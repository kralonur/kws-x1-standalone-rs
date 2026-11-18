//! USB install: the unpacker takes the upper region from the flasher over USB HID.

use crate::{
    APP_BASE, APP_LIMIT, CONTAINER_PREFIX, EXTENDED_PAYLOAD_LEN, SECTOR_BYTES, UPPER_BASE,
    UPPER_END,
    packed::{PACKED_MAGIC, Packed, UPPER_OVER_USB},
    region_hash,
};

/// Shared generic HID ID; the flasher also matches both strings.
pub const VID: u16 = 0x16c0;
pub const PID: u16 = 0x05df;
pub const MANUFACTURER: &str = "usb.pd.epr.240w@gmail.com";
pub const PRODUCT: &str = "KWS-X1 Installer";

pub const REPORT_BYTES: usize = 64;
/// Region bytes carried by one DATA report, after its 8-byte header.
pub const DATA_BYTES: usize = 56;
pub const VERSION: u8 = 2;
pub type Report = [u8; REPORT_BYTES];
/// A USB install image's filled record words, as the packer writes them.
pub type Identity = [u32; 7];

const MAGIC: [u8; 4] = *b"X1UP";
const QUERY: u8 = b'Q';
const DATA: u8 = b'D';
const INFO: u8 = b'I';
const ACK: u8 = b'A';
const SECTOR: usize = SECTOR_BYTES as usize;

/// Vendor-defined HID: one 64-byte input and output report.
pub const REPORT_DESCRIPTOR: [u8; 25] = [
    0x06, 0x00, 0xff, 0x09, 0x01, 0xa1, 0x01, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x40,
    0x09, 0x01, 0x81, 0x02, 0x09, 0x01, 0x91, 0x02, 0xc0,
];

/// Present in an unpacker built with USB install; `firmware-pack` checks for it.
pub const UNPACKER_MAGIC: [u32; 2] = [0x5553_4249, 0x0b6f_3a52];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Status {
    /// The sector is programmed; send from `next`.
    Next = 0,
    /// The whole region is programmed and its hash matches.
    Done = 1,
    /// The report was out of order; resend from `next`.
    BadOffset = 2,
    WriteFailed = 3,
    HashMismatch = 4,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reply {
    Info { needed: bool, identity: Identity },
    Ack { status: Status, next: u32 },
}

/// The record words of a USB install image; `record` has no upper stream.
pub fn identity(record: &Packed) -> Identity {
    [
        record.data,
        record.upper.len,
        record.upper.hash,
        UPPER_OVER_USB,
        record.main.len,
        record.main.hash,
        record.main.packed,
    ]
}

fn word(bytes: &[u8], at: usize) -> Option<u32> {
    Some(u32::from_le_bytes(*bytes.get(at..)?.first_chunk()?))
}

fn words<const N: usize>(bytes: &[u8], at: usize) -> Option<[u32; N]> {
    let mut out = [0; N];
    for (index, slot) in out.iter_mut().enumerate() {
        *slot = word(bytes, at + 4 * index)?;
    }
    Some(out)
}

fn put_words(out: &mut [u8], values: &[u32]) {
    for (chunk, value) in out.as_chunks_mut::<4>().0.iter_mut().zip(values) {
        *chunk = value.to_le_bytes();
    }
}

pub fn query() -> Report {
    let mut report = [0; REPORT_BYTES];
    report[0] = QUERY;
    report[1..5].copy_from_slice(&MAGIC);
    report[5] = VERSION;
    report
}

/// A DATA report; `bytes` holds at most [`DATA_BYTES`].
pub fn data(offset: u32, bytes: &[u8]) -> Option<Report> {
    if bytes.is_empty() || bytes.len() > DATA_BYTES {
        return None;
    }
    let mut report = [0; REPORT_BYTES];
    report[0] = DATA;
    report[1] = bytes.len() as u8;
    report[4..8].copy_from_slice(&offset.to_le_bytes());
    report[8..8 + bytes.len()].copy_from_slice(bytes);
    Some(report)
}

fn info(needed: bool, identity: &Identity) -> Report {
    let mut report = [0; REPORT_BYTES];
    report[0] = INFO;
    report[1] = VERSION;
    report[2] = u8::from(needed);
    put_words(&mut report[4..4 + 4 * identity.len()], identity);
    report
}

fn ack(status: Status, next: usize) -> Report {
    let mut report = [0; REPORT_BYTES];
    report[0] = ACK;
    report[1] = status as u8;
    report[4..8].copy_from_slice(&(next as u32).to_le_bytes());
    report
}

/// Decode a device report.
pub fn parse(report: &[u8]) -> Option<Reply> {
    match *report.first()? {
        INFO if *report.get(1)? == VERSION => Some(Reply::Info {
            needed: *report.get(2)? != 0,
            identity: words(report, 4)?,
        }),
        ACK => {
            let status = match *report.get(1)? {
                0 => Status::Next,
                1 => Status::Done,
                2 => Status::BadOffset,
                3 => Status::WriteFailed,
                4 => Status::HashMismatch,
                _ => return None,
            };
            Some(Reply::Ack {
                status,
                next: word(report, 4)?,
            })
        }
        _ => None,
    }
}

/// Device side: assembles DATA reports into sectors.
pub struct Receiver {
    identity: Identity,
    len: usize,
    present: bool,
    queried: bool,
    done: bool,
    at: usize,
    fill: usize,
}

impl Receiver {
    /// `present`: the installed upper region already matches `identity`.
    pub const fn new(identity: Identity, present: bool) -> Self {
        Self {
            identity,
            len: identity[1] as usize,
            present,
            queried: false,
            done: false,
            at: 0,
            fill: 0,
        }
    }

    pub const fn present(&self) -> bool {
        self.present
    }

    /// A host has sent a valid QUERY.
    pub const fn queried(&self) -> bool {
        self.queried
    }

    /// The region is installed and the host has been told.
    pub const fn done(&self) -> bool {
        self.done
    }

    /// Handle one host report; `sector` is one flash sector long.
    pub fn handle(
        &mut self,
        report: &[u8],
        sector: &mut [u8],
        mut program: impl FnMut(usize, &[u8]) -> bool,
        verify: impl FnOnce() -> bool,
    ) -> Option<Report> {
        match *report.first()? {
            QUERY if report.get(1..5)? == MAGIC && *report.get(5)? == VERSION => {
                self.queried = true;
                self.done = self.present;
                (self.at, self.fill) = (0, 0);
                Some(info(!self.present, &self.identity))
            }
            DATA => {
                let count = usize::from(*report.get(1)?);
                let offset = word(report, 4)? as usize;
                let bytes = report.get(8..8 + count);
                let end = self.fill + count;
                let (Some(bytes), true) = (
                    bytes,
                    self.queried
                        && !self.done
                        && (1..=DATA_BYTES).contains(&count)
                        && sector.len() == SECTOR
                        && offset == self.at + self.fill
                        && end <= SECTOR
                        && self.at + end <= self.len,
                ) else {
                    self.fill = 0;
                    return Some(ack(Status::BadOffset, self.at));
                };
                sector[self.fill..end].copy_from_slice(bytes);
                self.fill = end;
                if end != SECTOR && self.at + end != self.len {
                    return None;
                }
                self.fill = 0;
                if !program(self.at, &sector[..end]) {
                    return Some(ack(Status::WriteFailed, self.at));
                }
                self.at += end;
                if self.at != self.len {
                    return Some(ack(Status::Next, self.at));
                }
                self.at = 0;
                self.done = verify();
                Some(if self.done {
                    ack(Status::Done, self.len)
                } else {
                    ack(Status::HashMismatch, 0)
                })
            }
            _ => None,
        }
    }
}

/// Marks the upper region appended to a container file.
pub const SECTION_MAGIC: [u8; 8] = *b"X1UPPER\x02";
pub const SECTION_HEADER_BYTES: usize = SECTION_MAGIC.len() + 4 * 7;
/// The container's length in a USB install file.
pub const CONTAINER_BYTES: usize = CONTAINER_PREFIX.len() + EXTENDED_PAYLOAD_LEN;

/// The header that precedes the upper region in a USB install file.
pub fn section_header(identity: &Identity) -> [u8; SECTION_HEADER_BYTES] {
    let mut header = [0; SECTION_HEADER_BYTES];
    header[..8].copy_from_slice(&SECTION_MAGIC);
    put_words(&mut header[8..], identity);
    header
}

/// A USB install file's parts.
pub struct Section<'a> {
    pub identity: Identity,
    pub upper: &'a [u8],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SectionError {
    /// A USB install container without its upper region; it cannot complete.
    Missing,
    Invalid,
}

/// The record words of a USB install image in `container`, if it holds one.
fn usb_record(container: &[u8]) -> Option<Identity> {
    let payload = container.get(CONTAINER_PREFIX.len()..)?;
    let mut magic = [0; 8];
    put_words(&mut magic, &PACKED_MAGIC);
    payload
        .windows(magic.len())
        .enumerate()
        .filter(|(_, window)| *window == magic)
        .find_map(|(at, _)| {
            let identity: Identity = words(payload, at + magic.len())?;
            let plausible = identity[3] == UPPER_OVER_USB
                && (APP_BASE..APP_BASE + APP_LIMIT as u32).contains(&identity[0])
                && identity[1].is_multiple_of(4)
                && identity[1] <= UPPER_END - UPPER_BASE;
            plausible.then_some(identity)
        })
}

/// Split off a USB install file's section, checked against the container's record.
pub fn split(file: &[u8]) -> Result<(&[u8], Option<Section<'_>>), SectionError> {
    if file.len() <= CONTAINER_BYTES {
        return match usb_record(file) {
            Some(_) => Err(SectionError::Missing),
            None => Ok((file, None)),
        };
    }
    let (container, section) = file.split_at(CONTAINER_BYTES);
    let upper = section
        .get(SECTION_HEADER_BYTES..)
        .ok_or(SectionError::Invalid)?;
    let identity: Identity = words(section, SECTION_MAGIC.len()).ok_or(SectionError::Invalid)?;
    let mut record = [0; 4 * (2 + 7)];
    put_words(&mut record, &PACKED_MAGIC);
    put_words(&mut record[8..], &identity);
    let valid = section[..SECTION_MAGIC.len()] == SECTION_MAGIC
        && identity[3] == UPPER_OVER_USB
        && identity[1] as usize == upper.len()
        && identity[2] == region_hash(upper)
        && !upper.is_empty()
        && upper.len().is_multiple_of(4)
        && upper.len() <= (UPPER_END - UPPER_BASE) as usize
        && container[CONTAINER_PREFIX.len()..]
            .windows(record.len())
            .any(|window| window == record);
    if valid {
        Ok((container, Some(Section { identity, upper })))
    } else {
        Err(SectionError::Invalid)
    }
}
