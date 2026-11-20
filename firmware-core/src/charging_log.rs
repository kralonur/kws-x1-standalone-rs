//! Append-only charging log format and random-access, read-only CSV/FAT12 export.
//! All integers are little endian. The checksum word is programmed last.
/// Hardware erase sector size in bytes.
pub const SECTOR: usize = 8192;
/// XSL2/XSP1 sector header size, in bytes.
pub const HEADER: usize = 16;
/// XSL2/XSP1 entry size, including the commit checksum, in bytes.
pub const RECORD: usize = 16;
/// Number of entries that fit after the sector header.
pub const SAMPLES: usize = (SECTOR - HEADER) / RECORD;
// XSL2: one-second VI extrema summaries; XSP1: selective PD fragments.
const EXTREMA_MAGIC: u32 = 0x324c_5358;
const SELECTIVE_EVENT_MAGIC: u32 = 0x3150_5358;
/// Session limit used by the journal and the fixed-size export directory.
pub const MAX_SESSIONS: u32 = 32;
/// Maximum words in a PD packet: the header plus seven data objects.
pub const PACKET_WORDS: usize = 8;
/// The legacy three-sector allocation footer reserves even blank padding.
const LEGACY_GROUP_SECTORS: usize = 3;
const LEGACY_FOOTER_OFFSET: usize = SECTOR - 20;
const LEGACY_FOOTER: [u8; 8] = [0x58, 0x53, 0x43, 0x31, 0x1e, 0x65, 0xa2, 0xd9];
/// Number of erase sectors reserved for the journal.
pub const SECTORS: usize = 6;

/// Packet fragments may cross a sector, but never start one that cannot fit.
pub fn packet_fits(remaining: usize, words: usize, spare: bool) -> bool {
    words <= PACKET_WORDS && (words <= remaining || spare)
}

pub fn header(session: u32, segment: u32, event: bool) -> [u32; 4] {
    let mut w = [
        if event {
            SELECTIVE_EVENT_MAGIC
        } else {
            EXTREMA_MAGIC
        },
        session,
        segment,
        0,
    ];
    w[3] = checksum(&w[..3]);
    w
}

/// Reserved elapsed value; normal data must stop before u32::MAX milliseconds.
pub fn session_marker(session: u32) -> [u32; 4] {
    record(u32::MAX, session, 0)
}
pub fn is_event(bytes: &[u8]) -> bool {
    words(bytes).is_some_and(|w| w[0] == SELECTIVE_EVENT_MAGIC)
}
fn marker(w: [u32; 3]) -> Option<u32> {
    (w[0] == u32::MAX && w[2] == 0).then_some(w[1])
}
pub fn committed_slots(bytes: &[u8]) -> usize {
    if valid_header(bytes).is_none() {
        return 0;
    }
    bytes[HEADER..]
        .as_chunks::<RECORD>()
        .0
        .iter()
        .take(SAMPLES)
        .take_while(|&r| valid_record(r).is_some())
        .count()
}
pub fn latest_session(bytes: &[u8]) -> u32 {
    let Some(mut latest) = valid_header(bytes) else {
        return 0;
    };
    for r in bytes[HEADER..].as_chunks::<RECORD>().0.iter().take(SAMPLES) {
        let Some(w) = valid_record(r) else { break };
        if let Some(session) = marker(w) {
            latest = latest.max(session);
        }
    }
    latest
}
/// Never write past a torn record or append to an unrecognized header.
pub fn reusable_tail(bytes: &[u8], event: bool) -> Option<usize> {
    if bytes.len() != SECTOR || is_event(bytes) != event || valid_header(bytes).is_none() {
        return None;
    }
    let n = committed_slots(bytes);
    (n + 1 < SAMPLES && erased(&bytes[HEADER + n * RECORD..])).then_some(n)
}

fn erased(bytes: &[u8]) -> bool {
    bytes.iter().all(|&b| b == u8::MAX)
}

/// Nonblank data is never free, nor is erased padding in a legacy allocation.
pub fn sector_free(sectors: [&[u8]; SECTORS], index: usize) -> bool {
    let Some(sector) = sectors.get(index) else {
        return false;
    };
    let footer_sector =
        sectors[index / LEGACY_GROUP_SECTORS * LEGACY_GROUP_SECTORS + LEGACY_GROUP_SECTORS - 1];
    let footer =
        footer_sector.get(LEGACY_FOOTER_OFFSET..LEGACY_FOOTER_OFFSET + LEGACY_FOOTER.len());
    footer != Some(&LEGACY_FOOTER) && sector.len() == SECTOR && erased(sector)
}
pub const BLOCK_BYTES: usize = 512;
pub const BLOCKS: u32 = 512;
const ROOT_BLOCKS: usize = 5; // 80 entries: volume + 64 streams + status and raw storage
const FIRST_DATA: usize = 3 + ROOT_BLOCKS;
const FILES: usize = MAX_SESSIONS as usize * 2;
const RAW_BLOCKS: usize = SECTORS * SECTOR / BLOCK_BYTES;
const EVENT_CSV_HEADER: &[u8] = b"session,elapsed_ms,event_tag,raw_u32\r\n";
const EXTREMA_CSV_HEADER: &[u8] =
    b"session,elapsed_ms,avg_voltage_mV,avg_current_mA,min_voltage_mV,peak_current_mA\r\n";
// XSP1: three ten-digit fields, one eleven-digit field, commas and CRLF.
const ROW: usize = 46;
// XSL2: ten-digit session and time, two 5-digit voltages, two signed
// 5-digit currents, five separators and CRLF. Only the USB view is wider.
const MAX_ROW: usize = 49;

// Reflected CRC-32 polynomial; XSL2/XSP1 use the low 31 bits of the running CRC.
const CRC_POLYNOMIAL: u32 = 0xedb8_8320;

fn checksum(words: &[u32]) -> u32 {
    let mut crc = !0u32;
    for word in words {
        for byte in word.to_le_bytes() {
            crc ^= u32::from(byte);
            for _ in 0..8 {
                crc = (crc >> 1) ^ (CRC_POLYNOMIAL & 0u32.wrapping_sub(crc & 1));
            }
        }
    }
    // Never use the erased value as a commit marker.
    crc & 0x7fffffff
}

pub fn record(elapsed_ms: u32, mv: u32, ma: i32) -> [u32; 4] {
    let mut w = [elapsed_ms, mv, ma as u32, 0];
    w[3] = checksum(&w[..3]);
    w
}
/// An XSL2 summary: elapsed milliseconds (< u32::MAX), then average and
/// extrema pairs. Each pair has unsigned mV low and two's-complement mA high.
pub fn extrema_record(at: u32, avg_mv: u16, avg_ma: i16, min_mv: u16, peak_ma: i16) -> [u32; 4] {
    let pair = |mv: u16, ma: i16| u32::from(mv) | (u32::from(ma as u16) << 16);
    record(at, pair(avg_mv, avg_ma), pair(min_mv, peak_ma) as i32)
}
fn words(bytes: &[u8]) -> Option<[u32; 4]> {
    let b = bytes.first_chunk::<RECORD>()?;
    let w = b.as_chunks::<4>().0;
    Some(core::array::from_fn(|i| u32::from_le_bytes(w[i])))
}
pub fn valid_header(bytes: &[u8]) -> Option<u32> {
    let w = words(bytes)?;
    (matches!(w[0], SELECTIVE_EVENT_MAGIC | EXTREMA_MAGIC) && w[3] == checksum(&w[..3]))
        .then_some(w[1])
}
pub fn valid_record(bytes: &[u8]) -> Option<[u32; 3]> {
    let w = words(bytes)?;
    (w[3] == checksum(&w[..3])).then_some([w[0], w[1], w[2]])
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Stream {
    Pd,
    Extrema,
}
impl Stream {
    fn of(sector: &[u8]) -> Self {
        if is_event(sector) {
            Self::Pd
        } else {
            Self::Extrema
        }
    }
}

#[derive(Clone, Copy)]
struct File {
    session: u32,
    rows: usize,
    stream: Stream,
}
impl File {
    fn row_len(self) -> usize {
        if self.stream == Stream::Extrema {
            MAX_ROW
        } else {
            ROW
        }
    }
}
const EMPTY_FILE: File = File {
    session: 0,
    rows: 0,
    stream: Stream::Extrema,
};
pub struct Disk<'a> {
    sectors: [&'a [u8]; SECTORS],
    counts: [usize; SECTORS],
    order: [usize; SECTORS],
    files: [File; FILES],
    nfiles: usize,
}
impl<'a> Disk<'a> {
    pub fn new(sectors: [&'a [u8]; SECTORS]) -> Self {
        let counts = sectors.map(committed_slots);
        let mut order = core::array::from_fn(|i| i);
        order.sort_unstable_by_key(|&i| {
            let w = words(sectors[i]).unwrap_or([u32::MAX; 4]);
            (w[1], w[2], i)
        });
        let mut disk = Self {
            sectors,
            counts,
            order,
            files: [EMPTY_FILE; FILES],
            nfiles: 0,
        };
        for i in order {
            let Some(mut session) = valid_header(sectors[i]) else {
                continue;
            };
            let stream = Stream::of(sectors[i]);
            for r in sectors[i][HEADER..]
                .as_chunks::<RECORD>()
                .0
                .iter()
                .take(counts[i])
            {
                let Some(w) = valid_record(r) else { break };
                if let Some(next) = marker(w) {
                    session = next;
                    continue;
                }
                let file = (0..disk.nfiles)
                    .find(|&j| disk.files[j].session == session && disk.files[j].stream == stream);
                let j = if let Some(j) = file {
                    j
                } else {
                    if disk.nfiles == FILES {
                        continue;
                    }
                    let j = disk.nfiles;
                    disk.nfiles += 1;
                    disk.files[j] = File {
                        session,
                        rows: 0,
                        stream,
                    };
                    j
                };
                disk.files[j].rows += 1;
            }
        }
        // At most 64 directory entries, sorted once at mount. Insertion sort
        // avoids linking a general-purpose sorter into the main install region.
        let files = &mut disk.files[..disk.nfiles];
        for i in 1..files.len() {
            let file = files[i];
            let mut j = i;
            while j > 0 && (file.session, file.stream) < (files[j - 1].session, files[j - 1].stream)
            {
                files[j] = files[j - 1];
                j -= 1;
            }
            files[j] = file;
        }
        disk
    }
    fn filename(&self, i: usize) -> [u8; 11] {
        let f = self.files[i];
        if f.session > 9999 {
            let mut name = *b"LOG00000CSV";
            decimal(&mut name[3..8], i as u32 + 1);
            return name;
        }
        let mut name = *b"VI000002CSV";
        match f.stream {
            Stream::Extrema => {}
            Stream::Pd => {
                name[..2].copy_from_slice(b"PD");
                name[7] = b'1';
            }
        }
        decimal(&mut name[2..6], f.session);
        name
    }
    fn status(&self) -> [u8; BLOCK_BYTES] {
        let mut out = [b' '; BLOCK_BYTES];
        let heading = b"LOG STORAGE\r\nFREE SECTORS: 0/6\r\nSECTOR TYPE    FIRST ID   SLOTS\r\n";
        out[..heading.len()].copy_from_slice(heading);
        let free: [bool; SECTORS] = core::array::from_fn(|i| sector_free(self.sectors, i));
        out[b"LOG STORAGE\r\nFREE SECTORS: ".len()] =
            b'0' + free.iter().filter(|&&f| f).count() as u8;
        for i in 0..SECTORS {
            let mut row = *b"0      UNKNOWN 0000000000 0000\r\n";
            row[0] = b'1' + i as u8;
            let kind: &[u8; 7] = if free[i] {
                b"FREE   "
            } else if valid_header(self.sectors[i]).is_some() {
                if is_event(self.sectors[i]) {
                    b"PD SEL1"
                } else {
                    b"VI SEL2"
                }
            } else {
                b"UNKNOWN"
            };
            row[7..14].copy_from_slice(kind);
            decimal(&mut row[15..25], valid_header(self.sectors[i]).unwrap_or(0));
            decimal(&mut row[26..30], self.counts[i] as u32);
            let at = heading.len() + i * row.len();
            out[at..at + row.len()].copy_from_slice(&row);
        }
        let mut summary = *b"LATEST SESSION: 0000000000 / 0000000032\r\nLogger reuses tails; zero free sectors can still record.\r\n";
        decimal(
            &mut summary[16..26],
            self.sectors
                .map(latest_session)
                .into_iter()
                .max()
                .unwrap_or(0),
        );
        let at = heading.len() + SECTORS * 32;
        out[at..at + summary.len()].copy_from_slice(&summary);
        let policy = b"SEL2: 1s avg/min V/peak I.\r\n100mV/100mA; 60s heartbeat. Passive PD RX; known SOP routine omitted.\r\nNo clean-stop marker.\r\n";
        let at = at + summary.len();
        out[at..at + policy.len()].copy_from_slice(policy);
        out[510..].copy_from_slice(b"\r\n");
        out
    }
    fn csv_header(&self, i: usize) -> &'static [u8] {
        match self.files[i].stream {
            Stream::Pd => EVENT_CSV_HEADER,
            Stream::Extrema => EXTREMA_CSV_HEADER,
        }
    }
    fn len(&self, i: usize) -> usize {
        self.csv_header(i).len() + self.files[i].rows * self.files[i].row_len()
    }
    fn cluster(&self, i: usize) -> usize {
        2 + (0..i)
            .map(|j| self.len(j).div_ceil(BLOCK_BYTES))
            .sum::<usize>()
    }
    fn status_cluster(&self) -> usize {
        self.cluster(self.nfiles)
    }
    fn raw_cluster(&self) -> usize {
        self.status_cluster() + 1
    }
    fn row<'b>(&self, i: usize, mut index: usize, out: &'b mut [u8; MAX_ROW]) -> &'b [u8] {
        let f = self.files[i];
        let mut found = [0; 3];
        'sectors: for &j in &self.order {
            let Some(mut session) = valid_header(self.sectors[j]) else {
                continue;
            };
            if Stream::of(self.sectors[j]) != f.stream {
                continue;
            }
            for r in self.sectors[j][HEADER..]
                .as_chunks::<RECORD>()
                .0
                .iter()
                .take(self.counts[j])
            {
                // Disk is a read-only snapshot of the CRC-checked prefix above.
                let Some(v) = words(r) else { break };
                let w = [v[0], v[1], v[2]];
                if let Some(next) = marker(w) {
                    session = next;
                } else if session == f.session {
                    if index == 0 {
                        found = w;
                        break 'sectors;
                    }
                    index -= 1;
                }
            }
        }
        let row = &mut out[..f.row_len()];
        decimal(&mut row[..10], f.session);
        row[10] = b',';
        let fields = &mut row[11..];
        decimal(&mut fields[..10], found[0]);
        fields[10] = b',';
        if f.stream == Stream::Extrema {
            for (pair, field) in found[1..].iter().zip([11, 24]) {
                decimal(&mut fields[field..field + 5], u32::from(*pair as u16));
                fields[field + 5] = b',';
                signed_decimal(
                    &mut fields[field + 6..field + 12],
                    (*pair >> 16) as i16 as i32,
                );
            }
            fields[23] = b',';
        } else {
            decimal(&mut fields[11..21], found[1]);
            fields[21] = b',';
            decimal(&mut fields[22..33], found[2]);
        }
        let end = row.len();
        row[end - 2..].copy_from_slice(b"\r\n");
        row
    }
    pub fn read(&self, lba: u32, block: &mut [u8; BLOCK_BYTES]) {
        block.fill(0);
        match lba {
            0 => {
                block[..3].copy_from_slice(&[0xeb, 0x3c, 0x90]);
                block[3..11].copy_from_slice(b"MSDOS5.0");
                put16(block, 11, BLOCK_BYTES as u16);
                block[13] = 1;
                put16(block, 14, 1);
                block[16] = 1;
                put16(block, 17, (ROOT_BLOCKS * 16) as u16);
                put16(block, 19, BLOCKS as u16);
                block[21] = 0xf8;
                put16(block, 22, 2);
                put16(block, 24, 1);
                put16(block, 26, 1);
                block[38] = 0x29;
                put32(block, 39, EXTREMA_MAGIC);
                block[43..54].copy_from_slice(b"CHARGE LOG ");
                block[54..62].copy_from_slice(b"FAT12   ");
                block[510..].copy_from_slice(&[0x55, 0xaa]);
            }
            1..=2 => {
                let start = (lba as usize - 1) * BLOCK_BYTES;
                for cluster in 0..self.raw_cluster() + RAW_BLOCKS {
                    let value = if cluster == 0 {
                        0xff8
                    } else if cluster == 1 || cluster == self.status_cluster() {
                        0xfff
                    } else if cluster >= self.raw_cluster() {
                        if cluster + 1 == self.raw_cluster() + RAW_BLOCKS {
                            0xfff
                        } else {
                            (cluster + 1) as u16
                        }
                    } else {
                        if (0..self.nfiles).any(|i| {
                            cluster + 1 == self.cluster(i) + self.len(i).div_ceil(BLOCK_BYTES)
                        }) {
                            0xfff
                        } else {
                            (cluster + 1) as u16
                        }
                    };
                    let at = cluster + cluster / 2;
                    for (j, b) in (value << (4 * (cluster % 2)))
                        .to_le_bytes()
                        .iter()
                        .enumerate()
                    {
                        if let Some(out) = block.get_mut((at + j).wrapping_sub(start)) {
                            *out |= b;
                        }
                    }
                }
            }
            _ if (3..FIRST_DATA as u32).contains(&lba) => {
                for slot in 0..16 {
                    let entry = (lba as usize - 3) * 16 + slot;
                    let at = slot * 32;
                    if entry == 0 {
                        block[..11].copy_from_slice(b"CHARGE LOG ");
                        block[11] = 8;
                        continue;
                    }
                    let i = entry - 1;
                    let (name, cluster, len) = if i < self.nfiles {
                        (self.filename(i), self.cluster(i), self.len(i))
                    } else if i == self.nfiles {
                        (*b"STATUS  TXT", self.status_cluster(), BLOCK_BYTES)
                    } else if i == self.nfiles + 1 {
                        (*b"STORAGE BIN", self.raw_cluster(), SECTORS * SECTOR)
                    } else {
                        continue;
                    };
                    block[at..at + 11].copy_from_slice(&name);
                    block[at + 11] = 0x21;
                    put16(block, at + 24, 0x21);
                    put16(block, at + 26, cluster as u16);
                    put32(block, at + 28, len as u32);
                }
            }
            _ if lba >= FIRST_DATA as u32 && lba < BLOCKS => {
                let cluster = lba as usize - FIRST_DATA + 2;
                if cluster == self.status_cluster() {
                    block.copy_from_slice(&self.status());
                } else if (self.raw_cluster()..self.raw_cluster() + RAW_BLOCKS).contains(&cluster) {
                    let offset = (cluster - self.raw_cluster()) * BLOCK_BYTES;
                    if let Some(bytes) = self.sectors[offset / SECTOR]
                        .get(offset % SECTOR..offset % SECTOR + BLOCK_BYTES)
                    {
                        block.copy_from_slice(bytes);
                    }
                } else if let Some(i) = (0..self.nfiles).find(|&i| {
                    (self.cluster(i)..self.cluster(i) + self.len(i).div_ceil(BLOCK_BYTES))
                        .contains(&cluster)
                }) {
                    let start = (cluster - self.cluster(i)) * BLOCK_BYTES;
                    let end = (start + BLOCK_BYTES).min(self.len(i));
                    let mut at = start;
                    let row_len = self.files[i].row_len();
                    let mut row_buf = [0; MAX_ROW];
                    while at < end {
                        if at < self.csv_header(i).len() {
                            let n = (self.csv_header(i).len() - at).min(end - at);
                            block[at - start..at - start + n]
                                .copy_from_slice(&self.csv_header(i)[at..at + n]);
                            at += n;
                        } else {
                            let offset = at - self.csv_header(i).len();
                            let row = self.row(i, offset / row_len, &mut row_buf);
                            let within = offset % row_len;
                            let n = (row_len - within).min(end - at);
                            block[at - start..at - start + n]
                                .copy_from_slice(&row[within..within + n]);
                            at += n;
                        }
                    }
                }
            }
            _ => {}
        }
    }
}
fn decimal(out: &mut [u8], mut n: u32) {
    for b in out.iter_mut().rev() {
        *b = b'0' + (n % 10) as u8;
        n /= 10;
    }
}
fn signed_decimal(out: &mut [u8], n: i32) {
    out[0] = if n < 0 { b'-' } else { b'+' };
    decimal(&mut out[1..], n.unsigned_abs());
}
fn put16(b: &mut [u8], at: usize, v: u16) {
    b[at..at + 2].copy_from_slice(&v.to_le_bytes());
}
fn put32(b: &mut [u8], at: usize, v: u32) {
    b[at..at + 4].copy_from_slice(&v.to_le_bytes());
}
