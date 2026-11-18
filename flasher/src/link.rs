//! USB install: send the upper region to the unpacker over USB HID.

use anyhow::{Context, Result, bail, ensure};
use firmware_image::{
    SECTOR_BYTES,
    link::{self, DATA_BYTES, Identity, MANUFACTURER, PID, PRODUCT, Reply, Report, Status, VID},
};
use hidapi::{HidApi, HidDevice};
use std::{
    thread,
    time::{Duration, Instant},
};

// The unpacker checks the main stream before it enumerates.
const APPEAR_TIMEOUT: Duration = Duration::from_secs(90);
const APPEAR_POLL: Duration = Duration::from_millis(250);
// One sector erase and program, with margin.
const REPLY_TIMEOUT: Duration = Duration::from_secs(15);
const SECTOR: usize = SECTOR_BYTES as usize;

/// A USB install file's record words and upper region.
pub struct Part {
    pub identity: Identity,
    pub upper: Vec<u8>,
}

pub trait Port {
    fn send(&self, report: &Report) -> Result<()>;
    fn receive(&self, timeout: Duration) -> Result<Option<Report>>;
}

impl Port for HidDevice {
    fn send(&self, report: &Report) -> Result<()> {
        // HIDAPI requires a report ID byte; the link uses unnumbered reports.
        let mut buffer = [0; link::REPORT_BYTES + 1];
        buffer[1..].copy_from_slice(report);
        let written = self.write(&buffer)?;
        ensure!(
            written == buffer.len(),
            "Short HID write: {written}/65 bytes"
        );
        Ok(())
    }

    fn receive(&self, timeout: Duration) -> Result<Option<Report>> {
        let mut report = [0; link::REPORT_BYTES];
        let millis = i32::try_from(timeout.as_millis().max(1))?;
        let read = self.read_timeout(&mut report, millis)?;
        Ok((read == report.len()).then_some(report))
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum Outcome {
    Sent,
    /// The meter already held this build's upper region.
    AlreadyInstalled,
}

/// The waiting installer to resume; `--device` always starts from the bootloader.
pub fn resume(installers: &[String], requested: Option<&str>) -> Result<Option<String>> {
    match (requested, installers) {
        (Some(_), _) | (None, []) => Ok(None),
        (None, [only]) => Ok(Some(only.clone())),
        (None, _) => {
            bail!("Several meters are waiting in the installer; connect only one: {installers:?}")
        }
    }
}

/// The next reply `wanted` accepts; other reports are stale and skipped.
fn reply<T>(port: &impl Port, what: &str, wanted: impl Fn(Reply) -> Option<T>) -> Result<T> {
    let deadline = Instant::now() + REPLY_TIMEOUT;
    while let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
        if let Some(found) = port
            .receive(remaining)?
            .and_then(|report| link::parse(&report))
            .and_then(&wanted)
        {
            return Ok(found);
        }
    }
    bail!(
        "{what}: no reply within {:.0}s",
        REPLY_TIMEOUT.as_secs_f32()
    )
}

/// Check the installer is this file's whole build, then send the upper region.
pub fn install(port: &impl Port, part: &Part) -> Result<Outcome> {
    port.send(&link::query())?;
    let (needed, identity) = reply(port, "Installer query", |reply| match reply {
        Reply::Info { needed, identity } => Some((needed, identity)),
        Reply::Ack { .. } => None,
    })?;
    ensure!(
        identity == part.identity,
        "The meter's installer is from a different build than this file \
         (meter {identity:08x?}, file {:08x?}). Nothing was sent. Run the command \
         with the file this meter was flashed with, or hold Back+OK while connecting \
         USB and flash this file",
        part.identity
    );
    if !needed {
        return Ok(Outcome::AlreadyInstalled);
    }
    let upper = part.upper.as_slice();
    let mut offset = 0;
    while offset < upper.len() {
        let end = (offset + SECTOR).min(upper.len());
        let mut at = offset;
        for chunk in upper[offset..end].chunks(DATA_BYTES) {
            let report = link::data(at as u32, chunk).context("Building a data report")?;
            port.send(&report)?;
            at += chunk.len();
        }
        let (status, next) = reply(port, "Upper region sector", |reply| match reply {
            Reply::Ack { status, next } => Some((status, next)),
            Reply::Info { .. } => None,
        })?;
        match status {
            Status::Next if next as usize == end && end < upper.len() => {}
            Status::Done if next as usize == end && end == upper.len() => {
                return Ok(Outcome::Sent);
            }
            Status::WriteFailed => bail!(
                "The meter could not program its flash at upper offset {offset:#x}; \
                 run the same command again"
            ),
            Status::HashMismatch => bail!(
                "The installed upper region does not match this file; \
                 run the same command again"
            ),
            _ => bail!(
                "Unexpected installer reply {status:?} at upper offset {offset:#x}; \
                 run the same command again"
            ),
        }
        tracing::info!(
            bytes = end,
            total = upper.len(),
            "Upper region sector written"
        );
        offset = end;
    }
    bail!("The upper region is empty")
}

/// Paths of attached installers.
pub fn installers(api: &mut HidApi) -> Result<Vec<String>> {
    api.refresh_devices().context("Listing HID devices")?;
    Ok(api
        .device_list()
        .filter(|device| {
            device.vendor_id() == VID
                && device.product_id() == PID
                && device.manufacturer_string() == Some(MANUFACTURER)
                && device.product_string() == Some(PRODUCT)
        })
        .map(|device| device.path().to_string_lossy().into_owned())
        .collect())
}

/// Open the installer at `path`.
pub fn open(api: &HidApi, path: &str) -> Result<HidDevice> {
    let device = api
        .device_list()
        .find(|device| device.path().to_string_lossy() == path)
        .context("The installer disconnected")?;
    api.open_path(device.path())
        .context("Opening the installer; on Linux check hidraw permissions for 16c0:05df")
}

/// Wait for the installer to enumerate, then send it the region.
pub fn run(api: &mut HidApi, part: &Part) -> Result<Outcome> {
    tracing::info!("Waiting for the meter's installer; keep the cable connected");
    let deadline = Instant::now() + APPEAR_TIMEOUT;
    let mut last_error = None;
    while Instant::now() < deadline {
        match installers(api)?.as_slice() {
            [] => {}
            [path] => match open(api, path) {
                Ok(device) => return install(&device, part),
                // Permissions can settle a moment after the device appears.
                Err(error) => last_error = Some(error),
            },
            several => {
                bail!("Several installers are attached; connect only one meter: {several:?}")
            }
        }
        thread::sleep(APPEAR_POLL);
    }
    match last_error {
        Some(error) => Err(error),
        None => bail!(
            "The installer did not appear within {:.0}s. If the meter shows the \
             measurement screen, the update is complete. Otherwise keep it connected \
             and run the same command again",
            APPEAR_TIMEOUT.as_secs_f32()
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firmware_image::{link::Receiver, packed::UPPER_OVER_USB, region_hash};
    use std::{cell::RefCell, collections::VecDeque};

    /// The unpacker's receiver over a byte array standing in for flash.
    struct Meter {
        receiver: RefCell<Receiver>,
        flash: RefCell<Vec<u8>>,
        sector: RefCell<Vec<u8>>,
        replies: RefCell<VecDeque<Report>>,
        expected: u32,
        /// A sector offset that fails to program, and a byte to corrupt.
        fail_at: Option<usize>,
        corrupt: Option<usize>,
    }

    impl Meter {
        fn new(identity: Identity, present: bool) -> Self {
            Self {
                receiver: RefCell::new(Receiver::new(identity, present)),
                flash: RefCell::new(vec![0xff; identity[1] as usize]),
                sector: RefCell::new(vec![0; SECTOR]),
                replies: RefCell::new(VecDeque::new()),
                expected: identity[2],
                fail_at: None,
                corrupt: None,
            }
        }
    }

    impl Port for Meter {
        fn send(&self, report: &Report) -> Result<()> {
            let reply = self.receiver.borrow_mut().handle(
                report,
                &mut self.sector.borrow_mut(),
                |offset, bytes| {
                    if self.fail_at == Some(offset) {
                        return false;
                    }
                    self.flash.borrow_mut()[offset..offset + bytes.len()].copy_from_slice(bytes);
                    true
                },
                || {
                    let mut image = self.flash.borrow().clone();
                    if let Some(at) = self.corrupt {
                        image[at] ^= 1;
                    }
                    region_hash(&image) == self.expected
                },
            );
            self.replies.borrow_mut().extend(reply);
            Ok(())
        }

        fn receive(&self, _timeout: Duration) -> Result<Option<Report>> {
            Ok(self.replies.borrow_mut().pop_front())
        }
    }

    fn part(len: usize) -> Part {
        let upper: Vec<u8> = (0..len).map(|index| (index * 31 % 251) as u8).collect();
        let identity = [
            0x2_0000,
            len as u32,
            region_hash(&upper),
            UPPER_OVER_USB,
            0x2_4000,
            0x1234_5678,
            0x1_c000,
        ];
        Part { identity, upper }
    }

    fn blank(meter: &Meter) -> bool {
        meter.flash.borrow().iter().all(|byte| *byte == 0xff)
    }

    #[test]
    fn sends_a_region_with_a_partial_last_sector() {
        let part = part(3 * SECTOR + 1016);
        let meter = Meter::new(part.identity, false);
        assert_eq!(install(&meter, &part).unwrap(), Outcome::Sent);
        assert_eq!(*meter.flash.borrow(), part.upper);
        assert!(meter.receiver.borrow().done());
    }

    #[test]
    fn sends_a_region_of_whole_sectors() {
        let part = part(2 * SECTOR);
        let meter = Meter::new(part.identity, false);
        assert_eq!(install(&meter, &part).unwrap(), Outcome::Sent);
        assert_eq!(*meter.flash.borrow(), part.upper);
    }

    #[test]
    fn skips_an_installed_region() {
        let part = part(SECTOR + 8);
        let meter = Meter::new(part.identity, true);
        assert_eq!(install(&meter, &part).unwrap(), Outcome::AlreadyInstalled);
        assert!(meter.receiver.borrow().done());
        assert!(blank(&meter));
    }

    #[test]
    fn refuses_another_builds_installer() {
        let part = part(SECTOR);
        let mut identity = part.identity;
        identity[2] ^= 1;
        let meter = Meter::new(identity, false);
        let error = install(&meter, &part).unwrap_err().to_string();
        assert!(error.contains("different build"), "{error}");
        assert!(blank(&meter));
    }

    #[test]
    fn refuses_an_installer_with_the_same_upper_and_another_main() {
        let part = part(SECTOR);
        let mut identity = part.identity;
        identity[5] ^= 1;
        let meter = Meter::new(identity, true);
        let error = install(&meter, &part).unwrap_err().to_string();
        assert!(error.contains("different build"), "{error}");
        assert!(blank(&meter));
    }

    #[test]
    fn reports_a_failed_sector_write() {
        let part = part(2 * SECTOR + 4);
        let mut meter = Meter::new(part.identity, false);
        meter.fail_at = Some(SECTOR);
        let error = install(&meter, &part).unwrap_err().to_string();
        assert!(error.contains("could not program"), "{error}");
        assert!(!meter.receiver.borrow().done());
    }

    #[test]
    fn reports_a_hash_mismatch_and_allows_a_retry() {
        let part = part(SECTOR + 400);
        let mut meter = Meter::new(part.identity, false);
        meter.corrupt = Some(5);
        let error = install(&meter, &part).unwrap_err().to_string();
        assert!(error.contains("does not match"), "{error}");
        assert!(!meter.receiver.borrow().done());
        meter.corrupt = None;
        assert_eq!(install(&meter, &part).unwrap(), Outcome::Sent);
    }

    #[test]
    fn a_query_restarts_an_interrupted_transfer() {
        let part = part(2 * SECTOR);
        let meter = Meter::new(part.identity, false);
        meter.send(&link::query()).unwrap();
        meter
            .send(&link::data(0, &part.upper[..DATA_BYTES]).unwrap())
            .unwrap();
        assert_eq!(install(&meter, &part).unwrap(), Outcome::Sent);
        assert_eq!(*meter.flash.borrow(), part.upper);
    }

    #[test]
    fn rejects_out_of_order_and_overlong_data() {
        let part = part(DATA_BYTES + 4);
        let meter = Meter::new(part.identity, false);
        let upper = &part.upper;
        // Data before a query.
        meter.send(&link::data(0, &upper[..8]).unwrap()).unwrap();
        // A skipped offset, then a report past the region's end.
        meter.send(&link::query()).unwrap();
        meter.send(&link::data(8, &upper[..8]).unwrap()).unwrap();
        meter
            .send(&link::data(0, &upper[..DATA_BYTES]).unwrap())
            .unwrap();
        meter
            .send(&link::data(DATA_BYTES as u32, &upper[..8]).unwrap())
            .unwrap();
        let replies: Vec<_> = meter
            .replies
            .borrow()
            .iter()
            .filter_map(|report| link::parse(report))
            .collect();
        let bad = Reply::Ack {
            status: Status::BadOffset,
            next: 0,
        };
        assert_eq!(replies[0], bad);
        assert!(matches!(replies[1], Reply::Info { needed: true, .. }));
        assert_eq!(replies[2], bad);
        assert_eq!(replies[3], bad);
        assert_eq!(replies.len(), 4);
        assert!(blank(&meter));
    }

    #[test]
    fn splits_a_usb_install_file() {
        use firmware_image::{
            CONTAINER_PREFIX,
            link::{CONTAINER_BYTES, section_header, split},
            packed::PACKED_MAGIC,
        };
        let part = part(SECTOR + 12);
        let mut file = vec![0; CONTAINER_BYTES];
        assert!(split(&file).unwrap().1.is_none());
        file.extend_from_slice(&section_header(&part.identity));
        file.extend_from_slice(&part.upper);
        // The container must hold the same record.
        assert!(split(&file).is_err());
        let record: Vec<u8> = PACKED_MAGIC
            .iter()
            .chain(&part.identity)
            .flat_map(|word| word.to_le_bytes())
            .collect();
        let at = CONTAINER_PREFIX.len() + 0x100;
        file[at..at + record.len()].copy_from_slice(&record);
        let (container, section) = split(&file).unwrap();
        let section = section.unwrap();
        assert_eq!(container.len(), CONTAINER_BYTES);
        assert_eq!(
            (section.identity, section.upper),
            (part.identity, &part.upper[..])
        );
        let last = file.len() - 1;
        file[last] ^= 1;
        assert!(split(&file).is_err());
        file[last] ^= 1;
        file.truncate(last);
        assert!(split(&file).is_err());
    }

    #[test]
    fn resumes_only_a_single_installer_without_device() {
        let paths = |names: &[&str]| {
            names
                .iter()
                .map(|name| name.to_string())
                .collect::<Vec<_>>()
        };
        assert_eq!(resume(&paths(&[]), None).unwrap(), None);
        assert_eq!(resume(&paths(&["a"]), None).unwrap(), Some("a".into()));
        assert!(resume(&paths(&["a", "b"]), None).is_err());
        assert_eq!(resume(&paths(&["a"]), Some("x")).unwrap(), None);
    }
}
