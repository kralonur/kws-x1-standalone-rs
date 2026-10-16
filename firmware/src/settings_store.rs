//! Saved settings: 8-byte records appended to one claimed sector of the
//! original settings area through `crate::flash` (never erased). The newest
//! valid record wins; a full sector moves to the next blank one. The
//! calibration has a record per field beside the settings word; a newly
//! claimed sector starts with all of them, so it never depends on older ones.

use firmware_core::calibration::{FIELDS, Found};
use firmware_core::settings::Settings;

use crate::flash::{BLANK, ENERGY_SECTORS, SECTOR, SECTORS, blank, read, write};

/// First word of a sector claimed for settings ("KWST"). The diagnostic log
/// claims its own sectors with a different word and never takes this one.
const CLAIM: u32 = 0x4b57_5354;
/// Record: the encoded settings word, then its complement.
const RECORD: usize = 8;
/// First record, after the claim word, keeping records 8-byte aligned.
const FIRST: usize = 8;
/// Fills a sector too short for the next records: a word whose check is
/// not its complement, which every scan, older builds' too, skips.
const FILLER: u32 = 0;

/// Result of the last changed-settings save attempt.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Status {
    #[default]
    Saved,
    NoSpace,
    WriteFailed,
}

/// Where the next record goes, and what was last attempted.
pub struct Store {
    /// Sector base and the next free record, `None` when no claimed sector has room.
    next: Option<(usize, usize)>,
    saved: Settings,
    status: Status,
}

/// The newest settings word and calibration records found.
#[derive(Clone, Copy, Default)]
struct Records {
    settings: Option<Settings>,
    calibration: Found,
}

/// Read a claimed sector's valid records into `found`, later ones winning,
/// and return its first free record.
fn scan(base: usize, found: &mut Records) -> Option<usize> {
    let mut slot = base + FIRST;
    while slot + RECORD <= base + SECTOR {
        let (word, check) = (read(slot), read(slot + 4));
        if word == BLANK && check == BLANK {
            return Some(slot);
        }
        // A record interrupted mid-write fails the check and is skipped.
        if check == !word && !found.calibration.read(word) {
            found.settings = Settings::decode(word).or(found.settings);
        }
        slot += RECORD;
    }
    None
}

/// Settings sectors are claimed from the opposite end of `SECTORS` to the
/// diagnostic log.
fn candidates() -> impl Iterator<Item = usize> {
    SECTORS.iter().rev().copied()
}

impl Store {
    /// Find the saved settings. Sectors are claimed in `candidates` order
    /// and only the newest has room, so reading them in that order leaves
    /// the newest records; a full sector still supplies what no newer one
    /// replaced.
    pub fn open() -> (Self, Option<Settings>) {
        let mut next = None;
        let mut found = Records::default();
        for base in candidates().filter(|&base| read(base) == CLAIM) {
            if let Some(slot) = scan(base, &mut found) {
                next = next.or(Some((base, slot)));
            }
        }
        let loaded = (found.settings.is_some() || found.calibration.any()).then(|| Settings {
            calibration: found.calibration.calibration(),
            ..found.settings.unwrap_or_default()
        });
        let store = Self {
            next,
            saved: loaded.unwrap_or_default(),
            status: Status::Saved,
        };
        (store, loaded)
    }

    pub fn status(&self) -> Status {
        self.status
    }

    /// Append `settings` unless they are what was last attempted: the
    /// settings word if it changed, the calibration records if it did. One
    /// attempt per change: a failed write is not retried, the settings stay
    /// in RAM.
    pub fn save(&mut self, settings: Settings) -> bool {
        if settings == self.saved {
            return self.status == Status::Saved;
        }
        let word = settings.encode();
        let mut records = [0; 1 + FIELDS];
        records[0] = word;
        records[1..].copy_from_slice(&settings.calibration.records());
        let changed = match (
            word != self.saved.encode(),
            settings.calibration != self.saved.calibration,
        ) {
            (true, true) => &records[..],
            (true, false) => &records[..1],
            _ => &records[1..],
        };
        self.saved = settings;
        self.status = Status::NoSpace;
        let (base, mut slot, records) = match self.next {
            Some((base, slot)) if slot + RECORD * changed.len() <= base + SECTOR => {
                (base, slot, changed)
            }
            // A new sector starts with everything.
            next => {
                // Only one claimed sector may have room, as `open` takes it
                // for the newest: fill what this one has left.
                if let Some((base, mut slot)) = next {
                    while slot + RECORD <= base + SECTOR {
                        write(base, slot, &[FILLER, FILLER]);
                        slot += RECORD;
                    }
                    self.next = None;
                }
                let claimed = candidates()
                    .filter(|base| !ENERGY_SECTORS.contains(base))
                    .find(|&base| blank(base))
                    .and_then(|base| {
                        self.status = Status::WriteFailed;
                        write(base, base, &[CLAIM]).then_some(base)
                    });
                let Some(base) = claimed else {
                    self.next = None;
                    return false;
                };
                (base, base + FIRST, &records[..])
            }
        };
        let mut written = true;
        for &record in records {
            written = write(base, slot, &[record, !record]);
            // Move past the slot even after a failure: it may be partly programmed.
            slot += RECORD;
            if !written {
                break;
            }
        }
        self.status = if written {
            Status::Saved
        } else {
            Status::WriteFailed
        };
        self.next = (slot + RECORD <= base + SECTOR).then_some((base, slot));
        written
    }
}
