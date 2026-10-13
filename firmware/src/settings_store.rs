//! Saved settings: 8-byte records appended to one claimed sector of the
//! original settings area through `crate::flash` (never erased). The newest
//! valid record wins; a full sector moves to the next blank one.

use firmware_core::settings::Settings;

use crate::flash::{BLANK, ENERGY_SECTORS, SECTOR, SECTORS, blank, read, write};

/// First word of a sector claimed for settings ("KWST"). The diagnostic log
/// claims its own sectors with a different word and never takes this one.
const CLAIM: u32 = 0x4b57_5354;
/// Record: the encoded settings word, then its complement.
const RECORD: usize = 8;
/// First record, after the claim word, keeping records 8-byte aligned.
const FIRST: usize = 8;

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

/// The newest valid record of a claimed sector and its first free record.
fn scan(base: usize) -> (Option<Settings>, Option<usize>) {
    let mut last = None;
    let mut slot = base + FIRST;
    while slot + RECORD <= base + SECTOR {
        let (word, check) = (read(slot), read(slot + 4));
        if word == BLANK && check == BLANK {
            return (last, Some(slot));
        }
        // A record interrupted mid-write fails the check and is skipped.
        if check == !word
            && let Some(settings) = Settings::decode(word)
        {
            last = Some(settings);
        }
        slot += RECORD;
    }
    (last, None)
}

/// Settings sectors are claimed from the opposite end of `SECTORS` to the
/// diagnostic log.
fn candidates() -> impl Iterator<Item = usize> {
    SECTORS.iter().rev().copied()
}

impl Store {
    /// Find the saved settings: the claimed sector with room is current; a
    /// full one still supplies settings until a newer record exists.
    pub fn open() -> (Self, Option<Settings>) {
        let mut next = None;
        let mut current = None;
        let mut older = None;
        for base in candidates().filter(|&base| read(base) == CLAIM) {
            let (last, free) = scan(base);
            match free {
                Some(slot) if next.is_none() => {
                    next = Some((base, slot));
                    current = last;
                }
                _ => older = older.or(last),
            }
        }
        let loaded = current.or(older);
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

    /// Append `settings` unless they are what was last attempted. One attempt per
    /// change: a failed write is not retried, the settings stay in RAM.
    pub fn save(&mut self, settings: Settings) -> bool {
        if settings == self.saved {
            return self.status == Status::Saved;
        }
        self.saved = settings;
        self.status = Status::NoSpace;
        let next = self.next.or_else(|| {
            candidates()
                .filter(|base| !ENERGY_SECTORS.contains(base))
                .find(|&base| blank(base))
                .and_then(|base| {
                    self.status = Status::WriteFailed;
                    write(base, base, &[CLAIM]).then_some((base, base + FIRST))
                })
        });
        let Some((base, slot)) = next else {
            return false;
        };
        let word = settings.encode();
        let written = write(base, slot, &[word, !word]);
        self.status = if written {
            Status::Saved
        } else {
            Status::WriteFailed
        };
        // Move past the slot even after a failure: it may be partly programmed.
        let following = slot + RECORD;
        self.next = (following + RECORD <= base + SECTOR).then_some((base, following));
        written
    }
}
