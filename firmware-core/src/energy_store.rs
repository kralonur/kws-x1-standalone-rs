//! Two-sector energy journal. Boot preparation may migrate recognized legacy
//! logs after committing erase intent in their unused header words. Otherwise
//! the pair is claimed only after both sectors were checked blank. An intact
//! energy header authorizes recovery of an interrupted erase of its partner.
//! Never erase the sector containing the newest complete checkpoint.

use crate::energy::Totals;

/// Backend sector size in bytes; checked against `firmware::flash::SECTOR`.
pub const SECTOR_BYTES: usize = 8192;
/// Pair ownership/version marker ("EN01"), followed by its complement.
const CLAIM: u32 = 0x454e_3031;
const HEADER_BYTES: usize = 8;
/// Legacy logs reserve 16 header bytes but use only their first word. Each
/// remaining word can commit pair-migration intent ("ENMG"). A torn marker
/// is skipped, never reprogrammed; another blank header word is used instead.
const LEGACY_HEADER_BYTES: usize = 16;
const MIGRATION: u32 = 0x454e_4d47;
/// Sequence plus two u64 counters, their word-wise complements and a final
/// nonblank commit word. Complements alone can equal erased words, so they
/// cannot identify the last programmed word after a power cut.
const DATA_WORDS: usize = 5;
const RECORD_WORDS: usize = DATA_WORDS * 2 + 1;
/// Final record commit marker ("DONE"), written after all counter words.
const COMMIT: u32 = 0x444f_4e45;
const RECORD_BYTES: usize = RECORD_WORDS * 4;

/// Only the two reserved sectors are exposed by the firmware adapter.
pub trait Storage {
    fn read(&self, sector: usize, offset: usize) -> u32;
    fn write(&mut self, sector: usize, offset: usize, words: &[u32]) -> bool;
    fn erase(&mut self, sector: usize) -> bool;
    /// Recognize the old log's claim, whose bytes 4..16 are unused header
    /// space. Unknown data and settings must never qualify.
    fn is_legacy_log(&self, sector: usize) -> bool;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Preparation {
    Ready,
    Occupied,
    WriteFailed,
}

pub struct Store<F> {
    flash: F,
    current: usize,
    next: usize,
    sequence: u32,
    saved: Option<Totals>,
    claimed: bool,
    failed: bool,
}

fn header(flash: &impl Storage, sector: usize) -> bool {
    flash.read(sector, 0) == CLAIM && flash.read(sector, 4) == !CLAIM
}

fn blank(flash: &impl Storage, sector: usize) -> bool {
    (0..SECTOR_BYTES)
        .step_by(4)
        .all(|offset| flash.read(sector, offset) == u32::MAX)
}

fn record(flash: &impl Storage, sector: usize, offset: usize) -> Option<(u32, Totals)> {
    let words: [u32; RECORD_WORDS] = core::array::from_fn(|i| flash.read(sector, offset + i * 4));
    if words[RECORD_WORDS - 1] != COMMIT
        || (0..DATA_WORDS).any(|i| words[i + DATA_WORDS] != !words[i])
    {
        return None;
    }
    Some((
        words[0],
        Totals {
            ma_ms: u64::from(words[1]) | u64::from(words[2]) << 32,
            uw_ms: u64::from(words[3]) | u64::from(words[4]) << 32,
        },
    ))
}

/// Prepare only the reserved pair before protocol tasks start. No action is
/// taken for two blank sectors or a fully initialized journal. Legacy logs
/// are reclaimable, but only after durable intent proves both sectors were
/// checked: the survivor's marker remains through erasing/claiming its partner,
/// then the partner's energy header remains through erasing the survivor.
pub fn prepare(flash: &mut impl Storage) -> Preparation {
    let headers = [header(flash, 0), header(flash, 1)];
    if headers.iter().any(|found| *found) {
        for (sector, claimed) in headers.into_iter().enumerate() {
            if !claimed && !blank(flash, sector) && !flash.erase(sector) {
                return Preparation::WriteFailed;
            }
        }
        return Preparation::Ready;
    }
    let survivor = (0..2).find(|&sector| {
        flash.is_legacy_log(sector)
            && (4..LEGACY_HEADER_BYTES)
                .step_by(4)
                .any(|offset| flash.read(sector, offset) == MIGRATION)
    });
    let survivor = match survivor {
        Some(sector) => sector,
        None => {
            let blanks = [blank(flash, 0), blank(flash, 1)];
            if (0..2).any(|sector| !blanks[sector] && !flash.is_legacy_log(sector)) {
                return Preparation::Occupied;
            }
            if blanks.into_iter().all(|found| found) {
                return Preparation::Ready;
            }
            let slot = (0..2)
                .filter(|&sector| flash.is_legacy_log(sector))
                .flat_map(|sector| {
                    (4..LEGACY_HEADER_BYTES)
                        .step_by(4)
                        .map(move |offset| (sector, offset))
                })
                .find(|&(sector, offset)| flash.read(sector, offset) == u32::MAX);
            let Some((sector, offset)) = slot else {
                return Preparation::WriteFailed;
            };
            if !flash.write(sector, offset, &[MIGRATION]) {
                return Preparation::WriteFailed;
            }
            sector
        }
    };
    let partner = 1 - survivor;
    if !blank(flash, partner) && !flash.erase(partner) {
        return Preparation::WriteFailed;
    }
    if !flash.write(partner, 0, &[CLAIM, !CLAIM]) || !flash.erase(survivor) {
        return Preparation::WriteFailed;
    }
    Preparation::Ready
}

impl<F: Storage> Store<F> {
    /// Read-only recovery. Foreign contents are left untouched; unavailable
    /// storage is reported rather than reclaiming settings or stock data.
    pub fn open(flash: F) -> Option<Self> {
        let headers = [header(&flash, 0), header(&flash, 1)];
        let claimed = headers.iter().any(|found| *found);
        if !claimed && !(blank(&flash, 0) && blank(&flash, 1)) {
            return None;
        }
        let mut current = usize::from(!headers[0]);
        let mut next = [HEADER_BYTES; 2];
        let mut newest = None;
        for sector in 0..2 {
            if !headers[sector] {
                continue;
            }
            for offset in (HEADER_BYTES..=SECTOR_BYTES - RECORD_BYTES).step_by(RECORD_BYTES) {
                if (0..RECORD_WORDS).any(|i| flash.read(sector, offset + i * 4) != u32::MAX) {
                    next[sector] = offset + RECORD_BYTES;
                }
                if let Some((sequence, totals)) = record(&flash, sector, offset)
                    && newest.is_none_or(|(last, _)| sequence.wrapping_sub(last) as i32 > 0)
                {
                    newest = Some((sequence, totals));
                    current = sector;
                }
            }
        }
        // With no headers both sectors are blank; the first save claims 0.
        if !claimed {
            current = 0;
        }
        Some(Self {
            flash,
            current,
            next: next[current],
            sequence: newest.map_or(0, |(sequence, _)| sequence),
            saved: newest.map(|(_, totals)| totals),
            claimed,
            failed: false,
        })
    }

    pub fn totals(&self) -> Totals {
        self.saved.unwrap_or_default()
    }

    /// Append only changed totals. On rollover, erase the partner, claim it,
    /// and commit there before it becomes current. A failed operation stops
    /// further writes this boot; recovery scans past torn records next boot.
    pub fn save(&mut self, totals: Totals) -> bool {
        if self.failed {
            return false;
        }
        if self.saved == Some(totals) {
            return true;
        }
        let written = self.append(totals);
        self.failed = !written;
        written
    }

    fn append(&mut self, totals: Totals) -> bool {
        let mut sector = self.current;
        let mut offset = self.next;
        if !self.claimed {
            if !self.flash.write(sector, 0, &[CLAIM, !CLAIM]) {
                return false;
            }
            self.claimed = true;
        } else if offset + RECORD_BYTES > SECTOR_BYTES {
            sector = 1 - sector;
            // The current sector's intact header keeps ownership of the pair
            // through a reset at any point in this erase/claim sequence.
            if !blank(&self.flash, sector) && !self.flash.erase(sector) {
                return false;
            }
            if !self.flash.write(sector, 0, &[CLAIM, !CLAIM]) {
                return false;
            }
            offset = HEADER_BYTES;
        }
        let sequence = self.sequence.wrapping_add(1);
        let data = [
            sequence,
            totals.ma_ms as u32,
            (totals.ma_ms >> 32) as u32,
            totals.uw_ms as u32,
            (totals.uw_ms >> 32) as u32,
        ];
        let words: [u32; RECORD_WORDS] = core::array::from_fn(|i| {
            if i < data.len() {
                data[i]
            } else if i < data.len() * 2 {
                !data[i - data.len()]
            } else {
                COMMIT
            }
        });
        if !self.flash.write(sector, offset, &words) {
            return false;
        }
        self.current = sector;
        self.next = offset + RECORD_BYTES;
        self.sequence = sequence;
        self.saved = Some(totals);
        true
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec;
    use std::vec::Vec;

    #[derive(Clone)]
    struct Flash {
        words: Vec<u32>,
        remaining: Option<usize>,
        erases: usize,
    }

    impl Flash {
        fn new() -> Self {
            Self {
                words: vec![u32::MAX; SECTOR_BYTES / 4 * 2],
                remaining: None,
                erases: 0,
            }
        }
        fn step(&mut self) -> bool {
            match &mut self.remaining {
                Some(0) => false,
                Some(left) => {
                    *left -= 1;
                    true
                }
                None => true,
            }
        }
    }

    /// Diagnostic-log ownership tag recognized by this test adapter.
    const LEGACY_CLAIM: u32 = 0x4b57_4c47;

    impl Storage for Flash {
        fn is_legacy_log(&self, sector: usize) -> bool {
            self.read(sector, 0) == LEGACY_CLAIM
        }

        fn read(&self, sector: usize, offset: usize) -> u32 {
            self.words[(sector * SECTOR_BYTES + offset) / 4]
        }
        fn write(&mut self, sector: usize, offset: usize, words: &[u32]) -> bool {
            for (i, word) in words.iter().enumerate() {
                if !self.step() {
                    return false;
                }
                let index = (sector * SECTOR_BYTES + offset) / 4 + i;
                assert_eq!(self.words[index], u32::MAX);
                self.words[index] = *word;
            }
            true
        }
        fn erase(&mut self, sector: usize) -> bool {
            self.erases += 1;
            for offset in (0..SECTOR_BYTES).step_by(4) {
                if !self.step() {
                    return false;
                }
                self.words[(sector * SECTOR_BYTES + offset) / 4] = u32::MAX;
            }
            true
        }
    }

    fn totals(n: u64) -> Totals {
        Totals {
            ma_ms: n,
            uw_ms: n * 5000,
        }
    }

    #[test]
    fn foreign_sectors_are_never_claimed() {
        let mut flash = Flash::new();
        flash.words[SECTOR_BYTES / 4] = 0x12345678;
        assert!(Store::open(flash).is_none());
    }

    #[test]
    fn rotations_reset_and_unchanged_totals_survive_reopening() {
        let mut store = Store::open(Flash::new()).unwrap();
        for n in 1..=1000 {
            assert!(store.save(totals(n)));
        }
        assert!(store.flash.erases >= 2);
        assert!(store.save(Totals::default()));
        let mut reopened = Store::open(store.flash).unwrap();
        assert_eq!(reopened.totals(), Totals::default());
        let words = reopened.flash.words.clone();
        assert!(reopened.save(Totals::default()));
        assert_eq!(reopened.flash.words, words);
    }

    #[test]
    fn interrupted_appends_and_rollovers_keep_last_complete_checkpoint() {
        let mut store = Store::open(Flash::new()).unwrap();
        // Fill both sectors so the next rollover must erase the older one.
        let count = ((SECTOR_BYTES - HEADER_BYTES) / RECORD_BYTES * 2) as u64;
        for n in 1..=count {
            assert!(store.save(totals(n)));
        }
        let full = store.flash.clone();
        // Every word boundary through erase, header and new record.
        for stop in 0..SECTOR_BYTES / 4 + 2 + RECORD_WORDS {
            let mut flash = full.clone();
            flash.remaining = Some(stop);
            let mut interrupted = Store::open(flash).unwrap();
            assert!(!interrupted.save(totals(count + 1)));
            interrupted.flash.remaining = None;
            let mut recovered = Store::open(interrupted.flash).unwrap();
            assert_eq!(recovered.totals(), totals(count));
            assert!(recovered.save(totals(count + 2)));
            assert_eq!(
                Store::open(recovered.flash).unwrap().totals(),
                totals(count + 2)
            );
        }
        let mut store = Store::open(Flash::new()).unwrap();
        assert!(store.save(totals(1)));
        for stop in 0..RECORD_WORDS {
            let mut flash = store.flash.clone();
            flash.remaining = Some(stop);
            let mut interrupted = Store::open(flash).unwrap();
            assert!(!interrupted.save(totals(2)));
            let stopped = interrupted.flash.words.clone();
            assert!(!interrupted.save(totals(3)));
            assert_eq!(interrupted.flash.words, stopped);
            interrupted.flash.remaining = None;
            let mut recovered = Store::open(interrupted.flash).unwrap();
            assert_eq!(recovered.totals(), totals(1));
            assert!(recovered.save(totals(3)));
        }
    }

    fn legacy(flash: &mut Flash, sector: usize) {
        let start = sector * SECTOR_BYTES / 4;
        flash.words[start..start + SECTOR_BYTES / 4].fill(0);
        flash.words[start] = LEGACY_CLAIM;
        flash.words[start + 1..start + LEGACY_HEADER_BYTES / 4].fill(u32::MAX);
    }

    #[test]
    fn boot_migration_is_idempotent_and_preserves_unknown_data() {
        let mut blank = Flash::new();
        let unchanged = blank.words.clone();
        assert_eq!(prepare(&mut blank), Preparation::Ready);
        assert_eq!(blank.words, unchanged);
        assert_eq!(blank.erases, 0);

        for unknown in [0x1234_5678, 0x4b57_5354] {
            let mut flash = Flash::new();
            legacy(&mut flash, 0);
            flash.words[SECTOR_BYTES / 4] = unknown;
            let unchanged = flash.words.clone();
            assert_eq!(prepare(&mut flash), Preparation::Occupied);
            assert_eq!(flash.words, unchanged);
            assert_eq!(flash.erases, 0);
        }
        let mut saved = Store::open(Flash::new()).unwrap();
        for n in 1..=200 {
            assert!(saved.save(totals(n)));
        }
        let unchanged = saved.flash.words.clone();
        let erases = saved.flash.erases;
        assert_eq!(prepare(&mut saved.flash), Preparation::Ready);
        assert_eq!(saved.flash.words, unchanged);
        assert_eq!(saved.flash.erases, erases);
        assert_eq!(Store::open(saved.flash).unwrap().totals(), totals(200));
    }

    #[test]
    fn migration_recovers_at_every_word_boundary_and_keeps_saved_totals() {
        // These tests protect boot-time erasing: every interrupted claim or
        // erase must retain a durable intent/header in the other sector.
        for logs in [[true, false], [false, true], [true, true]] {
            let mut original = Flash::new();
            for (sector, log) in logs.into_iter().enumerate() {
                if log {
                    legacy(&mut original, sector);
                }
            }
            let steps = logs.iter().filter(|log| **log).count() * SECTOR_BYTES / 4 + 3;
            for stop in 0..steps {
                let mut interrupted = original.clone();
                interrupted.remaining = Some(stop);
                assert_eq!(prepare(&mut interrupted), Preparation::WriteFailed);
                interrupted.remaining = None;
                assert_eq!(prepare(&mut interrupted), Preparation::Ready);
                let unchanged = interrupted.words.clone();
                let erases = interrupted.erases;
                assert_eq!(prepare(&mut interrupted), Preparation::Ready);
                assert_eq!(interrupted.words, unchanged);
                assert_eq!(interrupted.erases, erases);
                let mut store = Store::open(interrupted).unwrap();
                assert_eq!(store.totals(), Totals::default());
                assert!(store.save(totals(7)));
                assert_eq!(Store::open(store.flash).unwrap().totals(), totals(7));
            }
        }
        let mut saved = Store::open(Flash::new()).unwrap();
        assert!(saved.save(totals(42)));
        legacy(&mut saved.flash, 1);
        for stop in 0..SECTOR_BYTES / 4 {
            let mut interrupted = saved.flash.clone();
            interrupted.remaining = Some(stop);
            assert_eq!(prepare(&mut interrupted), Preparation::WriteFailed);
            interrupted.remaining = None;
            assert_eq!(prepare(&mut interrupted), Preparation::Ready);
            assert_eq!(Store::open(interrupted).unwrap().totals(), totals(42));
        }
    }

    #[test]
    fn torn_migration_marker_uses_another_blank_header_word() {
        let mut flash = Flash::new();
        legacy(&mut flash, 0);
        // A partially programmed marker is not proof of erase intent. The
        // original log claim survives and both sectors are still recognizable.
        flash.words[1] = MIGRATION | 0xff00_0000;
        assert_eq!(prepare(&mut flash), Preparation::Ready);
        let mut store = Store::open(flash).unwrap();
        assert!(store.save(totals(1)));
        assert_eq!(Store::open(store.flash).unwrap().totals(), totals(1));
    }
}
