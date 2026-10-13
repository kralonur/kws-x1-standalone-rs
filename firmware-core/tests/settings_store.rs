//! Exercise the firmware's public settings store against a fake flash boundary.
//! A failed save must never become a successful repeat: restart-mode clearing
//! relies on the result, and failed brightness saves must remain visible.

use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

mod flash {
    use super::*;

    pub const BLANK: u32 = u32::MAX;
    pub const SECTOR: usize = 8192;
    pub const SECTORS: [usize; 6] = [0, SECTOR, 2 * SECTOR, 3 * SECTOR, 4 * SECTOR, 5 * SECTOR];
    pub const ENERGY_SECTORS: [usize; 2] = [2 * SECTOR, 3 * SECTOR];
    static WORDS: Mutex<Vec<u32>> = Mutex::new(Vec::new());
    pub static WRITES: AtomicUsize = AtomicUsize::new(0);
    static FAIL_AT: AtomicUsize = AtomicUsize::new(usize::MAX);

    pub fn reset(fill: u32, fail_at: usize) {
        *WORDS.lock().unwrap() = vec![fill; SECTOR / 4 * SECTORS.len()];
        WRITES.store(0, Ordering::Relaxed);
        FAIL_AT.store(fail_at, Ordering::Relaxed);
    }

    pub fn read(address: usize) -> u32 {
        WORDS.lock().unwrap()[address / 4]
    }

    pub fn blank(base: usize) -> bool {
        (0..SECTOR)
            .step_by(4)
            .all(|offset| read(base + offset) == BLANK)
    }

    pub fn write(base: usize, address: usize, words: &[u32]) -> bool {
        let attempt = WRITES.fetch_add(1, Ordering::Relaxed);
        if attempt >= FAIL_AT.load(Ordering::Relaxed) {
            return false;
        }
        assert!(SECTORS.contains(&base));
        assert!(address >= base && address + words.len() * 4 <= base + SECTOR);
        let mut flash = WORDS.lock().unwrap();
        for (index, word) in words.iter().enumerate() {
            let slot = &mut flash[address / 4 + index];
            assert_eq!(*slot, BLANK);
            *slot = *word;
        }
        true
    }
}

#[path = "../../firmware/src/settings_store.rs"]
mod settings_store;

#[test]
fn failed_claim_failed_record_and_full_storage_never_report_saved() {
    use firmware_core::settings::{EnergyMemory, Settings};
    use settings_store::{Status, Store};

    let changed = Settings {
        brightness: 3,
        energy_memory: EnergyMemory::Seconds5,
        ..Settings::default()
    };
    for fail_at in [0, 1] {
        flash::reset(flash::BLANK, fail_at);
        let (mut store, _) = Store::open();
        assert!(!store.save(changed));
        assert_eq!(store.status(), Status::WriteFailed);
        let attempts = flash::WRITES.load(Ordering::Relaxed);
        assert!(!store.save(changed));
        assert_eq!(store.status(), Status::WriteFailed);
        assert_eq!(flash::WRITES.load(Ordering::Relaxed), attempts);
        assert_eq!(Store::open().1, None);
    }
    flash::reset(0, usize::MAX);
    let (mut full, _) = Store::open();
    assert!(!full.save(changed));
    assert_eq!(full.status(), Status::NoSpace);
    assert!(!full.save(changed));
    assert_eq!(flash::WRITES.load(Ordering::Relaxed), 0);

    flash::reset(flash::BLANK, usize::MAX);
    let (mut writable, _) = Store::open();
    assert!(writable.save(changed));
    assert_eq!(writable.status(), Status::Saved);
    let attempts = flash::WRITES.load(Ordering::Relaxed);
    assert!(writable.save(changed));
    assert_eq!(flash::WRITES.load(Ordering::Relaxed), attempts);
    assert_eq!(Store::open().1, Some(changed));
}
