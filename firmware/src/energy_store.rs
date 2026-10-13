//! Hardware adapter for the portable two-sector energy journal.

use firmware_core::energy_store::{SECTOR_BYTES, Storage};

use crate::flash::{self, ENERGY_SECTORS};

const _: () = assert!(SECTOR_BYTES == flash::SECTOR);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Ready,
    Occupied,
    SaveFailed,
}

pub struct Flash;
pub type Store = firmware_core::energy_store::Store<Flash>;

/// Called once at boot, before I2C, display DMA or protocol tasks are active.
pub fn open() -> (Option<Store>, Status) {
    use firmware_core::energy_store::{Preparation, prepare};
    let mut flash = Flash;
    match prepare(&mut flash) {
        Preparation::Ready => {
            let store = Store::open(flash);
            let status = if store.is_some() {
                Status::Ready
            } else {
                Status::Occupied
            };
            (store, status)
        }
        Preparation::Occupied => (None, Status::Occupied),
        Preparation::WriteFailed => (None, Status::SaveFailed),
    }
}

impl Storage for Flash {
    fn is_legacy_log(&self, sector: usize) -> bool {
        flash::read(ENERGY_SECTORS[sector]) == flash::LOG_CLAIM
    }

    fn read(&self, sector: usize, offset: usize) -> u32 {
        flash::read(ENERGY_SECTORS[sector] + offset)
    }

    fn write(&mut self, sector: usize, offset: usize, words: &[u32]) -> bool {
        let base = ENERGY_SECTORS[sector];
        flash::write(base, base + offset, words)
    }

    fn erase(&mut self, sector: usize) -> bool {
        let base = ENERGY_SECTORS[sector];
        let erase: unsafe extern "C" fn(u32) -> u32 = core::hint::black_box(flash::erase_sector);
        // SAFETY: migration/recovery retains durable ownership in the other
        // sector, and never erases the newest checkpoint. This adapter fences
        // erases to the energy pair, excluding code, boot-swap and other stored
        // data. The RAM routine runs with interrupts masked. During boot no
        // I2C, panel DMA or protocol task is active; runtime meter_task holds
        // the I2C lease and has finished its previous panel DMA.
        // SysTick coalesces masked interrupts. Measure erase time with the
        // independent core cycle counter so energy integration and protocol
        // timers do not silently lose roughly 20 ms on every sector rollover.
        // SAFETY: no task owns DCB/DWT; enabling trace/cycle counting does not
        // change the SysTick driver or reset an existing debugger counter.
        let mut core = unsafe { cortex_m::Peripherals::steal() };
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        let errors = cortex_m::interrupt::free(|_| {
            use cortex_m::peripheral::{DWT, SCB, SYST};
            if SCB::is_pendst_pending() {
                SCB::clear_pendst();
                embassy_hc32::time_driver::systick_tick();
            }
            let period = SYST::get_reload() + 1;
            let phase = SYST::get_reload() - SYST::get_current();
            let start = DWT::cycle_count();
            // SAFETY: the address fence and RAM/interrupt requirements above hold.
            let errors = unsafe { erase(base as u32) };
            let elapsed = DWT::cycle_count().wrapping_sub(start);
            let ticks = (elapsed + phase) / period;
            // One tick remains pending in hardware and runs on unmask; replay
            // the others. Driver wakes only set executor flags, never run tasks.
            for _ in 1..ticks {
                embassy_hc32::time_driver::systick_tick();
            }
            errors
        });
        if errors != 0 {
            flash::note_failure(base, errors);
            return false;
        }
        let blank = flash::blank(base);
        if !blank {
            flash::note_failure(base, flash::VERIFY_FAILED);
        }
        blank
    }
}
