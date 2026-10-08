//! `pd-boot-diagnostic` only (`docs/SAFETY.md`). A restart loses the
//! screen and RAM (RSTF0 = PORF), so PD progress is appended to flash and the
//! next boot shows the previous boot's last entry. Restricted to
//! the original settings/statistics area, with these limits:
//! - only a sector of `SECTORS` that is entirely blank, or already claimed by
//!   this log; the last sector (0x7e000) is never used, it holds the FSWP
//!   boot-swap word at 0x7ffdc (HC32F460 RM section 7.9.8);
//! - program only, never erase; a word is programmed only if it reads blank;
//! - the EFM window (FPMTSW/FPMTEW, RM section 7.6.7) is narrowed to the sector
//!   for each write and restored afterwards; any error flag stops the log.

use crate::pd;
use core::cell::Cell;
use core::sync::atomic::{AtomicU16, Ordering};

const SECTOR: usize = 0x2000;
const SECTORS: [usize; 6] = [0x7c000, 0x7a000, 0x78000, 0x76000, 0x74000, 0x72000];
const CLAIM: u32 = 0x4b57_4c47; // "KWLG", first word of a claimed sector
/// 16-byte entries (first log image) are skipped; 32-byte entries carry the
/// lock/transfer/delay maxima too.
const ENTRY_V1: u32 = 0xb0;
const ENTRY: u32 = 0xb1;
const SLOT: usize = 16;
const ENTRY_LEN: usize = 32;
/// Entries per boot; the session's steps and errors fit, including the
/// cable breadcrumbs. One more is kept for a panic or fault record.
const PER_BOOT: u8 = 40;
/// Error codes of a fatal record (`fatal`); outside the session's codes.
pub const PANIC: u8 = 250;
pub const FAULT: u8 = 251;
/// `cable-log` stall detector: the display task stopped for 3 s.
#[allow(dead_code)]
pub const HANG: u8 = 252;

#[derive(Clone, Copy)]
struct State {
    base: usize,
    next: usize,
    boot: u16,
    written: u8,
    last: (u8, u8, u16),
}

static STATE: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    Cell<Option<State>>,
> = embassy_sync::blocking_mutex::Mutex::new(Cell::new(None));

/// Progress of the last published observation for a fatal record: furthest
/// step in bits 0..8, cable queries in bits 8..16. An atomic, so the
/// panic/fault handler never borrows shared state that may be mid-update.
static PROGRESS: AtomicU16 = AtomicU16::new(0);

pub static LAST: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    Cell<Option<pd::LastBoot>>,
> = embassy_sync::blocking_mutex::Mutex::new(Cell::new(None));

fn read(address: usize) -> u32 {
    unsafe { (address as *const u32).read_volatile() }
}

/// A valid current-format entry at `slot`.
fn entry(slot: usize) -> Option<[u32; 8]> {
    if read(slot) >> 24 != ENTRY || slot + ENTRY_LEN > slot / SECTOR * SECTOR + SECTOR {
        return None;
    }
    let words: [u32; 8] = core::array::from_fn(|i| read(slot + 4 * i));
    (words[7] == !words[..7].iter().fold(0, |a, w| a ^ w)).then_some(words)
}

/// Bytes used at `slot`: a 32-byte entry, else one 16-byte slot.
fn used(slot: usize) -> usize {
    if read(slot) >> 24 == ENTRY {
        ENTRY_LEN
    } else {
        SLOT
    }
}

/// Boot number of any entry format at `slot`.
fn boot_at(slot: usize) -> Option<u16> {
    matches!(read(slot) >> 24, ENTRY | ENTRY_V1).then(|| (read(slot) >> 8) as u16)
}

/// Program words at `address` (16-byte slot or the claim word) from RAM with
/// interrupts off: the code and vectors live in the flash being programmed.
/// Returns the accumulated EFM_FSR error bits, 0 on success.
#[inline(never)]
// SAFETY: this routine must run from RAM while flash programming is active.
#[unsafe(link_section = ".data.flash_log_program")]
unsafe extern "C" fn program(address: u32, words: *const u32, count: u32, window: u32) -> u32 {
    const EFM: u32 = 0x4001_0400;
    let faprt = EFM as *mut u32;
    let frmc = (EFM + 0x08) as *mut u32;
    let fwmc = (EFM + 0x0c) as *mut u32;
    let fsr = (EFM + 0x10) as *const u32;
    let fsclr = (EFM + 0x14) as *mut u32;
    let fpmtsw = (EFM + 0x20) as *mut u32;
    let fpmtew = (EFM + 0x24) as *mut u32;
    unsafe {
        faprt.write_volatile(0x0123);
        faprt.write_volatile(0x3210);
        let read_mode = frmc.read_volatile();
        frmc.write_volatile(read_mode & !(1 << 16)); // cache off (RM section 7.6)
        let (start, end) = (fpmtsw.read_volatile(), fpmtew.read_volatile());
        fpmtsw.write_volatile(window);
        fpmtew.write_volatile(window + 0x1fff);
        fsclr.write_volatile(0x3f);
        fwmc.write_volatile(1);
        fwmc.write_volatile(1 | 0b010 << 4); // single programming, read-back
        let mut errors = 0;
        let mut i = 0;
        while i < count {
            ((address + 4 * i) as *mut u32).write_volatile(words.add(i as usize).read_volatile());
            while fsr.read_volatile() & 1 << 8 == 0 {}
            errors |= fsr.read_volatile() & 0x2f; // COLERR PGMISMTCH PGSZERR PEPRTERR PEWERR
            fsclr.write_volatile(0x3f);
            if errors != 0 {
                break;
            }
            i += 1;
        }
        fwmc.write_volatile(1); // read-only
        fwmc.write_volatile(0);
        fpmtsw.write_volatile(start);
        fpmtew.write_volatile(end);
        frmc.write_volatile(read_mode | 1 << 24); // cache reset, then restore
        frmc.write_volatile(read_mode);
        faprt.write_volatile(0); // any write re-protects
        errors
    }
}

fn write(base: usize, address: usize, words: &[u32]) -> bool {
    let inside = SECTORS.contains(&base)
        && address.is_multiple_of(4)
        && address >= base
        && address + 4 * words.len() <= base + SECTOR;
    if !inside || (0..words.len()).any(|i| read(address + 4 * i) != !0) {
        return false;
    }
    let program: unsafe extern "C" fn(u32, *const u32, u32, u32) -> u32 =
        core::hint::black_box(program);
    let errors = cortex_m::interrupt::free(|_| unsafe {
        program(
            address as u32,
            words.as_ptr(),
            words.len() as u32,
            base as u32,
        )
    });
    errors == 0
        && words
            .iter()
            .enumerate()
            .all(|(i, &word)| read(address + 4 * i) == word)
}

/// End of the used part of a claimed sector (entries are appended in order).
fn used_end(base: usize) -> usize {
    let mut next = base + SLOT;
    let mut slot = base + SLOT;
    while slot < base + SECTOR {
        let step = used(slot);
        if (0..step / 4).any(|i| slot + 4 * i < base + SECTOR && read(slot + 4 * i) != !0) {
            next = slot + step;
        }
        slot += step;
    }
    next
}

/// Once, early in `main`: report the newest boot's last entry from all
/// claimed sectors, number this boot, and pick where it writes: the sector
/// holding the newest boot if it has room, else another claimed sector with
/// room, else a blank one, claimed now (never erased).
pub fn start() {
    let boot_of = |words: &[u32; 8]| (words[0] >> 8) as u16;
    let claimed = |base: usize| read(base) == CLAIM;
    let mut last_boot: Option<(u16, usize)> = None; // boot, its sector
    for base in SECTORS.iter().copied().filter(|&base| claimed(base)) {
        let mut slot = base + SLOT;
        while slot < base + SECTOR {
            if let Some(boot) = boot_at(slot)
                && last_boot.is_none_or(|(b, _)| boot >= b)
            {
                last_boot = Some((boot, base));
            }
            slot += used(slot);
        }
    }
    // That boot's last current-format entry, and when it first reached a step.
    let mut previous: Option<[u32; 8]> = None;
    let mut reached = [0u16; 2]; // steps 3 and 5
    if let Some((boot, base)) = last_boot {
        let mut slot = base + SLOT;
        while slot < base + SECTOR {
            if let Some(words) = entry(slot).filter(|w| boot_of(w) == boot) {
                for (i, step) in [3u8, 5].into_iter().enumerate() {
                    if reached[i] == 0 && words[0] as u8 >= step {
                        reached[i] = (words[2] & 0xffff).min(9999) as u16;
                    }
                }
                previous = Some(words);
            }
            slot += used(slot);
        }
    }
    let fatal = previous.is_some_and(|w| w[1] as u8 >= PANIC);
    let record = previous.map(|w| pd::BootRecord {
        chain: if fatal {
            [w[2], w[3], w[4], w[5], w[6]]
        } else {
            [0; 5]
        },
        step: w[0] as u8,
        error: w[1] as u8,
        packets: (w[1] >> 8) as u8,
        header: (w[1] >> 16) as u16,
        ms: if fatal {
            0
        } else {
            (w[2] & 0xffff).min(9999) as u16
        },
        step3_ms: reached[0],
        step5_ms: reached[1],
        poll_gap_ms: if fatal { 0 } else { (w[2] >> 16) as u8 },
        lock_wait_ms: if fatal { 0 } else { (w[2] >> 24) as u8 },
        transfer_ms: if fatal { 0 } else { w[3] as u8 },
        delay_over_ms: if fatal { 0 } else { (w[3] >> 8) as u8 },
    });
    let room = |base: usize| used_end(base) + ENTRY_LEN <= base + SECTOR;
    let blank = |base: usize| (0..SECTOR / 4).all(|i| read(base + 4 * i) == !0);
    let base = last_boot
        .map(|(_, base)| base)
        .filter(|&base| room(base))
        .or_else(|| {
            SECTORS
                .iter()
                .copied()
                .find(|&base| claimed(base) && room(base))
        })
        .or_else(|| {
            SECTORS
                .iter()
                .copied()
                .find(|&base| blank(base) && write(base, base, &[CLAIM]))
        });
    let boot = last_boot.map_or(1, |(b, _)| b.wrapping_add(1));
    LAST.lock(|cell| {
        cell.set(Some(pd::LastBoot {
            record,
            logging: base.is_some(),
        }))
    });
    if let Some(base) = base {
        let next = used_end(base);
        STATE.lock(|cell| {
            cell.set(Some(State {
                base,
                next,
                boot,
                written: 0,
                last: (0, 0, 0),
            }))
        });
    }
}

/// Append an entry when step or error changed (not on every packet, so the
/// per-boot budget lasts through the cable breadcrumbs).
pub fn save(observation: &pd::Observation) {
    // Session steps 1-9, cable steps 10-31: one step number; the packet
    // field holds the number of cable queries started in this boot.
    let key = (
        observation.boot_step.max(observation.cable_step),
        observation.rx_error,
        u16::from(observation.cable_queries),
    );
    STATE.lock(|cell| {
        let Some(mut state) = cell.get() else { return };
        if key == state.last
            || state.written >= PER_BOOT
            || state.next + ENTRY_LEN > state.base + SECTOR
        {
            return;
        }
        let w0 = ENTRY << 24 | u32::from(state.boot) << 8 | u32::from(key.0);
        let w1 = u32::from(key.1)
            | u32::from(key.2.min(255)) << 8
            | u32::from(observation.rx_last_header.unwrap_or(0)) << 16;
        let ms = embassy_time::Instant::now().as_millis().min(0xffff) as u32;
        let w2 = ms | u32::from(observation.poll_gap_ms) << 16;
        let max = |m: &core::sync::atomic::AtomicU32| {
            m.load(core::sync::atomic::Ordering::Relaxed).min(255)
        };
        let w2 = w2 | max(&crate::pd_trial::LOCK_WAIT_MAX) << 24;
        let w3 = max(&crate::pd_trial::TRANSFER_MAX) | max(&crate::pd_trial::DELAY_OVER_MAX) << 8;
        let mut words = [w0, w1, w2, w3, 0, 0, 0, 0];
        words[7] = !words[..7].iter().fold(0, |a, w| a ^ w);
        let ok = write(state.base, state.next, &words);
        state.next += ENTRY_LEN;
        state.written += 1;
        state.last = key;
        // Any failed write ends the log for this boot; nothing is retried.
        cell.set(ok.then_some(state));
    });
}

/// Keep the progress `fatal` records from `observation`.
pub fn note_progress(observation: &pd::Observation) {
    let step = observation.boot_step.max(observation.cable_step);
    PROGRESS.store(
        u16::from_le_bytes([step, observation.cable_queries]),
        Ordering::Relaxed,
    );
}

/// Last entry of this boot: the furthest step so far, `code` (`PANIC` or
/// `FAULT`) and five code addresses in words 2..6 (no time stamp).
/// Called with interrupts off from the panic/HardFault handler; may use
/// the one entry `save` keeps in reserve.
pub fn fatal(code: u8, chain: [u32; 5]) {
    let [step, queries] = PROGRESS.load(Ordering::Relaxed).to_le_bytes();
    STATE.lock(|cell| {
        let Some(state) = cell.get() else { return };
        if state.written > PER_BOOT || state.next + ENTRY_LEN > state.base + SECTOR {
            return;
        }
        let w0 = ENTRY << 24 | u32::from(state.boot) << 8 | u32::from(step);
        let w1 = u32::from(code) | u32::from(queries) << 8;
        let mut words = [w0, w1, chain[0], chain[1], chain[2], chain[3], chain[4], 0];
        words[7] = !words[..7].iter().fold(0, |a, w| a ^ w);
        write(state.base, state.next, &words);
        cell.set(None); // nothing after a fatal record
    });
}
