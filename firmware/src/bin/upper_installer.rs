//! Upper installer: programs the main firmware's upper code region
//! (`memory.x`), which the update path cannot write. Built with
//! `UPPER_BIN` naming that build's `objcopy --only-section=.upper` output;
//! flashed like any image, it runs once at boot, then the main firmware is
//! flashed over it and checks the region (`upper.rs`).
//!
//! Only sectors inside `[UPPER_BASE, UPPER_END)` are touched, and a sector is
//! erased only when it holds words that differ from the region and are not
//! blank. The backlight lights when the region matches, and blinks when it
//! could not be written.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embassy_hc32::gpio::{Level, OutputPin, Port};
use firmware_image::{UPPER_BASE, UPPER_END};
use hc32f460_pac as _;
use panic_halt as _;

#[path = "../flash.rs"]
#[allow(dead_code)]
mod flash;

use flash::{BLANK, SECTOR, program_blank, read};

/// This build's upper region bytes.
const UPPER: &[u8] = include_bytes!(env!("UPPER_BIN"));
const _: () = assert!(
    UPPER.len().is_multiple_of(4) && UPPER.len() <= (UPPER_END - UPPER_BASE) as usize,
    "the upper region must be whole words inside its flash range"
);

/// PH2 drives the backlight (`main.rs` `init_backlight`); high is full on.
const BACKLIGHT_PORT: Port = Port::H;
const BACKLIGHT_PIN: u8 = 2;
/// Failure blink half-period in core cycles, about half a second at the
/// clock the bootloader leaves running; only a visible rhythm matters.
const BLINK_CYCLES: u32 = 50_000_000;

#[entry]
fn main() -> ! {
    // The bootloader's interrupts may still be routed and enabled; this
    // image handles none of them.
    cortex_m::interrupt::disable();
    // SAFETY: this image owns PH2 and touches no other pin.
    let mut backlight =
        unsafe { OutputPin::new_high_drive(BACKLIGHT_PORT, BACKLIGHT_PIN, Level::Low) };
    if install() {
        backlight.set_high();
        loop {
            cortex_m::asm::wfi();
        }
    }
    loop {
        backlight.toggle();
        cortex_m::asm::delay(BLINK_CYCLES);
    }
}

/// The region's word at byte `offset`.
fn wanted(offset: usize) -> u32 {
    u32::from_le_bytes([
        UPPER[offset],
        UPPER[offset + 1],
        UPPER[offset + 2],
        UPPER[offset + 3],
    ])
}

/// Make flash from `UPPER_BASE` match [`UPPER`], sector by sector.
fn install() -> bool {
    for (index, part) in UPPER.chunks(SECTOR).enumerate() {
        let base = UPPER_BASE as usize + index * SECTOR;
        let offsets = (0..part.len()).step_by(4).map(|at| index * SECTOR + at);
        if offsets
            .clone()
            .all(|offset| read(UPPER_BASE as usize + offset) == wanted(offset))
        {
            continue;
        }
        let programmable = offsets.clone().all(|offset| {
            let word = read(UPPER_BASE as usize + offset);
            word == wanted(offset) || word == BLANK
        });
        if !programmable && !erase(base) {
            return false;
        }
        for offset in offsets {
            let address = UPPER_BASE as usize + offset;
            let word = wanted(offset);
            if read(address) != word && !program_blank(base, address, &[word]) {
                return false;
            }
        }
    }
    true
}

/// Erase the sector at `base`, only inside the upper region, then check it
/// reads blank.
fn erase(base: usize) -> bool {
    let inside = base.is_multiple_of(SECTOR)
        && base >= UPPER_BASE as usize
        && base + SECTOR <= UPPER_END as usize;
    if !inside {
        return false;
    }
    let erase: unsafe extern "C" fn(u32) -> u32 = core::hint::black_box(erase_sector);
    // SAFETY: `base` is a sector of the upper region, which holds no code
    // this image runs; the routine runs from RAM with interrupts off.
    let errors = cortex_m::interrupt::free(|_| unsafe { erase(base as u32) });
    errors == 0 && flash::blank(base)
}

/// Sector erase (HC32F460 RM section 7.6: PEMOD = 0b100, any word write in
/// the sector starts it), from RAM with the same unlock, window and cache
/// handling as `flash::program`. Returns the EFM_FSR error bits.
#[inline(never)]
// SAFETY: this routine must run from RAM while the erase is active.
#[unsafe(link_section = ".data.flash_erase")]
unsafe extern "C" fn erase_sector(base: u32) -> u32 {
    const EFM: u32 = 0x4001_0400;
    /// FWMC: PEMODE (bit 0) with PEMOD = sector erase (bits 6..4).
    const SECTOR_ERASE: u32 = 1 | 0b100 << 4;
    /// Any value written into the sector starts the erase.
    const START: u32 = 0;
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
        fpmtsw.write_volatile(base);
        fpmtew.write_volatile(base + 0x1fff);
        fsclr.write_volatile(0x3f);
        fwmc.write_volatile(1);
        fwmc.write_volatile(SECTOR_ERASE);
        (base as *mut u32).write_volatile(START);
        while fsr.read_volatile() & 1 << 8 == 0 {}
        let errors = fsr.read_volatile() & 0x2f; // COLERR PGMISMTCH PGSZERR PEPRTERR PEWERR
        fsclr.write_volatile(0x3f);
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
