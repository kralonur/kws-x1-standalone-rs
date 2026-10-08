//! Append-only programming of the original settings/statistics area
//! (`docs/SAFETY.md`), shared by the settings store and the diagnostic log:
//! - only the sectors in `SECTORS`; the last sector (0x7e000) is never used,
//!   it holds the FSWP boot-swap word at 0x7ffdc (HC32F460 RM section 7.9.8);
//! - program only, never erase; a word is programmed only if it reads blank;
//! - the EFM window (FPMTSW/FPMTEW, RM section 7.6.7) is narrowed to the sector
//!   for each write and restored afterwards; any error flag fails the write.

/// Flash sector size in bytes (RM section 7.3).
pub const SECTOR: usize = 0x2000;
/// The original settings/statistics sectors this firmware may claim.
pub const SECTORS: [usize; 6] = [0x7c000, 0x7a000, 0x78000, 0x76000, 0x74000, 0x72000];
/// An erased flash word.
pub const BLANK: u32 = !0;

/// Read one flash word.
pub fn read(address: usize) -> u32 {
    // SAFETY: callers pass word-aligned addresses inside on-chip flash.
    unsafe { (address as *const u32).read_volatile() }
}

/// Every word of the sector at `base` is erased.
pub fn blank(base: usize) -> bool {
    (0..SECTOR / 4).all(|i| read(base + 4 * i) == BLANK)
}

/// Program words at `address` (16-byte slot or the claim word) from RAM with
/// interrupts off: the code and vectors live in the flash being programmed.
/// Returns the accumulated EFM_FSR error bits, 0 on success.
#[inline(never)]
// SAFETY: this routine must run from RAM while flash programming is active.
#[unsafe(link_section = ".data.flash_program")]
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

pub fn write(base: usize, address: usize, words: &[u32]) -> bool {
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
