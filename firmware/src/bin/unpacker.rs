//! Unpacker: the first boot of a release image (`firmware_image::packed`).
//! It inflates both streams and checks their lengths and hashes, writes the
//! upper region, then writes the main firmware over itself and resets into
//! it. Everything but the reset path runs from RAM (`unpacker.x`), since the
//! flash it was loaded from is rewritten.
//!
//! Only the upper region and the application range are touched; a sector is
//! skipped when it already holds its bytes and erased only when a word is
//! neither wanted nor blank. The backlight is lit while it works and blinks
//! when the image is damaged or flash could not be written; flashing the
//! release again recovers.

#![no_std]
#![no_main]

use core::ops::Range;

use cortex_m_rt::entry;
use embassy_hc32::gpio::{Level, OutputPin, Port};
use firmware_image::{
    APP_BASE, APP_LIMIT, ERASE_CEILING, REGION_HASH_START, UPPER_BASE, UPPER_END,
    packed::{
        PACKED_MAGIC, PACKED_UNSET, Packed, Stream, WINDOW_BYTES, inflate_sectors,
        main_erase_margin,
    },
    region_hash_update,
};
use hc32f460_pac as _;
use panic_halt as _;

#[path = "../flash.rs"]
#[allow(dead_code)]
mod flash;

use flash::{BLANK, SECTOR, program_blank, read};

// Copy `.ramcode` (`unpacker.x`) from flash to RAM before cortex-m-rt
// initializes RAM; it runs from flash, so it calls nothing.
core::arch::global_asm!(
    ".section .text.__pre_init, \"ax\"
     .global __pre_init
     .type __pre_init, %function
     .thumb_func
     __pre_init:
     ldr r0, =__sramcode
     ldr r1, =__eramcode
     ldr r2, =__siramcode
     0:
     cmp r0, r1
     beq 1f
     ldr r3, [r2], #4
     str r3, [r0], #4
     b 0b
     1:
     bx lr
     .ltorg"
);

const UNSET_STREAM: Stream = Stream {
    len: PACKED_UNSET,
    hash: PACKED_UNSET,
    packed: PACKED_UNSET,
};

/// This image's streams; `firmware-pack` finds the record by its magic and
/// fills it in.
#[used]
static PACKED: Packed = Packed {
    magic: PACKED_MAGIC,
    data: PACKED_UNSET,
    upper: UNSET_STREAM,
    main: UNSET_STREAM,
};

/// PH2 drives the backlight (`main.rs` `init_backlight`); high is full on.
const BACKLIGHT_PORT: Port = Port::H;
const BACKLIGHT_PIN: u8 = 2;
/// Failure blink half-period in core cycles, about half a second at the
/// clock the bootloader leaves running; only a visible rhythm matters.
const BLINK_CYCLES: u32 = 50_000_000;

#[entry]
fn main() -> ! {
    // The bootloader's interrupts may still be routed and enabled; this
    // image handles none of them, and its vector table is overwritten.
    cortex_m::interrupt::disable();
    // SAFETY: this image owns PH2 and touches no other pin.
    let mut backlight =
        unsafe { OutputPin::new_high_drive(BACKLIGHT_PORT, BACKLIGHT_PIN, Level::High) };
    let mut window = [0; WINDOW_BYTES];
    if let Some(packed) = record()
        && intact(stream(&packed, Part::Upper), packed.upper, &mut window)
        && intact(stream(&packed, Part::Main), packed.main, &mut window)
        && install_upper(&packed, &mut window)
        && install_main(&packed, &mut window)
    {
        cortex_m::peripheral::SCB::sys_reset();
    }
    loop {
        backlight.toggle();
        cortex_m::asm::delay(BLINK_CYCLES);
    }
}

/// The filled record, when its streams are whole words inside this image
/// and their regions.
fn record() -> Option<Packed> {
    // SAFETY: a read of an initialized static; volatile so the placeholder
    // values are never folded in place of what the packer wrote.
    let packed = unsafe { core::ptr::read_volatile(&PACKED) };
    let streams_end = (packed.data as usize)
        .checked_add(packed.upper.packed as usize)?
        .checked_add(packed.main.packed as usize)?;
    let fits = packed.magic == PACKED_MAGIC
        && packed.data >= APP_BASE
        && streams_end <= APP_BASE as usize + APP_LIMIT
        && packed.upper.len.is_multiple_of(4)
        && packed.upper.len <= UPPER_END - UPPER_BASE
        && packed.main.len.is_multiple_of(4)
        && packed.main.len as usize <= APP_LIMIT;
    fits.then_some(packed)
}

enum Part {
    Upper,
    Main,
}

/// A stream's bytes in this image's flash.
fn stream(packed: &Packed, part: Part) -> &'static [u8] {
    let (start, len) = match part {
        Part::Upper => (packed.data, packed.upper.packed),
        Part::Main => (packed.data + packed.upper.packed, packed.main.packed),
    };
    // SAFETY: `record` checked both streams lie in this image's flash. Only
    // the main firmware's install erases any of it, and only sectors wholly
    // below the bytes the decompressor has yet to read (`main_erase_margin`).
    unsafe { core::slice::from_raw_parts(start as *const u8, len as usize) }
}

/// The stream inflates to exactly its recorded length and hash.
fn intact(bytes: &[u8], stream: Stream, window: &mut [u8; WINDOW_BYTES]) -> bool {
    let mut hash = REGION_HASH_START;
    let inflated = inflate_sectors(bytes, window, |chunk| {
        hash = region_hash_update(hash, chunk.bytes);
        true
    });
    inflated.is_ok_and(|len| len == stream.len as usize) && hash == stream.hash
}

fn install_upper(packed: &Packed, window: &mut [u8; WINDOW_BYTES]) -> bool {
    inflate_sectors(stream(packed, Part::Upper), window, |chunk| {
        write_sector(
            UPPER_BASE as usize + chunk.offset,
            chunk.bytes,
            UPPER_BASE..UPPER_END,
        )
    })
    .is_ok()
}

fn install_main(packed: &Packed, window: &mut [u8; WINDOW_BYTES]) -> bool {
    inflate_sectors(stream(packed, Part::Main), window, |chunk| {
        main_erase_margin(packed, chunk.offset, chunk.consumed).is_some()
            && write_sector(
                APP_BASE as usize + chunk.offset,
                chunk.bytes,
                APP_BASE..ERASE_CEILING,
            )
    })
    .is_ok()
}

/// Make the sector at `base` start with `bytes` (whole words): skip it when
/// it already does, erase it when a word is neither wanted nor blank, then
/// program the differing words. Only sectors inside `fence` are touched.
fn write_sector(base: usize, bytes: &[u8], fence: Range<u32>) -> bool {
    let inside = base.is_multiple_of(SECTOR)
        && base >= fence.start as usize
        && base + SECTOR <= fence.end as usize;
    if !inside {
        return false;
    }
    let words = (bytes.as_chunks::<4>().0.iter())
        .enumerate()
        .map(|(index, word)| (base + 4 * index, u32::from_le_bytes(*word)));
    if words.clone().all(|(address, word)| read(address) == word) {
        return true;
    }
    let programmable = words.clone().all(|(address, word)| {
        let found = read(address);
        found == word || found == BLANK
    });
    if !programmable && !erase(base) {
        return false;
    }
    words
        .into_iter()
        .all(|(address, word)| read(address) == word || program_blank(base, address, &[word]))
}

/// Erase the sector at `base`, then check it reads blank; `write_sector`
/// fences it.
fn erase(base: usize) -> bool {
    if !base.is_multiple_of(SECTOR) {
        return false;
    }
    let erase: unsafe extern "C" fn(u32) -> u32 = core::hint::black_box(flash::erase_sector);
    // SAFETY: the routine runs from RAM with interrupts off, and the caller
    // keeps `base` clear of code it runs.
    let errors = cortex_m::interrupt::free(|_| unsafe { erase(base as u32) });
    errors == 0 && flash::blank(base)
}
