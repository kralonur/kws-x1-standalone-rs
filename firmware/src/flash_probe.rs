//! `flash-probe` only: one read-only System page section on whether the
//! application can grow past the update erase ceiling (`docs/SAFETY.md`).
//! Nothing is written or erased.
//!
//! - row 1: the sectors from the erase ceiling to the settings, `.` erased,
//!   `#` holding data; row 2: the first used word above the ceiling;
//! - row 3: whether the last update wrote the padded payload's final 64 bytes;
//! - rows 4..: the bootloader's size and the offset and value of each of its
//!   literal words that look like a payload length or trailer address, which
//!   shows whether it checks a fixed checksum position.

use core::cell::Cell;
use core::ops::Range;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use crate::flash::{BLANK, SECTOR, read};
use crate::render::{hex, put};
use crate::ui::Lines;

/// Bootloader region: flash below the application base.
const BOOT_END: usize = 0x10000;
/// The update's exclusive erase ceiling and the first settings sector
/// (`firmware_image::ERASE_CEILING`, `flash::SECTORS`).
const ERASE_CEILING: usize = 0x44000;
const SETTINGS: usize = 0x72000;
/// The padded payload's last 64 bytes, past its last whole 512-byte block
/// (`firmware_image::EXTENDED_PAYLOAD_LEN` above the application base).
const TAIL: Range<usize> = 0x43600..0x43640;
/// Literal values around the payload length (`0x33420`, `0x33640`) up to
/// the trailer addresses and erase ceiling (`0x4340c`, `0x44000`).
const TRAILER_WORDS: Range<u32> = 0x3_3000..0x4_4001;

static PROBE: Mutex<CriticalSectionRawMutex, Cell<Option<Lines>>> = Mutex::new(Cell::new(None));

/// The probe section, read once on first use.
pub fn lines() -> Lines {
    PROBE.lock(|cell| {
        let lines = cell.get().unwrap_or_else(probe);
        cell.set(Some(lines));
        lines
    })
}

fn words(range: Range<usize>) -> impl Iterator<Item = (usize, u32)> {
    range.step_by(4).map(|address| (address, read(address)))
}

fn probe() -> Lines {
    let mut lines = [[b' '; 24]; 8];
    put(&mut lines[0], b"FLASH PROBE");
    for (index, base) in (ERASE_CEILING..SETTINGS).step_by(SECTOR).enumerate() {
        let used = words(base..base + SECTOR).any(|(_, word)| word != BLANK);
        lines[1][index] = if used { b'#' } else { b'.' };
    }
    match words(ERASE_CEILING..SETTINGS).find(|(_, word)| *word != BLANK) {
        Some((address, word)) => {
            put(&mut lines[2], b"USED 00000: 00000000");
            hex(&mut lines[2], 5, 5, address as u32);
            hex(&mut lines[2], 12, 8, word);
        }
        None => put(&mut lines[2], b"UPPER ALL BLANK"),
    }
    let tail: &[u8] = if words(TAIL).all(|(_, word)| word == 0) {
        b"TAIL WRITTEN"
    } else if words(TAIL).all(|(_, word)| word == BLANK) {
        b"TAIL NOT WRITTEN"
    } else {
        b"TAIL MIXED"
    };
    put(&mut lines[3], tail);
    let end = (0..BOOT_END)
        .step_by(4)
        .rev()
        .find(|address| read(*address) != BLANK)
        .map_or(0, |address| address + 4);
    put(&mut lines[4], b"BOOT END 00000");
    hex(&mut lines[4], 9, 5, end as u32);
    // Two `OOOO:VVVVV` hits (bootloader offset, value) per row.
    let mut hits = words(0..end).filter(|(_, word)| TRAILER_WORDS.contains(word));
    for line in &mut lines[5..] {
        for at in [0, 11] {
            let Some((address, word)) = hits.next() else {
                return lines;
            };
            hex(line, at, 4, address as u32);
            line[at + 4] = b':';
            hex(line, at + 5, 5, word);
        }
    }
    lines
}
