//! Read-only storage diagnostics: no claiming, programming or erasing.

use crate::flash::{self, BLANK, SECTOR, SECTORS};
use crate::render::{hex, put};
use crate::ui::Lines;

pub fn lines() -> Lines {
    let mut lines = [[b' '; 24]; 8];
    put(&mut lines[0], b"FLASH STORAGE");
    let (address, error) = flash::last_failure();
    put(&mut lines[1], b"ERR 00 AT 00000");
    hex(&mut lines[1], 4, 2, error);
    hex(&mut lines[1], 10, 5, address);
    for (row, base) in lines[2..].iter_mut().zip(SECTORS.iter().rev().copied()) {
        // Sector base, first word, then number of erased words (hex).
        // A sector is claimable only when all 0x0800 words are erased.
        put(row, b"00000 00000000 FREE 0000");
        hex(row, 0, 5, base as u32);
        hex(row, 6, 8, flash::read(base));
        let free = (0..SECTOR)
            .step_by(4)
            .filter(|offset| flash::read(base + offset) == BLANK)
            .count();
        hex(row, 20, 4, free as u32);
    }
    lines
}
