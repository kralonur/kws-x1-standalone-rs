//! The frame as a draw target, and the system detail lines.

use core::sync::atomic::Ordering;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_graphics::prelude::*;

use crate::ui::Lines;
use crate::{BUTTON_GAP_MS, FRAME_MS};

/// One full RGB565 frame in memory order.
pub type Frame = [u16; firmware_core::panel::PIXELS];
/// Panel size in pixels, as frame indices.
const WIDTH: usize = firmware_core::panel::WIDTH as usize;
const HEIGHT: usize = firmware_core::panel::HEIGHT as usize;

/// Frame pixels are stored byte-swapped: `panel::flush` sends each halfword
/// low byte first and the panel takes the high byte of an RGB565 pixel first.
pub const fn color(rgb565: u16) -> u16 {
    rgb565.swap_bytes()
}
/// Page background (black), in frame byte order.
const BACKGROUND: u16 = color(0x0000);

/// Fill the whole frame with the page background.
pub fn clear(frame: &mut Frame) {
    frame.fill(BACKGROUND);
}

/// The frame as an `embedded-graphics` draw target. Colours are stored in
/// [`color`]'s byte order; pixels outside the panel are dropped.
pub struct Canvas<'a>(pub &'a mut Frame);

impl OriginDimensions for Canvas<'_> {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

impl DrawTarget for Canvas<'_> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Rgb565>>,
    {
        for Pixel(point, rgb) in pixels {
            if let (Ok(x), Ok(y)) = (usize::try_from(point.x), usize::try_from(point.y))
                && x < WIDTH
                && y < HEIGHT
            {
                self.0[y * WIDTH + x] = color(RawU16::from(rgb).into_inner());
            }
        }
        Ok(())
    }
}

/// `digits` decimal digits of `value` (the low ones) at `row[at..]`.
fn decimal(row: &mut [u8; 24], at: usize, digits: usize, value: u32) {
    let mut value = value;
    for cell in row[at..at + digits].iter_mut().rev() {
        *cell = b'0' + (value % 10) as u8;
        value /= 10;
    }
}

/// `digits` hexadecimal digits of `value` (the low ones) at `row[at..]`.
fn hex(row: &mut [u8; 24], at: usize, digits: usize, value: u32) {
    let mut value = value;
    for cell in row[at..at + digits].iter_mut().rev() {
        *cell = b"0123456789ABCDEF"[(value & 0xf) as usize];
        value >>= 4;
    }
}

fn put(row: &mut [u8; 24], text: &[u8]) {
    row[..text.len()].copy_from_slice(text);
}

/// Counters shown on the system page.
pub struct Counters {
    pub step: u32,
    pub sr: u32,
    pub cr1: u32,
    pub cr4: u32,
    /// Peak stack use and stack size in bytes (`stack_use`).
    pub stack_peak: u32,
    pub stack_size: u32,
    pub reg_failures: u32,
    pub errors: u32,
    pub reads: u32,
    /// The reset cause flags (`RESET_FLAGS` bit order).
    pub reset_flags: u16,
}

/// Reset cause flag names, by bit (reference manual, reset control flag
/// register): power-on, reset pin, brown-out, voltage detectors 1 and 2,
/// watchdog, special watchdog, power-down, software, MPU error, RAM parity,
/// RAM ECC, clock failure, crystal failure and multiple causes.
const RESET_FLAGS: [&[u8]; 15] = [
    b"POWER",
    b"PIN",
    b"BROWNOUT",
    b"PVD1",
    b"PVD2",
    b"WDT",
    b"SWDT",
    b"PDOWN",
    b"SOFT",
    b"MPU",
    b"RAMPAR",
    b"RAMECC",
    b"CLOCK",
    b"XTAL",
    b"MULTI",
];

/// System page, one value per row: the I2C bus section, then timing and
/// memory. `STEP` is the bring-up step whose wait
/// hung (`0` = no timeout), which names the failure without needing a
/// debugger: 1 bus free, 2 start accepted, 3 tx buffer empty, 4 byte
/// transferred, 5 stop completed, 6 repeated start, 7 buffer before the read
/// address, 8 receive mode entered, 9 first byte received, 10 second byte
/// received.
pub fn system_lines(counters: Counters, latched: bool) -> [Lines; 2] {
    let mut rows = [[b' '; 24]; 8];
    // The latch also closes the bus without a failed transfer: after a PD
    // session left the PHY unverified, or when the bus never started.
    let transfer_failed = counters.errors != 0 || counters.step != 0;
    let head: &[u8] = match (latched, transfer_failed) {
        (false, _) => b"I2C FAULT  OK",
        (true, true) => b"I2C FAULT  BUS!",
        (true, false) => b"BUS LATCHED  NO I2C ERR",
    };
    put(&mut rows[0], head);
    put(&mut rows[1], b"STEP: 000000");
    decimal(&mut rows[1], 6, 6, counters.step);
    put(&mut rows[2], b"SR: 00000");
    hex(&mut rows[2], 4, 5, counters.sr);
    put(&mut rows[3], b"CR1: 00000 CR4: 00000");
    hex(&mut rows[3], 5, 5, counters.cr1);
    hex(&mut rows[3], 16, 5, counters.cr4);
    put(&mut rows[4], b"REG FAILS: 000000");
    decimal(&mut rows[4], 11, 6, counters.reg_failures);
    put(&mut rows[5], b"ERRORS: 000000");
    decimal(&mut rows[5], 8, 6, counters.errors);
    put(&mut rows[6], b"READS: 000000");
    decimal(&mut rows[6], 7, 6, counters.reads);

    let mut timing = [[b' '; 24]; 8];
    put(&mut timing[0], b"TIMING AND MEMORY");
    // The longest panel refresh, and the longest gap between two button
    // polls (normally `buttons::POLL_MS`), in the last `PEAK_WINDOW`, in ms.
    put(&mut timing[1], b"FRAME: 000000 MS");
    decimal(&mut timing[1], 7, 6, FRAME_MS.load(Ordering::Relaxed));
    put(&mut timing[2], b"BUTTON GAP: 000000 MS");
    decimal(&mut timing[2], 12, 6, BUTTON_GAP_MS.load(Ordering::Relaxed));
    // Peak stack bytes since boot / stack size: the margin left before an
    // overflow into `.bss` (see docs/SAFETY.md).
    put(&mut timing[3], b"STACK: 000000/000000");
    decimal(&mut timing[3], 7, 6, counters.stack_peak);
    decimal(&mut timing[3], 14, 6, counters.stack_size);
    // The flags are never cleared here: after a power-up only POWER is set,
    // so any other name is a reset since the meter last lost power.
    put(&mut timing[4], b"LAST RESET: 0000");
    hex(&mut timing[4], 12, 4, u32::from(counters.reset_flags));
    reset_names(&mut timing[5], counters.reset_flags);
    [rows, timing]
}

/// The names of the set reset cause `flags`, space separated, as many as fit
/// in `row`.
pub fn reset_names(row: &mut [u8], flags: u16) {
    let mut at = 0;
    for (bit, name) in RESET_FLAGS.iter().enumerate() {
        if flags & (1 << bit) != 0 && at + name.len() <= row.len() {
            row[at..at + name.len()].copy_from_slice(name);
            at += name.len() + 1;
        }
    }
}
