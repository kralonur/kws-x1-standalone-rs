//! The frame as a draw target, and the port and system detail lines.

use core::sync::atomic::Ordering;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_graphics::prelude::*;

use crate::ui::Lines;
use crate::{BUTTON_GAP_MAX_MS, FRAME_MS};

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

/// Ports page: frame and input-poll timing in ms, then each GPIO port's
/// input levels.
pub fn ports_lines(ports: &[u16; 6]) -> Lines {
    let mut rows = [[b' '; 24]; 8];
    put(&mut rows[0], b"FRAME 000000 GAP 000000");
    decimal(&mut rows[0], 6, 6, FRAME_MS.load(Ordering::Relaxed));
    decimal(
        &mut rows[0],
        17,
        6,
        BUTTON_GAP_MAX_MS.load(Ordering::Relaxed),
    );
    for ((row, name), &levels) in rows[1..].iter_mut().zip(b"ABCDEH").zip(ports) {
        put(row, b"PORT -: 0000");
        row[5] = *name;
        hex(row, 8, 4, levels.into());
    }
    rows
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
}

/// System page: one value per row. `STEP` is the bring-up step whose wait
/// hung (`0` = no timeout), which names the failure without needing a
/// debugger: 1 bus free, 2 start accepted, 3 tx buffer empty, 4 byte
/// transferred, 5 stop completed, 6 repeated start, 7 buffer before the read
/// address, 8 receive mode entered, 9 first byte received, 10 second byte
/// received.
pub fn irq_lines(counters: Counters, latched: bool) -> Lines {
    let mut rows = [[b' '; 24]; 8];
    put(
        &mut rows[0],
        if latched {
            b"I2C FAULT  BUS!"
        } else {
            b"I2C FAULT  OK"
        },
    );
    put(&mut rows[1], b"STEP: 000000");
    decimal(&mut rows[1], 6, 6, counters.step);
    put(&mut rows[2], b"SR: 00000");
    hex(&mut rows[2], 4, 5, counters.sr);
    put(&mut rows[3], b"CR1: 00000 CR4: 00000");
    hex(&mut rows[3], 5, 5, counters.cr1);
    hex(&mut rows[3], 16, 5, counters.cr4);
    // Peak stack bytes since boot / stack size: the margin left before an
    // overflow into `.bss` (see docs/SAFETY.md).
    put(&mut rows[4], b"STACK: 000000/000000");
    decimal(&mut rows[4], 7, 6, counters.stack_peak);
    decimal(&mut rows[4], 14, 6, counters.stack_size);
    put(&mut rows[5], b"REG FAILS: 000000");
    decimal(&mut rows[5], 11, 6, counters.reg_failures);
    put(&mut rows[6], b"ERRORS: 000000");
    decimal(&mut rows[6], 8, 6, counters.errors);
    put(&mut rows[7], b"READS: 000000");
    decimal(&mut rows[7], 7, 6, counters.reads);
    rows
}
