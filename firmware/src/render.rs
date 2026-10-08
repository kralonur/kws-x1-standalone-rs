//! Text drawing (`embedded-graphics` fonts) and the meter, port and I2C fault
//! pages, drawn straight into the frame.

use core::sync::atomic::Ordering;

use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_9X18_BOLD};
use embedded_graphics::mono_font::{MonoFont, MonoTextStyle};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Baseline, Text};

use crate::{BUTTON_GAP_MAX_MS, FRAME_MS, ina226};

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
pub const WHITE: u16 = color(0xffff);
pub const BLUE: u16 = color(0x001f);
pub const RED: u16 = color(0xf800);
const CYAN: u16 = color(0x07ff);

#[inline(never)]
pub unsafe fn clear_screen(framebuffer: *mut u16) {
    for offset in 0..firmware_core::panel::PIXELS {
        unsafe {
            framebuffer.add(offset).write_volatile(CYAN);
        }
    }
}

/// The frame as an `embedded-graphics` draw target. Colours are stored in
/// [`color`]'s byte order; pixels outside the panel are dropped.
struct Canvas<'a>(&'a mut Frame);

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

/// Meter, port and validation pages: 9 x 18 px bold glyphs on a 12 px pitch.
const LARGE_FONT: MonoFont = MonoFont {
    character_spacing: 3,
    ..FONT_9X18_BOLD
};
/// PD pages: 6 x 10 px glyphs on an 8 px pitch, 24 characters per row.
pub const SMALL_FONT: MonoFont = MonoFont {
    character_spacing: 2,
    ..FONT_6X10
};
/// Characters drawn per `Text`; longer lines continue in further chunks.
const TEXT_CHUNK: usize = 24;

/// Draw `bytes` in black with its top-left corner at (`x`, `y`). Bytes that
/// are not printable ASCII (such as unused `0` cells) are drawn as spaces.
pub fn draw_text(frame: &mut Frame, font: &MonoFont, x: usize, y: usize, bytes: &[u8]) {
    let style = MonoTextStyle::new(font, Rgb565::BLACK);
    let mut canvas = Canvas(frame);
    let mut origin = Point::new(x as i32, y as i32);
    for chunk in bytes.chunks(TEXT_CHUNK) {
        let mut printable = [b' '; TEXT_CHUNK];
        for (cell, &byte) in printable.iter_mut().zip(chunk) {
            if byte.is_ascii_graphic() {
                *cell = byte;
            }
        }
        let Ok(line) = core::str::from_utf8(&printable[..chunk.len()]) else {
            return;
        };
        let Ok(next) = Text::with_baseline(line, origin, style, Baseline::Top).draw(&mut canvas);
        origin = next;
    }
}

#[inline(never)]
unsafe fn text(framebuffer: *mut u16, x: usize, y: usize, bytes: &[u8]) {
    // SAFETY: `framebuffer` is the whole frame and the renderer is its sole owner.
    draw_text(
        unsafe { &mut *(framebuffer as *mut Frame) },
        &LARGE_FONT,
        x,
        y,
        bytes,
    );
}

#[inline(always)]
fn copy_4(dest: &mut [u8], offset: usize, src: &[u8; 4]) {
    dest[offset] = src[0];
    dest[offset + 1] = src[1];
    dest[offset + 2] = src[2];
    dest[offset + 3] = src[3];
}

#[inline(always)]
fn copy_6(dest: &mut [u8], offset: usize, src: &[u8; 6]) {
    dest[offset] = src[0];
    dest[offset + 1] = src[1];
    dest[offset + 2] = src[2];
    dest[offset + 3] = src[3];
    dest[offset + 4] = src[4];
    dest[offset + 5] = src[5];
}

// Format fixed point decimal: value is integer, divisor is 1000 for 3 decimals or 100 for 2
#[inline(never)]
fn fmt_fixed_3(mut val: u32, buf: &mut [u8; 6]) {
    for i in (3..6).rev() {
        buf[i] = b'0' + (val % 10) as u8;
        val /= 10;
    }
    buf[2] = b'.';
    buf[1] = b'0' + (val % 10) as u8;
    val /= 10;
    buf[0] = b'0' + (val % 10) as u8;
}

fn fmt_time(total_secs: u32, line: &mut [u8; 14]) {
    let hrs = (total_secs / 3600) % 100;
    let mins = (total_secs / 60) % 60;
    let secs = total_secs % 60;
    line[6] = b'0' + (hrs / 10) as u8;
    line[7] = b'0' + (hrs % 10) as u8;
    line[9] = b'0' + (mins / 10) as u8;
    line[10] = b'0' + (mins % 10) as u8;
    line[12] = b'0' + (secs / 10) as u8;
    line[13] = b'0' + (secs % 10) as u8;
}

#[inline(never)]
fn hex_digits(value: u16) -> [u8; 4] {
    let mut result = [b'0'; 4];
    for (i, digit) in result.iter_mut().enumerate() {
        let nibble = ((value >> ((3 - i) * 4)) & 0xf) as u8;
        *digit = if nibble < 10 {
            b'0' + nibble
        } else {
            b'A' + (nibble - 10)
        };
    }
    result
}

pub unsafe fn render_measure(
    framebuffer: *mut u16,
    raw_v: u16,
    raw_i: u16,
    total_secs: u32,
    alive: u16,
) {
    unsafe {
        clear_screen(framebuffer);
        text(framebuffer, 12, 6, b"USB PD METER");

        let v_mv = ina226::bus_millivolts(raw_v);
        let mut line_v = *b"VBUS: 00.000 V";
        let mut v_buf = [0u8; 6];
        fmt_fixed_3(v_mv, &mut v_buf);
        copy_6(&mut line_v, 6, &v_buf);
        text(framebuffer, 12, 30, &line_v);

        let i_raw = raw_i as i16;
        // Preserve the existing sensor conversion, including its signed range.
        #[allow(clippy::cast_abs_to_unsigned)]
        let i_ma = (i_raw.abs() as u32) / ina226::COUNTS_PER_MA;
        let mut line_i = *b"CURR:  00.000 A";
        if i_raw < 0 {
            line_i[6] = b'-';
        }
        let mut i_buf = [0u8; 6];
        fmt_fixed_3(i_ma, &mut i_buf);
        copy_6(&mut line_i, 7, &i_buf);
        text(framebuffer, 12, 54, &line_i);

        let mut line_p = *b"POWR: 00.000 W";
        let mut p_buf = [0u8; 6];
        fmt_fixed_3((v_mv * i_ma) / 1000, &mut p_buf);
        copy_6(&mut line_p, 6, &p_buf);
        text(framebuffer, 12, 78, &line_p);

        let mut line_t = *b"TIME: 00:00:00";
        fmt_time(total_secs, &mut line_t);
        text(framebuffer, 12, 106, &line_t);

        // 500 ms heartbeat square: proves meter_task and pulse_task both run.
        for y in 6..18 {
            for x in 222..234 {
                framebuffer.add(y * WIDTH + x).write_volatile(alive);
            }
        }
    }
}

#[inline(never)]
pub unsafe fn render_ports(framebuffer: *mut u16, ports: &[u16; 6]) {
    unsafe {
        clear_screen(framebuffer);
        let mut timing = *b"F:000000 G:000000";
        copy_6(&mut timing, 2, &decimal6(FRAME_MS.load(Ordering::Relaxed)));
        copy_6(
            &mut timing,
            11,
            &decimal6(BUTTON_GAP_MAX_MS.load(Ordering::Relaxed)),
        );
        text(framebuffer, 12, 6, &timing);
        const NAMES: [u8; 6] = *b"ABCDEH";
        for (i, &name) in NAMES.iter().enumerate() {
            let y = 28 + i * 17;
            let mut line = [name, b':', b' ', 0, 0, 0, 0];
            let h = hex_digits(ports[i]);
            copy_4(&mut line, 3, &h);
            text(framebuffer, 12, y, &line);
        }
    }
}

/// Six decimal digits (same rolling format as the fault counters).
fn decimal6(value: u32) -> [u8; 6] {
    let mut digits = [b'0'; 6];
    for (i, d) in digits.iter_mut().enumerate() {
        *d = b'0' + ((value / 10u32.pow(5 - i as u32)) % 10) as u8;
    }
    digits
}

/// Counters shown on the "I2C FAULT" validation page.
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

/// Validation page: one value per row (CR1/CR4 share one), because this is
/// read off a 135 px panel by eye. `STEP` is the bring-up step whose wait hung (`0` = no timeout), which
/// names the failure without needing a debugger:
/// 1 bus free, 2 start accepted, 3 tx buffer empty, 4 byte transferred,
/// 5 stop completed, 6 repeated start, 7 buffer before the read address,
/// 8 receive mode entered, 9 first byte received, 10 second byte received.
#[inline(never)]
pub unsafe fn render_irq_test(framebuffer: *mut u16, counters: Counters, latched: bool) {
    /// Five hex digits (covers `SR`'s and `CR1`'s low 20 bits).
    fn hex5(value: u32) -> [u8; 5] {
        let mut digits = [b'0'; 5];
        for (i, d) in digits.iter_mut().enumerate() {
            let nibble = ((value >> ((4 - i) * 4)) & 0xf) as u8;
            *d = if nibble < 10 {
                b'0' + nibble
            } else {
                b'A' + (nibble - 10)
            };
        }
        digits
    }

    unsafe {
        clear_screen(framebuffer);
        // Title doubles as the latch indicator, so the eight rows below stay one
        // value each: STEP names the wait that hung (see the legend above).
        text(
            framebuffer,
            6,
            4,
            if latched {
                b"I2C FAULT  BUS!"
            } else {
                b"I2C FAULT  OK"
            },
        );

        let mut line = *b"STEP  : 000000";
        copy_6(&mut line, 8, &decimal6(counters.step));
        text(framebuffer, 6, 20, &line);

        let mut line = *b"SR    : 00000";
        line[8..13].copy_from_slice(&hex5(counters.sr));
        text(framebuffer, 6, 36, &line);

        let mut line = *b"CR1 00000 CR4 00000";
        line[4..9].copy_from_slice(&hex5(counters.cr1));
        line[14..19].copy_from_slice(&hex5(counters.cr4));
        text(framebuffer, 6, 52, &line);

        // Peak stack bytes since boot / stack size: the margin left before an
        // overflow into `.bss` (see docs/SAFETY.md).
        let mut line = *b"STK 000000/000000";
        copy_6(&mut line, 4, &decimal6(counters.stack_peak));
        copy_6(&mut line, 11, &decimal6(counters.stack_size));
        text(framebuffer, 6, 68, &line);

        let mut line = *b"REGF  : 000000";
        copy_6(&mut line, 8, &decimal6(counters.reg_failures));
        text(framebuffer, 6, 84, &line);

        let mut line = *b"ERR   : 000000";
        copy_6(&mut line, 8, &decimal6(counters.errors));
        text(framebuffer, 6, 100, &line);

        let mut line = *b"RD    : 000000";
        copy_6(&mut line, 8, &decimal6(counters.reads));
        text(framebuffer, 6, 116, &line);
    }
}
