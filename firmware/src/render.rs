//! Bitmap font and the meter, port and I2C fault pages, drawn straight into
//! the frame.

use core::sync::atomic::Ordering;

use crate::{BUTTON_GAP_MAX_MS, FRAME_MS, ina226};

const CYAN: u16 = 0xff07;

// 3 columns x 5 rows bitmap font (same glyphs as the earlier application).
pub fn glyph(c: u8) -> u16 {
    match c {
        b'0' => 0b111_101_101_101_111,
        b'1' => 0b111_010_010_110_010,
        b'2' => 0b111_100_111_001_111,
        b'3' => 0b111_001_111_001_111,
        b'4' => 0b001_001_111_101_101,
        b'5' => 0b111_001_111_100_111,
        b'6' => 0b111_101_111_100_111,
        b'7' => 0b010_010_010_001_111,
        b'8' => 0b111_101_111_101_111,
        b'9' => 0b111_001_111_101_111,
        b'A' => 0b101_101_111_101_010,
        b'B' => 0b110_101_110_101_110,
        b'C' => 0b111_100_100_100_111,
        b'D' => 0b110_101_101_101_110,
        b'E' => 0b111_100_110_100_111,
        b'F' => 0b100_100_110_100_111,
        b'G' => 0b111_101_101_100_111,
        b'H' => 0b101_101_111_101_101,
        b'I' => 0b111_010_010_010_111,
        b'K' => 0b101_101_110_101_101,
        b'L' => 0b111_100_100_100_100,
        b'M' => 0b101_101_101_111_101,
        b'O' => 0b111_101_101_101_111,
        b'P' => 0b100_100_110_101_110,
        b'Q' => 0b001_111_101_101_111,
        b'R' => 0b101_101_110_101_110,
        b'S' => 0b111_001_111_100_111,
        b'T' => 0b010_010_010_010_111,
        b'U' => 0b111_101_101_101_101,
        b'V' => 0b010_101_101_101_101,
        b'W' => 0b101_111_101_101_101,
        b'X' => 0b101_101_010_101_101,
        b'Y' => 0b010_010_010_101_101,
        b'/' => 0b001_001_010_100_100,
        b'_' => 0b000_000_000_000_111,
        b'.' => 0b010_000_000_000_000,
        b'-' => 0b000_000_111_000_000,
        b':' => 0b000_010_000_010_000,
        b'*' => 0b101_010_111_010_101,
        b'[' => 0b111_100_100_100_111,
        b']' => 0b111_001_001_001_111,
        _ => 0,
    }
}

#[inline(never)]
pub unsafe fn clear_screen(framebuffer: *mut u16) {
    for offset in 0..32400 {
        unsafe {
            framebuffer.add(offset).write_volatile(CYAN);
        }
    }
}

#[inline(never)]
unsafe fn text(framebuffer: *mut u16, mut x: usize, y: usize, bytes: &[u8]) {
    for &c in bytes {
        let bitmap = glyph(c);
        for row in 0..5 {
            for column in 0..3 {
                if bitmap & (1 << (row * 3 + 2 - column)) != 0 {
                    for dy in 0..3 {
                        for dx in 0..3 {
                            let px = x + column * 3 + dx;
                            let py = y + row * 3 + dy;
                            if px < 240 && py < 135 {
                                unsafe {
                                    framebuffer.add(py * 240 + px).write_volatile(0);
                                }
                            }
                        }
                    }
                }
            }
        }
        x += 12;
    }
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
        text(framebuffer, 12, 6, b"KOWSI X1 EMBASSY");

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
                framebuffer.add(y * 240 + x).write_volatile(alive);
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
