//! Graphical pages: the overview (meter) and settings screens, drawn with
//! `embedded-graphics` primitives and `u8g2-fonts` text on any RGB565 target.
//! The renderer gathers the values; nothing here touches hardware.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyle, Rectangle, StyledDrawable};
use firmware_core::settings::{Item, Settings};
use u8g2_fonts::FontRenderer;
use u8g2_fonts::fonts;
use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};

/// Voltage and current: Inconsolata Bold 19 px, numerals only.
const READING: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_inb19_mn>().with_ignore_unknown_chars(true);
/// Power: Inconsolata Bold 27 px, numerals only.
const POWER: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_inb27_mn>().with_ignore_unknown_chars(true);
/// Titles and units: t0 16 px bold.
const BOLD: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_t0_16b_mr>().with_ignore_unknown_chars(true);
/// Settings rows: t0 14 px bold.
const ROW: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_t0_14b_mr>().with_ignore_unknown_chars(true);
/// Status line, peaks and the page counter: ProFont 12 px.
const PLAIN: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_profont12_mr>().with_ignore_unknown_chars(true);

const ACCENT: Rgb565 = Rgb565::new(11, 52, 28);
const VALUE: Rgb565 = Rgb565::new(30, 61, 30);
const POWER_COLOR: Rgb565 = Rgb565::new(20, 56, 15);
const MUTED: Rgb565 = Rgb565::new(21, 43, 22);
const RULE: Rgb565 = Rgb565::new(5, 14, 7);

/// Screen width and the left/right text margins, in pixels.
const WIDTH: u32 = 240;
const LEFT: i32 = 8;
const RIGHT: i32 = 232;
/// Header: title baseline and the rule under it.
const TITLE_BASELINE: i32 = 15;
const HEADER_RULE_Y: i32 = 21;
/// Overview: voltage/current baseline, the rule under them, the current
/// column, the power baseline, the peak column and the status line.
const READING_BASELINE: i32 = 49;
const READING_RULE_Y: i32 = 59;
const CURRENT_LEFT: i32 = 124;
const POWER_BASELINE: i32 = 100;
const PEAK_LEFT: i32 = 150;
const PEAK_BASELINES: [i32; 3] = [79, 93, 107];
const STATUS_BASELINE: i32 = 128;
/// Gap between a number and its unit, in pixels.
const UNIT_GAP: i32 = 4;

/// One meter reading. `ma` is signed: negative when current flows back.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Reading {
    pub mv: u32,
    pub ma: i32,
}

impl Reading {
    pub fn mw(&self) -> u32 {
        self.mv * self.ma.unsigned_abs() / 1000
    }
}

/// Highest voltage, current magnitude and power seen since the last reset.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Peaks {
    pub mv: u32,
    pub ma: u32,
    pub mw: u32,
}

impl Peaks {
    pub fn update(&mut self, reading: Reading) {
        self.mv = self.mv.max(reading.mv);
        self.ma = self.ma.max(reading.ma.unsigned_abs());
        self.mw = self.mw.max(reading.mw());
    }
}

/// The charging protocol the meter can identify from the PD controller.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Protocol {
    /// Nothing identified yet.
    #[default]
    Unknown,
    /// Type-C current advertisement without PD; `ma` is 0 for default USB power.
    TypeC { ma: u16 },
    /// The source offered PD capabilities, but no contract is known.
    PdOffer,
    /// Fixed PD contract; `revision` is the major PD revision (2 or 3).
    Pd {
        revision: Option<u8>,
        mv: u16,
        ma: u16,
    },
    /// Programmable power supply contract.
    Pps { mv: u16, ma: u16 },
}

/// Page header: title on the left, one-based page position on the right.
pub struct Header<'a> {
    pub title: &'a str,
    pub position: (u8, u8),
}

/// Everything the overview page shows. `reading` is `None` before the first
/// sample and while the bus is latched.
pub struct Meter {
    pub reading: Option<Reading>,
    pub peaks: Peaks,
    pub protocol: Protocol,
    /// Attached CC pin (1 or 2), when known.
    pub cc: Option<u8>,
    /// Heartbeat dot colour (alternates every pulse, red on a bus fault).
    pub alive: Rgb565,
}

/// Small text buffer for formatted numbers and labels; writes past the end are dropped.
struct Label {
    bytes: [u8; 32],
    len: usize,
}

impl Label {
    fn new() -> Self {
        Self {
            bytes: [0; 32],
            len: 0,
        }
    }

    fn push(mut self, text: &[u8]) -> Self {
        for &byte in text {
            if let Some(cell) = self.bytes.get_mut(self.len) {
                *cell = byte;
                self.len += 1;
            }
        }
        self
    }

    /// `value / 10^places` with `places` decimals, e.g. `fixed(2012, 2)` is "20.12".
    fn fixed(self, mut value: u32, places: u32) -> Self {
        let mut digits = [0u8; 12];
        let mut start = digits.len();
        let mut place = 0;
        loop {
            if place == places && places != 0 {
                start -= 1;
                digits[start] = b'.';
            }
            start -= 1;
            digits[start] = b'0' + (value % 10) as u8;
            value /= 10;
            place += 1;
            if value == 0 && place > places {
                break;
            }
        }
        self.push(&digits[start..])
    }

    /// Two digits with a leading zero, e.g. "01".
    fn two_digits(self, value: u8) -> Self {
        self.push(&[b'0' + value / 10 % 10, b'0' + value % 10])
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).unwrap_or("")
    }
}

/// Draw `content` on `baseline` and return the x just past it.
fn text<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    font: &FontRenderer,
    content: &str,
    position: Point,
    horizontal: HorizontalAlignment,
    color: Rgb565,
) -> i32 {
    // Glyph lookups cannot fail (unknown characters are skipped) and drawing
    // into the frame is infallible.
    font.render_aligned(
        content,
        position,
        VerticalPosition::Baseline,
        horizontal,
        FontColor::Transparent(color),
        target,
    )
    .ok()
    .flatten()
    .map_or(position.x, |area| area.top_left.x + area.size.width as i32)
}

fn fill<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    shape: impl StyledDrawable<PrimitiveStyle<Rgb565>, Color = Rgb565>,
    color: Rgb565,
) {
    shape
        .draw_styled(&PrimitiveStyle::with_fill(color), target)
        .ok();
}

fn rule<D: DrawTarget<Color = Rgb565>>(target: &mut D, y: i32) {
    fill(
        target,
        Rectangle::new(Point::new(LEFT - 2, y), Size::new(WIDTH - 12, 1)),
        RULE,
    );
}

/// A number followed by its unit in the accent colour.
fn value_with_unit<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    font: &FontRenderer,
    value: &Label,
    unit: &str,
    position: Point,
    color: Rgb565,
) {
    let end = text(
        target,
        font,
        value.as_str(),
        position,
        HorizontalAlignment::Left,
        color,
    );
    text(
        target,
        &BOLD,
        unit,
        Point::new(end + UNIT_GAP, position.y),
        HorizontalAlignment::Left,
        ACCENT,
    );
}

pub fn draw_header<D: DrawTarget<Color = Rgb565>>(target: &mut D, header: &Header<'_>) {
    text(
        target,
        &BOLD,
        header.title,
        Point::new(LEFT, TITLE_BASELINE),
        HorizontalAlignment::Left,
        ACCENT,
    );
    let (index, count) = header.position;
    let counter = Label::new().two_digits(index).push(b"/").two_digits(count);
    text(
        target,
        &PLAIN,
        counter.as_str(),
        Point::new(RIGHT, TITLE_BASELINE),
        HorizontalAlignment::Right,
        MUTED,
    );
    rule(target, HEADER_RULE_Y);
}

/// Two decimals; power drops to one from 100 W so it keeps five characters.
fn hundredths(milli: u32) -> Label {
    Label::new().fixed(milli / 10, 2)
}
fn watts(mw: u32) -> Label {
    if mw >= 100_000 {
        Label::new().fixed(mw / 100, 1)
    } else {
        hundredths(mw)
    }
}

/// The overview page: volts and amps side by side, watts below, peaks on
/// the right of the watts and the protocol status line at the bottom.
pub fn draw_meter<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    meter: &Meter,
) {
    draw_header(target, header);
    let (volts, amps, power, color) = match meter.reading {
        Some(reading) => {
            let sign: &[u8] = if reading.ma < 0 { b"-" } else { b"" };
            (
                hundredths(reading.mv),
                Label::new()
                    .push(sign)
                    .fixed(reading.ma.unsigned_abs() / 10, 2),
                watts(reading.mw()),
                VALUE,
            )
        }
        None => (
            Label::new().push(b"--.--"),
            Label::new().push(b"-.--"),
            Label::new().push(b"--.--"),
            MUTED,
        ),
    };
    value_with_unit(
        target,
        &READING,
        &volts,
        "V",
        Point::new(LEFT, READING_BASELINE),
        color,
    );
    value_with_unit(
        target,
        &READING,
        &amps,
        "A",
        Point::new(CURRENT_LEFT, READING_BASELINE),
        color,
    );
    rule(target, READING_RULE_Y);
    value_with_unit(
        target,
        &POWER,
        &power,
        "W",
        Point::new(LEFT, POWER_BASELINE),
        if meter.reading.is_some() {
            POWER_COLOR
        } else {
            MUTED
        },
    );

    text(
        target,
        &BOLD,
        "MAX",
        Point::new(PEAK_LEFT, PEAK_BASELINES[0]),
        HorizontalAlignment::Left,
        ACCENT,
    );
    let peaks = [
        hundredths(meter.peaks.mv).push(b"V"),
        hundredths(meter.peaks.ma).push(b"A"),
        watts(meter.peaks.mw).push(b"W"),
    ];
    for (peak, baseline) in peaks.iter().zip(PEAK_BASELINES) {
        text(
            target,
            &PLAIN,
            peak.as_str(),
            Point::new(RIGHT, baseline),
            HorizontalAlignment::Right,
            MUTED,
        );
    }

    text(
        target,
        &PLAIN,
        status(meter.protocol, meter.cc).as_str(),
        Point::new(LEFT, STATUS_BASELINE),
        HorizontalAlignment::Left,
        MUTED,
    );
    fill(
        target,
        Circle::with_center(Point::new(RIGHT - 3, STATUS_BASELINE - 4), 6),
        meter.alive,
    );
}

/// "PD 3.0 | CC1 | 20V 3.00A": protocol, attached CC pin and contract.
fn status(protocol: Protocol, cc: Option<u8>) -> Label {
    let label = Label::new().push(match protocol {
        Protocol::Unknown => b"NO SOURCE" as &[u8],
        Protocol::TypeC { ma: 0 } => b"USB",
        Protocol::TypeC { .. } => b"TYPE-C",
        Protocol::PdOffer | Protocol::Pd { revision: None, .. } => b"PD",
        Protocol::Pd {
            revision: Some(3), ..
        } => b"PD 3.0",
        Protocol::Pd { .. } => b"PD 2.0",
        Protocol::Pps { .. } => b"PPS",
    });
    let label = match cc {
        Some(cc) => label.push(b" | CC").push(&[b'0' + cc % 10]),
        None => label,
    };
    match protocol {
        Protocol::Unknown => label,
        Protocol::TypeC { ma: 0 } => label.push(b" | 5V"),
        Protocol::TypeC { ma } => label.push(b" | ").fixed(u32::from(ma) / 100, 1).push(b"A"),
        Protocol::PdOffer => label.push(b" | OFFER"),
        Protocol::Pd { mv, ma, .. } => label
            .push(b" | ")
            .fixed(u32::from(mv) / 1000, 0)
            .push(b"V ")
            .fixed(u32::from(ma) / 10, 2)
            .push(b"A"),
        Protocol::Pps { mv, ma } => label
            .push(b" | ")
            .fixed(u32::from(mv) / 10, 2)
            .push(b"V ")
            .fixed(u32::from(ma) / 10, 2)
            .push(b"A"),
    }
}

/// Settings rows: first baseline and pitch.
const SETTINGS_BASELINE: i32 = 46;
const SETTINGS_PITCH: i32 = 26;

/// The settings page; the row under `cursor` is marked and brightened.
pub fn draw_settings<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    settings: Settings,
    cursor: Item,
) {
    draw_header(target, header);
    for (index, item) in Item::ALL.into_iter().enumerate() {
        let baseline = SETTINGS_BASELINE + index as i32 * SETTINGS_PITCH;
        let selected = item == cursor;
        if selected {
            text(
                target,
                &ROW,
                ">",
                Point::new(LEFT, baseline),
                HorizontalAlignment::Left,
                ACCENT,
            );
        }
        let (name, value) = match item {
            Item::Brightness => (
                "BRIGHTNESS",
                Label::new()
                    .fixed(u32::from(settings.brightness_percent()), 0)
                    .push(b"%"),
            ),
            Item::ResetPeaks => ("RESET MAX", Label::new().push(b"CLEAR")),
            Item::DebugPages => (
                "DEBUG PAGES",
                Label::new().push(if settings.debug_pages { b"ON" } else { b"OFF" }),
            ),
        };
        text(
            target,
            &ROW,
            name,
            Point::new(LEFT + 14, baseline),
            HorizontalAlignment::Left,
            if selected { VALUE } else { MUTED },
        );
        text(
            target,
            &ROW,
            value.as_str(),
            Point::new(RIGHT, baseline),
            HorizontalAlignment::Right,
            if selected { ACCENT } else { MUTED },
        );
        if index + 1 < Item::ALL.len() {
            rule(target, baseline + SETTINGS_PITCH / 2 - 2);
        }
    }
}
