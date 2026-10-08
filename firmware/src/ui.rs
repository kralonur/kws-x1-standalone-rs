//! Graphical pages: the meter and settings screens, drawn with
//! `embedded-graphics` primitives and `u8g2-fonts` text on any RGB565 target.
//! The renderer gathers the values; nothing here touches hardware.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{
    Circle, Line, PrimitiveStyle, Rectangle, RoundedRectangle, StyledDrawable, Triangle,
};
use firmware_core::settings::{BRIGHTNESS_PERCENT, Item, Settings};
use u8g2_fonts::FontRenderer;
use u8g2_fonts::fonts;
use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};

/// Readings: Logisoso, 28 px digits (numerals and punctuation only).
const DIGITS: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_logisoso28_tn>().with_ignore_unknown_chars(true);
/// Units, badges and settings rows: Helvetica Bold 14.
const LABEL: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_helvB14_tr>().with_ignore_unknown_chars(true);
/// Peak row and notes: Helvetica Bold 10.
const SMALL: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_helvB10_tr>().with_ignore_unknown_chars(true);

pub const BACKGROUND: Rgb565 = Rgb565::BLACK;
const TEXT: Rgb565 = Rgb565::WHITE;
const VOLTS: Rgb565 = Rgb565::new(31, 56, 0);
const AMPS: Rgb565 = Rgb565::new(6, 60, 10);
const WATTS: Rgb565 = Rgb565::new(31, 34, 6);
const MUTED: Rgb565 = Rgb565::new(15, 31, 15);
const RULE: Rgb565 = Rgb565::new(7, 14, 7);
const HIGHLIGHT: Rgb565 = Rgb565::new(5, 11, 5);
/// Protocol badge colours.
const PD3: Rgb565 = Rgb565::new(6, 56, 12);
const PD2: Rgb565 = Rgb565::new(4, 48, 24);
const PPS: Rgb565 = Rgb565::new(31, 20, 26);
const TYPE_C: Rgb565 = Rgb565::new(28, 56, 28);

/// Meter layout, in pixels: digits end at `VALUE_RIGHT`, units start at
/// `UNIT_LEFT`, and the protocol column starts right of `DIVIDER_X`.
const VALUE_RIGHT: i32 = 136;
const UNIT_LEFT: i32 = 142;
const DIVIDER_X: i32 = 170;
/// Baselines of the voltage, current and power rows.
const ROW_BASELINES: [i32; 3] = [36, 73, 110];
/// Top of the peak row and its rule.
const PEAK_RULE_Y: i32 = 117;
const PEAK_BASELINE: i32 = 131;
/// Protocol column: left edge, width and centre.
const SIDE_LEFT: i32 = 175;
const SIDE_WIDTH: u32 = 62;
const SIDE_CENTER: i32 = SIDE_LEFT + SIDE_WIDTH as i32 / 2;

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

/// Everything the meter page shows. `reading` is `None` before the first
/// sample and while the bus is latched.
pub struct Meter {
    pub reading: Option<Reading>,
    pub peaks: Peaks,
    pub protocol: Protocol,
    /// Heartbeat dot colour (alternates every pulse, red on a bus fault).
    pub alive: Rgb565,
}

/// Small text buffer for formatted numbers and labels; writes past the end are dropped.
struct Label {
    bytes: [u8; 16],
    len: usize,
}

impl Label {
    fn new() -> Self {
        Self {
            bytes: [0; 16],
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

    /// `value / 10^places` with `places` decimals, e.g. `fixed(20012, 3)` is "20.012".
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

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).unwrap_or("")
    }
}

fn text<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    font: &FontRenderer,
    content: &str,
    position: Point,
    vertical: VerticalPosition,
    horizontal: HorizontalAlignment,
    color: Rgb565,
) {
    // Glyph lookups cannot fail (unknown characters are skipped) and drawing
    // into the frame is infallible.
    font.render_aligned(
        content,
        position,
        vertical,
        horizontal,
        FontColor::Transparent(color),
        target,
    )
    .ok();
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

fn outline<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    shape: impl StyledDrawable<PrimitiveStyle<Rgb565>, Color = Rgb565>,
    color: Rgb565,
    width: u32,
) {
    shape
        .draw_styled(&PrimitiveStyle::with_stroke(color, width), target)
        .ok();
}

/// Volts with three decimals, amps with three, watts with two (one from 100 W).
fn volts(mv: u32) -> Label {
    Label::new().fixed(mv, 3)
}
fn amps(ma: i32) -> Label {
    let sign: &[u8] = if ma < 0 { b"-" } else { b"" };
    Label::new().push(sign).fixed(ma.unsigned_abs(), 3)
}
fn watts(mw: u32) -> Label {
    if mw >= 100_000 {
        Label::new().fixed(mw / 100, 1)
    } else {
        Label::new().fixed(mw / 10, 2)
    }
}

/// The main page: volts, amps and watts on the left, the protocol on the
/// right and the peaks along the bottom.
pub fn draw_meter<D: DrawTarget<Color = Rgb565>>(target: &mut D, meter: &Meter) {
    let rows: [(Label, &str, Rgb565); 3] = match meter.reading {
        Some(reading) => [
            (volts(reading.mv), "V", VOLTS),
            (amps(reading.ma), "A", AMPS),
            (watts(reading.mw()), "W", WATTS),
        ],
        None => [
            (Label::new().push(b"--.---"), "V", MUTED),
            (Label::new().push(b"-.---"), "A", MUTED),
            (Label::new().push(b"--.--"), "W", MUTED),
        ],
    };
    for ((value, unit, color), baseline) in rows.iter().zip(ROW_BASELINES) {
        text(
            target,
            &DIGITS,
            value.as_str(),
            Point::new(VALUE_RIGHT, baseline),
            VerticalPosition::Baseline,
            HorizontalAlignment::Right,
            *color,
        );
        text(
            target,
            &LABEL,
            unit,
            Point::new(UNIT_LEFT, baseline),
            VerticalPosition::Baseline,
            HorizontalAlignment::Left,
            *color,
        );
    }

    outline(
        target,
        Line::new(
            Point::new(DIVIDER_X, 6),
            Point::new(DIVIDER_X, PEAK_RULE_Y - 6),
        ),
        RULE,
        1,
    );
    draw_protocol(target, meter.protocol);
    draw_peaks(target, meter.peaks);
    fill(
        target,
        Circle::new(Point::new(229, PEAK_BASELINE - 9), 7),
        meter.alive,
    );
}

fn draw_peaks<D: DrawTarget<Color = Rgb565>>(target: &mut D, peaks: Peaks) {
    fill(
        target,
        Rectangle::new(Point::new(0, PEAK_RULE_Y), Size::new(240, 1)),
        RULE,
    );
    // Upward triangle: "maximum".
    fill(
        target,
        Triangle::new(
            Point::new(4, PEAK_BASELINE),
            Point::new(14, PEAK_BASELINE),
            Point::new(9, PEAK_BASELINE - 9),
        ),
        MUTED,
    );
    let columns = [
        (Label::new().fixed(peaks.mv / 10, 2).push(b"V"), 22, VOLTS),
        (Label::new().fixed(peaks.ma / 10, 2).push(b"A"), 92, AMPS),
        (watts(peaks.mw).push(b"W"), 156, WATTS),
    ];
    for (value, x, color) in columns {
        text(
            target,
            &SMALL,
            value.as_str(),
            Point::new(x, PEAK_BASELINE),
            VerticalPosition::Baseline,
            HorizontalAlignment::Left,
            color,
        );
    }
}

fn draw_protocol<D: DrawTarget<Color = Rgb565>>(target: &mut D, protocol: Protocol) {
    let (badge, color): (&str, Rgb565) = match protocol {
        Protocol::Unknown => ("N/A", RULE),
        Protocol::TypeC { ma: 0 } => ("USB", TYPE_C),
        Protocol::TypeC { .. } => ("USB-C", TYPE_C),
        Protocol::PdOffer | Protocol::Pd { revision: None, .. } => ("PD", PD3),
        Protocol::Pd {
            revision: Some(3), ..
        } => ("PD3.0", PD3),
        Protocol::Pd { .. } => ("PD2.0", PD2),
        Protocol::Pps { .. } => ("PPS", PPS),
    };
    let badge_box = Rectangle::new(Point::new(SIDE_LEFT, 6), Size::new(SIDE_WIDTH, 24));
    fill(
        target,
        RoundedRectangle::with_equal_corners(badge_box, Size::new(6, 6)),
        color,
    );
    text(
        target,
        &LABEL,
        badge,
        badge_box.center(),
        VerticalPosition::Center,
        HorizontalAlignment::Center,
        if protocol == Protocol::Unknown {
            MUTED
        } else {
            BACKGROUND
        },
    );

    let icon_top = 38;
    match protocol {
        Protocol::Unknown => plug(target, icon_top, RULE),
        Protocol::TypeC { .. } => plug(target, icon_top, color),
        _ => bolt(target, icon_top, color),
    }

    let (first, second) = match protocol {
        Protocol::Unknown => (Label::new().push(b"--"), Label::new().push(b"--")),
        Protocol::TypeC { ma: 0 } => (Label::new().push(b"5V"), Label::new().push(b"DEFAULT")),
        Protocol::TypeC { ma } => (
            Label::new().push(b"5V"),
            Label::new().fixed(u32::from(ma) / 100, 1).push(b"A"),
        ),
        Protocol::PdOffer => (Label::new().push(b"5V"), Label::new().push(b"NO REQ")),
        Protocol::Pd { mv, ma, .. } => (
            Label::new().fixed(u32::from(mv) / 1000, 0).push(b"V"),
            Label::new().fixed(u32::from(ma) / 10, 2).push(b"A"),
        ),
        Protocol::Pps { mv, ma } => (
            Label::new().fixed(u32::from(mv) / 10, 2).push(b"V"),
            Label::new().fixed(u32::from(ma) / 10, 2).push(b"A"),
        ),
    };
    for (line, baseline) in [(first, 90), (second, 108)] {
        text(
            target,
            &LABEL,
            line.as_str(),
            Point::new(SIDE_CENTER, baseline),
            VerticalPosition::Baseline,
            HorizontalAlignment::Center,
            TEXT,
        );
    }
}

/// Lightning bolt, 22 x 34 px, centred in the protocol column.
fn bolt<D: DrawTarget<Color = Rgb565>>(target: &mut D, top: i32, color: Rgb565) {
    let x = SIDE_CENTER;
    fill(
        target,
        Triangle::new(
            Point::new(x + 6, top),
            Point::new(x - 10, top + 20),
            Point::new(x + 3, top + 20),
        ),
        color,
    );
    fill(
        target,
        Triangle::new(
            Point::new(x - 3, top + 14),
            Point::new(x + 10, top + 14),
            Point::new(x - 6, top + 34),
        ),
        color,
    );
}

/// USB plug: two prongs, a body and a cable, centred in the protocol column.
fn plug<D: DrawTarget<Color = Rgb565>>(target: &mut D, top: i32, color: Rgb565) {
    let x = SIDE_CENTER;
    for prong in [x - 7, x + 4] {
        fill(
            target,
            Rectangle::new(Point::new(prong, top), Size::new(3, 8)),
            color,
        );
    }
    fill(
        target,
        RoundedRectangle::with_equal_corners(
            Rectangle::new(Point::new(x - 11, top + 8), Size::new(22, 16)),
            Size::new(4, 4),
        ),
        color,
    );
    fill(
        target,
        Rectangle::new(Point::new(x - 2, top + 24), Size::new(4, 10)),
        color,
    );
}

/// Settings rows: top of the first row, row pitch and height.
const SETTINGS_TOP: i32 = 34;
const SETTINGS_PITCH: i32 = 32;
const SETTINGS_ROW_HEIGHT: u32 = 28;

/// The settings page; `cursor` is highlighted.
pub fn draw_settings<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    settings: Settings,
    cursor: Item,
) {
    gear(target, Point::new(16, 15), TEXT);
    text(
        target,
        &LABEL,
        "SETTINGS",
        Point::new(32, 15),
        VerticalPosition::Center,
        HorizontalAlignment::Left,
        TEXT,
    );
    fill(
        target,
        Rectangle::new(Point::new(0, 28), Size::new(240, 1)),
        RULE,
    );

    for (index, item) in Item::ALL.into_iter().enumerate() {
        let top = SETTINGS_TOP + index as i32 * SETTINGS_PITCH;
        let row = Rectangle::new(Point::new(4, top), Size::new(232, SETTINGS_ROW_HEIGHT));
        let selected = item == cursor;
        if selected {
            fill(
                target,
                RoundedRectangle::with_equal_corners(row, Size::new(6, 6)),
                HIGHLIGHT,
            );
            fill(
                target,
                Rectangle::new(
                    Point::new(4, top + 4),
                    Size::new(3, SETTINGS_ROW_HEIGHT - 8),
                ),
                VOLTS,
            );
        }
        let middle = top + SETTINGS_ROW_HEIGHT as i32 / 2;
        let name = match item {
            Item::Brightness => "Brightness",
            Item::ResetPeaks => "Reset max",
            Item::DebugPages => "Debug pages",
        };
        text(
            target,
            &LABEL,
            name,
            Point::new(14, middle),
            VerticalPosition::Center,
            HorizontalAlignment::Left,
            if selected { TEXT } else { MUTED },
        );
        match item {
            Item::Brightness => brightness_bars(target, middle, settings.brightness),
            Item::ResetPeaks => text(
                target,
                &SMALL,
                "PRESS",
                Point::new(228, middle),
                VerticalPosition::Center,
                HorizontalAlignment::Right,
                MUTED,
            ),
            Item::DebugPages => toggle(target, middle, settings.debug_pages),
        }
    }
}

/// One bar per level, rising left to right; lit up to the current level.
fn brightness_bars<D: DrawTarget<Color = Rgb565>>(target: &mut D, middle: i32, level: u8) {
    for index in 0..BRIGHTNESS_PERCENT.len() {
        let height = 6 + 3 * index as u32;
        let x = 168 + 12 * index as i32;
        let color = if index <= usize::from(level) {
            VOLTS
        } else {
            RULE
        };
        fill(
            target,
            Rectangle::new(
                Point::new(x, middle + 9 - height as i32),
                Size::new(8, height),
            ),
            color,
        );
    }
}

/// Pill switch: green with the knob right when on, grey with it left when off.
fn toggle<D: DrawTarget<Color = Rgb565>>(target: &mut D, middle: i32, on: bool) {
    let track = Rectangle::new(Point::new(192, middle - 9), Size::new(36, 18));
    fill(
        target,
        RoundedRectangle::with_equal_corners(track, Size::new(9, 9)),
        if on { PD3 } else { RULE },
    );
    let knob_x = if on { 212 } else { 194 };
    fill(
        target,
        Circle::new(Point::new(knob_x, middle - 7), 14),
        TEXT,
    );
}

/// Eight-toothed gear, 16 px across, centred on `center`.
fn gear<D: DrawTarget<Color = Rgb565>>(target: &mut D, center: Point, color: Rgb565) {
    for (dx, dy) in [
        (0, -7),
        (5, -5),
        (7, 0),
        (5, 5),
        (0, 7),
        (-5, 5),
        (-7, 0),
        (-5, -5),
    ] {
        fill(
            target,
            Rectangle::with_center(center + Point::new(dx, dy), Size::new(4, 4)),
            color,
        );
    }
    fill(target, Circle::with_center(center, 12), color);
    fill(target, Circle::with_center(center, 5), BACKGROUND);
}
