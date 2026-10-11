//! Graphical pages: overview, line voltages, PD profile and PPS selectors,
//! Quick Charge trigger, settings and the scrolling detail lists, drawn with
//! `embedded-graphics` primitives and `u8g2-fonts` text on any RGB565 target.
//! The renderer gathers the values; nothing here touches hardware.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{
    Circle, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle, StyledDrawable,
    Triangle,
};
use firmware_core::buttons::{
    Detail, Level, PpsFocus, QC2_VOLTAGES, Qc2Focus, QcOffer, SettingsFocus, Tab,
};
use firmware_core::settings::{BRIGHTNESS_PERCENT, Item, QcFindings, Rotation, Settings};
use u8g2_fonts::FontRenderer;
use u8g2_fonts::fonts;
use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};

use crate::line_adc::LineVoltages;
use crate::qc_drive::Status;
use qc_protocol::detect::{Outcome, Report, Stage};
use qc_protocol::qc2::{Failure, Voltage};
use qc_protocol::qc3::{MIN_MV, STEP_MV};

/// Voltage and current: Inconsolata Bold 19 px, numerals only.
const READING: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_inb19_mn>().with_ignore_unknown_chars(true);
/// Power: Inconsolata Bold 27 px, numerals only.
const POWER: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_inb27_mn>().with_ignore_unknown_chars(true);
/// Titles, units and profile voltages: t0 16 px bold.
const BOLD: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_t0_16b_mr>().with_ignore_unknown_chars(true);
/// Settings rows: t0 14 px bold.
const ROW: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_t0_14b_mr>().with_ignore_unknown_chars(true);
/// Labels, hints, peaks and the page counter: ProFont 12 px.
const PLAIN: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_profont12_mr>().with_ignore_unknown_chars(true);

const BLACK: Rgb565 = Rgb565::BLACK;
const WHITE: Rgb565 = Rgb565::new(30, 61, 30);
const YELLOW: Rgb565 = Rgb565::new(31, 58, 4);
const CYAN: Rgb565 = Rgb565::new(6, 54, 31);
const GREEN: Rgb565 = Rgb565::new(10, 62, 10);
const MAGENTA: Rgb565 = Rgb565::new(31, 24, 30);
const ORANGE: Rgb565 = Rgb565::new(31, 40, 4);
const MUTED: Rgb565 = Rgb565::new(21, 43, 22);
const RULE: Rgb565 = Rgb565::new(5, 14, 7);
/// Dark card fills matching the accents above.
const YELLOW_TINT: Rgb565 = Rgb565::new(5, 10, 0);
const CYAN_TINT: Rgb565 = Rgb565::new(1, 9, 6);
const GREEN_TINT: Rgb565 = Rgb565::new(1, 10, 1);
const MAGENTA_TINT: Rgb565 = Rgb565::new(6, 4, 6);

/// Screen width and the left/right margins, in pixels.
const WIDTH: u32 = 240;
const LEFT: i32 = 8;
const RIGHT: i32 = 232;
/// Header: title baseline and the rule under it.
const TITLE_BASELINE: i32 = 15;
const HEADER_RULE_Y: i32 = 21;
/// Header tab strip, in pixels: each tab's top and height, the shown tab's
/// label padding, a folded tab's width, the space between tabs, the
/// tab-level underline's height and the group-level marker's width.
const TAB_TOP: i32 = 3;
const TAB_HEIGHT: u32 = 16;
const TAB_PADDING: i32 = 6;
const FOLDED_TAB: u32 = 6;
const TAB_GAP: i32 = 3;
const TAB_UNDERLINE: u32 = 2;
const TAB_MARKER: u32 = 3;
const TAB_CORNER: Size = Size::new(3, 3);
/// Group dots on the header's right: diameter and centre spacing, in pixels.
const GROUP_DOT: u32 = 6;
const GROUP_DOT_PITCH: i32 = 10;
/// The group dots' centre line, level with the tab labels' capitals.
const GROUP_DOT_Y: i32 = TITLE_BASELINE - 5;
/// Button hint baseline, along the bottom edge.
const HINT_BASELINE: i32 = 131;
/// Card corner radius.
const CORNER: Size = Size::new(5, 5);
/// Gap between a number and its unit, in pixels.
const UNIT_GAP: i32 = 3;

/// One meter reading. `ma` is signed: negative when the charger is on the
/// plug side and current flows from plug to receptacle.
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

/// A supply level: fixed or PPS, in mV and mA.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Supply {
    pub mv: u16,
    pub ma: u16,
    pub pps: bool,
}

/// One PPS range the charger offers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PpsRange {
    pub min_mv: u16,
    pub max_mv: u16,
    pub max_ma: u16,
}

/// Most power objects one Source_Capabilities message can carry (USB PD
/// section 6.4.1: up to seven data objects).
pub const MAX_OBJECTS: usize = 7;

/// What the charger offers and what this meter previewed and agreed, as the
/// selector pages show it.
#[derive(Clone, Copy, Debug, Default)]
pub struct Offer {
    /// The sink session runs, so profiles can be requested.
    pub session: bool,
    /// This boot keeps PD silent for Quick Charge, so no offer is heard.
    pub quick_charge: bool,
    fixed: [Supply; MAX_OBJECTS],
    fixed_count: usize,
    pps: [PpsRange; MAX_OBJECTS],
    pps_count: usize,
    /// The profile Left/Right previewed; nothing is sent before OK.
    pub preview: Option<Supply>,
    /// The contract in force.
    pub contract: Option<Supply>,
}

impl Offer {
    pub fn push_fixed(&mut self, mv: u16, ma: u16) {
        if let Some(slot) = self.fixed.get_mut(self.fixed_count) {
            *slot = Supply { mv, ma, pps: false };
            self.fixed_count += 1;
        }
    }

    pub fn push_pps(&mut self, range: PpsRange) {
        if let Some(slot) = self.pps.get_mut(self.pps_count) {
            *slot = range;
            self.pps_count += 1;
        }
    }

    /// Advertised maximum current of the fixed profile at `mv`.
    pub fn fixed_current(&self, mv: u16) -> Option<u16> {
        self.fixed()
            .iter()
            .find(|profile| profile.mv == mv)
            .map(|profile| profile.ma)
    }

    fn fixed(&self) -> &[Supply] {
        &self.fixed[..self.fixed_count]
    }

    fn pps(&self) -> &[PpsRange] {
        &self.pps[..self.pps_count]
    }
}

/// Page header.
pub enum Header<'a> {
    /// A page in the groups: its group's tabs as a strip, each in its own
    /// colour and only the shown one labelled, so a group fits many tabs;
    /// and a dot per group on the right.
    Tabs {
        tabs: &'a [Tab],
        /// Index of the shown tab in `tabs`.
        current: usize,
        /// The shown group's index and the group count.
        group: (usize, usize),
        /// What Left/Right move: the dots light for groups, the shown tab is
        /// underlined for tabs.
        level: Level,
        /// The PD session page is the passthrough sniffer.
        passthrough: bool,
    },
    /// The settings, outside the groups.
    Settings,
}

/// A tab's label and colour.
fn tab_style(tab: Tab, passthrough: bool) -> (&'static str, Rgb565) {
    match tab {
        Tab::Overview => ("OVERVIEW", CYAN),
        Tab::Lines => ("LINES", YELLOW),
        Tab::Fixed => ("PD PROFILES", GREEN),
        Tab::Pps => ("PPS", MAGENTA),
        Tab::Qc2 => ("QC 2.0", ORANGE),
        Tab::Qc3 => ("QC 3.0", CYAN),
        Tab::Detail { page, .. } => match page {
            Detail::Pd if passthrough => ("PD SNIFF", GREEN),
            Detail::Pd => ("PD SESSION", GREEN),
            Detail::Charger => ("CHARGER", ORANGE),
            Detail::Cable => ("CABLE", CYAN),
            Detail::System => ("SYSTEM", YELLOW),
        },
    }
}

/// Everything the overview page shows. `reading` is `None` before the first
/// sample and while the bus is latched.
pub struct Meter {
    pub reading: Option<Reading>,
    pub peaks: Peaks,
    pub protocol: Protocol,
    /// Attached CC pin (1 or 2), when known.
    pub cc: Option<u8>,
    /// The frame is shown turned 180 degrees, so the plug is on the right.
    pub flipped: bool,
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

    /// Milli-units with two decimals, e.g. 2012 mV is "2.01".
    fn hundredths(self, milli: u32) -> Self {
        self.fixed(milli / 10, 2)
    }

    /// Watts from mW: two decimals, one from 100 W so it keeps five characters.
    fn watts(self, mw: u32) -> Self {
        if mw >= 100_000 {
            self.fixed(mw / 100, 1)
        } else {
            self.hundredths(mw)
        }
    }

    /// Whole volts when exact ("9V"), otherwise two decimals ("9.20V").
    fn volts(self, mv: u16) -> Self {
        let mv = u32::from(mv);
        if mv.is_multiple_of(1000) {
            self.fixed(mv / 1000, 0).push(b"V")
        } else {
            self.hundredths(mv).push(b"V")
        }
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).unwrap_or("")
    }
}

/// Draw `content` on its baseline and return the x just past it.
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

/// A rounded card filled with `tint`, outlined in `border` when given.
fn card<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    area: Rectangle,
    tint: Rgb565,
    border: Option<Rgb565>,
) {
    let mut style = PrimitiveStyleBuilder::new().fill_color(tint);
    if let Some(border) = border {
        style = style.stroke_color(border).stroke_width(2);
    }
    RoundedRectangle::with_equal_corners(area, CORNER)
        .draw_styled(&style.build(), target)
        .ok();
}

fn rule<D: DrawTarget<Color = Rgb565>>(target: &mut D, y: i32) {
    fill(
        target,
        Rectangle::new(Point::new(LEFT - 2, y), Size::new(WIDTH - 12, 1)),
        RULE,
    );
}

/// A number followed by its unit.
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
        color,
    );
}

/// The tab strip and group dots, or the settings title, and the rule.
fn draw_header<D: DrawTarget<Color = Rgb565>>(target: &mut D, header: &Header<'_>) {
    rule(target, HEADER_RULE_Y);
    let &Header::Tabs {
        tabs,
        current,
        group: (shown_group, groups),
        level,
        passthrough,
    } = header
    else {
        fill(
            target,
            Rectangle::new(Point::new(LEFT - 2, 5), Size::new(3, 13)),
            YELLOW,
        );
        text(
            target,
            &BOLD,
            "SETTINGS",
            Point::new(LEFT + 5, TITLE_BASELINE),
            HorizontalAlignment::Left,
            YELLOW,
        );
        return;
    };
    let mut x = LEFT - 2;
    let mut shown_color = WHITE;
    for (index, tab) in tabs.iter().enumerate() {
        let (label, color) = tab_style(*tab, passthrough);
        let shown = index == current;
        let width = if shown {
            let label_width = BOLD
                .get_rendered_dimensions(label, Point::zero(), VerticalPosition::Baseline)
                .map_or(0, |dimensions| dimensions.advance.x);
            (label_width + 2 * TAB_PADDING) as u32
        } else {
            FOLDED_TAB
        };
        let area = Rectangle::new(Point::new(x, TAB_TOP), Size::new(width, TAB_HEIGHT));
        // The shown tab marks the level: a bar beside its name while
        // Left/Right move between groups, an underline while they move
        // between tabs, a filled tab once the page itself has them.
        let line = Size::zero();
        let (marker, corner, label_color) = match level {
            _ if !shown => (area, TAB_CORNER, color),
            Level::Groups => (
                Rectangle::new(area.top_left, Size::new(TAB_MARKER, TAB_HEIGHT)),
                line,
                color,
            ),
            Level::Tabs => (
                Rectangle::new(
                    Point::new(x, HEADER_RULE_Y + 1 - TAB_UNDERLINE as i32),
                    Size::new(width, TAB_UNDERLINE),
                ),
                line,
                color,
            ),
            Level::Page => (area, TAB_CORNER, BLACK),
        };
        fill(
            target,
            RoundedRectangle::with_equal_corners(marker, corner),
            color,
        );
        if shown {
            shown_color = color;
            text(
                target,
                &BOLD,
                label,
                Point::new(x + TAB_PADDING, TITLE_BASELINE),
                HorizontalAlignment::Left,
                label_color,
            );
        }
        x += width as i32 + TAB_GAP;
    }
    let lit = if level == Level::Groups {
        shown_color
    } else {
        MUTED
    };
    for index in 0..groups {
        let offset = GROUP_DOT_PITCH * (groups - 1 - index) as i32;
        fill(
            target,
            Circle::with_center(
                Point::new(RIGHT - GROUP_DOT as i32 / 2 - offset, GROUP_DOT_Y),
                GROUP_DOT,
            ),
            if index == shown_group { lit } else { RULE },
        );
    }
}

/// The bottom hint: `content`, the page's own, unless Left/Right move
/// between groups.
fn hint<D: DrawTarget<Color = Rgb565>>(target: &mut D, header: &Header<'_>, content: &str) {
    text(
        target,
        &PLAIN,
        if matches!(
            header,
            Header::Tabs {
                level: Level::Groups,
                ..
            }
        ) {
            "< > groups    OK tabs"
        } else {
            content
        },
        Point::new(LEFT, HINT_BASELINE),
        HorizontalAlignment::Left,
        MUTED,
    );
}

/// Badge text and colour for a protocol.
fn protocol_badge(protocol: Protocol) -> (&'static str, Rgb565) {
    match protocol {
        Protocol::Unknown => ("NONE", RULE),
        Protocol::TypeC { ma: 0 } => ("USB", WHITE),
        Protocol::TypeC { .. } => ("TYPE-C", WHITE),
        Protocol::PdOffer | Protocol::Pd { revision: None, .. } => ("PD", GREEN),
        Protocol::Pd {
            revision: Some(3), ..
        } => ("PD 3.0", GREEN),
        Protocol::Pd { .. } => ("PD 2.0", CYAN),
        Protocol::Pps { .. } => ("PPS", MAGENTA),
    }
}

/// Colour of a supply voltage: cooler for low, warmer for high.
fn level_color(mv: u16) -> Rgb565 {
    match mv {
        0..=5_500 => WHITE,
        5_501..=9_500 => CYAN,
        9_501..=12_500 => GREEN,
        12_501..=15_500 => YELLOW,
        _ => ORANGE,
    }
}

/// Overview cards: voltage and current on top, power and peaks below.
const VOLTS_CARD: Rectangle = Rectangle::new(Point::new(4, 25), Size::new(114, 41));
const AMPS_CARD: Rectangle = Rectangle::new(Point::new(122, 25), Size::new(114, 41));
const POWER_CARD: Rectangle = Rectangle::new(Point::new(4, 70), Size::new(150, 48));
const PEAKS_CARD: Rectangle = Rectangle::new(Point::new(158, 70), Size::new(78, 48));
/// Card label baseline and value baseline, from the card top.
const CARD_LABEL: i32 = 11;
const CARD_VALUE: i32 = 35;
const POWER_VALUE: i32 = 43;
/// Below this current magnitude no flow arrow is drawn, in mA: a chosen
/// margin over the shunt's zero offset (40 INA226 counts), not a measured one.
const FLOW_MIN_MA: u32 = 20;
/// Right edge of the flow chevrons on the bottom row, in pixels: clear of
/// the heartbeat dot.
const FLOW_RIGHT: i32 = RIGHT - 10;
/// Protocol pill along the bottom edge.
const PILL: Rectangle = Rectangle::new(Point::new(4, 121), Size::new(52, 13));

fn card_label<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    area: Rectangle,
    label: &str,
    color: Rgb565,
) {
    text(
        target,
        &PLAIN,
        label,
        area.top_left + Point::new(6, CARD_LABEL),
        HorizontalAlignment::Left,
        color,
    );
}

/// The overview page.
pub fn draw_meter<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    meter: &Meter,
) {
    draw_header(target, header);
    let (volts, amps, power) = match meter.reading {
        // The meter works in either direction (charger on the plug or on the
        // receptacle), so the current is shown as a magnitude.
        Some(reading) => (
            Label::new().hundredths(reading.mv),
            Label::new().hundredths(reading.ma.unsigned_abs()),
            Label::new().watts(reading.mw()),
        ),
        None => (
            Label::new().push(b"--.--"),
            Label::new().push(b"-.--"),
            Label::new().push(b"--.--"),
        ),
    };
    let cards = [
        (VOLTS_CARD, YELLOW_TINT, YELLOW, "VOLTAGE", volts, "V"),
        (AMPS_CARD, CYAN_TINT, CYAN, "CURRENT", amps, "A"),
    ];
    for (area, tint, color, label, value, unit) in &cards {
        card(target, *area, *tint, None);
        card_label(target, *area, label, *color);
        value_with_unit(
            target,
            &READING,
            value,
            unit,
            area.top_left + Point::new(6, CARD_VALUE),
            *color,
        );
    }
    card(target, POWER_CARD, GREEN_TINT, None);
    card_label(target, POWER_CARD, "POWER", GREEN);
    value_with_unit(
        target,
        &POWER,
        &power,
        "W",
        POWER_CARD.top_left + Point::new(6, POWER_VALUE),
        GREEN,
    );

    card(target, PEAKS_CARD, MAGENTA_TINT, None);
    card_label(target, PEAKS_CARD, "MAX", MAGENTA);
    let peaks = [
        (Label::new().hundredths(meter.peaks.mv).push(b"V"), YELLOW),
        (Label::new().hundredths(meter.peaks.ma).push(b"A"), CYAN),
        (Label::new().watts(meter.peaks.mw).push(b"W"), GREEN),
    ];
    for (index, (peak, color)) in peaks.iter().enumerate() {
        text(
            target,
            &PLAIN,
            peak.as_str(),
            Point::new(
                PEAKS_CARD.top_left.x + PEAKS_CARD.size.width as i32 - 6,
                PEAKS_CARD.top_left.y + 22 + 11 * index as i32,
            ),
            HorizontalAlignment::Right,
            *color,
        );
    }

    let (badge, color) = protocol_badge(meter.protocol);
    fill(
        target,
        RoundedRectangle::with_equal_corners(PILL, Size::new(6, 6)),
        color,
    );
    text(
        target,
        &PLAIN,
        badge,
        Point::new(PILL.center().x, HINT_BASELINE - 1),
        HorizontalAlignment::Center,
        BLACK,
    );
    text(
        target,
        &PLAIN,
        status(meter.protocol, meter.cc).as_str(),
        Point::new(
            PILL.top_left.x + PILL.size.width as i32 + 6,
            HINT_BASELINE - 1,
        ),
        HorizontalAlignment::Left,
        WHITE,
    );
    if let Some(reading) = meter.reading {
        flow(target, reading.ma, meter.flipped);
    }
    fill(
        target,
        Circle::with_center(Point::new(RIGHT - 1, HINT_BASELINE - 5), 6),
        meter.alive,
    );
}

/// Which way the current flows, as chevrons on the bottom row. Upright, the
/// plug is on the left and the receptacle on the right; `flipped` (the frame
/// turned 180 degrees) swaps them. Positive current runs from the receptacle
/// to the plug (charger on the receptacle), negative the other way.
fn flow<D: DrawTarget<Color = Rgb565>>(target: &mut D, ma: i32, flipped: bool) {
    if ma.unsigned_abs() < FLOW_MIN_MA {
        return;
    }
    let toward_plug = ma > 0;
    let chevrons = if toward_plug != flipped {
        "< < <"
    } else {
        "> > >"
    };
    text(
        target,
        &BOLD,
        chevrons,
        Point::new(FLOW_RIGHT, HINT_BASELINE - 1),
        HorizontalAlignment::Right,
        CYAN,
    );
}

/// "CC1  20V 3.00A": attached CC pin and contract.
fn status(protocol: Protocol, cc: Option<u8>) -> Label {
    let label = match cc {
        Some(cc) => Label::new().push(b"CC").push(&[b'0' + cc % 10, b' ', b' ']),
        None => Label::new(),
    };
    match protocol {
        Protocol::Unknown => label.push(b"no source"),
        Protocol::TypeC { ma: 0 } => label.push(b"5V default"),
        Protocol::TypeC { ma } => label.push(b"5V ").hundredths(u32::from(ma)).push(b"A"),
        Protocol::PdOffer => label.push(b"offer, 5V"),
        Protocol::Pd { mv, ma, .. } | Protocol::Pps { mv, ma } => label
            .volts(mv)
            .push(b" ")
            .hundredths(u32::from(ma))
            .push(b"A"),
    }
}

/// Line cards, two by two: the data pair on top, the CC pair below.
const DP_CARD: Rectangle = VOLTS_CARD;
const DM_CARD: Rectangle = AMPS_CARD;
const CC1_CARD: Rectangle = Rectangle::new(Point::new(4, 70), Size::new(114, 41));
const CC2_CARD: Rectangle = Rectangle::new(Point::new(122, 70), Size::new(114, 41));

/// A reading in volts from mV, or a blank one before the first.
fn volts_or_blank(mv: Option<u32>) -> Label {
    match mv {
        Some(mv) => Label::new().hundredths(mv),
        None => Label::new().push(b"-.--"),
    }
}

/// One line's voltage from a scan, blank without one.
fn line_volts(lines: Option<LineVoltages>, mv: fn(&LineVoltages) -> u16) -> Label {
    volts_or_blank(lines.map(|l| u32::from(mv(&l))))
}

/// Tinted cards each showing a reading in volts: area, tint, accent, label
/// and value.
fn volt_cards<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    cards: &[(Rectangle, Rgb565, Rgb565, &str, Label)],
) {
    for (area, tint, color, label, value) in cards {
        card(target, *area, *tint, None);
        card_label(target, *area, label, *color);
        value_with_unit(
            target,
            &READING,
            value,
            "V",
            area.top_left + Point::new(6, CARD_VALUE),
            *color,
        );
    }
}

/// The line voltage page. `lines` is `None` before the first scan and after
/// a failed one.
pub fn draw_lines<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    lines: Option<LineVoltages>,
) {
    draw_header(target, header);
    let value = |mv| line_volts(lines, mv);
    volt_cards(
        target,
        &[
            (DP_CARD, YELLOW_TINT, YELLOW, "D+", value(|l| l.dp_mv)),
            (DM_CARD, CYAN_TINT, CYAN, "D-", value(|l| l.dm_mv)),
            (CC1_CARD, GREEN_TINT, GREEN, "CC1", value(|l| l.cc1_mv)),
            (CC2_CARD, MAGENTA_TINT, MAGENTA, "CC2", value(|l| l.cc2_mv)),
        ],
    );
    hint(target, header, "uncalibrated    < > tabs");
}

/// How the PD side of the port stands, for the Quick Charge hints.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QcMode {
    /// No PD contract: requests reach the charger.
    Plain,
    /// A PD contract holds the charger, which then ignores Quick Charge; OK
    /// restarts it into a silent boot.
    Pd,
    /// This boot keeps PD silent (`NextBoot::silent`); Back on the stopped
    /// page restarts the charger into PD.
    Silent,
}

/// A Quick Charge tab under a PD contract, which the charger then ignores:
/// only how to switch.
fn pd_mode<D: DrawTarget<Color = Rgb565>>(target: &mut D, header: &Header<'_>) {
    notice(target, "PD MODE", "the charger ignores QC under PD", GREEN);
    hint(target, header, "OK switch to QC, tests up to 20V");
}

/// A PD tab while this boot keeps PD silent for Quick Charge: only how to
/// switch back.
fn qc_mode<D: DrawTarget<Color = Rgb565>>(target: &mut D, header: &Header<'_>) {
    notice(
        target,
        "QC MODE",
        "PD is off while Quick Charge runs",
        ORANGE,
    );
    hint(target, header, "BACK on a stopped QC tab: PD");
}

/// Hint on a stopped Quick Charge tab: the probe comes first while this
/// boot has none, else `start`.
fn qc_start_hint(mode: QcMode, qc: QcOffer, start: &'static str) -> &'static str {
    match (mode, qc.report()) {
        (_, None) => "OK test the charger, up to 20V",
        (QcMode::Silent, Some(_)) => "OK start    BACK back to PD",
        (_, Some(_)) => start,
    }
}

/// The status pill on the Quick Charge 2.0 tab: on the voltage it reports,
/// or on the request while entering or after a failed entry.
fn qc2_pill(status: Status, request: Option<Voltage>) -> Option<(Voltage, &'static str, Rgb565)> {
    Some(match status {
        Status::Entering => (request?, "WAIT", ORANGE),
        Status::Failed(Failure::NotDcp) => (request?, "NO DCP", MAGENTA),
        Status::Failed(Failure::NoEntry) => (request?, "NO QC", MAGENTA),
        Status::Active(voltage, Some(Outcome::Waiting)) => (voltage, "SETTING", ORANGE),
        Status::Active(voltage, Some(Outcome::Ignored)) => (voltage, "IGNORED", MAGENTA),
        Status::Active(voltage, _) => (voltage, "ACTIVE", GREEN),
        Status::Off | Status::Stepping(_) | Status::Detecting(_) | Status::Detected(_) => {
            return None;
        }
    })
}

/// Baseline of the probe's verdict under the Quick Charge 2.0 rows.
const QC2_PROBE_BASELINE: i32 = 116;

/// Everything the Quick Charge 2.0 tab shows.
pub struct Qc2 {
    /// The running trigger; `None` while stopped.
    pub focus: Option<Qc2Focus>,
    pub mode: QcMode,
    pub status: Status,
    pub qc: QcOffer,
}

/// The Quick Charge 2.0 tab, laid out as the fixed PD profiles: one row per
/// voltage, the choice framed, the status on its row. Voltages the probe
/// found the charger ignores are greyed out and cannot be chosen.
pub fn draw_qc2<D: DrawTarget<Color = Rgb565>>(target: &mut D, header: &Header<'_>, page: &Qc2) {
    let &Qc2 {
        focus,
        mode,
        status,
        qc,
    } = page;
    draw_header(target, header);
    if focus.is_none() && mode == QcMode::Pd {
        pd_mode(target, header);
        return;
    }
    let preview = match focus {
        Some(Qc2Focus::Choosing { preview, .. }) => Some(preview),
        _ => None,
    };
    let pill = qc2_pill(status, focus.map(Qc2Focus::request));
    for (row, voltage) in QC2_VOLTAGES.into_iter().enumerate() {
        let taken = qc.takes(voltage);
        let detail = match (taken, voltage, qc.report()) {
            // 20 V is Class B only (NCP4371 Table 4).
            (_, Voltage::V20, None) => "Class B only",
            (true, ..) => "",
            (false, Voltage::V20, _) => "needs Class B",
            (false, ..) => "charger ignored it",
        };
        selector_row(
            target,
            row,
            Label::new().volts(voltage.millivolts()).as_str(),
            if taken {
                level_color(voltage.millivolts())
            } else {
                MUTED
            },
            [detail, ""],
            preview == Some(voltage),
            pill.filter(|(at, ..)| *at == voltage)
                .map(|(_, label, color)| (label, color)),
        );
    }
    if let Some(report) = qc.report() {
        let line = Label::new()
            .push(b"probe: ")
            .push(report_summary(report).as_str().as_bytes());
        text(
            target,
            &PLAIN,
            line.as_str(),
            Point::new(LEFT, QC2_PROBE_BASELINE),
            HorizontalAlignment::Left,
            CYAN,
        );
    }
    let hint_text = match (focus, status) {
        (None, _) => qc_start_hint(mode, qc, "OK choose a voltage    < > tabs"),
        (Some(Qc2Focus::Choosing { .. }), _) => "< > choose  OK request  BACK cancel",
        (_, Status::Entering) => "handshake    BACK stop",
        (_, Status::Failed(Failure::NotDcp)) => "D+ D- not shorted    BACK",
        (_, Status::Failed(Failure::NoEntry)) => "D- stayed high    BACK",
        (_, Status::Active(_, Some(Outcome::Waiting))) => "waiting for VBUS    BACK stop",
        (_, Status::Active(_, Some(Outcome::Ignored))) => "charger kept its output",
        _ => "OK change    BACK stop",
    };
    hint(target, header, hint_text);
}

/// The probe, on either Quick Charge tab: what it is asking the charger.
pub fn draw_probe<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    status: Status,
) {
    draw_header(target, header);
    let step;
    let (title, detail, color) = match status {
        Status::Failed(Failure::NotDcp) => {
            ("NO CHARGING PORT", "D+ and D- are not shorted", MAGENTA)
        }
        Status::Failed(Failure::NoEntry) => ("NO QUICK CHARGE", "D- stayed high", MAGENTA),
        Status::Detecting(stage) => (
            "TESTING THE CHARGER",
            match stage {
                Stage::Fixed(voltage) => {
                    step = Label::new()
                        .push(b"asking for ")
                        .volts(voltage.millivolts());
                    step.as_str()
                }
                Stage::Back => "back to 5V",
                Stage::Continuous => "QC 3.0 continuous mode",
                Stage::StepUp(sent) => {
                    step = Label::new()
                        .push(b"QC 3.0 steps +")
                        .hundredths(u32::from(sent) * u32::from(STEP_MV))
                        .push(b"V");
                    step.as_str()
                }
            },
            ORANGE,
        ),
        _ => ("TESTING THE CHARGER", "Quick Charge handshake", ORANGE),
    };
    notice(target, title, detail, color);
    hint(
        target,
        header,
        if matches!(status, Status::Failed(_)) {
            "BACK"
        } else {
            "up to 20V on the output    BACK stop"
        },
    );
}

/// Everything the Quick Charge 3.0 tab shows.
pub struct Qc3 {
    /// The output the steps aim for in mV, while the trigger runs.
    pub target_mv: Option<u16>,
    pub mode: QcMode,
    pub status: Status,
    pub qc: QcOffer,
    pub lines: Option<LineVoltages>,
    pub reading: Option<Reading>,
}

/// The Quick Charge 3.0 tab, laid out as the PPS page: the step range, the
/// target and the output, and the driven lines as measured.
pub fn draw_qc3<D: DrawTarget<Color = Rgb565>>(target: &mut D, header: &Header<'_>, page: &Qc3) {
    let &Qc3 {
        target_mv,
        mode,
        status,
        qc,
        lines,
        reading,
    } = page;
    draw_header(target, header);
    if target_mv.is_none() && mode == QcMode::Pd {
        pd_mode(target, header);
        return;
    }
    if !qc.steps() {
        notice(target, "NO QC 3.0", "the charger ignored the steps", CYAN);
        hint(target, header, "< > tabs");
        return;
    }
    // The range the probe found, else both classes' (NCP4371 Table 4).
    let class_range = |max: Voltage, class: &[u8]| {
        Label::new()
            .fixed(u32::from(MIN_MV) / 100, 1)
            .push(b"-")
            .fixed(u32::from(max.millivolts()) / 1000, 0)
            .push(b"V by ")
            .push(step_size().as_str().as_bytes())
            .push(class)
    };
    let ranges = match qc.report().and_then(step_range) {
        Some(range) => [Some(range), None],
        None => [
            Some(class_range(Voltage::V12, b"  Class A")),
            Some(class_range(Voltage::V20, b"  Class B")),
        ],
    };
    for (index, range) in ranges.iter().flatten().enumerate() {
        text(
            target,
            &PLAIN,
            range.as_str(),
            Point::new(LEFT, RANGE_BASELINE + RANGE_PITCH * index as i32),
            HorizontalAlignment::Left,
            CYAN,
        );
    }

    let set = match target_mv {
        Some(mv) => Label::new().hundredths(u32::from(mv)),
        None => Label::new().push(b"--.--"),
    };
    card(
        target,
        PPS_VOLTS,
        if target_mv.is_some() { RULE } else { BLACK },
        Some(if target_mv.is_some() { YELLOW } else { RULE }),
    );
    card_label(target, PPS_VOLTS, "SET VOLTAGE", YELLOW);
    value_with_unit(
        target,
        &READING,
        &set,
        "V",
        PPS_VOLTS.top_left + PPS_VALUE,
        if target_mv.is_some() { WHITE } else { YELLOW },
    );
    if target_mv.is_some() {
        arrows(target, PPS_VOLTS, YELLOW);
    }
    card(target, PPS_AMPS, BLACK, Some(RULE));
    card_label(target, PPS_AMPS, "VBUS", CYAN);
    value_with_unit(
        target,
        &READING,
        &volts_or_blank(reading.map(|r| r.mv)),
        "V",
        PPS_AMPS.top_left + PPS_VALUE,
        CYAN,
    );

    let readback = Label::new()
        .push(b"D+ ")
        .push(line_volts(lines, |l| l.dp_mv).as_str().as_bytes())
        .push(b"V  D- ")
        .push(line_volts(lines, |l| l.dm_mv).as_str().as_bytes())
        .push(b"V");
    text(
        target,
        &PLAIN,
        readback.as_str(),
        Point::new(LEFT, PPS_STATUS_BASELINE),
        HorizontalAlignment::Left,
        WHITE,
    );
    let (state, color) = match status {
        Status::Entering => ("WAIT", ORANGE),
        Status::Failed(Failure::NotDcp) => ("NO DCP", MAGENTA),
        Status::Failed(Failure::NoEntry) => ("NO QC", MAGENTA),
        Status::Stepping(mv) if Some(mv) == target_mv => ("ACTIVE", GREEN),
        Status::Stepping(_) => ("STEPPING", ORANGE),
        _ => ("", MUTED),
    };
    text(
        target,
        &PLAIN,
        state,
        Point::new(RIGHT, PPS_STATUS_BASELINE),
        HorizontalAlignment::Right,
        color,
    );
    hint(
        target,
        header,
        match (target_mv, status) {
            (None, _) => qc_start_hint(mode, qc, "OK start at 5V    < > tabs"),
            (_, Status::Entering) => "handshake    BACK stop",
            (_, Status::Failed(Failure::NotDcp)) => "D+ D- not shorted    BACK",
            (_, Status::Failed(Failure::NoEntry)) => "D- stayed high    BACK",
            _ => "< > 0.2V    BACK stop",
        },
    );
}

/// The probe's verdict: the version, then the class (Class A ends at 12 V,
/// Class B adds 20 V).
fn report_summary(report: Report) -> Label {
    if !report.v9 && !report.steps {
        return Label::new().push(b"5V ONLY");
    }
    let version: &[u8] = if report.steps { b"QC3 " } else { b"QC2 " };
    Label::new()
        .push(version)
        .push(if report.v20 { b"B" } else { b"A" })
}

/// The Quick Charge 3.0 step range, from 3.6 V to the class maximum, e.g.
/// "3.6-12V by 0.2V"; `None` without steps.
fn step_range(report: Report) -> Option<Label> {
    Some(
        step_span(report)?
            .push(b" by ")
            .push(step_size().as_str().as_bytes()),
    )
}

/// From 3.6 V to the class maximum, e.g. "3.6-12V"; `None` without steps.
fn step_span(report: Report) -> Option<Label> {
    if !report.steps {
        return None;
    }
    let max = if report.v20 {
        Voltage::V20
    } else {
        Voltage::V12
    };
    Some(
        Label::new()
            .fixed(u32::from(MIN_MV) / 100, 1)
            .push(b"-")
            .fixed(u32::from(max.millivolts()) / 1000, 0)
            .push(b"V"),
    )
}

/// "0.2V", the Quick Charge 3.0 step.
fn step_size() -> Label {
    Label::new().fixed(u32::from(STEP_MV) / 100, 1).push(b"V")
}

/// Sections of the charger page's Quick Charge part: Quick Charge 2.0 and
/// 3.0 apart after a probe, one otherwise.
pub fn qc_charger_sections(findings: Option<QcFindings>) -> u32 {
    if matches!(findings, Some(QcFindings::Detected(_))) {
        2
    } else {
        1
    }
}

/// Section `section` of [`qc_charger_sections`]: what each Quick Charge
/// version took in the last probe, or how to get one.
pub fn qc_charger_lines(findings: Option<QcFindings>, mode: QcMode, section: u32) -> Lines {
    let mut rows = [[b' '; 24]; 8];
    let mut row = 0;
    let mut line = |text: &[u8]| {
        for (cell, &byte) in rows[row].iter_mut().zip(text) {
            *cell = byte.to_ascii_uppercase();
        }
        row += 1;
    };
    match findings {
        Some(QcFindings::Detected(report)) if section == 0 => {
            line(b"QUICK CHARGE 2.0");
            if !report.v9 {
                line(b"NOT SUPPORTED");
                return rows;
            }
            line(if report.v20 {
                b"CLASS B, UP TO 20V"
            } else {
                b"CLASS A, UP TO 12V"
            });
            let fixed: [(&[u8], bool); 4] = [
                (b"5V ", true),
                (b"9V ", report.v9),
                (b"12V", report.v12),
                (b"20V", report.v20),
            ];
            for (volts, taken) in fixed {
                let answer: &[u8] = if taken { b"  YES" } else { b"  NO" };
                line(Label::new().push(volts).push(answer).as_str().as_bytes());
            }
        }
        Some(QcFindings::Detected(report)) => {
            line(b"QUICK CHARGE 3.0");
            match step_span(report) {
                Some(span) => {
                    line(b"SUPPORTED");
                    line(
                        Label::new()
                            .push(b"RANGE ")
                            .push(span.as_str().as_bytes())
                            .as_str()
                            .as_bytes(),
                    );
                    line(
                        Label::new()
                            .push(b"STEP ")
                            .push(step_size().as_str().as_bytes())
                            .as_str()
                            .as_bytes(),
                    );
                }
                None => {
                    line(b"NOT SUPPORTED");
                    line(b"NO 0.2V STEPS");
                }
            }
        }
        Some(QcFindings::Failed(Failure::NotDcp)) => {
            line(b"QUICK CHARGE");
            line(b"NOT A CHARGING PORT");
            line(b"D+ D- NOT SHORTED");
        }
        Some(QcFindings::Failed(Failure::NoEntry)) => {
            line(b"QUICK CHARGE");
            line(b"NO QUICK CHARGE");
            line(b"D- STAYED HIGH");
        }
        None if mode == QcMode::Pd => {
            line(b"QUICK CHARGE");
            line(b"NOT SCANNED YET");
            line(b"HOLD OK: FULL SCAN");
            line(b"(QC, PD AND EPR)");
        }
        None => {
            line(b"QUICK CHARGE");
            line(b"NOT TESTED YET");
            line(b"HOLD OK: TEST QC");
        }
    }
    rows
}

/// Centred notice for a selector page with nothing to select.
fn notice<D: DrawTarget<Color = Rgb565>>(target: &mut D, title: &str, detail: &str, color: Rgb565) {
    text(
        target,
        &BOLD,
        title,
        Point::new(WIDTH as i32 / 2, 66),
        HorizontalAlignment::Center,
        color,
    );
    text(
        target,
        &PLAIN,
        detail,
        Point::new(WIDTH as i32 / 2, 86),
        HorizontalAlignment::Center,
        MUTED,
    );
}

/// Fixed profile rows: top, pitch and the most shown at once.
const PROFILE_TOP: i32 = 25;
const PROFILE_PITCH: i32 = 19;
const PROFILE_ROWS: usize = 5;

/// Detail columns of a selector row, from the left edge.
const ROW_DETAILS_X: [i32; 2] = [78, 130];
/// A selector row's status pill.
const ROW_PILL_X: i32 = 178;
const ROW_PILL: Size = Size::new(52, 12);

/// Selector row `row`: `title` in `color`, two details, framed while
/// `selected`, and a status pill on the right.
fn selector_row<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    row: usize,
    title: &str,
    color: Rgb565,
    details: [&str; 2],
    selected: bool,
    pill: Option<(&str, Rgb565)>,
) {
    let top = PROFILE_TOP + row as i32 * PROFILE_PITCH;
    if selected {
        let area = Rectangle::new(Point::new(4, top), Size::new(232, PROFILE_PITCH as u32 - 2));
        card(target, area, RULE, Some(color));
    }
    let baseline = top + 13;
    text(
        target,
        &BOLD,
        title,
        Point::new(LEFT + 4, baseline),
        HorizontalAlignment::Left,
        color,
    );
    for (detail, x) in details.into_iter().zip(ROW_DETAILS_X) {
        text(
            target,
            &PLAIN,
            detail,
            Point::new(x, baseline),
            HorizontalAlignment::Left,
            if selected { WHITE } else { MUTED },
        );
    }
    if let Some((label, color)) = pill {
        let pill = Rectangle::new(Point::new(ROW_PILL_X, top + 3), ROW_PILL);
        fill(
            target,
            RoundedRectangle::with_equal_corners(pill, Size::new(6, 6)),
            color,
        );
        text(
            target,
            &PLAIN,
            label,
            Point::new(pill.center().x, baseline),
            HorizontalAlignment::Center,
            BLACK,
        );
    }
}

/// The fixed PD profile selector. While `editing`, the previewed profile is
/// framed; the contract in force is marked ACTIVE. Before the sink session
/// has the charger's offer, the page only shows what is known.
pub fn draw_fixed<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    offer: &Offer,
    editing: bool,
) {
    draw_header(target, header);
    if offer.quick_charge {
        qc_mode(target, header);
        return;
    }
    let profiles = offer.fixed();
    if profiles.is_empty() {
        notice(target, "NO PD CHARGER", "plug in a PD source", GREEN);
        return;
    }
    let preview = offer
        .preview
        .filter(|preview| editing && !preview.pps)
        .and_then(|preview| profiles.iter().position(|p| p.mv == preview.mv));
    // Keep the previewed row in view when there are more rows than fit.
    let first = preview.map_or(0, |index| {
        index
            .saturating_sub(PROFILE_ROWS - 1)
            .min(profiles.len().saturating_sub(PROFILE_ROWS))
    });
    for (row, (index, profile)) in profiles
        .iter()
        .enumerate()
        .skip(first)
        .take(PROFILE_ROWS)
        .enumerate()
    {
        let amps = Label::new().hundredths(u32::from(profile.ma)).push(b"A");
        let watts = u32::from(profile.mv) * u32::from(profile.ma) / 1_000_000;
        let watts = Label::new().fixed(watts, 0).push(b"W");
        let active = offer
            .contract
            .is_some_and(|contract| !contract.pps && contract.mv == profile.mv);
        selector_row(
            target,
            row,
            Label::new().volts(profile.mv).as_str(),
            level_color(profile.mv),
            [amps.as_str(), watts.as_str()],
            preview == Some(index),
            active.then_some(("ACTIVE", GREEN)),
        );
    }
    hint(
        target,
        header,
        if !offer.session {
            VIEW_ONLY
        } else if editing {
            "< > choose  OK request  BACK cancel"
        } else {
            "OK select a profile    < > tabs"
        },
    );
}

/// Hint on the selector pages before the sink session has an offer.
const VIEW_ONLY: &str = "waiting for the charger    < > tabs";

/// PPS value boxes.
const PPS_VOLTS: Rectangle = Rectangle::new(Point::new(4, 52), Size::new(114, 44));
const PPS_AMPS: Rectangle = Rectangle::new(Point::new(122, 52), Size::new(114, 44));
/// A PPS box's value baseline, from the box's top-left corner.
const PPS_VALUE: Point = Point::new(8, 37);
/// Baseline of the line under the PPS boxes: the output and the state.
const PPS_STATUS_BASELINE: i32 = 112;
/// The range lines above the PPS boxes: the first baseline and the pitch.
const RANGE_BASELINE: i32 = 34;
const RANGE_PITCH: i32 = 12;

/// The PPS selector: the advertised ranges, the voltage and current boxes
/// (the one Left/Right change is framed), and the live reading.
pub fn draw_pps<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    offer: &Offer,
    focus: Option<PpsFocus>,
    live: Option<Reading>,
) {
    draw_header(target, header);
    if offer.quick_charge {
        qc_mode(target, header);
        return;
    }
    let ranges = offer.pps();
    if ranges.is_empty() {
        notice(
            target,
            "NO PPS",
            "this charger offers no PPS range",
            MAGENTA,
        );
        return;
    }
    for (index, range) in ranges.iter().take(2).enumerate() {
        let line = Label::new()
            .hundredths(u32::from(range.min_mv))
            .push(b"-")
            .hundredths(u32::from(range.max_mv))
            .push(b"V  up to ")
            .hundredths(u32::from(range.max_ma))
            .push(b"A");
        text(
            target,
            &PLAIN,
            line.as_str(),
            Point::new(LEFT, RANGE_BASELINE + RANGE_PITCH * index as i32),
            HorizontalAlignment::Left,
            MAGENTA,
        );
    }

    let set = offer.preview.filter(|preview| preview.pps);
    let (volts, amps) = match set {
        Some(set) => (
            Label::new().hundredths(u32::from(set.mv)),
            Label::new().hundredths(u32::from(set.ma)),
        ),
        None => (Label::new().push(b"--.--"), Label::new().push(b"-.--")),
    };
    let boxes = [
        (
            PPS_VOLTS,
            "SET VOLTAGE",
            volts,
            "V",
            PpsFocus::Voltage,
            YELLOW,
        ),
        (
            PPS_AMPS,
            "CURRENT LIMIT",
            amps,
            "A",
            PpsFocus::Current,
            CYAN,
        ),
    ];
    for (area, label, value, unit, field, color) in &boxes {
        let focused = focus == Some(*field);
        let (tint, border) = if focused {
            (RULE, Some(*color))
        } else {
            (BLACK, Some(RULE))
        };
        card(target, *area, tint, border);
        card_label(target, *area, label, *color);
        value_with_unit(
            target,
            &READING,
            value,
            unit,
            area.top_left + PPS_VALUE,
            if focused { WHITE } else { *color },
        );
        if focused {
            arrows(target, *area, *color);
        }
    }

    let applied = set.is_some() && offer.contract == set;
    let state = match (set, applied) {
        (_, true) => ("ACTIVE", GREEN),
        (Some(_), false) => ("NOT APPLIED", ORANGE),
        (None, false) => ("", MUTED),
    };
    if let Some(live) = live {
        let line = Label::new()
            .push(b"OUT ")
            .hundredths(live.mv)
            .push(b"V ")
            .hundredths(live.ma.unsigned_abs())
            .push(b"A");
        text(
            target,
            &PLAIN,
            line.as_str(),
            Point::new(LEFT, PPS_STATUS_BASELINE),
            HorizontalAlignment::Left,
            WHITE,
        );
    }
    text(
        target,
        &PLAIN,
        state.0,
        Point::new(RIGHT, PPS_STATUS_BASELINE),
        HorizontalAlignment::Right,
        state.1,
    );
    hint(
        target,
        header,
        match focus {
            _ if !offer.session => VIEW_ONLY,
            Some(PpsFocus::Voltage) => "< > volts    OK next: current",
            Some(PpsFocus::Current) => "< > amps    OK request",
            None => "OK adjust PPS    < > tabs",
        },
    );
}

/// Up and down arrows at a box's right edge: Right raises it, Left lowers it.
fn arrows<D: DrawTarget<Color = Rgb565>>(target: &mut D, area: Rectangle, color: Rgb565) {
    let x = area.top_left.x + area.size.width as i32 - 10;
    let y = area.top_left.y + area.size.height as i32 / 2 + 6;
    fill(
        target,
        Triangle::new(
            Point::new(x, y - 15),
            Point::new(x - 5, y - 10),
            Point::new(x + 5, y - 10),
        ),
        color,
    );
    fill(
        target,
        Triangle::new(
            Point::new(x, y + 4),
            Point::new(x - 5, y - 1),
            Point::new(x + 5, y - 1),
        ),
        color,
    );
}

/// Settings rows: first baseline and pitch.
const SETTINGS_BASELINE: i32 = 44;
const SETTINGS_PITCH: i32 = 27;

/// The settings page: the cursor row is highlighted, and the value being
/// changed is framed.
pub fn draw_settings<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    settings: Settings,
    focus: SettingsFocus,
) {
    draw_header(target, header);
    for (index, item) in Item::ALL.into_iter().enumerate() {
        let baseline = SETTINGS_BASELINE + index as i32 * SETTINGS_PITCH;
        let selected = item == focus.row();
        if selected {
            card(
                target,
                Rectangle::new(Point::new(4, baseline - 15), Size::new(232, 22)),
                RULE,
                None,
            );
            text(
                target,
                &ROW,
                ">",
                Point::new(LEFT, baseline),
                HorizontalAlignment::Left,
                YELLOW,
            );
        }
        let name = match item {
            Item::Brightness => "BRIGHTNESS",
            Item::Rotation => "ROTATION",
            Item::ResetPeaks => "RESET MAX",
        };
        text(
            target,
            &ROW,
            name,
            Point::new(LEFT + 14, baseline),
            HorizontalAlignment::Left,
            if selected { WHITE } else { MUTED },
        );
        match item {
            Item::Brightness => brightness_bars(
                target,
                baseline,
                settings.brightness,
                focus == SettingsFocus::Brightness,
            ),
            Item::Rotation => {
                let editing = focus == SettingsFocus::Rotation;
                if editing {
                    card(
                        target,
                        Rectangle::new(Point::new(154, baseline - 16), Size::new(78, 24)),
                        BLACK,
                        Some(YELLOW),
                    );
                }
                let value = match settings.rotation {
                    Rotation::Auto => "AUTO",
                    Rotation::Normal => "NORMAL",
                    Rotation::Flipped => "FLIPPED",
                };
                text(
                    target,
                    &ROW,
                    value,
                    Point::new(193, baseline),
                    HorizontalAlignment::Center,
                    if selected { YELLOW } else { MUTED },
                );
            }
            Item::ResetPeaks => {
                text(
                    target,
                    &ROW,
                    "OK",
                    Point::new(RIGHT - 4, baseline),
                    HorizontalAlignment::Right,
                    if selected { MAGENTA } else { MUTED },
                );
            }
        }
    }
    hint(
        target,
        header,
        match focus {
            SettingsFocus::Brightness => "< > dimmer/brighter    OK done",
            SettingsFocus::Rotation => "< > auto/normal/flipped  OK done",
            _ => "< > choose  OK change  BACK exit",
        },
    );
}

/// One bar per brightness level, rising left to right, lit up to `level`;
/// framed while being changed.
fn brightness_bars<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    baseline: i32,
    level: u8,
    editing: bool,
) {
    let left = 160;
    if editing {
        card(
            target,
            Rectangle::new(Point::new(left - 6, baseline - 16), Size::new(78, 24)),
            BLACK,
            Some(YELLOW),
        );
    }
    for index in 0..BRIGHTNESS_PERCENT.len() {
        let height = 4 + 3 * index as u32;
        let color = if index <= usize::from(level) {
            YELLOW
        } else {
            RULE
        };
        fill(
            target,
            Rectangle::new(
                Point::new(left + 13 * index as i32, baseline + 2 - height as i32),
                Size::new(9, height),
            ),
            color,
        );
    }
}

/// One section of a detail list: up to eight rows of 24 characters; row 0
/// is the section heading, and blank rows are left out.
pub type Lines = [[u8; 24]; 8];

/// How a detail list was laid out, for the input and PD tasks.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ListView {
    /// Last row the list can start at.
    pub end: u16,
    /// Section of the row at the top of the screen.
    pub section: u16,
}

/// Detail list rows on screen, their pitch and the first baseline (below
/// the header rule).
const LIST_ROWS: u16 = 8;
const LIST_PITCH: i32 = 12;
const LIST_BASELINE: i32 = 34;
/// Scrollbar track along the right edge.
const SCROLL_TRACK: Rectangle = Rectangle::new(Point::new(RIGHT + 1, 25), Size::new(3, 94));

/// A detail page in `page`'s colour: the non-blank rows of `sections` from
/// row `top`, with a scrollbar lit while `scrolling`.
pub fn draw_detail<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    page: Detail,
    sections: impl IntoIterator<Item = Lines>,
    top: u16,
    scrolling: bool,
) -> ListView {
    let (_, accent) = tab_style(Tab::detail(page), false);
    draw_header(target, header);
    let mut rows = 0u16;
    let mut view = ListView::default();
    for (section, lines) in sections.into_iter().enumerate() {
        for (index, line) in lines.iter().enumerate() {
            let mut printable = [b' '; 24];
            for (cell, &byte) in printable.iter_mut().zip(line) {
                if byte.is_ascii_graphic() {
                    *cell = byte;
                }
            }
            let content = core::str::from_utf8(&printable).unwrap_or("").trim_end();
            if content.is_empty() {
                continue;
            }
            if rows <= top {
                view.section = section as u16;
            }
            if let Some(slot) = rows.checked_sub(top).filter(|slot| *slot < LIST_ROWS) {
                let (x, color) = if index == 0 {
                    (LEFT, accent)
                } else {
                    (LEFT + 6, WHITE)
                };
                text(
                    target,
                    &PLAIN,
                    content,
                    Point::new(x, LIST_BASELINE + LIST_PITCH * i32::from(slot)),
                    HorizontalAlignment::Left,
                    color,
                );
            }
            rows = rows.saturating_add(1);
        }
    }
    view.end = rows.saturating_sub(LIST_ROWS);
    if view.end > 0 {
        fill(target, SCROLL_TRACK, RULE);
        let track = SCROLL_TRACK.size.height;
        let thumb = (track * u32::from(LIST_ROWS) / u32::from(rows)).max(8);
        let offset = (track - thumb) * u32::from(top.min(view.end)) / u32::from(view.end);
        fill(
            target,
            Rectangle::new(
                SCROLL_TRACK.top_left + Point::new(0, offset as i32),
                Size::new(SCROLL_TRACK.size.width, thumb),
            ),
            if scrolling { accent } else { MUTED },
        );
    }
    hint(
        target,
        header,
        match (scrolling, view.end > 0) {
            (true, _) => "< > scroll    OK done",
            (false, true) => "OK scroll    < > tabs",
            (false, false) => "< > tabs    BACK groups",
        },
    );
    view
}
