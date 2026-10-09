//! Graphical pages: overview, PD profile and PPS selectors, settings and
//! the scrolling detail lists, drawn with `embedded-graphics` primitives and `u8g2-fonts` text on any
//! RGB565 target. The renderer gathers the values; nothing here touches
//! hardware.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{
    Circle, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle, StyledDrawable,
    Triangle,
};
use firmware_core::buttons::{Detail, PpsFocus, SettingsFocus};
use firmware_core::settings::{BRIGHTNESS_PERCENT, Item, Rotation, Settings};
use u8g2_fonts::FontRenderer;
use u8g2_fonts::fonts;
use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};

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

/// Page header: title on the left, one-based page position (when the page
/// is in a cycle) on the right.
pub struct Header<'a> {
    pub title: &'a str,
    pub position: Option<(u8, u8)>,
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

/// Title in the page's accent colour, page counter and rule.
fn draw_header<D: DrawTarget<Color = Rgb565>>(target: &mut D, header: &Header<'_>, accent: Rgb565) {
    fill(
        target,
        Rectangle::new(Point::new(LEFT - 2, 5), Size::new(3, 13)),
        accent,
    );
    text(
        target,
        &BOLD,
        header.title,
        Point::new(LEFT + 5, TITLE_BASELINE),
        HorizontalAlignment::Left,
        accent,
    );
    if let Some((index, count)) = header.position {
        let counter = Label::new().two_digits(index).push(b"/").two_digits(count);
        text(
            target,
            &PLAIN,
            counter.as_str(),
            Point::new(RIGHT, TITLE_BASELINE),
            HorizontalAlignment::Right,
            MUTED,
        );
    }
    rule(target, HEADER_RULE_Y);
}

fn hint<D: DrawTarget<Color = Rgb565>>(target: &mut D, content: &str) {
    text(
        target,
        &PLAIN,
        content,
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
/// Flow arrow length and head half-height, in pixels, and its centre's
/// offset from the card's top-right corner.
const FLOW_LENGTH: i32 = 16;
const FLOW_HEAD: i32 = 4;
const FLOW_CENTRE: Point = Point::new(-16, 7);
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
    draw_header(target, header, CYAN);
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
        if *area == AMPS_CARD
            && let Some(reading) = meter.reading
        {
            flow_arrow(target, *area, reading.ma, *color);
        }
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
    fill(
        target,
        Circle::with_center(Point::new(RIGHT - 1, HINT_BASELINE - 5), 6),
        meter.alive,
    );
}

/// Which way the current flows between the meter's ends, as the meter sits:
/// plug on the left, receptacle on the right. Positive current runs from the
/// receptacle to the plug (charger on the receptacle), so the arrow points
/// left; negative current points it right.
fn flow_arrow<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    area: Rectangle,
    ma: i32,
    color: Rgb565,
) {
    if ma.unsigned_abs() < FLOW_MIN_MA {
        return;
    }
    let centre =
        Point::new(area.top_left.x + area.size.width as i32, area.top_left.y) + FLOW_CENTRE;
    // +1 points right, -1 points left.
    let toward = if ma < 0 { 1 } else { -1 };
    let tip = centre + Point::new(toward * FLOW_LENGTH / 2, 0);
    let base = tip - Point::new(toward * 2 * FLOW_HEAD, 0);
    let tail = centre - Point::new(toward * FLOW_LENGTH / 2, 0);
    fill(
        target,
        Rectangle::with_corners(
            Point::new(tail.x.min(base.x), centre.y - 1),
            Point::new(tail.x.max(base.x), centre.y + 1),
        ),
        color,
    );
    fill(
        target,
        Triangle::new(
            tip,
            base - Point::new(0, FLOW_HEAD),
            base + Point::new(0, FLOW_HEAD),
        ),
        color,
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

/// The fixed PD profile selector. While `editing`, the previewed profile is
/// framed; the contract in force is marked ACTIVE. Before the sink session
/// has the charger's offer, the page only shows what is known.
pub fn draw_fixed<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    offer: &Offer,
    editing: bool,
) {
    draw_header(target, header, GREEN);
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
        let top = PROFILE_TOP + row as i32 * PROFILE_PITCH;
        let color = level_color(profile.mv);
        let area = Rectangle::new(Point::new(4, top), Size::new(232, PROFILE_PITCH as u32 - 2));
        let selected = preview == Some(index);
        if selected {
            card(target, area, RULE, Some(color));
        }
        let baseline = top + 13;
        text(
            target,
            &BOLD,
            Label::new().volts(profile.mv).as_str(),
            Point::new(LEFT + 4, baseline),
            HorizontalAlignment::Left,
            color,
        );
        text(
            target,
            &PLAIN,
            Label::new()
                .hundredths(u32::from(profile.ma))
                .push(b"A")
                .as_str(),
            Point::new(78, baseline),
            HorizontalAlignment::Left,
            if selected { WHITE } else { MUTED },
        );
        let watts = u32::from(profile.mv) * u32::from(profile.ma) / 1_000_000;
        text(
            target,
            &PLAIN,
            Label::new().fixed(watts, 0).push(b"W").as_str(),
            Point::new(130, baseline),
            HorizontalAlignment::Left,
            if selected { WHITE } else { MUTED },
        );
        if offer
            .contract
            .is_some_and(|contract| !contract.pps && contract.mv == profile.mv)
        {
            let pill = Rectangle::new(Point::new(178, top + 3), Size::new(52, 12));
            fill(
                target,
                RoundedRectangle::with_equal_corners(pill, Size::new(6, 6)),
                GREEN,
            );
            text(
                target,
                &PLAIN,
                "ACTIVE",
                Point::new(pill.center().x, top + 13),
                HorizontalAlignment::Center,
                BLACK,
            );
        }
    }
    hint(
        target,
        if !offer.session {
            VIEW_ONLY
        } else if editing {
            "< > choose  OK request  BACK cancel"
        } else {
            "OK select a profile    < > pages"
        },
    );
}

/// Hint on the selector pages before the sink session has an offer.
const VIEW_ONLY: &str = "waiting for the charger    < > pages";

/// PPS value boxes.
const PPS_VOLTS: Rectangle = Rectangle::new(Point::new(4, 52), Size::new(114, 44));
const PPS_AMPS: Rectangle = Rectangle::new(Point::new(122, 52), Size::new(114, 44));

/// The PPS selector: the advertised ranges, the voltage and current boxes
/// (the one Left/Right change is framed), and the live reading.
pub fn draw_pps<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    offer: &Offer,
    focus: PpsFocus,
    live: Option<Reading>,
) {
    draw_header(target, header, MAGENTA);
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
            Point::new(LEFT, 34 + 12 * index as i32),
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
        let focused = focus == *field;
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
            area.top_left + Point::new(8, 37),
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
            Point::new(LEFT, 112),
            HorizontalAlignment::Left,
            WHITE,
        );
    }
    text(
        target,
        &PLAIN,
        state.0,
        Point::new(RIGHT, 112),
        HorizontalAlignment::Right,
        state.1,
    );
    hint(
        target,
        match focus {
            _ if !offer.session => VIEW_ONLY,
            PpsFocus::Voltage => "< > volts    OK next: current",
            PpsFocus::Current => "< > amps    OK request",
            _ => "OK adjust PPS    < > pages",
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
    draw_header(target, header, YELLOW);
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
    let accent = match page {
        Detail::Pd => GREEN,
        Detail::Charger => ORANGE,
        Detail::Cable => CYAN,
        Detail::System => YELLOW,
    };
    draw_header(target, header, accent);
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
        match (scrolling, view.end > 0) {
            (true, _) => "< > scroll    OK done",
            (false, true) => "OK scroll    < > pages",
            (false, false) => "< > pages    BACK home",
        },
    );
    view
}
