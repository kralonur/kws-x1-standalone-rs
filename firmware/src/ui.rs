//! Graphical pages: overview, line voltages, PD profile, PPS and AVS selectors,
//! Quick Charge trigger, settings and the scrolling detail lists, drawn with
//! `embedded-graphics` primitives and `u8g2-fonts` text on any RGB565 target.
//! The renderer gathers the values; nothing here touches hardware.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{
    Circle, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle,
    StyledDrawable, Triangle,
};
use firmware_core::buttons::{
    Detail, GraphFocus, Level, PpsFocus, Qc2Choice, QcOffer, RESET_STEPS, SettingsFocus, Tab,
};
use firmware_core::dcp::Dcp;
use firmware_core::energy::Totals;
use firmware_core::power_history::{GraphTrace, GraphWindow, History, Sample};
use firmware_core::settings::{
    BRIGHTNESS_PERCENT, EnergyMemory, Item, QcFindings, Rotation, Settings,
};
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
/// Outlined cards' labels: Nokia Small, proportional, so its digits are
/// narrow.
const SMALL: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_NokiaSmallPlain_tr>().with_ignore_unknown_chars(true);

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
/// The reset hold's sweep, a neutral fill lighter than every tint, so each
/// accent stays readable over it.
const SWEEP: Rgb565 = Rgb565::new(10, 20, 10);

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

/// The charging protocol in force, as far as the meter can tell.
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
    /// SPR adjustable voltage supply contract.
    Avs { mv: u16, ma: u16 },
    /// Quick Charge 2.0 at this requested output, in mV.
    Qc2 { mv: u16 },
    /// Quick Charge 3.0 continuous mode; `mv` is the output the steps set,
    /// when this meter sends them.
    Qc3 { mv: Option<u16> },
    /// A charging port telling its current by fixed D+/D− voltages.
    Dcp(Dcp),
    /// A BC1.2 dedicated charging port (D+ shorted to D−), as the Quick
    /// Charge test found it.
    Bc12,
}

/// What kind of supply a level is.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Kind {
    #[default]
    Fixed,
    Pps,
    Avs,
}

/// A supply level, in mV and mA.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Supply {
    pub mv: u16,
    pub ma: u16,
    pub kind: Kind,
}

/// One PPS range the charger offers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PpsRange {
    pub min_mv: u16,
    pub max_mv: u16,
    pub max_ma: u16,
}

/// One SPR AVS range the charger offers: its current up to 15 V, and above
/// 15 V when the range reaches 20 V.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AvsRange {
    pub min_mv: u16,
    pub max_mv: u16,
    pub ma_15v: u16,
    pub ma_20v: u16,
    /// EPR AVS power limit; SPR AVS instead uses the two current fields.
    pub pdp_w: Option<u16>,
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
    pub pd_notice: Option<&'static str>,
    /// This boot keeps PD silent for Quick Charge, so no offer is heard.
    pub quick_charge: bool,
    fixed: [Supply; MAX_OBJECTS],
    fixed_count: usize,
    pps: [PpsRange; MAX_OBJECTS],
    pps_count: usize,
    avs: [AvsRange; MAX_OBJECTS],
    avs_count: usize,
    /// The profile Left/Right previewed; nothing is sent before OK.
    pub preview: Option<Supply>,
    /// The contract in force.
    pub contract: Option<Supply>,
}

impl Offer {
    pub fn push_fixed(&mut self, mv: u16, ma: u16) {
        if let Some(slot) = self.fixed.get_mut(self.fixed_count) {
            *slot = Supply {
                mv,
                ma,
                kind: Kind::Fixed,
            };
            self.fixed_count += 1;
        }
    }

    pub fn push_pps(&mut self, range: PpsRange) {
        if let Some(slot) = self.pps.get_mut(self.pps_count) {
            *slot = range;
            self.pps_count += 1;
        }
    }

    pub fn push_avs(&mut self, range: AvsRange) {
        if let Some(slot) = self.avs.get_mut(self.avs_count) {
            *slot = range;
            self.avs_count += 1;
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

    fn avs(&self) -> &[AvsRange] {
        &self.avs[..self.avs_count]
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
        Tab::Graph => ("GRAPH", CYAN),
        Tab::Lines => ("LINES", YELLOW),
        Tab::Energy => ("ENERGY", GREEN),
        Tab::Fixed => ("PD PROFILES", GREEN),
        Tab::Pps => ("PPS", MAGENTA),
        Tab::Avs => ("AVS", YELLOW),
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
    /// Heartbeat pulses since boot, which step the flow chevrons' shimmer.
    pub pulses: u32,
    /// The reset hold's progress over the peaks, out of [`RESET_STEPS`].
    pub reset_hold: u8,
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
    fn fixed(self, value: u32, places: u32) -> Self {
        self.fixed_u64(u64::from(value), places)
    }

    fn fixed_u64(self, mut value: u64, places: u32) -> Self {
        // Twenty decimal digits for u64, plus a decimal point and leading zero.
        let mut digits = [0u8; 22];
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

/// The reset hold's sweep over `area`: `steps` of [`RESET_STEPS`] filled
/// from the left, over the card's fill and under its text.
fn sweep<D: DrawTarget<Color = Rgb565>>(target: &mut D, area: Rectangle, steps: u8) {
    let width = area.size.width * u32::from(steps) / u32::from(RESET_STEPS);
    fill(
        target,
        RoundedRectangle::with_equal_corners(
            Rectangle::new(area.top_left, Size::new(width, area.size.height)),
            CORNER,
        ),
        SWEEP,
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
            "< > groups  OK tabs"
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
        Protocol::Unknown => ("NONE", MUTED),
        Protocol::TypeC { ma: 0 } => ("USB", WHITE),
        Protocol::TypeC { .. } => ("TYPE-C", WHITE),
        Protocol::PdOffer | Protocol::Pd { revision: None, .. } => ("PD", GREEN),
        Protocol::Pd {
            revision: Some(3), ..
        } => ("PD 3.0", GREEN),
        Protocol::Pd { .. } => ("PD 2.0", CYAN),
        Protocol::Pps { .. } => ("PPS", MAGENTA),
        Protocol::Avs { .. } => ("AVS", YELLOW),
        Protocol::Qc2 { .. } => ("QC 2.0", ORANGE),
        Protocol::Qc3 { .. } => ("QC 3.0", ORANGE),
        Protocol::Dcp(Dcp::Bias1V2) => ("1.2V DCP", YELLOW),
        Protocol::Dcp(_) => ("DIVIDER", YELLOW),
        Protocol::Bc12 => ("BC1.2", WHITE),
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

/// The least padding inside a card, between its edge and its content, in
/// pixels. Cards fill the page from under the header rule to 2 px above the
/// hint row's ink, so a page with fewer cards gives each more room, its
/// content centred.
const PAD: i32 = 5;
const CONTENT_BOTTOM: i32 = HINT_BASELINE - PLAIN_INK - 3;
/// Rows the fonts ink above their baseline, measured by rendering them:
/// `PLAIN` capitals and digits, `SMALL`, `READING` and `POWER` digits.
const PLAIN_INK: i32 = 8;
const SMALL_INK: i32 = 7;
const READING_INK: i32 = 19;
const POWER_INK: i32 = 27;

/// Outlined cards, on the overview and lines pages: an outline in
/// the card's colour on the bare background, with its label in `SMALL`
/// sitting on the top edge, the edge running through the label's middle
/// and cut away round it. The first label's ink starts just under the
/// header rule, the next label's 2 clear rows under the cards above.
const OUTLINED_GAP: i32 = 2;
/// Rows of label ink above and below a card's top edge.
const LABEL_ABOVE_EDGE: i32 = SMALL_INK / 2;
const LABEL_BELOW_EDGE: i32 = SMALL_INK - LABEL_ABOVE_EDGE;
const OUTLINED_TOP: i32 = HEADER_RULE_Y + 2 + LABEL_ABOVE_EDGE;
/// A label's left edge, from its card's.
const OUTLINED_LABEL_LEFT: i32 = PAD + 2;
/// Rows from one outlined card's bottom to the next one's top edge.
const OUTLINED_SPACING: i32 = OUTLINED_GAP + LABEL_ABOVE_EDGE;
/// The height of `rows` outlined cards filling the page, and the top edge
/// of the card `row` rows down.
const fn outlined_height(rows: i32) -> u32 {
    ((CONTENT_BOTTOM + 1 - OUTLINED_TOP - (rows - 1) * OUTLINED_SPACING) / rows) as u32
}
const fn outlined_top(row: i32, height: u32) -> i32 {
    OUTLINED_TOP + row * (height as i32 + OUTLINED_SPACING)
}
/// The baseline, under an outlined card's top edge, of an `ink` row value
/// centred between the label's lower half and the bottom edge.
const fn outlined_value(height: u32, ink: i32) -> i32 {
    let top = LABEL_BELOW_EDGE + 1;
    top + (height as i32 - 1 - top - ink) / 2 + ink
}
/// Card label baseline and value baseline, from the card top.
const CARD_LABEL: i32 = 11;
const CARD_VALUE: i32 = 35;
/// Overview cards: voltage and current on top, power and peaks below, the
/// protocol strip at the bottom. A value starts 2 px under the label's
/// lower half and pads 3 px above the bottom edge.
const OVERVIEW_PAD: i32 = 3;
/// A value's baseline under its card's top edge, for an `ink` row font.
const fn overview_value(ink: i32) -> i32 {
    LABEL_BELOW_EDGE + 2 + ink
}
const fn overview_height(ink: i32) -> u32 {
    (overview_value(ink) + OVERVIEW_PAD + 1) as u32
}
const OVERVIEW_VALUE: i32 = overview_value(READING_INK);
const OVERVIEW_HEIGHT: u32 = overview_height(READING_INK);
const VOLTS_CARD: Rectangle =
    Rectangle::new(Point::new(4, OUTLINED_TOP), Size::new(114, OVERVIEW_HEIGHT));
const AMPS_CARD: Rectangle = Rectangle::new(
    Point::new(122, OUTLINED_TOP),
    Size::new(114, OVERVIEW_HEIGHT),
);
const POWER_TOP: i32 = outlined_top(1, OVERVIEW_HEIGHT);
const POWER_VALUE: i32 = overview_value(POWER_INK);
const POWER_HEIGHT: u32 = overview_height(POWER_INK);
const POWER_CARD: Rectangle =
    Rectangle::new(Point::new(4, POWER_TOP), Size::new(150, POWER_HEIGHT));
const PEAKS_CARD: Rectangle =
    Rectangle::new(Point::new(158, POWER_TOP), Size::new(78, POWER_HEIGHT));
/// The peaks' pitch, and their first baseline with the three rows centred
/// in the card (they sit right, clear of the label).
const PEAKS_PITCH: i32 = 9;
const PEAKS_FIRST: i32 = (POWER_HEIGHT as i32 - PLAIN_INK - 2 * PEAKS_PITCH) / 2 + PLAIN_INK;
/// The protocol strip, an outlined card along the bottom in the protocol's
/// colour: its `PLAIN` line, the protocol then its status, sits like a
/// value, padding 3 px above the bottom edge.
const STRIP_VALUE: i32 = LABEL_BELOW_EDGE + 2 + PLAIN_INK;
const STRIP_HEIGHT: u32 = (STRIP_VALUE + OVERVIEW_PAD + 1) as u32;
const PROTOCOL_STRIP: Rectangle = Rectangle::new(
    Point::new(4, CONTENT_BOTTOM + 1 - STRIP_HEIGHT as i32),
    Size::new(232, STRIP_HEIGHT),
);
const _: () =
    assert!(POWER_TOP + POWER_HEIGHT as i32 + OUTLINED_SPACING <= PROTOCOL_STRIP.top_left.y);
/// Below this current magnitude no flow arrow is drawn, in mA: a chosen
/// margin over the shunt's zero offset (40 INA226 counts), not a measured one.
const FLOW_MIN_MA: u32 = 20;
/// The strip's status baseline, and its chevrons' (`BOLD`, 10 rows of
/// ink) one row lower, centred on it.
const STRIP_BASELINE: i32 = PROTOCOL_STRIP.top_left.y + STRIP_VALUE;
const STRIP_CHEVRON_BASELINE: i32 = STRIP_BASELINE + 1;
/// Heartbeat dot diameter, and its centre at the strip's right padding.
const DOT: u32 = 6;
const STRIP_DOT: Point = Point::new(
    PROTOCOL_STRIP.top_left.x + PROTOCOL_STRIP.size.width as i32 - PAD - DOT as i32 / 2,
    STRIP_BASELINE - PLAIN_INK / 2,
);
/// Right edge of the flow chevrons, clear of the heartbeat dot, and how
/// many there are.
const FLOW_RIGHT: i32 = STRIP_DOT.x - DOT as i32;
const FLOW_CHEVRONS: u32 = 3;

/// An outlined card: an outline in `color` with `label` on its top edge,
/// the edge cut away round the label. `steps` of the reset sweep fill the
/// inside under the outline.
fn outlined_card<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    area: Rectangle,
    label: &str,
    color: Rgb565,
    steps: u8,
) {
    sweep(target, area, steps);
    labelled_outline(target, area, label, color, color);
}

/// A 1 px rounded outline round `area`, `corner` the corner radius.
fn rounded_outline<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    area: Rectangle,
    corner: Size,
    color: Rgb565,
) {
    RoundedRectangle::with_equal_corners(area, corner)
        .draw_styled(&PrimitiveStyle::with_stroke(color, 1), target)
        .ok();
}

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

/// An outline in `outline` with `label` in `color` on its top edge, the
/// edge cut away round the label.
fn labelled_outline<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    area: Rectangle,
    label: &str,
    color: Rgb565,
    outline: Rgb565,
) {
    rounded_outline(target, area, CORNER, outline);
    let left = area.top_left.x + OUTLINED_LABEL_LEFT;
    fill(
        target,
        Rectangle::new(
            Point::new(left - 2, area.top_left.y),
            Size::new((width(&SMALL, label) + 4) as u32, 1),
        ),
        BLACK,
    );
    text(
        target,
        &SMALL,
        label,
        Point::new(left, area.top_left.y + LABEL_BELOW_EDGE),
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
        (VOLTS_CARD, YELLOW, "VOLTAGE", volts, "V"),
        (AMPS_CARD, CYAN, "CURRENT", amps, "A"),
    ];
    for (area, color, label, value, unit) in &cards {
        outlined_card(target, *area, label, *color, 0);
        value_with_unit(
            target,
            &READING,
            value,
            unit,
            area.top_left + Point::new(PAD, OVERVIEW_VALUE),
            *color,
        );
    }
    outlined_card(target, POWER_CARD, "POWER", GREEN, 0);
    value_with_unit(
        target,
        &POWER,
        &power,
        "W",
        POWER_CARD.top_left + Point::new(PAD, POWER_VALUE),
        GREEN,
    );

    outlined_card(target, PEAKS_CARD, "MAX", MAGENTA, meter.reset_hold);
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
                PEAKS_CARD.top_left.x + PEAKS_CARD.size.width as i32 - PAD,
                PEAKS_CARD.top_left.y + PEAKS_FIRST + PEAKS_PITCH * index as i32,
            ),
            HorizontalAlignment::Right,
            *color,
        );
    }

    let (badge, color) = protocol_badge(meter.protocol);
    outlined_card(target, PROTOCOL_STRIP, "PROTOCOL", color, 0);
    let badge_end = text(
        target,
        &PLAIN,
        badge,
        Point::new(PROTOCOL_STRIP.top_left.x + PAD, STRIP_BASELINE),
        HorizontalAlignment::Left,
        color,
    );
    text(
        target,
        &PLAIN,
        status(meter.protocol, meter.cc).as_str(),
        Point::new(badge_end + PAD, STRIP_BASELINE),
        HorizontalAlignment::Left,
        WHITE,
    );
    if let Some(reading) = meter.reading {
        flow(target, reading.ma, meter.flipped, meter.pulses);
    }
    fill(target, Circle::with_center(STRIP_DOT, DOT), meter.alive);
    hint(target, header, "< > tabs  hold OK reset");
}

/// Space between the trace and window labels, and the full-scale label's
/// baseline below the plot's top line, in pixels.
const GRAPH_LABEL_GAP: i32 = 6;
const GRAPH_SCALE_LABEL_BELOW_TOP: i32 = 10;
/// Power graph: the now and max rows' baselines above the plot, the right
/// edges of their volts and amps columns, and the plot.
const GRAPH_NOW_BASELINE: i32 = 34;
const GRAPH_MAX_BASELINE: i32 = 46;
const GRAPH_VOLTS_RIGHT: i32 = 100;
const GRAPH_AMPS_RIGHT: i32 = 160;
const PLOT: Rectangle = Rectangle::new(Point::new(LEFT - 4, 51), Size::new(230, 70));
/// The graph's full scale is the peak plus this headroom, in percent,
/// rounded up to one of these steps (times a power of ten), so the top line
/// is a round number the trace stays below: e.g. 22 W becomes 25 W.
const GRAPH_HEADROOM_PERCENT: u64 = 10;
const GRAPH_SCALE_STEPS: [u64; 10] = [10, 12, 15, 20, 25, 30, 40, 50, 60, 80];

/// Full scale for a `peak` in milli-units, in milli-units.
fn graph_scale(peak: u32) -> u32 {
    let wanted = (u64::from(peak) * (100 + GRAPH_HEADROOM_PERCENT)).div_ceil(100);
    let mut decade = 1;
    loop {
        if let Some(step) = GRAPH_SCALE_STEPS
            .iter()
            .map(|step| step * decade)
            .find(|step| *step >= wanted)
        {
            return u32::try_from(step).unwrap_or(u32::MAX);
        }
        decade *= 10;
    }
}

/// A full scale in milli-units as its shortest exact decimal, e.g. 25000
/// is "25", 1500 is "1.5" and 250 is "0.25".
fn scale_label(milli: u32) -> Label {
    if milli.is_multiple_of(1000) {
        Label::new().fixed(milli / 1000, 0)
    } else if milli.is_multiple_of(100) {
        Label::new().fixed(milli / 100, 1)
    } else {
        Label::new().fixed(milli / 10, 2)
    }
}

/// One summary row: `label`, then volts, amps and watts in the overview's
/// colours, or dashes without a reading.
fn graph_row<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    label: &str,
    color: Rgb565,
    sample: Option<Sample>,
    baseline: i32,
) {
    text(
        target,
        &PLAIN,
        label,
        Point::new(LEFT, baseline),
        HorizontalAlignment::Left,
        color,
    );
    let values = match sample {
        Some(sample) => [
            Label::new().hundredths(sample.mv.into()).push(b"V"),
            Label::new().hundredths(sample.ma.into()).push(b"A"),
            Label::new().watts(sample.mw).push(b"W"),
        ],
        None => [
            Label::new().push(b"--.--V"),
            Label::new().push(b"-.--A"),
            Label::new().push(b"--.--W"),
        ],
    };
    let columns = [
        (GRAPH_VOLTS_RIGHT, YELLOW),
        (GRAPH_AMPS_RIGHT, CYAN),
        (RIGHT, GREEN),
    ];
    for (value, (right, color)) in values.iter().zip(columns) {
        text(
            target,
            &PLAIN,
            value.as_str(),
            Point::new(right, baseline),
            HorizontalAlignment::Right,
            color,
        );
    }
}

/// How far `content` advances in `font`, in pixels.
fn width(font: &FontRenderer, content: &str) -> i32 {
    font.get_rendered_dimensions(content, Point::zero(), VerticalPosition::Baseline)
        .map_or(0, |dimensions| dimensions.advance.x)
}

/// The graph: `trace` over the last `window`, newest on the right, from zero
/// to a round full scale above the window's highest second. `focus` is what Left/Right
/// change, once opened.
pub fn draw_graph<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    history: &History,
    trace: GraphTrace,
    window: GraphWindow,
    focus: Option<GraphFocus>,
) {
    draw_header(target, header);
    let max = history.max(window);
    graph_row(target, "NOW", WHITE, history.latest(), GRAPH_NOW_BASELINE);
    graph_row(target, "MAX", MAGENTA, max, GRAPH_MAX_BASELINE);

    let top = PLOT.top_left.y;
    let bottom = top + PLOT.size.height as i32 - 1;
    // Full scale, half scale and zero.
    for y in [top, (top + bottom) / 2, bottom] {
        fill(
            target,
            Rectangle::new(
                Point::new(PLOT.top_left.x, y),
                Size::new(PLOT.size.width, 1),
            ),
            RULE,
        );
    }
    // The overview's colours for each quantity.
    let (color, tint) = match trace {
        GraphTrace::Power => (GREEN, GREEN_TINT),
        GraphTrace::Voltage => (YELLOW, YELLOW_TINT),
        GraphTrace::Current => (CYAN, CYAN_TINT),
    };
    let scale = graph_scale(max.map_or(0, |max| trace.value(max)));
    // Drawn first, so the trace stays on top where they meet.
    text(
        target,
        &PLAIN,
        scale_label(scale).push(trace.unit().as_bytes()).as_str(),
        Point::new(PLOT.top_left.x + 2, top + GRAPH_SCALE_LABEL_BELOW_TOP),
        HorizontalAlignment::Left,
        MUTED,
    );
    // A column per second, or, when the window has more seconds than the
    // plot has pixels, the highest of the seconds a pixel column covers, so
    // a short peak stays visible and still reaches the top.
    let seconds = window.seconds();
    let columns = seconds.min(PLOT.size.width as usize);
    let mut previous = None;
    let mut column = (0, None);
    for (second, sample) in history.recent(window).enumerate() {
        let index = second * columns / seconds;
        if index != column.0 {
            plot_column(target, column, columns, scale, (color, tint), &mut previous);
            column = (index, None);
        }
        column.1 = column.1.max(sample.map(|sample| trace.value(sample)));
    }
    plot_column(target, column, columns, scale, (color, tint), &mut previous);

    // Bottom right: the trace in its colour, then the window; arrows round
    // the one Left/Right change.
    let framed = |content: &str, focused: bool| {
        if focused {
            Label::new()
                .push(b"< ")
                .push(content.as_bytes())
                .push(b" >")
        } else {
            Label::new().push(content.as_bytes())
        }
    };
    let span = framed(window.label(), focus == Some(GraphFocus::Window));
    let span_width = PLAIN
        .get_rendered_dimensions(span.as_str(), Point::zero(), VerticalPosition::Baseline)
        .map_or(0, |dimensions| dimensions.advance.x);
    text(
        target,
        &PLAIN,
        span.as_str(),
        Point::new(RIGHT, HINT_BASELINE),
        HorizontalAlignment::Right,
        if focus == Some(GraphFocus::Window) {
            CYAN
        } else {
            MUTED
        },
    );
    text(
        target,
        &PLAIN,
        framed(trace.label(), focus == Some(GraphFocus::Trace)).as_str(),
        Point::new(RIGHT - span_width - GRAPH_LABEL_GAP, HINT_BASELINE),
        HorizontalAlignment::Right,
        color,
    );
    hint(
        target,
        header,
        match focus {
            None => "OK choose    < > tabs",
            Some(GraphFocus::Trace) => "< > trace  OK time",
            Some(GraphFocus::Window) => "< > time  OK done",
        },
    );
}

/// Plot column `index` of `columns` at `value` (none: a gap) against
/// `scale`, as a line in `color` over a `tint` fill, joined to the
/// `previous` column's point.
fn plot_column<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    (index, value): (usize, Option<u32>),
    columns: usize,
    scale: u32,
    (color, tint): (Rgb565, Rgb565),
    previous: &mut Option<Point>,
) {
    let Some(value) = value else {
        *previous = None;
        return;
    };
    let width = PLOT.size.width as usize;
    let left = PLOT.top_left.x + (width * index / columns) as i32;
    let right = PLOT.top_left.x + (width * (index + 1) / columns) as i32;
    let bottom = PLOT.top_left.y + PLOT.size.height as i32 - 1;
    let rise = (u64::from(value) * u64::from(PLOT.size.height - 1) / u64::from(scale)) as i32;
    let point = Point::new((left + right) / 2, bottom - rise);
    fill(
        target,
        Rectangle::new(
            Point::new(left, point.y),
            Size::new((right - left) as u32, rise as u32 + 1),
        ),
        tint,
    );
    Line::new(previous.unwrap_or(point), point)
        .draw_styled(&PrimitiveStyle::with_stroke(color, 1), target)
        .ok();
    *previous = Some(point);
}

/// Totals remain visible on their own monitor tab without displacing live
/// readings or peaks on the overview.
pub fn draw_energy<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    totals: Totals,
    memory: EnergyMemory,
    save_status: crate::energy_store::Status,
) {
    draw_header(target, header);
    let cards = [
        (
            Rectangle::new(Point::new(4, 25), Size::new(232, 39)),
            "CHARGE",
            totals.milliamp_hours(),
            "Ah",
            CYAN,
            CYAN_TINT,
        ),
        (
            Rectangle::new(Point::new(4, 68), Size::new(232, 39)),
            "ENERGY",
            totals.milliwatt_hours(),
            "Wh",
            GREEN,
            GREEN_TINT,
        ),
    ];
    for (area, label, value, unit, color, tint) in cards {
        card(target, area, tint, None);
        card_label(target, area, label, color);
        value_with_unit(
            target,
            &READING,
            &Label::new().fixed_u64(value, 3),
            unit,
            area.top_left + Point::new(6, CARD_VALUE),
            color,
        );
    }
    let status = if memory == EnergyMemory::Off {
        Label::new().push(b"RAM ONLY - RESTART RESETS")
    } else if save_status == crate::energy_store::Status::Occupied {
        Label::new().push(b"ENERGY AREA OCCUPIED")
    } else if save_status == crate::energy_store::Status::SaveFailed {
        Label::new().push(b"FLASH SAVE FAILED")
    } else {
        Label::new()
            .push(b"SAVE EVERY ")
            .push(memory.label().as_bytes())
            .push(b" - FLASH WEAR")
    };
    text(
        target,
        &PLAIN,
        status.as_str(),
        Point::new(LEFT, VERSION_BASELINE),
        HorizontalAlignment::Left,
        if memory != EnergyMemory::Off && save_status != crate::energy_store::Status::Ready {
            ORANGE
        } else {
            MUTED
        },
    );
    hint(target, header, "reset totals in settings    < > tabs");
}

/// Which way the current flows, as chevrons in the protocol strip. Upright, the
/// plug is on the left and the receptacle on the right; `flipped` (the frame
/// turned 180 degrees) swaps them. Positive current runs from the receptacle
/// to the plug (charger on the receptacle), negative the other way.
fn flow<D: DrawTarget<Color = Rgb565>>(target: &mut D, ma: i32, flipped: bool, pulses: u32) {
    if ma.unsigned_abs() < FLOW_MIN_MA {
        return;
    }
    let toward_plug = ma > 0;
    let (chevron, leftward) = if toward_plug != flipped {
        ("<", true)
    } else {
        (">", false)
    };
    // One chevron lit at a time, stepping with each pulse in the direction
    // of the flow.
    let step = (pulses % FLOW_CHEVRONS) as i32;
    let lit = if leftward {
        FLOW_CHEVRONS as i32 - 1 - step
    } else {
        step
    };
    let pitch = width(&BOLD, "> ");
    let first = FLOW_RIGHT - (FLOW_CHEVRONS as i32 - 1) * pitch - width(&BOLD, chevron);
    for index in 0..FLOW_CHEVRONS as i32 {
        text(
            target,
            &BOLD,
            chevron,
            Point::new(first + index * pitch, STRIP_CHEVRON_BASELINE),
            HorizontalAlignment::Left,
            if index == lit { CYAN } else { MUTED },
        );
    }
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
        Protocol::Pd { mv, ma, .. } | Protocol::Pps { mv, ma } | Protocol::Avs { mv, ma } => label
            .volts(mv)
            .push(b" ")
            .hundredths(u32::from(ma))
            .push(b"A"),
        Protocol::Qc2 { mv } | Protocol::Qc3 { mv: Some(mv) } => label.volts(mv),
        Protocol::Qc3 { mv: None } => label.push(b"continuous"),
        Protocol::Dcp(dcp) => match dcp.milliamps() {
            Some(ma) => label.push(b"5V ").hundredths(ma).push(b"A"),
            None => label.push(b"5V"),
        },
        Protocol::Bc12 => label.push(b"5V shorted"),
    }
}

/// Line cards, two by two: the data pair on top, the CC pair below.
/// Two rows of them fill the page.
const LINE_CARD_HEIGHT: u32 = outlined_height(2);
const LINE_VALUE: i32 = outlined_value(LINE_CARD_HEIGHT, READING_INK);
const DP_CARD: Rectangle = Rectangle::new(
    Point::new(4, outlined_top(0, LINE_CARD_HEIGHT)),
    Size::new(114, LINE_CARD_HEIGHT),
);
const DM_CARD: Rectangle = Rectangle::new(
    Point::new(122, outlined_top(0, LINE_CARD_HEIGHT)),
    Size::new(114, LINE_CARD_HEIGHT),
);
const CC1_CARD: Rectangle = Rectangle::new(
    Point::new(4, outlined_top(1, LINE_CARD_HEIGHT)),
    Size::new(114, LINE_CARD_HEIGHT),
);
const CC2_CARD: Rectangle = Rectangle::new(
    Point::new(122, outlined_top(1, LINE_CARD_HEIGHT)),
    Size::new(114, LINE_CARD_HEIGHT),
);

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

/// Outlined cards each showing a reading in volts: area, accent, label and
/// value.
fn volt_cards<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    cards: &[(Rectangle, Rgb565, &str, Label)],
) {
    for (area, color, label, value) in cards {
        outlined_card(target, *area, label, *color, 0);
        value_with_unit(
            target,
            &READING,
            value,
            "V",
            area.top_left + Point::new(PAD, LINE_VALUE),
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
            (DP_CARD, YELLOW, "D+", value(|l| l.dp_mv)),
            (DM_CARD, CYAN, "D-", value(|l| l.dm_mv)),
            (CC1_CARD, GREEN, "CC1", value(|l| l.cc1_mv)),
            (CC2_CARD, MAGENTA, "CC2", value(|l| l.cc2_mv)),
        ],
    );
    hint(target, header, "< > tabs  approximate");
}

/// How the PD side of the port stands, for the Quick Charge hints.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QcMode {
    /// No PD contract: requests reach the charger.
    Plain,
    /// A PD contract holds the charger, which then ignores Quick Charge; OK
    /// restarts it into a silent boot.
    Pd,
    /// This boot keeps PD silent (`NextBoot::silent`); OK on a PD tab
    /// restarts the charger into PD.
    Silent,
}

/// A Quick Charge tab under a PD contract, which the charger then ignores:
/// only how to switch.
fn pd_mode<D: DrawTarget<Color = Rgb565>>(target: &mut D, header: &Header<'_>) {
    notice(target, "PD MODE", "the charger ignores QC under PD", GREEN);
    hint(target, header, "OK switch to QC, tests up to 20V");
}

/// A PD tab while this boot keeps PD silent for Quick Charge: only how to
/// switch back, as [`pd_mode`] switches to Quick Charge.
fn qc_mode<D: DrawTarget<Color = Rgb565>>(target: &mut D, header: &Header<'_>) {
    notice(
        target,
        "QC MODE",
        "PD is off while Quick Charge runs",
        ORANGE,
    );
    hint(target, header, "OK switch back to PD");
}

/// Hint on a Quick Charge tab with nothing of its own running: the probe
/// comes first while this boot has none, else `start`.
fn qc_start_hint(qc: QcOffer, start: &'static str) -> &'static str {
    match qc.report() {
        None => "OK test the charger, up to 20V",
        Some(_) => start,
    }
}

/// Hint on a Quick Charge tab while its request runs, before `running`
/// once the charger follows.
fn qc_running_hint(status: Status, running: &'static str) -> &'static str {
    match status {
        Status::Entering => "handshake    < > tabs",
        Status::Failed(Failure::NotDcp) => "D+ D- not shorted    OK change",
        Status::Failed(Failure::NoEntry) => "D- stayed high    OK change",
        Status::Active(_, Some(Outcome::Waiting)) => "waiting for VBUS    < > tabs",
        Status::Active(_, Some(Outcome::Ignored)) => "charger kept its output    OK change",
        _ => running,
    }
}

/// The status pill on the Quick Charge 2.0 tab: on the voltage it reports,
/// or on the request while entering or after a failed entry.
fn qc2_pill(status: Status, request: Voltage) -> Option<(Voltage, &'static str, Rgb565)> {
    Some(match status {
        Status::Entering => (request, "WAIT", ORANGE),
        Status::Failed(Failure::NotDcp) => (request, "NO DCP", MAGENTA),
        Status::Failed(Failure::NoEntry) => (request, "NO QC", MAGENTA),
        Status::Active(voltage, Some(Outcome::Waiting)) => (voltage, "SETTING", ORANGE),
        Status::Active(voltage, Some(Outcome::Ignored)) => (voltage, "IGNORED", MAGENTA),
        Status::Active(voltage, _) => (voltage, "ACTIVE", GREEN),
        Status::Off | Status::Stepping(_) | Status::Detecting(_) | Status::Detected(_) => {
            return None;
        }
    })
}

/// Everything the Quick Charge 2.0 tab shows.
pub struct Qc2 {
    /// The row being chosen, while the choice is open.
    pub preview: Option<Qc2Choice>,
    /// The voltage asked of the charger, kept while other tabs show.
    pub request: Option<Voltage>,
    pub mode: QcMode,
    pub status: Status,
    pub qc: QcOffer,
}

/// The Quick Charge 2.0 tab, laid out as the fixed PD profiles: one row per
/// voltage and an off row, the choice framed, the status on its row.
/// Voltages the probe found the charger ignores are greyed out and cannot be
/// chosen.
pub fn draw_qc2<D: DrawTarget<Color = Rgb565>>(target: &mut D, header: &Header<'_>, page: &Qc2) {
    let &Qc2 {
        preview,
        request,
        mode,
        status,
        qc,
    } = page;
    draw_header(target, header);
    if mode == QcMode::Pd {
        pd_mode(target, header);
        return;
    }
    let pill = request.and_then(|request| qc2_pill(status, request));
    for (row, choice) in Qc2Choice::ALL.into_iter().enumerate() {
        let selected = preview == Some(choice);
        let Qc2Choice::Volts(voltage) = choice else {
            selector_row(target, row, "OFF", WHITE, ["plain USB", ""], selected, None);
            continue;
        };
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
            selected,
            pill.filter(|(at, ..)| *at == voltage)
                .map(|(_, label, color)| (label, color)),
        );
    }
    let hint_text = match (preview, request) {
        (Some(_), _) => "< > choose  OK request  BACK cancel",
        (None, None) => qc_start_hint(qc, "OK choose a voltage    < > tabs"),
        (None, Some(_)) => qc_running_hint(status, "OK change    < > tabs"),
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
    /// The output the steps aim for in mV, kept while other tabs show.
    pub target_mv: Option<u16>,
    /// Left/Right move the target.
    pub adjusting: bool,
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
        adjusting,
        mode,
        status,
        qc,
        lines,
        reading,
    } = page;
    draw_header(target, header);
    if mode == QcMode::Pd {
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
        if adjusting { RULE } else { BLACK },
        Some(if adjusting { YELLOW } else { RULE }),
    );
    card_label(target, PPS_VOLTS, "SET VOLTAGE", YELLOW);
    value_with_unit(
        target,
        &READING,
        &set,
        "V",
        PPS_VOLTS.top_left + PPS_VALUE,
        if adjusting { WHITE } else { YELLOW },
    );
    if adjusting {
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
        match target_mv {
            _ if adjusting => "< > 0.2V    OK done",
            None => qc_start_hint(qc, "OK start at 5V    < > tabs"),
            Some(_) => qc_running_hint(status, "OK adjust    < > tabs"),
        },
    );
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
            line(b"HOLD OK: FULL RESCAN");
            line(b"(QC AND PD)");
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
/// Startup found the upper code region missing or from another build: the
/// PD stack cannot run, so only this screen is shown.
pub fn draw_upper_missing<D: DrawTarget<Color = Rgb565>>(target: &mut D) {
    notice(
        target,
        "PD CODE MISSING",
        "flash the release image again",
        YELLOW,
    );
}

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
        .filter(|preview| editing && preview.kind == Kind::Fixed)
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
            .is_some_and(|contract| contract.kind == Kind::Fixed && contract.mv == profile.mv);
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
    scrollbar(
        target,
        profiles.len() as u32,
        PROFILE_ROWS as u32,
        first as u32,
        if editing { GREEN } else { MUTED },
    );
    hint(
        target,
        header,
        offer.pd_notice.unwrap_or(if !offer.session {
            VIEW_ONLY
        } else if editing {
            "< > choose  OK request  BACK cancel"
        } else if profiles.len() > PROFILE_ROWS {
            "OK select/scroll    < > tabs"
        } else {
            "OK select a profile    < > tabs"
        }),
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

    let set = offer.preview.filter(|preview| preview.kind == Kind::Pps);
    let (volts, amps) = set_values(set);
    value_box(
        target,
        PPS_VOLTS,
        "SET VOLTAGE",
        volts,
        "V",
        YELLOW,
        focus == Some(PpsFocus::Voltage),
    );
    value_box(
        target,
        PPS_AMPS,
        "CURRENT LIMIT",
        amps,
        "A",
        CYAN,
        focus == Some(PpsFocus::Current),
    );
    supply_status(target, offer, set, live);
    hint(
        target,
        header,
        offer.pd_notice.unwrap_or(match focus {
            _ if !offer.session => VIEW_ONLY,
            Some(PpsFocus::Voltage) => "< > volts    OK next: current",
            Some(PpsFocus::Current) => "< > amps    OK request",
            None => "OK adjust PPS    < > tabs",
        }),
    );
}

/// The SPR/EPR AVS selector: the advertised ranges, the voltage box (framed while
/// Left/Right change it), the current that voltage asks for, and the live
/// reading. The meter asks for the most the range allows at the set voltage.
pub fn draw_avs<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    offer: &Offer,
    adjusting: bool,
    live: Option<Reading>,
) {
    draw_header(target, header);
    if offer.quick_charge {
        qc_mode(target, header);
        return;
    }
    let ranges = offer.avs();
    if ranges.is_empty() {
        notice(target, "NO AVS", "this charger offers no AVS range", YELLOW);
        return;
    }
    for (index, range) in ranges.iter().take(2).enumerate() {
        let line = Label::new()
            .hundredths(u32::from(range.min_mv))
            .push(b"-")
            .hundredths(u32::from(range.max_mv))
            .push(b"V  ")
            .hundredths(u32::from(range.ma_15v))
            .push(b"A");
        let line = if let Some(pdp) = range.pdp_w {
            Label::new()
                .hundredths(u32::from(range.min_mv))
                .push(b"-")
                .hundredths(u32::from(range.max_mv))
                .push(b"V EPR ")
                .fixed(u32::from(pdp), 0)
                .push(b"W")
        } else if range.ma_20v == 0 {
            line
        } else {
            line.push(b" / ")
                .hundredths(u32::from(range.ma_20v))
                .push(b"A")
        };
        text(
            target,
            &PLAIN,
            line.as_str(),
            Point::new(LEFT, RANGE_BASELINE + RANGE_PITCH * index as i32),
            HorizontalAlignment::Left,
            YELLOW,
        );
    }
    let set = offer.preview.filter(|preview| preview.kind == Kind::Avs);
    let (volts, amps) = set_values(set);
    value_box(
        target,
        PPS_VOLTS,
        "SET VOLTAGE",
        volts,
        "V",
        YELLOW,
        adjusting,
    );
    value_box(target, PPS_AMPS, "CURRENT", amps, "A", CYAN, false);
    supply_status(target, offer, set, live);
    hint(
        target,
        header,
        offer.pd_notice.unwrap_or(if !offer.session {
            VIEW_ONLY
        } else if adjusting {
            "< > volts    OK request"
        } else {
            "OK adjust AVS    < > tabs"
        }),
    );
}

/// The set voltage and current, or placeholders before a preview.
fn set_values(set: Option<Supply>) -> (Label, Label) {
    match set {
        Some(set) => (
            Label::new().hundredths(u32::from(set.mv)),
            Label::new().hundredths(u32::from(set.ma)),
        ),
        None => (Label::new().push(b"--.--"), Label::new().push(b"-.--")),
    }
}

/// One PPS/AVS value box, framed with arrows while Left/Right change it.
fn value_box<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    area: Rectangle,
    label: &str,
    value: Label,
    unit: &str,
    color: Rgb565,
    focused: bool,
) {
    let (tint, border) = if focused {
        (RULE, Some(color))
    } else {
        (BLACK, Some(RULE))
    };
    card(target, area, tint, border);
    card_label(target, area, label, color);
    value_with_unit(
        target,
        &READING,
        &value,
        unit,
        area.top_left + PPS_VALUE,
        if focused { WHITE } else { color },
    );
    if focused {
        arrows(target, area, color);
    }
}

/// The line under the PPS/AVS boxes: the live output and whether the set
/// level is the contract in force.
fn supply_status<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    offer: &Offer,
    set: Option<Supply>,
    live: Option<Reading>,
) {
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
const SETTINGS_BASELINE: i32 = 35;
const SETTINGS_PITCH: i32 = 18;
/// Compact row highlights and edit frames leave room for all five settings.
const SETTINGS_ROW_HEIGHT: u32 = 17;
/// Baseline of the firmware version line, below the five rows and above the hint.
const VERSION_BASELINE: i32 = 117;
/// This firmware's version, from its Cargo package.
const VERSION: &str = env!("CARGO_PKG_VERSION");

/// The settings page: the cursor row is highlighted, and the value being
/// changed is framed.
pub fn draw_settings<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    settings: Settings,
    focus: SettingsFocus,
    save_status: crate::settings_store::Status,
) {
    draw_header(target, header);
    for (index, item) in Item::ALL.into_iter().enumerate() {
        let baseline = SETTINGS_BASELINE + index as i32 * SETTINGS_PITCH;
        let selected = item == focus.row();
        if selected {
            card(
                target,
                Rectangle::new(
                    Point::new(4, baseline - 13),
                    Size::new(232, SETTINGS_ROW_HEIGHT),
                ),
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
            Item::EnergyMemory => "ENERGY MEMORY",
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
                        Rectangle::new(
                            Point::new(154, baseline - 13),
                            Size::new(78, SETTINGS_ROW_HEIGHT),
                        ),
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
            Item::EnergyMemory => {
                if focus == SettingsFocus::EnergyMemory {
                    card(
                        target,
                        Rectangle::new(
                            Point::new(154, baseline - 13),
                            Size::new(78, SETTINGS_ROW_HEIGHT),
                        ),
                        BLACK,
                        Some(YELLOW),
                    );
                }
                text(
                    target,
                    &ROW,
                    settings.energy_memory.label(),
                    Point::new(193, baseline),
                    HorizontalAlignment::Center,
                    if selected { YELLOW } else { MUTED },
                );
            }
        }
    }
    // A failed save stays visible instead of silently promising persistence.
    let (label, color) = match save_status {
        crate::settings_store::Status::Saved => ("VERSION", MUTED),
        crate::settings_store::Status::NoSpace => ("SETTINGS AREA FULL", ORANGE),
        crate::settings_store::Status::WriteFailed => ("SETTINGS WRITE FAILED", ORANGE),
    };
    text(
        target,
        &PLAIN,
        label,
        Point::new(LEFT + 14, VERSION_BASELINE),
        HorizontalAlignment::Left,
        color,
    );
    text(
        target,
        &PLAIN,
        VERSION,
        Point::new(RIGHT - 4, VERSION_BASELINE),
        HorizontalAlignment::Right,
        MUTED,
    );
    hint(
        target,
        header,
        match focus {
            SettingsFocus::Brightness => "< > dimmer/brighter    OK done",
            SettingsFocus::Rotation => "< > auto/normal/flipped  OK done",
            SettingsFocus::EnergyMemory => "shorter saves = more wear   OK done",
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
            Rectangle::new(
                Point::new(left - 6, baseline - 13),
                Size::new(78, SETTINGS_ROW_HEIGHT),
            ),
            BLACK,
            Some(YELLOW),
        );
    }
    for index in 0..BRIGHTNESS_PERCENT.len() {
        let height = 2 + 3 * index as u32;
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
/// Shortest scrollbar thumb in pixels, so a long list still shows its position.
const SCROLL_THUMB_MIN: u32 = 8;

/// A scrollbar on `SCROLL_TRACK` for `visible` of `total` rows from row
/// `first`; nothing when every row fits.
fn scrollbar<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    total: u32,
    visible: u32,
    first: u32,
    color: Rgb565,
) {
    let end = total.saturating_sub(visible);
    if end == 0 {
        return;
    }
    fill(target, SCROLL_TRACK, RULE);
    let track = SCROLL_TRACK.size.height;
    let thumb = (track * visible / total).max(SCROLL_THUMB_MIN);
    let offset = (track - thumb) * first.min(end) / end;
    fill(
        target,
        Rectangle::new(
            SCROLL_TRACK.top_left + Point::new(0, offset as i32),
            Size::new(SCROLL_TRACK.size.width, thumb),
        ),
        color,
    );
}

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
    scrollbar(
        target,
        u32::from(rows),
        u32::from(LIST_ROWS),
        u32::from(top),
        if scrolling { accent } else { MUTED },
    );
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
