//! Graphical pages: overview, line voltages, PD profile, PPS and AVS selectors,
//! Quick Charge trigger, settings and the scrolling detail lists, drawn with
//! `embedded-graphics` primitives and `u8g2-fonts` text on any RGB565 target.
//! The renderer gathers the values; nothing here touches hardware.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{
    Circle, Line, PrimitiveStyle, Rectangle, RoundedRectangle, StyledDrawable, Triangle,
};
use firmware_core::buttons::{
    Detail, GraphFocus, Level, PpsFocus, Qc2Choice, QcOffer, RESET_STEPS, RippleFocus,
    SettingsFocus, Tab,
};
use firmware_core::calibration::{Calibration, Quantity, Step, Wizard};
use firmware_core::dcp::Dcp;
use firmware_core::energy::Totals;
use firmware_core::power_history::{GraphTrace, GraphWindow, History, Sample};
use firmware_core::settings::{
    BRIGHTNESS_PERCENT, EnergyMemory, FcpFindings, Item, QcFindings, Rotation, ScpFindings,
    Settings,
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
/// The graph's now/max values, which must fit one line: 6x13, monospaced.
const GRAPH_VALUES: FontRenderer =
    FontRenderer::new::<fonts::u8g2_font_6x13_mr>().with_ignore_unknown_chars(true);
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
/// Voltage, current and power in the same colour on every page; charge
/// takes current's and energy power's.
const VOLTS: Rgb565 = YELLOW;
const AMPS: Rgb565 = CYAN;
const WATTS: Rgb565 = GREEN;
/// Dark fills under the graph's trace, matching the colours above.
const VOLTS_TINT: Rgb565 = Rgb565::new(5, 10, 0);
const AMPS_TINT: Rgb565 = Rgb565::new(1, 9, 6);
const WATTS_TINT: Rgb565 = Rgb565::new(1, 10, 1);
/// Each charging protocol's colour: its tab, its page's cards and its
/// overview badge. Every tab in a group has a colour of its own; a family
/// (PD, Quick Charge, FCP and SCP) shares a hue in different tones.
const PD_ACCENT: Rgb565 = GREEN;
const PPS_ACCENT: Rgb565 = Rgb565::new(20, 63, 20);
const AVS_ACCENT: Rgb565 = Rgb565::new(2, 46, 10);
const QC2_ACCENT: Rgb565 = ORANGE;
const QC3_ACCENT: Rgb565 = Rgb565::new(31, 50, 18);
const SCP_ACCENT: Rgb565 = MAGENTA;
const FCP_ACCENT: Rgb565 = Rgb565::new(31, 44, 31);
/// Ripple spectrum bars, dim purple behind the cyan trace, and their peak
/// labels.
const SPECTRUM: Rgb565 = Rgb565::new(12, 6, 14);
const SPECTRUM_LABEL: Rgb565 = MAGENTA;
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
    TypeC {
        ma: u16,
    },
    /// The source offered PD capabilities, but no contract is known.
    PdOffer,
    /// Fixed PD contract; `revision` is the major PD revision (2 or 3).
    Pd {
        revision: Option<u8>,
        mv: u16,
        ma: u16,
    },
    /// Programmable power supply contract.
    Pps {
        mv: u16,
        ma: u16,
    },
    /// SPR adjustable voltage supply contract.
    Avs {
        mv: u16,
        ma: u16,
    },
    /// Quick Charge 2.0 at this requested output, in mV.
    Qc2 {
        mv: u16,
    },
    /// Quick Charge 3.0 continuous mode; `mv` is the output the steps set,
    /// when this meter sends them.
    Qc3 {
        mv: Option<u16>,
    },
    /// SCP capability identified by register reads, not an output contract.
    ScpDetected,
    Scp {
        mv: u16,
        ma: u16,
    },
    /// FCP or SCP data another device sends on D−, not told apart yet.
    SingleWire,
    /// FCP at `mv`: this meter's applied request, or, for transactions
    /// another device sends, the output its latest acknowledged voltage
    /// write requested while VBUS is near it.
    Fcp {
        mv: Option<u16>,
    },
    /// SCP transactions another device sends, as for [`Self::Fcp`].
    DrivenScp {
        mv: Option<u16>,
    },
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

/// A tab's label and colour, each different from the rest of its group's,
/// so the folded bars can be told apart.
fn tab_style(tab: Tab, passthrough: bool) -> (&'static str, Rgb565) {
    match tab {
        Tab::Overview => ("OVERVIEW", CYAN),
        Tab::Graph => ("GRAPH", ORANGE),
        Tab::Ripple => ("RIPPLE", MAGENTA),
        Tab::Lines => ("LINES", YELLOW),
        Tab::Energy => ("ENERGY", GREEN),
        Tab::Fixed => ("PD PROFILES", PD_ACCENT),
        Tab::Pps => ("PPS", PPS_ACCENT),
        Tab::Avs => ("AVS", AVS_ACCENT),
        Tab::Qc2 => ("QC 2.0", QC2_ACCENT),
        Tab::Qc3 => ("QC 3.0", QC3_ACCENT),
        Tab::Scp => ("SCP", SCP_ACCENT),
        Tab::Fcp => ("FCP", FCP_ACCENT),
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

    /// Hours, minutes and seconds from ms, e.g. "1:02:03"; hours grow past 99.
    fn duration(self, ms: u64) -> Self {
        let seconds = ms / 1000;
        let two = |value: u64| [b'0' + (value / 10) as u8, b'0' + (value % 10) as u8];
        self.fixed_u64(seconds / 3600, 0)
            .push(b":")
            .push(&two(seconds / 60 % 60))
            .push(b":")
            .push(&two(seconds % 60))
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
        Protocol::PdOffer | Protocol::Pd { revision: None, .. } => ("PD", PD_ACCENT),
        Protocol::Pd {
            revision: Some(3), ..
        } => ("PD 3.0", PD_ACCENT),
        Protocol::Pd { .. } => ("PD 2.0", CYAN),
        Protocol::Pps { .. } => ("PPS", PPS_ACCENT),
        Protocol::Avs { .. } => ("AVS", AVS_ACCENT),
        Protocol::Qc2 { .. } => ("QC 2.0", QC2_ACCENT),
        Protocol::Qc3 { .. } => ("QC 3.0", QC3_ACCENT),
        Protocol::ScpDetected | Protocol::Scp { .. } | Protocol::DrivenScp { .. } => {
            ("SCP", SCP_ACCENT)
        }
        Protocol::Fcp { .. } => ("FCP", FCP_ACCENT),
        Protocol::SingleWire => ("FCP/SCP", SCP_ACCENT),
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
/// `PLAIN` capitals and digits, `SMALL`, `GRAPH_VALUES`, `READING` and
/// `POWER` digits.
const PLAIN_INK: i32 = 8;
const SMALL_INK: i32 = 7;
const GRAPH_VALUES_INK: i32 = 9;
const READING_INK: i32 = 19;
const POWER_INK: i32 = 27;

/// Outlined cards, on the overview, lines and energy pages: an outline in
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
/// The peaks card's width: its longest row, six `PLAIN` characters 6 px
/// wide ("20.00V", "100.0W"), its padding and a little room; the power
/// card takes the rest of the row.
const PEAKS_WIDTH: u32 = 52;
const PEAKS_CARD: Rectangle = Rectangle::new(
    Point::new(236 - PEAKS_WIDTH as i32, POWER_TOP),
    Size::new(PEAKS_WIDTH, POWER_HEIGHT),
);
const POWER_CARD: Rectangle = Rectangle::new(
    Point::new(4, POWER_TOP),
    Size::new(PEAKS_CARD.top_left.x as u32 - 8, POWER_HEIGHT),
);
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
        (VOLTS_CARD, VOLTS, "VOLTAGE", volts, "V"),
        (AMPS_CARD, AMPS, "CURRENT", amps, "A"),
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
    outlined_card(target, POWER_CARD, "POWER", WATTS, 0);
    value_with_unit(
        target,
        &POWER,
        &power,
        "W",
        POWER_CARD.top_left + Point::new(PAD, POWER_VALUE),
        WATTS,
    );

    outlined_card(target, PEAKS_CARD, "MAX", MAGENTA, meter.reset_hold);
    let peaks = [
        (Label::new().hundredths(meter.peaks.mv).push(b"V"), VOLTS),
        (Label::new().hundredths(meter.peaks.ma).push(b"A"), AMPS),
        (Label::new().watts(meter.peaks.mw).push(b"W"), WATTS),
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

/// Space between the trace and window labels, and the baseline of the
/// labels inside the plot (full scale, trace, window) below its top line,
/// in pixels.
const GRAPH_LABEL_GAP: i32 = 6;
const GRAPH_SCALE_LABEL_BELOW_TOP: i32 = 10;
/// Power graph: the now/max values in an outlined card above the plot,
/// sitting like a value and padding 3 px above its bottom edge, and the
/// plot filling the rest 2 clear rows under it.
const GRAPH_ROW_VALUE: i32 = LABEL_BELOW_EDGE + 2 + GRAPH_VALUES_INK;
const GRAPH_ROW_CARD: Rectangle = Rectangle::new(
    Point::new(4, OUTLINED_TOP),
    Size::new(232, (GRAPH_ROW_VALUE + OVERVIEW_PAD + 1) as u32),
);
const PLOT_TOP: i32 = GRAPH_ROW_CARD.top_left.y + GRAPH_ROW_CARD.size.height as i32 + OUTLINED_GAP;
const PLOT: Rectangle = Rectangle::new(
    Point::new(LEFT - 4, PLOT_TOP),
    Size::new(230, (CONTENT_BOTTOM - PLOT_TOP + 1) as u32),
);
/// Right edge of the labels inside the plot.
const PLOT_RIGHT: i32 = PLOT.top_left.x + PLOT.size.width as i32 - 2;
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

/// The now/max card: the latest second's and the window's highest volts,
/// amps and watts ("9.01/9.10V") in the overview's colours, dashes for
/// either without a reading, spread across the card with equal gaps under
/// a `NOW/MAX` label.
fn graph_row<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    now: Option<Sample>,
    max: Option<Sample>,
    reset_hold: u8,
) {
    let pair = |value: fn(Label, Sample) -> Label, blank: &[u8], unit: &[u8]| {
        let half = |sample: Option<Sample>, label: Label| match sample {
            Some(sample) => value(label, sample),
            None => label.push(blank),
        };
        half(max, half(now, Label::new()).push(b"/")).push(unit)
    };
    let values = [
        pair(|l, s| l.hundredths(s.mv.into()), b"--.--", b"V"),
        pair(|l, s| l.hundredths(s.ma.into()), b"-.--", b"A"),
        pair(|l, s| l.watts(s.mw), b"--.--", b"W"),
    ];
    let area = GRAPH_ROW_CARD;
    outlined_card(target, area, "NOW/MAX", WHITE, reset_hold);
    let label = Point::new(
        area.top_left.x + OUTLINED_LABEL_LEFT,
        area.top_left.y + LABEL_BELOW_EDGE,
    );
    text(
        target,
        &SMALL,
        "/MAX",
        label + Point::new(width(&SMALL, "NOW"), 0),
        HorizontalAlignment::Left,
        MAGENTA,
    );
    let left = area.top_left.x + PAD;
    let right = area.top_left.x + area.size.width as i32 - PAD;
    let widths = values
        .each_ref()
        .map(|value| width(&GRAPH_VALUES, value.as_str()));
    let gap = (right - left - widths.iter().sum::<i32>()) / (widths.len() as i32 - 1);
    let mut x = left;
    for ((value, width), color) in values.iter().zip(widths).zip([VOLTS, AMPS, WATTS]) {
        text(
            target,
            &GRAPH_VALUES,
            value.as_str(),
            Point::new(x, area.top_left.y + GRAPH_ROW_VALUE),
            HorizontalAlignment::Left,
            color,
        );
        x += width + gap;
    }
}

/// How far `content` advances in `font`, in pixels.
fn width(font: &FontRenderer, content: &str) -> i32 {
    font.get_rendered_dimensions(content, Point::zero(), VerticalPosition::Baseline)
        .map_or(0, |dimensions| dimensions.advance.x)
}

/// The graph: `trace` over the last `window`, newest on the right, from zero
/// to a round full scale above the window's highest second. `focus` is what Left/Right
/// change, once opened; `reset_hold` the reset hold's progress.
pub fn draw_graph<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    history: &History,
    trace: GraphTrace,
    window: GraphWindow,
    focus: Option<GraphFocus>,
    reset_hold: u8,
) {
    draw_header(target, header);
    let max = history.max(window);
    graph_row(target, history.latest(), max, reset_hold);

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
        GraphTrace::Power => (WATTS, WATTS_TINT),
        GraphTrace::Voltage => (VOLTS, VOLTS_TINT),
        GraphTrace::Current => (AMPS, AMPS_TINT),
    };
    let scale = graph_scale(max.map_or(0, |max| trace.value(max)));
    let label_baseline = top + GRAPH_SCALE_LABEL_BELOW_TOP;
    // Drawn first, so the trace stays on top where they meet.
    text(
        target,
        &PLAIN,
        scale_label(scale).push(trace.unit().as_bytes()).as_str(),
        Point::new(PLOT.top_left.x + 2, label_baseline),
        HorizontalAlignment::Left,
        MUTED,
    );
    // Top right of the plot, under the trace like the scale: the trace in
    // its colour, then the window; arrows round the one Left/Right change.
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
    let span_width = width(&PLAIN, span.as_str());
    text(
        target,
        &PLAIN,
        span.as_str(),
        Point::new(PLOT_RIGHT, label_baseline),
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
        Point::new(PLOT_RIGHT - span_width - GRAPH_LABEL_GAP, label_baseline),
        HorizontalAlignment::Right,
        color,
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

    hint(
        target,
        header,
        match focus {
            None => "< > tabs  OK choose  hold OK reset",
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

/// Energy cards: charge, energy and time, three rows filling the page.
const ENERGY_CARD_HEIGHT: u32 = outlined_height(3);
const ENERGY_VALUE: i32 = outlined_value(ENERGY_CARD_HEIGHT, READING_INK);

/// Totals remain visible on their own monitor tab without displacing live
/// readings or peaks on the overview; `reset_hold` is the reset hold's
/// progress over them.
pub fn draw_energy<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    totals: Totals,
    memory: EnergyMemory,
    save_status: crate::energy_store::Status,
    reset_hold: u8,
) {
    draw_header(target, header);
    let cards = [
        (
            "CHARGE",
            Label::new().fixed_u64(totals.milliamp_hours(), 3),
            "Ah",
            AMPS,
        ),
        (
            "ENERGY",
            Label::new().fixed_u64(totals.milliwatt_hours(), 3),
            "Wh",
            WATTS,
        ),
        ("TIME", Label::new().duration(totals.elapsed_ms), "", WHITE),
    ];
    for (index, (label, value, unit, color)) in cards.iter().enumerate() {
        let top = outlined_top(index as i32, ENERGY_CARD_HEIGHT);
        let area = Rectangle::new(Point::new(4, top), Size::new(232, ENERGY_CARD_HEIGHT));
        outlined_card(target, area, label, *color, reset_hold);
        value_with_unit(
            target,
            &READING,
            value,
            unit,
            Point::new(area.top_left.x + PAD, top + ENERGY_VALUE),
            *color,
        );
    }
    // How the totals are kept, as the hint's note: the cards fill the page.
    let note: &[u8] = match (memory, save_status) {
        (EnergyMemory::Off, _) => b"RAM only",
        (_, crate::energy_store::Status::Occupied) => b"flash taken",
        (_, crate::energy_store::Status::SaveFailed) => b"save failed",
        (_, crate::energy_store::Status::Ready) => b"save ",
    };
    let mut hint_text = Label::new().push(b"< > tabs  hold OK reset  ").push(note);
    if memory != EnergyMemory::Off && save_status == crate::energy_store::Status::Ready {
        hint_text = hint_text.push(memory.label().as_bytes());
    }
    hint(target, header, hint_text.as_str());
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
        Protocol::Pd { mv, ma, .. }
        | Protocol::Pps { mv, ma }
        | Protocol::Avs { mv, ma }
        | Protocol::Scp { mv, ma } => label
            .volts(mv)
            .push(b" ")
            .hundredths(u32::from(ma))
            .push(b"A"),
        Protocol::Qc2 { mv }
        | Protocol::Qc3 { mv: Some(mv) }
        | Protocol::Fcp { mv: Some(mv) }
        | Protocol::DrivenScp { mv: Some(mv) } => label.volts(mv),
        Protocol::Qc3 { mv: None } => label.push(b"continuous"),
        Protocol::ScpDetected => label.push(b"detected, 5V"),
        Protocol::SingleWire | Protocol::Fcp { mv: None } | Protocol::DrivenScp { mv: None } => {
            label.push(b"D- data")
        }
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

/// Waveform/statistics can be held independently of the live bus reading.
pub struct Ripple<'a> {
    pub reading: Option<&'a firmware_core::ripple::Reading>,
    pub reference: Option<u16>,
    pub bus: Option<Reading>,
    pub hold: bool,
    /// The selected timebase; a capture is labelled with its own.
    pub timebase: firmware_core::ripple::Timebase,
    pub scale: firmware_core::ripple::Scale,
    /// Open: the setting Left/Right change; OK focuses the next.
    pub focus: Option<RippleFocus>,
    /// ADC1's clock in Hz; without one, no timebase is reachable.
    pub clock_hz: Option<u32>,
}

/// One space of ProFont 12 between run/hold and the capture status, in pixels.
const RIPPLE_STATUS_GAP: i32 = 6;

/// Estimated voltage scale and nominal clock-derived timebase, not calibration.
pub fn draw_ripple<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    page: &Ripple<'_>,
) {
    use crate::line_adc::REFERENCE_MV;
    use firmware_core::ripple::{HORIZONTAL_DIVISIONS, Scale, VERTICAL_DIVISIONS};
    use firmware_core::spectrum::{self, FULL_HEIGHT};
    // Peak frequencies show tenths of a kHz below 100 kHz, whole kHz above.
    const KILO_HZ_DECIMALS_BELOW: u32 = 100_000;
    // Pixels: metrics above a four-division, 56-pixel plot, live bus/scale
    // below it, then run/hold, status, nominal timebase and the control hint.
    const VALUE_Y: i32 = 37;
    const BUS_Y: i32 = 109;
    const TOP: i32 = 40;
    const BOTTOM: i32 = 96;
    const STATUS_Y: i32 = 120;
    const COLUMN_PITCH: i32 = 2;
    // Arrows round the setting Left/Right change, as on the graph, in
    // yellow; `color` otherwise.
    let setting = |target: &mut D, content: &str, field, point, alignment, color| {
        let focused = page.focus == Some(field);
        let label = if focused {
            Label::new()
                .push(b"< ")
                .push(content.as_bytes())
                .push(b" >")
        } else {
            Label::new().push(content.as_bytes())
        };
        let color = if focused { YELLOW } else { color };
        text(target, &PLAIN, label.as_str(), point, alignment, color);
        width(&PLAIN, label.as_str())
    };
    draw_header(target, header);
    let normalized = page.reading.zip(page.reference);
    let mv = normalized.and_then(|(r, reference)| r.peak_to_peak_mv(reference, REFERENCE_MV));
    let rms = normalized.and_then(|(r, reference)| r.rms_tenths_mv(reference, REFERENCE_MV));
    let value = mv.map_or_else(
        || Label::new().push(b"--"),
        |mv| Label::new().fixed(u32::from(mv), 0),
    );
    value_with_unit(
        target,
        &BOLD,
        &value,
        "mV p-p",
        Point::new(LEFT, VALUE_Y),
        VOLTS,
    );
    let rms = match rms {
        Some(rms) => Label::new()
            .push(b"RMS ")
            .fixed(u32::from(rms), 1)
            .push(b"mV"),
        None => Label::new().push(b"RMS --mV"),
    };
    text(
        target,
        &PLAIN,
        rms.as_str(),
        Point::new(RIGHT, VALUE_Y - 1),
        HorizontalAlignment::Right,
        WHITE,
    );
    let bus = match page.bus {
        Some(bus) => Label::new()
            .fixed(bus.mv, 3)
            .push(b"V ")
            .push(if bus.ma < 0 { b"-" } else { b"" })
            .fixed(bus.ma.unsigned_abs(), 3)
            .push(b"A"),
        None => Label::new().push(b"--V --A"),
    };
    text(
        target,
        &PLAIN,
        bus.as_str(),
        Point::new(LEFT, BUS_Y),
        HorizontalAlignment::Left,
        WHITE,
    );
    let plot = page
        .reading
        .map(|r| r.plot_scale(page.scale, page.reference, REFERENCE_MV));
    let scale = match (plot.and_then(|plot| plot.mv_per_div), page.scale) {
        (Some(mv), scale) => Label::new()
            .push(if scale == Scale::Auto {
                b"AUTO "
            } else {
                b"LOCK "
            })
            .fixed(u32::from(mv), 0)
            .push(b"mV/div"),
        // Before a capture: the chosen scale, not yet a voltage.
        (None, Scale::Auto) if page.reading.is_none() => Label::new().push(b"AUTO"),
        (None, scale) if page.reading.is_none() => Label::new()
            .push(b"LOCK ")
            .fixed(u32::from(scale.millivolts().unwrap_or(0)), 0)
            .push(b"mV/div"),
        (None, _) => Label::new().push(b"RAW AUTO"),
    };
    setting(
        target,
        scale.as_str(),
        RippleFocus::Scale,
        Point::new(RIGHT, BUS_Y),
        HorizontalAlignment::Right,
        MUTED,
    );
    if let (Some(reading), Some(plot)) = (page.reading, plot) {
        let y = |count: u16| {
            let count = i32::from(count).clamp(plot.min, plot.max);
            BOTTOM - (count - plot.min) * (BOTTOM - TOP) / (plot.max - plot.min)
        };
        for division in 0..=VERTICAL_DIVISIONS as i32 {
            let row = TOP + division * (BOTTOM - TOP) / VERTICAL_DIVISIONS as i32;
            Line::new(Point::new(LEFT, row), Point::new(RIGHT - 1, row))
                .into_styled(PrimitiveStyle::with_stroke(RULE, 1))
                .draw(target)
                .ok();
        }
        for division in 0..=HORIZONTAL_DIVISIONS as i32 {
            let x = LEFT + division * (RIGHT - LEFT - 1) / HORIZONTAL_DIVISIONS as i32;
            Line::new(Point::new(x, TOP), Point::new(x, BOTTOM))
                .into_styled(PrimitiveStyle::with_stroke(RULE, 1))
                .draw(target)
                .ok();
        }
        // The spectrum behind the trace: frequency left to right, heights
        // relative to the strongest bin, peak frequencies listed at the top.
        let spectrum = &reading.spectrum;
        let bar = |height: u8| i32::from(height) * (BOTTOM - TOP) / i32::from(FULL_HEIGHT);
        for (column, &height) in spectrum.bars.iter().enumerate() {
            let x = LEFT + column as i32;
            if bar(height) > 0 {
                Line::new(Point::new(x, BOTTOM), Point::new(x, BOTTOM - bar(height)))
                    .into_styled(PrimitiveStyle::with_stroke(SPECTRUM, 1))
                    .draw(target)
                    .ok();
            }
        }
        let rate = page
            .clock_hz
            .and_then(|hz| Some(reading.timebase.sampling(hz)?.rate_hz(hz)));
        if let Some(rate) = rate {
            let mut peaks = Label::new();
            for (index, position) in spectrum.peaks.iter().enumerate() {
                let Some(position) = *position else { break };
                let hz = spectrum::frequency_hz(position, rate);
                if index > 0 {
                    peaks = peaks.push(b" ");
                }
                peaks = peaks.push(&[b'1' + index as u8, b':']);
                // Rounded to the shown digit: 100 Hz or 1 kHz.
                peaks = if hz < KILO_HZ_DECIMALS_BELOW {
                    peaks.fixed((hz + 50) / 100, 1)
                } else {
                    peaks.fixed((hz + 500) / 1000, 0)
                };
                peaks = peaks.push(b"k");
            }
            text(
                target,
                &SMALL,
                peaks.as_str(),
                Point::new(RIGHT - 2, TOP + SMALL_INK + 1),
                HorizontalAlignment::Right,
                SPECTRUM_LABEL,
            );
        }
        let mut previous = None;
        for (column, bin) in reading.trace.iter().enumerate() {
            let x = LEFT + column as i32 * COLUMN_PITCH;
            let low = y(bin.min);
            let high = y(bin.max);
            Line::new(Point::new(x, low), Point::new(x, high))
                .into_styled(PrimitiveStyle::with_stroke(VOLTS, 1))
                .draw(target)
                .ok();
            let centre = Point::new(x, (low + high) / 2);
            if let Some(previous) = previous {
                Line::new(previous, centre)
                    .into_styled(PrimitiveStyle::with_stroke(VOLTS, 1))
                    .draw(target)
                    .ok();
            }
            previous = Some(centre);
        }
        // Peak numbers at the foot of their bars, beside rather than over
        // the bar (a "1" would vanish into it), and over the trace so they
        // stay legible. Right of the bar, or left where the plot ends.
        for (index, position) in spectrum.peaks.iter().enumerate() {
            let Some(position) = *position else { break };
            let digit = [b'1' + index as u8];
            let digit = core::str::from_utf8(&digit).unwrap_or("");
            let x = LEFT + spectrum::column(position) as i32;
            let (x, alignment) = if x + 2 + width(&SMALL, digit) < RIGHT {
                (x + 2, HorizontalAlignment::Left)
            } else {
                (x - 1, HorizontalAlignment::Right)
            };
            text(
                target,
                &SMALL,
                digit,
                Point::new(x, BOTTOM - 2),
                alignment,
                SPECTRUM_LABEL,
            );
        }
    } else {
        text(
            target,
            &BOLD,
            "NO CAPTURE",
            Point::new(WIDTH as i32 / 2, (TOP + BOTTOM) / 2),
            HorizontalAlignment::Center,
            MUTED,
        );
    }
    let clipped = page.reading.is_some_and(|r| r.clipped());
    let limited = plot.is_some_and(|plot| plot.limited);
    let capture_width = setting(
        target,
        if page.hold { "HOLD" } else { "RUN" },
        RippleFocus::Capture,
        Point::new(LEFT, STATUS_Y),
        HorizontalAlignment::Left,
        if page.hold { WHITE } else { MUTED },
    );
    let status = match page.reading {
        None => "NO CAPTURE",
        Some(_) if clipped => "CLIPPED",
        Some(_) if mv.is_none() => "NO REF",
        Some(_) if limited => "OFF SCALE",
        Some(r) if r.triggered => "T",
        Some(_) => "",
    };
    text(
        target,
        &PLAIN,
        status,
        Point::new(LEFT + capture_width + RIPPLE_STATUS_GAP, STATUS_Y),
        HorizontalAlignment::Left,
        if clipped || limited { ORANGE } else { MUTED },
    );
    // ASCII `us` is microseconds; `~` marks a nominal, unmeasured timebase.
    let timebase = page.reading.map_or(page.timebase, |r| r.timebase);
    let timebase = match page.clock_hz.and_then(|hz| timebase.sampling(hz)) {
        Some(_) => Label::new()
            .push(b"~")
            .fixed(timebase.us_per_div(), 0)
            .push(b"us/div"),
        None => Label::new().push(b"--us/div"),
    };
    setting(
        target,
        timebase.as_str(),
        RippleFocus::Time,
        Point::new(RIGHT, STATUS_Y),
        HorizontalAlignment::Right,
        MUTED,
    );
    hint(
        target,
        header,
        match page.focus {
            None => "< > tabs  OK open",
            Some(RippleFocus::Time) => "< > time  OK scale  hold OK reset",
            Some(RippleFocus::Scale) => "< > scale  OK run/hold  hold OK reset",
            Some(RippleFocus::Capture) => "< > run/hold  OK time  hold OK reset",
        },
    );
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
    hint(target, header, "< > tabs  OK switch to QC, up to 20V");
}

/// A PD tab while this boot keeps PD silent for Quick Charge: only how to
/// switch back, as [`pd_mode`] switches to Quick Charge.
fn qc_mode<D: DrawTarget<Color = Rgb565>>(target: &mut D, header: &Header<'_>) {
    notice(target, "LEGACY MODE", "PD is off for QC / SCP", ORANGE);
    hint(target, header, "< > tabs  OK switch back to PD");
}

/// Hint on a Quick Charge tab with nothing of its own running: the probe
/// comes first while this boot has none, else `start`.
fn qc_start_hint(qc: QcOffer, start: &'static str) -> &'static str {
    match qc.report() {
        None => "< > tabs  OK test charger, up to 20V",
        Some(_) => start,
    }
}

/// Hint on a Quick Charge tab while its request runs, before `running`
/// once the charger follows.
fn qc_running_hint(status: Status, running: &'static str) -> &'static str {
    match status {
        Status::Entering => "< > tabs  handshake",
        Status::Failed(Failure::NotDcp) => "< > tabs  OK change  no D+ D- short",
        Status::Failed(Failure::NoEntry) => "< > tabs  OK change  D- stayed high",
        Status::Active(_, Some(Outcome::Waiting)) => "< > tabs  waiting for VBUS",
        Status::Active(_, Some(Outcome::Ignored)) => "< > tabs  OK change  output unchanged",
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
        Status::Off
        | Status::Stepping(_)
        | Status::Detecting(_)
        | Status::Detected(_)
        | Status::Scp(..)
        | Status::Fcp(..) => {
            return None;
        }
    })
}

fn scp_error(error: crate::scp_drive::Failure) -> &'static str {
    use crate::scp_drive::Failure;
    use scp_protocol::Error;
    match error {
        Failure::Entry => "LEGACY ENTRY FAILED",
        Failure::DriveLevel => "LINE LEVEL CHECK FAILED",
        Failure::Vbus => "VBUS CHECK FAILED",
        Failure::Clock => "MICROSECOND CLOCK FAILED",
        Failure::Silent => "NO DIGITAL RESPONSE",
        Failure::Control(error) => match error {
            scp_protocol::control::ControlError::Wire(error) => scp_error(Failure::Wire(error)),
            scp_protocol::control::ControlError::Request(_) => "UNSUPPORTED SETPOINT",
            scp_protocol::control::ControlError::Readback => "REGISTER READBACK FAILED",
            scp_protocol::control::ControlError::Protection => "PROTECTION / MODE LOST",
        },
        Failure::Wire(error) => match error {
            Error::Transport(never) => match never {},
            Error::Timing => "EDGE TIMING FAILED",
            Error::Timeout => "NO DIGITAL RESPONSE",
            Error::Framing => "INVALID SYNC OR PING",
            Error::Parity => "PARITY CHECK FAILED",
            Error::Crc => "CRC CHECK FAILED",
            Error::Nack => "REGISTER REJECTED",
            Error::Ack => "INVALID ACK BYTE",
            Error::NotScp => "TYPE NOT PROGRAMMABLE",
            Error::NotFcp => "NO FCP VOLTAGES",
        },
    }
}

/// The SCP card's one line, laid out as a PPS or AVS range: the voltage
/// range, the maximum current and, only when the adapter reports it, the
/// rated power. Power is never derived from the voltage and current maxima,
/// which need not be available together. Steps are on Charger info.
fn scp_range(cap: scp_protocol::Capabilities) -> Label {
    let line = Label::new()
        .hundredths(u32::from(cap.min_mv))
        .push(b"-")
        .hundredths(u32::from(cap.max_mv))
        .push(b"V  ")
        .hundredths(u32::from(cap.max_ma))
        .push(b"A");
    // Whole watts as EPR AVS shows them; a tenth only when there is one.
    match cap.max_power_dw {
        Some(dw) if dw % 10 == 0 => line.push(b"  ").fixed(u32::from(dw / 10), 0).push(b"W"),
        Some(dw) => line.push(b"  ").fixed(u32::from(dw), 1).push(b"W"),
        None => line,
    }
}

pub fn scp_charger_lines(
    findings: Option<ScpFindings>,
    status: Option<crate::scp_drive::Status>,
) -> Lines {
    let mut rows = [[b' '; 24]; 8];
    let mut row = 0;
    let mut line = |text: &[u8]| {
        if let Some(target) = rows.get_mut(row) {
            for (cell, &byte) in target.iter_mut().zip(text) {
                *cell = byte;
            }
            row += 1;
        }
    };
    line(b"SUPERCHARGE (SCP)");
    match findings {
        Some(ScpFindings::Confirmed) => {
            if let Some(crate::scp_drive::Status::Detected(adapter)) = status {
                line(match adapter.modes {
                    scp_protocol::Modes::LowVoltage => b"SCP DETECTED (LVC)",
                    scp_protocol::Modes::HighVoltage => b"SCP DETECTED (HVC)",
                    scp_protocol::Modes::Both => b"SCP DETECTED (LVC+HVC)",
                });
                if let Some(cap) = adapter.capabilities {
                    let vout = Label::new()
                        .push(b"VOUT: ")
                        .fixed(u32::from(cap.min_mv) / 100, 1)
                        .push(b"-")
                        .fixed(u32::from(cap.max_mv) / 100, 1)
                        .push(b"V");
                    line(vout.as_str().as_bytes());
                    let iout = Label::new()
                        .push(b"IOUT: ")
                        .hundredths(u32::from(cap.min_ma))
                        .push(b"-")
                        .hundredths(u32::from(cap.max_ma))
                        .push(b"A");
                    line(iout.as_str().as_bytes());
                    if let (Some(vstep), Some(istep)) = (cap.vstep_mv, cap.istep_ma) {
                        let step = Label::new()
                            .push(b"STEP: ")
                            .fixed(u32::from(vstep), 0)
                            .push(b"MV / ")
                            .fixed(u32::from(istep), 0)
                            .push(b"MA");
                        line(step.as_str().as_bytes());
                    }
                    if let Some(dw) = cap.max_power_dw {
                        let power = Label::new()
                            .push(b"MAX POWER: ")
                            .fixed(u32::from(dw), 1)
                            .push(b"W");
                        line(power.as_str().as_bytes());
                    }
                } else {
                    line(b"TYPE REGISTER VERIFIED");
                    line(b"NO OUTPUT REQUESTED");
                }
            } else {
                line(b"SCP DETECTED");
                line(b"TYPE REGISTER VERIFIED");
                line(b"NO OUTPUT REQUESTED");
            }
        }
        Some(ScpFindings::Unverified) => {
            line(b"SCP NOT VERIFIED");
            if let Some(crate::scp_drive::Status::Failed(error)) = status {
                line(scp_error(error).as_bytes());
            }
            line(b"SUPPORT NOT DETERMINED");
        }
        Some(ScpFindings::Absent) => {
            line(b"NO SCP");
            if let Some(crate::scp_drive::Status::Failed(error)) = status {
                line(scp_error(error).as_bytes());
            }
        }
        None => {
            line(b"NOT TESTED YET");
        }
    }
    rows
}

/// The charger page's FCP section, after SCP's and laid out the same: the
/// test's findings, with the voltages and the reason when this boot ran it.
pub fn fcp_charger_lines(
    findings: Option<FcpFindings>,
    adapter: Option<scp_protocol::fcp::Adapter>,
    status: Option<crate::scp_drive::FcpStatus>,
) -> Lines {
    let mut rows = [[b' '; 24]; 8];
    let mut row = 0;
    let mut line = |text: &[u8]| {
        if let Some(target) = rows.get_mut(row) {
            for (cell, &byte) in target.iter_mut().zip(text) {
                *cell = byte;
            }
            row += 1;
        }
    };
    line(b"FAST CHARGE (FCP)");
    let failure = match status {
        Some(crate::scp_drive::FcpStatus::Failed(error)) => Some(scp_error(error)),
        _ => None,
    };
    match findings {
        Some(FcpFindings::Detected {
            max_mv,
            max_power_mw,
        }) => {
            line(b"FCP DETECTED");
            let vout = match adapter {
                Some(adapter) => adapter
                    .levels()
                    .iter()
                    .enumerate()
                    .fold(Label::new().push(b"VOUT: "), |label, (index, &mv)| {
                        let label = if index > 0 { label.push(b" / ") } else { label };
                        label.fixed(u32::from(mv) / 100, 1)
                    })
                    .push(b"V"),
                None => Label::new()
                    .push(b"VOUT: UP TO ")
                    .fixed(u32::from(max_mv) / 100, 1)
                    .push(b"V"),
            };
            line(vout.as_str().as_bytes());
            if let Some(mw) = max_power_mw {
                let power = Label::new()
                    .push(b"MAX POWER: ")
                    .fixed(mw / 100, 1)
                    .push(b"W");
                line(power.as_str().as_bytes());
            }
        }
        Some(FcpFindings::Unverified) => {
            line(b"FCP NOT VERIFIED");
            if let Some(failure) = failure {
                line(failure.as_bytes());
            }
            line(b"SUPPORT NOT DETERMINED");
        }
        Some(FcpFindings::Absent) => {
            line(b"NO FCP");
            if let Some(failure) = failure {
                line(failure.as_bytes());
            }
        }
        None => line(b"NOT TESTED YET"),
    }
    line(b"HOLD OK: FULL RESCAN");
    rows
}

pub struct Scp {
    pub probing: bool,
    pub mode: QcMode,
    pub findings: Option<ScpFindings>,
    pub status: Option<crate::scp_drive::Status>,
    pub selection: Option<firmware_core::scp::Selection>,
    pub focus: Option<PpsFocus>,
    pub active: bool,
    pub reading: Option<Reading>,
}

pub fn draw_scp<D: DrawTarget<Color = Rgb565>>(target: &mut D, header: &Header<'_>, scp: &Scp) {
    let Scp {
        probing,
        mode,
        findings,
        status,
        ..
    } = *scp;
    draw_header(target, header);
    if mode == QcMode::Pd {
        notice(
            target,
            "PD MODE",
            "PD must stop before SCP detection",
            GREEN,
        );
        hint(target, header, "< > tabs  OK switch to SCP detection");
        return;
    }
    if !probing && let Some(selection) = scp.selection {
        let output = selection.output();
        let cap = selection.adapter().capabilities;
        let (volts_box, amps_box) = range_card(target, "SCP RANGE", SCP_ACCENT, cap.map(scp_range));
        value_box(
            target,
            volts_box,
            "SET VOLTAGE",
            Label::new().hundredths(u32::from(output.millivolts())),
            "V",
            VOLTS,
            scp.focus == Some(PpsFocus::Voltage),
        );
        value_box(
            target,
            amps_box,
            "CURRENT LIMIT",
            Label::new().hundredths(u32::from(output.milliamps())),
            "A",
            AMPS,
            scp.focus == Some(PpsFocus::Current),
        );
        let live = scp.reading.map(|reading| {
            Label::new()
                .hundredths(reading.mv)
                .push(b"V  ")
                .hundredths(reading.ma.unsigned_abs())
                .push(b"A")
        });
        let (caption, color) = match status {
            Some(crate::scp_drive::Status::Output { applied: true, .. }) => ("ACTIVE", GREEN),
            Some(crate::scp_drive::Status::Output { applied: false, .. }) => ("SETTING", ORANGE),
            Some(crate::scp_drive::Status::Stopping) => ("STOPPING", ORANGE),
            Some(crate::scp_drive::Status::Failed(_)) => ("FAILED", MAGENTA),
            _ if scp.active => ("ENTERING", ORANGE),
            _ => ("", MUTED),
        };
        let failure = match status {
            Some(crate::scp_drive::Status::Failed(error)) => Some(scp_error(error)),
            _ => None,
        };
        output_card(
            target,
            "OUTPUT",
            failure.unwrap_or_else(|| live.as_ref().map_or("--.--V  -.--A", Label::as_str)),
            (caption, color),
        );
        hint(
            target,
            header,
            match scp.focus {
                Some(PpsFocus::Voltage) => "< > volts  OK next: current",
                Some(PpsFocus::Current) => "< > amps  OK request",
                None if scp.active => "< > tabs  OK adjust  HOLD OK stop",
                None => "< > tabs  OK adjust  HOLD OK rescan",
            },
        );
        return;
    }
    if !probing
        && findings == Some(ScpFindings::Confirmed)
        && let Some(crate::scp_drive::Status::Detected(adapter)) = status
    {
        let lines = match adapter.capabilities {
            Some(cap) => scp_range(cap),
            None => Label::new().push(b"ratings not reported"),
        };
        let (volts_box, amps_box) = range_card(target, "SCP RANGE", SCP_ACCENT, [lines]);
        let (volts, amps) = match adapter.capabilities {
            Some(cap) => (
                Label::new().hundredths(u32::from(cap.max_mv)),
                Label::new().hundredths(u32::from(cap.max_ma)),
            ),
            None => (Label::new().push(b"--.--"), Label::new().push(b"-.--")),
        };
        value_box(target, volts_box, "MAX VOLTAGE", volts, "V", VOLTS, false);
        value_box(target, amps_box, "MAX CURRENT", amps, "A", AMPS, false);
        let mode_text = match adapter.modes {
            scp_protocol::Modes::LowVoltage => "LVC",
            scp_protocol::Modes::HighVoltage => "HVC",
            scp_protocol::Modes::Both => "LVC + HVC",
        };
        output_card(target, "MODE", mode_text, ("DETECTED", GREEN));
        hint(target, header, "< > tabs  OK rescan SCP");
        return;
    }
    if probing {
        notice(target, "TESTING SCP", "adapter identification", ORANGE);
    } else {
        notice(
            target,
            match findings {
                Some(ScpFindings::Confirmed) => "SCP DETECTED",
                Some(ScpFindings::Unverified) => "SCP NOT VERIFIED",
                Some(ScpFindings::Absent) => "NO SCP",
                None => "SCP NOT TESTED",
            },
            if let Some(crate::scp_drive::Status::Failed(error)) = status {
                scp_error(error)
            } else {
                "no output requested"
            },
            SCP_ACCENT,
        );
    }
    hint(
        target,
        header,
        if probing {
            "BACK stop"
        } else {
            "< > tabs  OK test SCP"
        },
    );
}

/// Everything the FCP tab shows.
pub struct Fcp {
    pub mode: QcMode,
    pub probing: bool,
    /// The row being chosen, while the choice is open.
    pub preview: Option<usize>,
    pub adapter: Option<scp_protocol::fcp::Adapter>,
    /// The voltage asked of the adapter in mV, kept while other tabs show.
    pub request: Option<u16>,
    /// The current attempt's latest status.
    pub status: Option<crate::scp_drive::FcpStatus>,
}

/// The FCP tab: the probe's result until it finds an adapter, then one row
/// per voltage the adapter lists and an off row, laid out as the Quick
/// Charge 2.0 tab.
pub fn draw_fcp<D: DrawTarget<Color = Rgb565>>(target: &mut D, header: &Header<'_>, page: &Fcp) {
    use crate::scp_drive::FcpStatus;
    let &Fcp {
        mode,
        probing,
        preview,
        adapter,
        request,
        status,
    } = page;
    draw_header(target, header);
    if mode == QcMode::Pd {
        notice(
            target,
            "PD MODE",
            "PD must stop before FCP detection",
            GREEN,
        );
        hint(target, header, "< > tabs  OK switch to FCP detection");
        return;
    }
    let failure = match status {
        Some(FcpStatus::Failed(error)) => Some(error),
        _ => None,
    };
    if probing {
        notice(target, "TESTING FCP", "adapter identification", ORANGE);
        hint(target, header, "BACK stop");
        return;
    }
    let Some(adapter) = adapter else {
        notice(
            target,
            match failure {
                Some(error) if error.absent() => "NO FCP",
                Some(_) => "FCP NOT VERIFIED",
                None => "FCP NOT TESTED",
            },
            failure.map_or("no output requested", scp_error),
            FCP_ACCENT,
        );
        hint(target, header, "< > tabs  OK test FCP");
        return;
    };
    let pill = match status {
        Some(FcpStatus::Output { applied: true, .. }) => ("ACTIVE", GREEN),
        Some(FcpStatus::Output { applied: false, .. }) => ("SETTING", ORANGE),
        Some(FcpStatus::Stopping) => ("STOPPING", ORANGE),
        Some(FcpStatus::Failed(_)) => ("FAILED", MAGENTA),
        _ => ("WAIT", ORANGE),
    };
    let levels = adapter.levels();
    for (row, &mv) in levels.iter().enumerate() {
        // The rated power is the adapter's, shown once at its top voltage.
        let power = adapter
            .max_power_mw
            .filter(|_| row + 1 == levels.len())
            .map(|mw| match mw % 1_000 {
                0 => Label::new().fixed(mw / 1_000, 0),
                _ => Label::new().fixed(mw / 100, 1),
            })
            .map(|watts| watts.push(b"W rated"));
        selector_row(
            target,
            row,
            Label::new().volts(mv).as_str(),
            level_color(mv),
            [power.as_ref().map_or("", Label::as_str), ""],
            preview == Some(row),
            (request == Some(mv)).then_some(pill),
        );
    }
    selector_row(
        target,
        levels.len(),
        "OFF",
        WHITE,
        ["plain USB", ""],
        preview == Some(levels.len()),
        None,
    );
    let hint_text = match (preview, failure, request) {
        (Some(_), ..) => "< > choose  OK request  BACK cancel",
        (None, Some(error), _) => scp_error(error),
        (None, None, None) => "< > tabs  OK choose  HOLD OK rescan",
        (None, None, Some(_)) => "< > tabs  OK change  HOLD OK stop",
    };
    hint(target, header, hint_text);
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
        (None, None) => qc_start_hint(qc, "< > tabs  OK choose a voltage"),
        (None, Some(_)) => qc_running_hint(status, "< > tabs  OK change"),
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
            "BACK stop  up to 20V on the output"
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
        notice(
            target,
            "NO QC 3.0",
            "the charger ignored the steps",
            QC3_ACCENT,
        );
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
    let (volts_box, amps_box) =
        range_card(target, "STEPS", QC3_ACCENT, ranges.into_iter().flatten());

    let set = match target_mv {
        Some(mv) => Label::new().hundredths(u32::from(mv)),
        None => Label::new().push(b"--.--"),
    };
    value_box(target, volts_box, "SET VOLTAGE", set, "V", VOLTS, adjusting);
    value_box(
        target,
        amps_box,
        "VBUS",
        volts_or_blank(reading.map(|r| r.mv)),
        "V",
        VOLTS,
        false,
    );

    let readback = Label::new()
        .push(b"D+ ")
        .push(line_volts(lines, |l| l.dp_mv).as_str().as_bytes())
        .push(b"V  D- ")
        .push(line_volts(lines, |l| l.dm_mv).as_str().as_bytes())
        .push(b"V");
    let state = match status {
        Status::Entering => ("WAIT", ORANGE),
        Status::Failed(Failure::NotDcp) => ("NO DCP", MAGENTA),
        Status::Failed(Failure::NoEntry) => ("NO QC", MAGENTA),
        Status::Stepping(mv) if Some(mv) == target_mv => ("ACTIVE", GREEN),
        Status::Stepping(_) => ("STEPPING", ORANGE),
        _ => ("", MUTED),
    };
    output_card(target, "LINES", readback.as_str(), state);
    hint(
        target,
        header,
        match target_mv {
            _ if adjusting => "< > 0.2V  OK done",
            None => qc_start_hint(qc, "< > tabs  OK start at 5V"),
            Some(_) => qc_running_hint(status, "< > tabs  OK adjust"),
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
            line(b"(QC, SCP, FCP AND PD)");
        }
        None => {
            line(b"QUICK CHARGE");
            line(b"NOT TESTED YET");
            line(b"HOLD OK: SCAN QC/SCP/FCP");
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
        rounded_outline(target, area, CORNER, color);
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
        rounded_outline(target, pill, Size::new(6, 6), color);
        text(
            target,
            &PLAIN,
            label,
            Point::new(pill.center().x, baseline),
            HorizontalAlignment::Center,
            color,
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
        notice(target, "NO PD CHARGER", "plug in a PD source", PD_ACCENT);
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
            "< > tabs  OK select/scroll"
        } else {
            "< > tabs  OK select a profile"
        }),
    );
}

/// Hint on the selector pages before the sink session has an offer.
const VIEW_ONLY: &str = "< > tabs  waiting for the charger";

/// PPS, AVS and QC 3.0 pages: the ranges card on top, as tall as its one
/// or two `PLAIN` lines (sitting like values, 2 px apart), the value boxes
/// filling the rest, and the output card along the bottom, shaped like the
/// overview's protocol strip.
const RANGE_LINES: usize = 2;
const RANGE_PITCH: i32 = PLAIN_INK + 2;
const RANGE_BASELINE: i32 = OUTLINED_TOP + STRIP_VALUE;
const OUTPUT_CARD: Rectangle = Rectangle::new(
    Point::new(4, CONTENT_BOTTOM + 1 - STRIP_HEIGHT as i32),
    Size::new(232, STRIP_HEIGHT),
);
const OUTPUT_BASELINE: i32 = OUTPUT_CARD.top_left.y + STRIP_VALUE;
/// Text inside those cards starts at their padding.
const CARD_TEXT_LEFT: i32 = 4 + PAD;
const CARD_TEXT_RIGHT: i32 = 4 + 232 - PAD;

/// The ranges card, outlined in the page's `color`, around up to two lines;
/// returns the voltage and current boxes' areas, filling the space down to
/// the output card.
fn range_card<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    label: &str,
    color: Rgb565,
    lines: impl IntoIterator<Item = Label>,
) -> (Rectangle, Rectangle) {
    let mut shown: [Option<Label>; RANGE_LINES] = [None, None];
    for (slot, line) in shown.iter_mut().zip(lines) {
        *slot = Some(line);
    }
    let count = shown.iter().flatten().count().max(1) as i32;
    let height = STRIP_VALUE + (count - 1) * RANGE_PITCH + OVERVIEW_PAD + 1;
    let card = Rectangle::new(Point::new(4, OUTLINED_TOP), Size::new(232, height as u32));
    outlined_card(target, card, label, color, 0);
    for (index, line) in shown.iter().flatten().enumerate() {
        text(
            target,
            &PLAIN,
            line.as_str(),
            Point::new(CARD_TEXT_LEFT, RANGE_BASELINE + RANGE_PITCH * index as i32),
            HorizontalAlignment::Left,
            color,
        );
    }
    let top = OUTLINED_TOP + height + OUTLINED_SPACING;
    let size = Size::new(
        114,
        (OUTPUT_CARD.top_left.y - OUTLINED_SPACING - top) as u32,
    );
    (
        Rectangle::new(Point::new(4, top), size),
        Rectangle::new(Point::new(122, top), size),
    )
}

/// The output card: `line` on the left, `state` on the right, the card
/// outlined in the state's colour.
fn output_card<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    label: &str,
    line: &str,
    (state, color): (&str, Rgb565),
) {
    outlined_card(target, OUTPUT_CARD, label, color, 0);
    text(
        target,
        &PLAIN,
        line,
        Point::new(CARD_TEXT_LEFT, OUTPUT_BASELINE),
        HorizontalAlignment::Left,
        WHITE,
    );
    text(
        target,
        &PLAIN,
        state,
        Point::new(CARD_TEXT_RIGHT, OUTPUT_BASELINE),
        HorizontalAlignment::Right,
        color,
    );
}

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
            PPS_ACCENT,
        );
        return;
    }
    let (volts_box, amps_box) = range_card(
        target,
        "PPS RANGE",
        PPS_ACCENT,
        ranges.iter().map(|range| {
            Label::new()
                .hundredths(u32::from(range.min_mv))
                .push(b"-")
                .hundredths(u32::from(range.max_mv))
                .push(b"V  ")
                .hundredths(u32::from(range.max_ma))
                .push(b"A")
        }),
    );

    let set = offer.preview.filter(|preview| preview.kind == Kind::Pps);
    let (volts, amps) = set_values(set);
    value_box(
        target,
        volts_box,
        "SET VOLTAGE",
        volts,
        "V",
        VOLTS,
        focus == Some(PpsFocus::Voltage),
    );
    value_box(
        target,
        amps_box,
        "CURRENT LIMIT",
        amps,
        "A",
        AMPS,
        focus == Some(PpsFocus::Current),
    );
    supply_status(target, offer, set, live);
    hint(
        target,
        header,
        offer.pd_notice.unwrap_or(match focus {
            _ if !offer.session => VIEW_ONLY,
            Some(PpsFocus::Voltage) => "< > volts  OK next: current",
            Some(PpsFocus::Current) => "< > amps  OK request",
            None => "< > tabs  OK adjust PPS",
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
        notice(
            target,
            "NO AVS",
            "this charger offers no AVS range",
            AVS_ACCENT,
        );
        return;
    }
    let line = |range: &AvsRange| {
        let line = Label::new()
            .hundredths(u32::from(range.min_mv))
            .push(b"-")
            .hundredths(u32::from(range.max_mv))
            .push(b"V  ")
            .hundredths(u32::from(range.ma_15v))
            .push(b"A");
        // EPR AVS states its power instead of a current.
        if let Some(pdp) = range.pdp_w {
            Label::new()
                .hundredths(u32::from(range.min_mv))
                .push(b"-")
                .hundredths(u32::from(range.max_mv))
                .push(b"V  ")
                .fixed(u32::from(pdp), 0)
                .push(b"W  EPR")
        } else if range.ma_20v == 0 {
            line
        } else {
            line.push(b" / ")
                .hundredths(u32::from(range.ma_20v))
                .push(b"A")
        }
    };
    let (volts_box, amps_box) =
        range_card(target, "AVS RANGE", AVS_ACCENT, ranges.iter().map(line));
    let set = offer.preview.filter(|preview| preview.kind == Kind::Avs);
    let (volts, amps) = set_values(set);
    value_box(
        target,
        volts_box,
        "SET VOLTAGE",
        volts,
        "V",
        VOLTS,
        adjusting,
    );
    value_box(target, amps_box, "CURRENT", amps, "A", AMPS, false);
    supply_status(target, offer, set, live);
    hint(
        target,
        header,
        offer.pd_notice.unwrap_or(if !offer.session {
            VIEW_ONLY
        } else if adjusting {
            "< > volts  OK request"
        } else {
            "< > tabs  OK adjust AVS"
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

/// One PPS/AVS/QC 3.0 value box: an outlined card, dim until Left/Right
/// change it, then outlined in its colour with arrows.
fn value_box<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    area: Rectangle,
    label: &str,
    value: Label,
    unit: &str,
    color: Rgb565,
    focused: bool,
) {
    labelled_outline(
        target,
        area,
        label,
        color,
        if focused { color } else { RULE },
    );
    value_with_unit(
        target,
        &READING,
        &value,
        unit,
        area.top_left + Point::new(PAD, outlined_value(area.size.height, READING_INK)),
        if focused { WHITE } else { color },
    );
    if focused {
        arrows(target, area, color);
    }
}

/// The output card under the PPS/AVS boxes: the live output and whether
/// the set level is the contract in force.
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
    let line = match live {
        Some(live) => Label::new()
            .hundredths(live.mv)
            .push(b"V  ")
            .hundredths(live.ma.unsigned_abs())
            .push(b"A"),
        None => Label::new().push(b"--.--V  -.--A"),
    };
    output_card(target, "OUTPUT", line.as_str(), state);
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
/// Settings row highlight and edit frame height.
const SETTINGS_ROW_HEIGHT: u32 = 17;

/// The frame round a settings value while Left/Right change it.
fn value_frame<D: DrawTarget<Color = Rgb565>>(target: &mut D, left: i32, baseline: i32) {
    rounded_outline(
        target,
        Rectangle::new(
            Point::new(left, baseline - 13),
            Size::new(78, SETTINGS_ROW_HEIGHT),
        ),
        CORNER,
        YELLOW,
    );
}
/// Baseline of the firmware version line, below the rows and above the hint;
/// the energy page's status line shares it.
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
    // The reset hold's progress over the calibration row, out of
    // [`RESET_STEPS`].
    reset_hold: u8,
) {
    draw_header(target, header);
    for (index, item) in Item::ALL.into_iter().enumerate() {
        let baseline = SETTINGS_BASELINE + index as i32 * SETTINGS_PITCH;
        let selected = item == focus.row();
        if selected && item == Item::Calibration {
            sweep(
                target,
                Rectangle::new(
                    Point::new(4, baseline - 13),
                    Size::new(232, SETTINGS_ROW_HEIGHT),
                ),
                reset_hold,
            );
        }
        if selected {
            // Dim while its value is being changed, which then takes the
            // highlight.
            rounded_outline(
                target,
                Rectangle::new(
                    Point::new(4, baseline - 13),
                    Size::new(232, SETTINGS_ROW_HEIGHT),
                ),
                CORNER,
                if focus == SettingsFocus::Row(item) {
                    YELLOW
                } else {
                    RULE
                },
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
            Item::Calibration => "CALIBRATION",
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
                    value_frame(target, 154, baseline);
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
                    value_frame(target, 154, baseline);
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
            Item::Calibration => {
                text(
                    target,
                    &ROW,
                    if settings.calibration == Calibration::default() {
                        "FACTORY"
                    } else {
                        "CUSTOM"
                    },
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
            SettingsFocus::Brightness => "< > dimmer/brighter  OK done",
            SettingsFocus::Rotation => "< > auto/normal/flipped  OK done",
            SettingsFocus::EnergyMemory => "< > interval  OK done (shorter wears)",
            SettingsFocus::Row(Item::Calibration) => "OK calibrate  hold OK factory reset",
            _ => "< > choose  OK change  BACK exit",
        },
    );
}

/// Calibration page: the step's title and what to do, then the steps.
const CALIBRATION_TITLE: i32 = 40;
const CALIBRATION_DETAIL: i32 = 56;
/// The entry box, under the instruction, and the live reading under it.
const CALIBRATION_BOX: Rectangle =
    Rectangle::new(Point::new(4, 64), Size::new(150, OVERVIEW_HEIGHT));
const CALIBRATION_LIVE: i32 = 104;

/// The calibration steps: the zero step shows the live current; the
/// current and voltage steps an entry box starting at this meter's
/// average, with the live reading under it. A refusal shows in orange.
pub fn draw_calibration<D: DrawTarget<Color = Rgb565>>(
    target: &mut D,
    header: &Header<'_>,
    wizard: &Wizard,
    live: Option<Reading>,
) {
    draw_header(target, header);
    let (title, detail, (unit, color), hint_text) = match wizard.step {
        Step::Zero => (
            "1/3  ZERO CURRENT",
            "Unplug the load, keep the power on",
            ("A", AMPS),
            "hold OK zero  OK skip  BACK cancel",
        ),
        Step::Entry {
            quantity: Quantity::Current,
            ..
        } => (
            "2/3  CURRENT",
            "Steady load: enter your meter's amps",
            ("A", AMPS),
            "< > adjust  OK next  BACK cancel",
        ),
        Step::Entry {
            quantity: Quantity::Voltage,
            ..
        } => (
            "3/3  VOLTAGE",
            "Enter your meter's volts",
            ("V", VOLTS),
            "< > adjust  OK save  BACK cancel",
        ),
    };
    text(
        target,
        &BOLD,
        title,
        Point::new(LEFT, CALIBRATION_TITLE),
        HorizontalAlignment::Left,
        YELLOW,
    );
    text(
        target,
        &PLAIN,
        detail,
        Point::new(LEFT, CALIBRATION_DETAIL),
        HorizontalAlignment::Left,
        WHITE,
    );
    let live_value = |reading: Reading| match wizard.step {
        Step::Entry {
            quantity: Quantity::Voltage,
            ..
        } => reading.mv,
        _ => reading.ma.unsigned_abs(),
    };
    let entered = match wizard.step.entered() {
        Some(value) => Label::new().fixed(value, 3),
        None if wizard.step == Step::Zero => match live {
            Some(reading) => Label::new().fixed(live_value(reading), 3),
            None => Label::new().push(b"-.---"),
        },
        None => Label::new().push(b"-.---"),
    };
    let editing = wizard.step != Step::Zero;
    value_box(
        target,
        CALIBRATION_BOX,
        if editing { "YOUR METER" } else { "THIS METER" },
        entered,
        unit,
        color,
        editing,
    );
    if editing {
        let now = match live {
            Some(reading) => Label::new()
                .push(b"This meter now ")
                .fixed(live_value(reading), 3)
                .push(unit.as_bytes()),
            None => Label::new()
                .push(b"This meter now -.---")
                .push(unit.as_bytes()),
        };
        text(
            target,
            &PLAIN,
            now.as_str(),
            Point::new(LEFT, CALIBRATION_LIVE),
            HorizontalAlignment::Left,
            MUTED,
        );
    }
    if let Some(refusal) = wizard.refusal {
        text(
            target,
            &PLAIN,
            refusal.message(),
            Point::new(LEFT, VERSION_BASELINE),
            HorizontalAlignment::Left,
            ORANGE,
        );
    }
    hint(target, header, hint_text);
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
        value_frame(target, left - 6, baseline);
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

/// The most characters a detail row can show: rows start 6 px right of
/// `LEFT`, and the list font advances 6 px per character (measured), so 36
/// end at x = 230, clear of the scrollbar at `RIGHT + 1`.
pub const WIDE_COLUMNS: usize = 36;

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
/// row `top`, with a scrollbar lit while `scrolling`. Rows are [`Lines`]
/// wide, or up to [`WIDE_COLUMNS`].
pub fn draw_detail<D: DrawTarget<Color = Rgb565>, const WIDTH: usize>(
    target: &mut D,
    header: &Header<'_>,
    page: Detail,
    sections: impl IntoIterator<Item = [[u8; WIDTH]; 8]>,
    top: u16,
    scrolling: bool,
) -> ListView {
    const { assert!(WIDTH <= WIDE_COLUMNS, "detail rows wider than the screen") };
    let (_, accent) = tab_style(Tab::detail(page), false);
    draw_header(target, header);
    let mut rows = 0u16;
    let mut view = ListView::default();
    for (section, lines) in sections.into_iter().enumerate() {
        for (index, line) in lines.iter().enumerate() {
            let mut printable = [b' '; WIDTH];
            for (cell, &byte) in printable.iter_mut().zip(line) {
                if byte.is_ascii_graphic() {
                    *cell = byte;
                }
            }
            let content = core::str::from_utf8(&printable)
                .unwrap_or("")
                .trim_ascii_end();
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
    // Only the sink's PD session page scans; passthrough only listens.
    let sink = matches!(
        header,
        Header::Tabs {
            passthrough: false,
            ..
        }
    );
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
            (true, _) => "< > scroll  OK done",
            (false, true) => match page {
                Detail::Charger => "< > tabs  OK scroll  hold OK scan",
                Detail::Pd if sink => "< > tabs  OK scroll  hold OK scan",
                Detail::Cable => "< > tabs  OK scroll  hold OK read",
                _ => "< > tabs  OK scroll",
            },
            (false, false) => match page {
                Detail::Charger => "< > tabs  hold OK scan",
                Detail::Pd if sink => "< > tabs  hold OK scan",
                Detail::Cable => "< > tabs  hold OK read",
                _ => "< > tabs  BACK groups",
            },
        },
    );
    view
}
