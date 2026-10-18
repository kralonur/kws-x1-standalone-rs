//! Button sampling and debouncing (pure logic).
//!
//! The original firmware configures four button inputs as active-low with
//! pull-ups: PB2, PB10, PH0 and PC14, each treating a low level as pressed.
//! They are the Left, Right, OK and Back buttons, as identified on
//! the device.

use crate::calibration::{Calibration, Mean, Wizard};
use crate::power_history::{GraphTrace, GraphWindow};
use crate::ripple::{Scale as RippleScale, Timebase as RippleTimebase};
use crate::settings::{FcpFindings, Item, NextBoot, QcFindings, ScpFindings, Screen, Settings};
use qc_protocol::detect::Report;
use qc_protocol::qc2::Voltage;
use qc_protocol::qc3::{MIN_MV, STEP_MV};

pub const COUNT: usize = 4;
/// Pin identifiers, matching the original configuration.
pub const PINS: [&str; COUNT] = ["PB2", "PB10", "PH0", "PC14"];

/// Number of consecutive identical samples required to change a stable state.
pub const DEFAULT_THRESHOLD: u8 = 3;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Debounced {
    stable: u8,
    counters: [u8; COUNT],
}

impl Debounced {
    /// Feed one raw sample (bit set = pressed) and return the stable pressed mask.
    ///
    /// A state change needs `threshold` consecutive samples, so a single glitch
    /// cannot flip a reported button.
    pub fn update(&mut self, raw_pressed: u8, threshold: u8) -> u8 {
        let threshold = threshold.max(1);
        for index in 0..COUNT {
            let bit = 1u8 << index;
            let sampled = raw_pressed & bit != 0;
            let stable = self.stable & bit != 0;
            if sampled == stable {
                self.counters[index] = 0;
                continue;
            }
            self.counters[index] = self.counters[index].saturating_add(1);
            if self.counters[index] >= threshold {
                self.counters[index] = 0;
                if sampled {
                    self.stable |= bit;
                } else {
                    self.stable &= !bit;
                }
            }
        }
        self.stable
    }

    pub fn stable(&self) -> u8 {
        self.stable
    }
}

/// Button bits of the stable pressed mask (bit `n` is `PINS[n]`).
pub const LEFT: u8 = 1 << 0;
pub const RIGHT: u8 = 1 << 1;
pub const OK: u8 = 1 << 2;
pub const BACK: u8 = 1 << 3;

/// `raw` with Left and Right swapped, for a screen turned 180 degrees: the
/// button on the viewer's left is then the Right one.
pub fn swap_arrows(raw: u8) -> u8 {
    let swapped = |from, to| if raw & from != 0 { to } else { 0 };
    raw & !(LEFT | RIGHT) | swapped(LEFT, RIGHT) | swapped(RIGHT, LEFT)
}

/// Button sampling period in milliseconds; the hold times below are counted
/// in these samples.
pub const POLL_MS: u32 = 10;
/// A long OK press (cable query on the cable page).
const LONG_OK_SAMPLES: u32 = 500 / POLL_MS;
/// Holding OK on the overview, graph or energy tab resets what it shows: the
/// sweep shows after a tap's length, so a tap never flashes it, then fills
/// for a second; the hold itself is the confirmation. Chosen, not measured.
const RESET_DELAY_SAMPLES: u32 = 200 / POLL_MS;
const RESET_SWEEP_SAMPLES: u32 = 1000 / POLL_MS;
/// The sweep's progress steps; each redraws the page, so they stay coarse.
pub const RESET_STEPS: u8 = 12;
/// A held Left or Right repeats after this delay, at this period.
const REPEAT_DELAY_SAMPLES: u32 = 500 / POLL_MS;
const REPEAT_SAMPLES: u32 = 100 / POLL_MS;
/// PPS steps per press and per repeat: 20 mV is the PPS voltage resolution
/// and 50 mA the PPS current resolution (USB PD Table 6.21).
const PPS_MV_STEP: i16 = 20;
const PPS_MV_REPEAT: i16 = 100;
const PPS_MA_STEP: i16 = 50;
const PPS_MA_REPEAT: i16 = 250;
/// SPR AVS voltage steps per press and per repeat: 100 mV is its effective
/// resolution (USB PD Table 6.22).
const AVS_MV_STEP: i16 = 100;
const AVS_MV_REPEAT: i16 = 500;

/// The screen and what its Left, Right and OK act on, one level at a time:
/// Left/Right move between groups, OK goes into a group's tabs, OK again
/// opens the shown tab, and Back goes up a level. Each level carries only its
/// own state, so e.g. a PPS edit on a closed tab cannot exist. Back on the
/// overview opens the settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Page {
    /// Left/Right move between groups; the group's tab is shown.
    Groups(Tab),
    /// Left/Right move between the shown group's tabs.
    Tabs(Tab),
    /// Left/Right and OK act on the tab itself.
    Open(Open),
    /// Opened by Back on the overview.
    Settings(SettingsFocus),
}

impl Default for Page {
    fn default() -> Self {
        Self::Groups(Tab::Overview)
    }
}

impl Page {
    /// Where a boot opens for its saved `screen`: that tab, or, for a Quick
    /// Charge tab in a PD-silent boot, its probe, which runs at once.
    pub fn resume(screen: Screen, silent: bool) -> Self {
        match Tab::from(screen) {
            Tab::Qc2 if silent => Self::Open(Open::Probe(QcPage::Qc2)),
            Tab::Qc3 if silent => Self::Open(Open::Probe(QcPage::Qc3)),
            tab => Self::Tabs(tab),
        }
    }

    /// A held Left/Right repeats wherever it changes a value or scrolls.
    /// Moving between groups or tabs and the capture toggle stay one step
    /// per press, so a hold never flies past a screen or flickers a toggle.
    fn repeats(self) -> bool {
        !matches!(
            self,
            Self::Groups(_) | Self::Tabs(_) | Self::Open(Open::Ripple(RippleFocus::Capture))
        )
    }
}

/// The page groups, in Left/Right order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Group {
    /// Live readings.
    Monitor,
    /// Fixed PD, PPS, AVS and Quick Charge requests.
    Trigger,
    /// The detail lists.
    Info,
}

/// The most tabs a group has.
pub const MAX_TABS: usize = 7;

impl Group {
    pub const ALL: [Self; 3] = [Self::Monitor, Self::Trigger, Self::Info];

    /// The group's tabs, in Left/Right order.
    const fn tabs(self) -> &'static [Tab] {
        const MONITOR: [Tab; 5] = [
            Tab::Overview,
            Tab::Lines,
            Tab::Energy,
            Tab::Graph,
            Tab::Ripple,
        ];
        const TRIGGER: [Tab; MAX_TABS] = [
            Tab::Fixed,
            Tab::Pps,
            Tab::Avs,
            Tab::Qc2,
            Tab::Qc3,
            Tab::Scp,
            Tab::Fcp,
        ];
        const INFO: [Tab; 4] = [
            Tab::detail(Detail::Pd),
            Tab::detail(Detail::Charger),
            Tab::detail(Detail::Cable),
            Tab::detail(Detail::System),
        ];
        match self {
            Self::Monitor => &MONITOR,
            Self::Trigger => &TRIGGER,
            Self::Info => &INFO,
        }
    }

    /// The group's tabs `passthrough` leaves, in Left/Right order.
    pub fn available(self, passthrough: bool) -> impl Iterator<Item = Tab> {
        self.tabs()
            .iter()
            .copied()
            .filter(move |tab| tab.available(passthrough))
    }

    /// Index among the groups with a tab `passthrough` leaves, and their
    /// count.
    pub fn position(self, passthrough: bool) -> (usize, usize) {
        let shown = |group: &&Self| group.available(passthrough).next().is_some();
        (
            Self::ALL[..self as usize].iter().filter(shown).count(),
            Self::ALL.iter().filter(shown).count(),
        )
    }
}

/// A page within its group, shown but not acted on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Tab {
    #[default]
    Overview,
    /// Recent power.
    Graph,
    /// D+, D−, CC1 and CC2 line voltages.
    Lines,
    /// Accumulated charge and energy.
    Energy,
    /// Uncalibrated ADC ripple capture.
    Ripple,
    /// Fixed PD profile selector.
    Fixed,
    /// PPS voltage and current selector.
    Pps,
    /// SPR AVS voltage selector.
    Avs,
    /// Quick Charge 2.0 fixed voltages and the probe.
    Qc2,
    /// Quick Charge 3.0 continuous mode.
    Qc3,
    /// SCP adapter identification and explicit output control.
    Scp,
    /// FCP adapter identification and its fixed output voltages.
    Fcp,
    /// A detail list shown from row `top`.
    Detail { page: Detail, top: u16 },
}

impl From<Tab> for Screen {
    /// The screen to come back to; a detail list comes back at its top. The
    /// graph's history does not outlast a restart, so it comes back as the
    /// overview.
    fn from(tab: Tab) -> Self {
        match tab {
            Tab::Overview | Tab::Graph => Self::Overview,
            Tab::Lines => Self::Lines,
            Tab::Energy => Self::Energy,
            Tab::Ripple => Self::Ripple,
            Tab::Fixed => Self::Fixed,
            Tab::Pps => Self::Pps,
            Tab::Avs => Self::Avs,
            Tab::Qc2 => Self::Qc2,
            Tab::Qc3 => Self::Qc3,
            Tab::Scp => Self::Scp,
            Tab::Fcp => Self::Fcp,
            Tab::Detail { page, .. } => match page {
                Detail::Pd => Self::PdDetails,
                Detail::Charger => Self::Charger,
                Detail::Cable => Self::Cable,
                Detail::System => Self::System,
            },
        }
    }
}

impl From<Screen> for Tab {
    fn from(screen: Screen) -> Self {
        match screen {
            Screen::Overview => Self::Overview,
            Screen::Lines => Self::Lines,
            Screen::Energy => Self::Energy,
            Screen::Ripple => Self::Ripple,
            Screen::Fixed => Self::Fixed,
            Screen::Pps => Self::Pps,
            Screen::Avs => Self::Avs,
            Screen::Qc2 => Self::Qc2,
            Screen::Qc3 => Self::Qc3,
            Screen::Scp => Self::Scp,
            Screen::Fcp => Self::Fcp,
            Screen::PdDetails => Self::detail(Detail::Pd),
            Screen::Charger => Self::detail(Detail::Charger),
            Screen::Cable => Self::detail(Detail::Cable),
            Screen::System => Self::detail(Detail::System),
        }
    }
}

impl Tab {
    /// A detail list at its top.
    pub const fn detail(page: Detail) -> Self {
        Self::Detail { page, top: 0 }
    }

    pub const fn group(self) -> Group {
        match self {
            Self::Overview | Self::Graph | Self::Lines | Self::Energy | Self::Ripple => {
                Group::Monitor
            }
            Self::Fixed | Self::Pps | Self::Avs | Self::Qc2 | Self::Qc3 | Self::Scp | Self::Fcp => {
                Group::Trigger
            }
            Self::Detail { .. } => Group::Info,
        }
    }

    /// The same tab as `other`, wherever either list is scrolled.
    pub fn same(self, other: Self) -> bool {
        match (self, other) {
            (Self::Detail { page, .. }, Self::Detail { page: other, .. }) => page == other,
            _ => self == other,
        }
    }

    /// Whether the tab has anything to show: in passthrough the meter only
    /// listens, so the selectors and the charger and cable lists, which need
    /// a sink session, are left out.
    pub fn available(self, passthrough: bool) -> bool {
        !passthrough
            || matches!(
                self,
                Self::Overview
                    | Self::Graph
                    | Self::Lines
                    | Self::Energy
                    | Self::Ripple
                    | Self::Detail {
                        page: Detail::Pd | Detail::System,
                        ..
                    }
            )
    }

    /// The available tab `forward` (Right) or back from this one in its
    /// group, wrapping; a new list starts at its top.
    fn next(self, forward: bool, passthrough: bool) -> Self {
        let tabs = self.group().tabs();
        let index = tabs.iter().position(|tab| tab.same(self)).unwrap_or(0);
        cycle_where(tabs, index, forward, |tab| tab.available(passthrough)).unwrap_or(self)
    }
}

/// An open tab and what Left/Right and OK do there. The overview and the
/// lines have nothing to open.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Open {
    /// Left/Right preview fixed profiles; OK requests one.
    Fixed,
    /// Left/Right change the PPS voltage or current.
    Pps(PpsFocus),
    /// Left/Right change the SPR AVS voltage.
    Avs,
    /// The Quick Charge probe runs; `page` then opens with what it found.
    Probe(QcPage),
    /// Left/Right preview the Quick Charge 2.0 voltages the charger takes,
    /// and off; OK requests one.
    Qc2(Qc2Choice),
    /// Left/Right move the Quick Charge 3.0 steps' target.
    Qc3,
    /// The read-only SCP probe runs until a result or Back.
    Scp,
    /// Preview SCP voltage, then current; a final OK applies the request.
    ScpSet(PpsFocus),
    /// The read-only FCP probe runs until a result or Back.
    Fcp,
    /// Left/Right preview the adapter's FCP voltages by index, then off;
    /// OK requests one.
    FcpSet(usize),
    /// Left/Right scroll the list, shown from row `top`.
    Detail { page: Detail, top: u16 },
    /// Left/Right change what the graph plots, or its window.
    Graph(GraphFocus),
    /// Left/Right change the focused ripple setting; OK focuses the next.
    Ripple(RippleFocus),
}

impl Open {
    /// The tab this is open on.
    pub const fn tab(self) -> Tab {
        match self {
            Self::Fixed => Tab::Fixed,
            Self::Pps(_) => Tab::Pps,
            Self::Avs => Tab::Avs,
            Self::Probe(page) => page.tab(),
            Self::Qc2(_) => Tab::Qc2,
            Self::Qc3 => Tab::Qc3,
            Self::Scp | Self::ScpSet(_) => Tab::Scp,
            Self::Fcp | Self::FcpSet(_) => Tab::Fcp,
            Self::Detail { page, top } => Tab::Detail { page, top },
            Self::Graph(_) => Tab::Graph,
            Self::Ripple(_) => Tab::Ripple,
        }
    }
}

/// What Left/Right move at the current level.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Level {
    Groups,
    Tabs,
    /// Something within the page.
    Page,
}

/// What the renderer shows: the tab and, when open, what it is doing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum View {
    Overview,
    /// What Left/Right change, once opened.
    Graph(Option<GraphFocus>),
    Lines,
    Energy,
    Ripple,
    Fixed {
        choosing: bool,
    },
    Pps(Option<PpsFocus>),
    /// Whether Left/Right change the voltage.
    Avs {
        adjusting: bool,
    },
    /// The previewed row, while choosing.
    Qc2(Option<Qc2Choice>),
    Scp {
        probing: bool,
    },
    /// Whether the probe runs, and the previewed row while choosing.
    Fcp {
        probing: bool,
        preview: Option<usize>,
    },
    /// The Quick Charge probe runs, on either Quick Charge tab.
    Probe,
    /// Whether Left/Right move the target.
    Qc3 {
        adjusting: bool,
    },
    Detail {
        page: Detail,
        top: u16,
        scrolling: bool,
    },
    Settings(SettingsFocus),
}

impl View {
    /// A Quick Charge tab, which shows the trigger and D+/D−.
    pub const fn quick_charge(self) -> bool {
        matches!(
            self,
            Self::Qc2(_) | Self::Qc3 { .. } | Self::Probe | Self::Scp { .. } | Self::Fcp { .. }
        )
    }

    /// The line voltages are read here: shown, or on the overview used to
    /// name the charging protocol.
    pub const fn shows_lines(self) -> bool {
        matches!(self, Self::Overview | Self::Lines) || self.quick_charge()
    }
}

/// What Left/Right change on the open graph.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GraphFocus {
    Trace,
    Window,
}

/// What Left/Right change on the open ripple tab, in OK's order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RippleFocus {
    Time,
    Scale,
    Capture,
}

impl RippleFocus {
    const fn next(self) -> Self {
        match self {
            Self::Time => Self::Scale,
            Self::Scale => Self::Capture,
            Self::Capture => Self::Time,
        }
    }
}

/// What Left/Right change on the open PPS tab.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PpsFocus {
    Voltage,
    Current,
}

/// Where a Quick Charge probe returns when it finishes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QcPage {
    Qc2,
    Qc3,
    /// A test started from Charger info returns there, not to QC controls.
    Charger,
}

impl QcPage {
    pub const fn tab(self) -> Tab {
        match self {
            Self::Qc2 => Tab::Qc2,
            Self::Qc3 => Tab::Qc3,
            Self::Charger => Tab::detail(Detail::Charger),
        }
    }

    /// Open this tab: the probe first, unless this boot already ran it.
    /// Quick Charge 2.0 opens its choice at the voltage in force, else 5 V;
    /// Quick Charge 3.0 steps from where they are, else from 5 V, and stays
    /// closed on a charger without steps.
    fn open(self, input: &mut Input) -> Page {
        match (self, input.trigger) {
            (Self::Charger, _) => Page::Tabs(Tab::detail(Detail::Charger)),
            _ if input.qc.report().is_none() => Page::Open(Open::Probe(self)),
            (Self::Qc2, Some(QcTarget::Volts(voltage))) => {
                Page::Open(Open::Qc2(Qc2Choice::Volts(voltage)))
            }
            (Self::Qc2, _) => Page::Open(Open::Qc2(Qc2Choice::Volts(Voltage::V5))),
            (Self::Qc3, _) if !input.qc.steps() => Page::Tabs(Tab::Qc3),
            (Self::Qc3, Some(QcTarget::Steps(_))) => Page::Open(Open::Qc3),
            (Self::Qc3, _) => {
                input.trigger = Some(QcTarget::Steps(Voltage::V5.millivolts()));
                Page::Open(Open::Qc3)
            }
        }
    }
}

/// A row of the Quick Charge 2.0 choice.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Qc2Choice {
    Volts(Voltage),
    /// Release D+/D−, so the charger returns to plain USB at 5 V; this also
    /// ends Quick Charge 3.0 steps.
    Off,
}

impl Qc2Choice {
    /// The rows, in Left/Right order.
    pub const ALL: [Self; 5] = [
        Self::Volts(Voltage::V5),
        Self::Volts(Voltage::V9),
        Self::Volts(Voltage::V12),
        Self::Volts(Voltage::V20),
        Self::Off,
    ];

    /// What the trigger asks of the charger once this is chosen.
    const fn target(self) -> Option<QcTarget> {
        match self {
            Self::Volts(voltage) => Some(QcTarget::Volts(voltage)),
            Self::Off => None,
        }
    }
}

/// What this boot's probe found the charger takes; before a probe, nothing
/// is ruled out.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct QcOffer(Option<Report>);

impl QcOffer {
    pub const fn report(self) -> Option<Report> {
        self.0
    }

    /// The charger followed a request for `voltage`; 5 V is where Quick
    /// Charge starts, so it always is.
    pub fn takes(self, voltage: Voltage) -> bool {
        self.0.is_none_or(|report| match voltage {
            Voltage::V5 => true,
            Voltage::V9 => report.v9,
            Voltage::V12 => report.v12,
            Voltage::V20 => report.v20,
        })
    }

    /// The charger followed Quick Charge 3.0 steps.
    pub fn steps(self) -> bool {
        self.0.is_none_or(|report| report.steps)
    }

    /// Highest output the steps may ask for: 20 V once the charger took it
    /// (Class B), else 12 V (Class A, NCP4371 Table 4).
    pub fn steps_max(self) -> Voltage {
        if self.0.is_some_and(|report| report.v20) {
            Voltage::V20
        } else {
            Voltage::V12
        }
    }
}

/// What the Quick Charge trigger asks of the charger.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QcTarget {
    Volts(Voltage),
    /// Read-only SCP uses the same exclusive line owner as QC.
    Scp(u8),
    /// Explicit standalone output request; never used by automatic scans.
    ScpOutput {
        attempt: u8,
        mv: u16,
        ma: u16,
    },
    /// Read-only FCP identification, as for SCP.
    Fcp(u8),
    /// One of the adapter's FCP voltages, in mV; explicit, as for SCP.
    FcpOutput {
        attempt: u8,
        mv: u16,
    },
    /// Quick Charge 3.0 continuous mode, stepping toward this output in mV.
    Steps(u16),
    /// Try each request in turn and report which the charger followed.
    Detect,
}

impl QcTarget {
    /// The Quick Charge 2.0 voltage asked for, if this is one.
    pub const fn volts(self) -> Option<Voltage> {
        match self {
            Self::Volts(voltage) => Some(voltage),
            _ => None,
        }
    }

    /// The Quick Charge 3.0 steps' target in mV, if this is one.
    pub const fn steps(self) -> Option<u16> {
        match self {
            Self::Steps(mv) => Some(mv),
            _ => None,
        }
    }
}

/// The settings page cursor, and whether its value is being changed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SettingsFocus {
    /// Left/Right move the cursor.
    Row(Item),
    /// Left/Right change the value.
    Brightness,
    Rotation,
    EnergyMemory,
    /// The calibration steps run; Back cancels them.
    Calibrate(Wizard),
}

impl Default for SettingsFocus {
    fn default() -> Self {
        Self::Row(Item::default())
    }
}

impl SettingsFocus {
    /// The highlighted row.
    pub fn row(self) -> Item {
        match self {
            Self::Row(item) => item,
            Self::Brightness => Item::Brightness,
            Self::Rotation => Item::Rotation,
            Self::EnergyMemory => Item::EnergyMemory,
            Self::Calibrate(_) => Item::Calibration,
        }
    }
}

/// The detail lists, in Left/Right order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Detail {
    /// PD session state.
    #[default]
    Pd,
    /// What the charger said about itself.
    Charger,
    /// Cable E-marker details.
    Cable,
    /// I2C fault counters, timing and stack use.
    System,
}

/// `items[index ± 1]`, wrapping.
fn cycle<T: Copy>(items: &[T], index: usize, forward: bool) -> T {
    let next = if forward {
        index + 1
    } else {
        index + items.len() - 1
    };
    items[next % items.len()]
}

/// The first of `items` after `index` (before it, unless `forward`) that
/// `keep` accepts, wrapping round to `items[index]` itself.
fn cycle_where<T: Copy>(
    items: &[T],
    index: usize,
    forward: bool,
    keep: impl Fn(T) -> bool,
) -> Option<T> {
    let count = items.len();
    (1..=count)
        .map(|offset| {
            if forward {
                items[(index + offset) % count]
            } else {
                items[(index + count - offset) % count]
            }
        })
        .find(|item| keep(*item))
}

/// The tab the group `forward` (Right) or back from `tab`'s shows: the one
/// it last showed (`last`, by `Group::ALL` index), else its first available.
fn next_group(last: &[Tab; Group::ALL.len()], tab: Tab, forward: bool, passthrough: bool) -> Tab {
    let shown = |group: Group| {
        let last = last[group as usize];
        if last.group() == group && last.available(passthrough) {
            Some(last)
        } else {
            group.available(passthrough).next()
        }
    };
    cycle_where(&Group::ALL, tab.group() as usize, forward, |group| {
        shown(group).is_some()
    })
    .and_then(shown)
    .unwrap_or(tab)
}

impl Page {
    /// The shown tab; `None` for the settings.
    pub const fn tab(self) -> Option<Tab> {
        match self {
            Self::Groups(tab) | Self::Tabs(tab) => Some(tab),
            Self::Open(open) => Some(open.tab()),
            Self::Settings(_) => None,
        }
    }

    pub const fn level(self) -> Level {
        match self {
            Self::Groups(_) => Level::Groups,
            Self::Tabs(_) => Level::Tabs,
            Self::Open(_) | Self::Settings(_) => Level::Page,
        }
    }

    pub const fn view(self) -> View {
        match self {
            Self::Groups(tab) | Self::Tabs(tab) => match tab {
                Tab::Overview => View::Overview,
                Tab::Graph => View::Graph(None),
                Tab::Lines => View::Lines,
                Tab::Energy => View::Energy,
                Tab::Ripple => View::Ripple,
                Tab::Fixed => View::Fixed { choosing: false },
                Tab::Pps => View::Pps(None),
                Tab::Avs => View::Avs { adjusting: false },
                Tab::Qc2 => View::Qc2(None),
                Tab::Qc3 => View::Qc3 { adjusting: false },
                Tab::Scp => View::Scp { probing: false },
                Tab::Fcp => View::Fcp {
                    probing: false,
                    preview: None,
                },
                Tab::Detail { page, top } => View::Detail {
                    page,
                    top,
                    scrolling: false,
                },
            },
            Self::Open(open) => match open {
                Open::Fixed => View::Fixed { choosing: true },
                Open::Pps(focus) => View::Pps(Some(focus)),
                Open::Avs => View::Avs { adjusting: true },
                Open::Probe(_) => View::Probe,
                Open::Qc2(preview) => View::Qc2(Some(preview)),
                Open::Qc3 => View::Qc3 { adjusting: true },
                Open::Scp => View::Scp { probing: true },
                Open::ScpSet(_) => View::Scp { probing: false },
                Open::Fcp => View::Fcp {
                    probing: true,
                    preview: None,
                },
                Open::FcpSet(row) => View::Fcp {
                    probing: false,
                    preview: Some(row),
                },
                Open::Detail { page, top } => View::Detail {
                    page,
                    top,
                    scrolling: true,
                },
                Open::Graph(focus) => View::Graph(Some(focus)),
                Open::Ripple(_) => View::Ripple,
            },
            Self::Settings(focus) => View::Settings(focus),
        }
    }

    fn is_charger(self) -> bool {
        self.tab()
            .is_some_and(|tab| tab.same(Tab::detail(Detail::Charger)))
    }

    /// The shown list's top row.
    fn top_mut(&mut self) -> Option<&mut u16> {
        match self {
            Self::Groups(Tab::Detail { top, .. })
            | Self::Tabs(Tab::Detail { top, .. })
            | Self::Open(Open::Detail { top, .. }) => Some(top),
            _ => None,
        }
    }

    /// Left/Right change a value or scroll here, rather than moving between
    /// groups, tabs or rows.
    pub fn editing(self) -> bool {
        matches!(
            self,
            Self::Open(
                Open::Fixed
                    | Open::Pps(_)
                    | Open::Avs
                    | Open::Qc2(_)
                    | Open::Qc3
                    | Open::FcpSet(_)
                    | Open::Detail { .. }
                    | Open::Graph(_)
                    | Open::Ripple(_)
            ) | Self::Settings(
                SettingsFocus::Brightness
                    | SettingsFocus::Rotation
                    | SettingsFocus::EnergyMemory
                    | SettingsFocus::Calibrate(_)
            )
        )
    }
}

/// A request for the PD session, sent by the input task. Previews only change
/// what is shown; nothing is transmitted before [`PdCommand::Confirm`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PdCommand {
    /// Preview the next (+1) or previous (-1) fixed profile; 0 previews the
    /// current or first one.
    StepFixed(i8),
    /// Preview a PPS voltage change in mV; 0 previews the first PPS range.
    PpsVoltage(i16),
    /// Preview a PPS current change in mA.
    PpsCurrent(i16),
    /// Preview an SPR AVS voltage change in mV; 0 previews the first AVS range.
    AvsVoltage(i16),
    /// Request the previewed profile.
    Confirm,
    /// Ask the cable for its E-marker identity (from fixed 5 V only).
    QueryCable,
    /// Ask the partner for its sink PDOs and SOP identity at fixed 5 V (PD3).
    QueryPort,
    /// Ask an EPR capable charger for its EPR offer (informational only).
    QueryEpr,
    /// Restart the meter through a charger power cycle, into the saved
    /// `Settings::next_boot`.
    Restart,
}

/// One coherent input state, published by the input task as a unit.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Input {
    /// Stable pressed mask; bit `n` is `PINS[n]`.
    pub pressed: u8,
    pub page: Page,
    /// The PD controller only listens (passthrough mode, chosen at boot).
    pub passthrough: bool,
    /// A live PD session excludes every legacy line request.
    pub pd_active: bool,
    pub settings: Settings,
    /// Reset request counters: the meter task clears the overview's peaks,
    /// the graph's history or the energy totals when one changes.
    pub peaks_reset: u8,
    pub graph_reset: u8,
    pub energy_reset: u8,
    /// How far the reset hold has swept, out of [`RESET_STEPS`]; 0 when none
    /// shows.
    pub reset_hold: u8,
    pub qc: QcOffer,
    /// What the Quick Charge trigger asks of the charger, kept while other
    /// tabs show; `None` leaves D+/D− undriven. A PD contract (the charger
    /// then ignores Quick Charge), passthrough or a restart clears it.
    pub trigger: Option<QcTarget>,
    /// Probe generation: stale completion from the preceding attempt is ignored.
    pub scp_attempt: u8,
    /// This attachment's volatile output preview; never restored from settings.
    pub scp_selection: Option<crate::scp::Selection>,
    /// FCP probe or output generation, as for SCP, and the adapter the
    /// probe found; this boot only, since the settings word has no room for
    /// FCP findings.
    pub fcp_attempt: u8,
    pub fcp_adapter: Option<scp_protocol::fcp::Adapter>,
    /// What the graph plots and how far back it reaches; this boot only.
    pub graph_trace: GraphTrace,
    pub graph_window: GraphWindow,
    /// Freeze only the ripple waveform/statistics; bus V/A stay live.
    pub ripple_hold: bool,
    /// Ripple time per division and vertical scale; this boot only.
    pub ripple_timebase: RippleTimebase,
    pub ripple_scale: RippleScale,
}

impl Input {
    /// What the Quick Charge trigger asks of the charger now: the probe
    /// while it is shown, else the request in force.
    pub fn qc_request(self) -> Option<QcTarget> {
        if self.passthrough || self.pd_active {
            return None;
        }
        match self.page {
            Page::Open(Open::Probe(_)) => Some(QcTarget::Detect),
            Page::Open(Open::Scp) => Some(QcTarget::Scp(self.scp_attempt)),
            Page::Open(Open::Fcp) => Some(QcTarget::Fcp(self.fcp_attempt)),
            _ => self.trigger,
        }
    }

    pub fn scp_scanning(self) -> bool {
        matches!(self.settings.next_boot, NextBoot::ScanQuickCharge { .. })
            && self.settings.resume == Some(Screen::Scp)
    }

    /// The scan's last no-PD stage, after SCP.
    pub fn fcp_scanning(self) -> bool {
        matches!(self.settings.next_boot, NextBoot::ScanQuickCharge { .. })
            && self.settings.resume == Some(Screen::Fcp)
    }

    /// The reset request counter a completed OK hold on the shown tab
    /// advances, if the tab has one.
    fn reset_counter(&mut self) -> Option<&mut u8> {
        match self.page.tab()? {
            Tab::Overview => Some(&mut self.peaks_reset),
            Tab::Graph => Some(&mut self.graph_reset),
            Tab::Energy => Some(&mut self.energy_reset),
            _ => None,
        }
    }
}

/// Navigation and settings, owned by the input task rather than the renderer,
/// so a render backlog never loses taps.
///
/// Left and Right act when released, unless the press already repeated; OK
/// acts when released unless it was held long. Held on the overview, graph or
/// energy tab, OK sweeps a reset across it, which resets once full.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Navigation {
    debounce: Debounced,
    input: Input,
    /// The current Left/Right press already repeated.
    repeated: bool,
    /// Samples the current Left/Right press has been held.
    held: u32,
    /// Samples OK has been held, and whether that hold already acted.
    ok_held: u32,
    ok_long: bool,
    /// A sink session runs, so the PD and PPS pages can select.
    selectable: bool,
    /// The meter's plug sits in the charger, and whether this boot runs the
    /// 5 A cable test.
    plug_in_charger: bool,
    cable_test: bool,
    /// This boot keeps PD silent (`NextBoot::silent`): no session takes PD
    /// commands but the restart.
    silent: bool,
    /// Last row the shown detail list can start at, as the renderer laid it
    /// out.
    list_end: u16,
    /// The tab each group last showed, by `Group::ALL` index, so moving
    /// between groups returns to it.
    last: [Tab; Group::ALL.len()],
    /// The meter's averaged raw reading, for the calibration steps.
    mean: Option<Mean>,
}

impl Navigation {
    /// Start on `page` (e.g. a diagnostic boot page) with saved `settings`.
    pub fn new(page: Page, settings: Settings) -> Self {
        let mut navigation = Self::default();
        navigation.input.page = page;
        navigation.input.settings = settings;
        navigation
    }

    /// Whether a sink session runs. Without one (passthrough, no charger) the
    /// PD and PPS pages only show the offer, and an open selection closes.
    pub fn set_selectable(&mut self, selectable: bool) {
        self.selectable = selectable;
        self.input.pd_active = selectable;
        if selectable
            && let Page::Open(open @ (Open::Scp | Open::ScpSet(_) | Open::Fcp | Open::FcpSet(_))) =
                self.input.page
        {
            self.input.page = Page::Tabs(open.tab());
        }
        if selectable {
            self.input.trigger = None;
        }
        if let Page::Open(open @ (Open::Fixed | Open::Pps(_) | Open::Avs)) = self.input.page
            && !selectable
        {
            self.input.page = Page::Tabs(open.tab());
        }
    }

    /// Whether the PD controller only listens; a tab passthrough cannot
    /// use closes to the overview.
    pub fn set_passthrough(&mut self, passthrough: bool) {
        self.input.passthrough = passthrough;
        if passthrough {
            self.input.trigger = None;
        }
        if let Some(tab) = self.input.page.tab()
            && !tab.available(passthrough)
        {
            self.input.page = Page::default();
        }
    }

    /// The meter's plug sits in the charger (`plug_in_charger`), and this
    /// boot runs the 5 A cable test (`testing`). Then the full scan (a long
    /// OK on the charger page) ends in the test, and Back there ends it.
    /// The test keeps the charger page shown (`update`), its tabs level
    /// so OK scrolls it.
    pub fn set_cable_test(&mut self, plug_in_charger: bool, testing: bool) {
        self.plug_in_charger = plug_in_charger;
        self.cable_test = testing;
        if testing && !self.input.page.is_charger() {
            self.input.page = Page::Tabs(Tab::detail(Detail::Charger));
        }
    }

    /// Whether this boot keeps PD silent for Quick Charge, and whether the
    /// charger has had time to give PD up (`settled`). Quick Charge mode is
    /// saved only to outlast the charger's own restarts until then, so a
    /// later plug-in starts with PD again.
    pub fn set_silent(&mut self, silent: bool, settled: bool) {
        self.silent = silent;
        if settled && self.input.settings.next_boot == NextBoot::QuickCharge {
            self.input.settings.set_next_boot(NextBoot::Normal, None);
        }
    }

    /// The probe found `report`: keep it, and open the Quick Charge tab that
    /// waited for it (the scan keeps its own page).
    pub fn finish_probe(&mut self, report: Report) {
        self.input.qc = QcOffer(Some(report));
        if let Page::Open(Open::Probe(page)) = self.input.page
            && !self.scanning()
            && page != QcPage::Charger
        {
            self.input.page = page.open(&mut self.input);
        }
    }

    /// The probe found no Quick Charge. Quick Charge mode then ends with this
    /// boot: a charger still retrying PD would otherwise keep restarting the
    /// meter into the same failed probe. The next boot runs PD and reopens
    /// this tab, which keeps the result (the scan keeps its own steps).
    pub fn fail_probe(&mut self) {
        if self.silent && self.input.settings.next_boot == NextBoot::QuickCharge {
            let tab = self.input.page.tab().map(Screen::from);
            self.input.settings.set_next_boot(NextBoot::Normal, tab);
        }
    }

    /// This boot runs the full scan's Quick Charge part, which keeps the
    /// probe on screen until it ends; Back abandons the scan.
    fn scanning(&self) -> bool {
        matches!(
            self.input.settings.next_boot,
            NextBoot::ScanQuickCharge { .. }
        )
    }

    /// Finish a Charger-page test: return directly without PD, or keep the
    /// full scan's findings and restart into its PD part.
    pub fn finish_scan(&mut self, findings: QcFindings) -> Option<PdCommand> {
        let local = self.input.page == Page::Open(Open::Probe(QcPage::Charger));
        if self.input.scp_scanning() || self.input.fcp_scanning() || (!local && !self.scanning()) {
            return None;
        }
        if local {
            self.input.settings.next_boot = NextBoot::ScanQuickCharge {
                then_cable_test: false,
            };
        }
        self.input.settings.qc_findings = Some(findings);
        self.input.settings.scp_findings = None;
        self.input.settings.fcp_findings = None;
        self.input.scp_selection = None;
        self.input.settings.resume = Some(Screen::Scp);
        self.input.scp_attempt = self.input.scp_attempt.wrapping_add(1);
        self.input.trigger = None;
        self.input.page = Page::Open(Open::Scp);
        None
    }

    /// Accept ratings only from the current explicit detection attempt.
    pub fn set_scp_adapter(&mut self, attempt: u8, adapter: scp_protocol::Adapter) {
        if attempt == self.input.scp_attempt && self.input.page == Page::Open(Open::Scp) {
            self.input.scp_selection = crate::scp::Selection::new(adapter);
        }
    }

    /// A failed output ends its request, releasing the lines: OK starts a
    /// new one, never an automatic retry.
    pub fn fail_scp_output(&mut self, attempt: u8) {
        if matches!(self.input.trigger, Some(QcTarget::ScpOutput { attempt: current, .. }) if current == attempt)
        {
            self.input.trigger = None;
        }
    }

    /// Keep the current FCP probe's result, as for SCP, and the adapter it
    /// found. In the scan this is the last no-PD stage, which then ends.
    pub fn finish_fcp(
        &mut self,
        attempt: u8,
        findings: FcpFindings,
        adapter: Option<scp_protocol::fcp::Adapter>,
    ) -> Option<PdCommand> {
        if attempt != self.input.fcp_attempt || self.input.page != Page::Open(Open::Fcp) {
            return None;
        }
        self.input.settings.fcp_findings = Some(findings);
        self.input.fcp_adapter = adapter;
        self.input.trigger = None;
        if !self.input.fcp_scanning() {
            self.input.page = Page::Tabs(Tab::Fcp);
            return None;
        }
        self.end_scan_without_pd()
    }

    /// A failed FCP output ends its request, as for SCP.
    pub fn fail_fcp_output(&mut self, attempt: u8) {
        if matches!(self.input.trigger, Some(QcTarget::FcpOutput { attempt: current, .. }) if current == attempt)
        {
            self.input.trigger = None;
        }
    }

    /// Keep SCP evidence: absence only when the adapter showed it, never
    /// from a test that went wrong.
    pub fn finish_scp(&mut self, attempt: u8, findings: ScpFindings) -> Option<PdCommand> {
        if self.input.scp_attempt != attempt || self.input.page != Page::Open(Open::Scp) {
            return None;
        }
        self.input.settings.scp_findings = Some(findings);
        self.input.trigger = None;
        if !self.input.scp_scanning() {
            self.input.page = Page::Tabs(Tab::Scp);
            return None;
        }
        // The scan goes on to FCP in this boot.
        self.input.settings.fcp_findings = None;
        self.input.fcp_adapter = None;
        self.input.settings.resume = Some(Screen::Fcp);
        self.input.fcp_attempt = self.input.fcp_attempt.wrapping_add(1);
        self.input.page = Page::Open(Open::Fcp);
        None
    }

    /// The scan's no-PD stages are done: show the charger page, restarting
    /// into the PD part when this boot kept PD silent.
    fn end_scan_without_pd(&mut self) -> Option<PdCommand> {
        let NextBoot::ScanQuickCharge { then_cable_test } = self.input.settings.next_boot else {
            return None;
        };
        self.input.page = Page::Tabs(Tab::detail(Detail::Charger));
        // Without PD the scan ends in this boot, already on the charger
        // page: a saved resume screen would open the next plug-in there.
        if !self.silent {
            self.input.settings.set_next_boot(NextBoot::Normal, None);
            return None;
        }
        let boot = if then_cable_test {
            NextBoot::CableTest
        } else {
            NextBoot::ScanPd
        };
        self.input
            .settings
            .set_next_boot(boot, Some(Screen::Charger));
        Some(PdCommand::Restart)
    }

    /// The last row the shown detail list can start at; a list that got
    /// shorter scrolls back.
    pub fn set_list_end(&mut self, end: u16) {
        self.list_end = end;
        if let Some(top) = self.input.page.top_mut() {
            *top = (*top).min(end);
        }
    }

    /// The meter's averaged raw reading; `None` while it is not read.
    pub fn set_mean(&mut self, mean: Option<Mean>) {
        self.mean = mean;
    }

    /// Feed one sample; returns the input state and any PD session request.
    pub fn update(&mut self, raw_pressed: u8) -> (Input, Option<PdCommand>) {
        let page_before = self.input.page;
        let before = self.debounce.stable();
        let after = self.debounce.update(raw_pressed, DEFAULT_THRESHOLD);
        let rising = after & !before;
        let falling = before & !after;
        self.input.pressed = after;
        let mut command = None;

        // A Left/Right press steps when released, unless it repeated.
        if !self.repeated {
            if falling & LEFT != 0 {
                command = self.step(false, false);
            } else if falling & RIGHT != 0 {
                command = self.step(true, false);
            }
        }
        let left_right = after & (LEFT | RIGHT);
        if left_right == LEFT || left_right == RIGHT {
            self.held += 1;
            if self.held >= REPEAT_DELAY_SAMPLES
                && (self.held - REPEAT_DELAY_SAMPLES).is_multiple_of(REPEAT_SAMPLES)
                && self.input.page.repeats()
            {
                self.repeated = true;
                command = self.step(left_right == RIGHT, true);
            }
        } else if left_right == 0 {
            self.repeated = false;
            self.held = 0;
        }

        self.input.reset_hold = 0;
        if after & OK != 0 {
            self.ok_held += 1;
            if self.ok_held == LONG_OK_SAMPLES
                && let Some(query) = self.long_ok()
            {
                self.ok_long = true;
                command = query;
            }
            let swept = self.ok_held.saturating_sub(RESET_DELAY_SAMPLES);
            if !self.ok_long && self.sweeps() {
                if swept >= RESET_SWEEP_SAMPLES {
                    self.reset();
                    self.ok_long = true;
                } else {
                    // Below `RESET_STEPS`, as `swept` is below the sweep.
                    self.input.reset_hold =
                        (swept * u32::from(RESET_STEPS) / RESET_SWEEP_SAMPLES) as u8;
                }
            }
        } else {
            if falling & OK != 0 && !self.ok_long {
                command = self.ok().or(command);
            }
            self.ok_held = 0;
            self.ok_long = false;
        }
        if rising & BACK != 0 {
            if self.cable_test && self.input.page.is_charger() {
                command = Some(PdCommand::Restart);
            } else if self.scanning() {
                self.input.settings.set_next_boot(NextBoot::Normal, None);
                self.input.trigger = None;
                self.input.page = Page::Tabs(Tab::detail(Detail::Charger));
                command = self.silent.then_some(PdCommand::Restart);
            } else {
                self.back();
            }
        }
        // The 5 A cable test tells the charger the path carries 5 A, so the
        // meter stays on the charger page: it scrolls, asks for the EPR offer
        // or ends the test, and never selects another profile.
        if self.cable_test {
            if !self.input.page.is_charger() {
                self.input.page = page_before;
            }
            command = command.filter(|c| matches!(c, PdCommand::QueryEpr | PdCommand::Restart));
        }
        // The scan's Quick Charge part keeps its probe on screen.
        if self.input.scp_scanning() {
            self.input.page = Page::Open(Open::Scp);
        } else if self.input.fcp_scanning() {
            self.input.page = Page::Open(Open::Fcp);
        } else if self.scanning() {
            self.input.page = Page::Open(Open::Probe(QcPage::Qc2));
        }
        if let Some(tab) = self.input.page.tab() {
            self.last[tab.group() as usize] = tab;
        }
        // A hold lasts only while Ripple is shown: coming back starts live.
        if self.input.page.tab() != Some(Tab::Ripple) {
            self.input.ripple_hold = false;
        }
        if self.silent {
            command = command.filter(|c| *c == PdCommand::Restart);
        }
        // A restart starts from 5 V, which a charger still holding a Quick
        // Charge request never returns to: release D+/D− first.
        if command == Some(PdCommand::Restart) {
            self.input.trigger = None;
        }
        (self.input, command)
    }

    /// A completed OK hold resets something on the shown page.
    fn sweeps(&mut self) -> bool {
        self.input.page == Page::Settings(SettingsFocus::Row(Item::Calibration))
            || self.input.reset_counter().is_some()
    }

    /// What a completed OK hold resets: the calibration on its row, else
    /// the shown tab's readings.
    fn reset(&mut self) {
        match self.input.reset_counter() {
            Some(counter) => *counter = counter.wrapping_add(1),
            None => self.input.settings.calibration = Calibration::default(),
        }
    }

    /// What a long OK does here: `None` when the page has no long OK, else
    /// the PD command it sends, if any.
    fn long_ok(&mut self) -> Option<Option<PdCommand>> {
        // Open ripple: back to the default timebase, auto scale and live.
        if let Page::Open(Open::Ripple(_)) = self.input.page {
            let input = &mut self.input;
            input.ripple_timebase = RippleTimebase::default();
            input.ripple_scale = RippleScale::default();
            input.ripple_hold = false;
            return Some(None);
        }
        if let Page::Settings(SettingsFocus::Calibrate(wizard)) = &mut self.input.page {
            wizard.hold_ok(self.mean);
            return Some(None);
        }
        // Hold OK stops a running SCP output, else rescans the adapter.
        if self.input.page == Page::Tabs(Tab::Scp)
            && matches!(self.input.trigger, Some(QcTarget::ScpOutput { .. }))
        {
            self.input.trigger = None;
            return Some(None);
        }
        if self.input.page == Page::Tabs(Tab::Scp) && !self.selectable && !self.input.passthrough {
            self.input.trigger = None;
            self.input.scp_selection = None;
            self.input.scp_attempt = self.input.scp_attempt.wrapping_add(1);
            self.input.page = Page::Open(Open::Scp);
            return Some(None);
        }
        // The same for FCP.
        if self.input.page == Page::Tabs(Tab::Fcp)
            && matches!(self.input.trigger, Some(QcTarget::FcpOutput { .. }))
        {
            self.input.trigger = None;
            return Some(None);
        }
        if self.input.page == Page::Tabs(Tab::Fcp) && !self.selectable && !self.input.passthrough {
            self.input.trigger = None;
            self.input.fcp_adapter = None;
            self.input.fcp_attempt = self.input.fcp_attempt.wrapping_add(1);
            self.input.page = Page::Open(Open::Fcp);
            return Some(None);
        }
        let Some(Tab::Detail { page, .. }) = self.input.page.tab() else {
            return None;
        };
        match page {
            Detail::Pd if self.selectable && !self.input.passthrough && !self.cable_test => {
                Some(Some(PdCommand::QueryPort))
            }
            Detail::Cable => Some(Some(PdCommand::QueryCable)),
            Detail::Charger if self.cable_test => Some(Some(PdCommand::QueryEpr)),
            // The full scan: Quick Charge first, with PD silent (a charger
            // ignores it under a PD contract), then PD again with its EPR
            // offer, ending here.
            Detail::Charger if self.selectable => {
                self.input.settings.set_next_boot(
                    NextBoot::ScanQuickCharge {
                        then_cable_test: self.plug_in_charger,
                    },
                    Some(Screen::Qc2),
                );
                Some(Some(PdCommand::Restart))
            }
            // Without PD, test directly and return to Charger info.
            Detail::Charger => {
                self.input.page = Page::Open(Open::Probe(QcPage::Charger));
                Some(None)
            }
            _ => None,
        }
    }

    fn back(&mut self) {
        let page = &mut self.input.page;
        *page = match *page {
            Page::Groups(Tab::Overview) => Page::Settings(SettingsFocus::default()),
            Page::Groups(_) | Page::Settings(SettingsFocus::Row(_)) => Page::default(),
            Page::Settings(focus) => Page::Settings(SettingsFocus::Row(focus.row())),
            Page::Tabs(tab) => Page::Groups(tab),
            Page::Open(Open::Scp) => {
                self.input.trigger = None;
                Page::Tabs(Tab::Scp)
            }
            // Cancel the edit, as on PPS: the output in force stays, and
            // the preview returns to it.
            Page::Open(Open::ScpSet(_)) => {
                if let (Some(selection), Some(QcTarget::ScpOutput { mv, ma, .. })) =
                    (&mut self.input.scp_selection, self.input.trigger)
                {
                    selection.reset_to(mv, ma);
                }
                Page::Tabs(Tab::Scp)
            }
            Page::Open(Open::Fcp) => {
                self.input.trigger = None;
                Page::Tabs(Tab::Fcp)
            }
            Page::Open(Open::Probe(QcPage::Charger)) => {
                self.input.trigger = None;
                Page::Tabs(Tab::detail(Detail::Charger))
            }
            Page::Open(open) => Page::Tabs(open.tab()),
        };
    }

    /// Left (`forward` false) or Right; `repeat` for a held button.
    fn step(&mut self, forward: bool, repeat: bool) -> Option<PdCommand> {
        let sign = if forward { 1 } else { -1 };
        let input = &mut self.input;
        match &mut input.page {
            Page::Groups(tab) => *tab = next_group(&self.last, *tab, forward, input.passthrough),
            Page::Tabs(tab) => *tab = tab.next(forward, input.passthrough),
            Page::Open(Open::Detail { top, .. }) => {
                *top = if forward {
                    top.saturating_add(1).min(self.list_end)
                } else {
                    top.saturating_sub(1)
                };
            }
            Page::Settings(SettingsFocus::Row(item)) => {
                let index = Item::ALL.iter().position(|row| row == item).unwrap_or(0);
                *item = cycle(&Item::ALL, index, forward);
            }
            Page::Settings(SettingsFocus::Brightness) => input.settings.step_brightness(forward),
            Page::Settings(SettingsFocus::Rotation) => input.settings.step_rotation(forward),
            Page::Settings(SettingsFocus::EnergyMemory) => {
                input.settings.step_energy_memory(forward)
            }
            Page::Settings(SettingsFocus::Calibrate(wizard)) => wizard.step(forward, repeat),
            Page::Open(Open::Fixed) => return Some(PdCommand::StepFixed(sign as i8)),
            Page::Open(Open::Qc2(preview)) => {
                let all = Qc2Choice::ALL;
                let index = all.iter().position(|row| row == preview).unwrap_or(0);
                let qc = input.qc;
                *preview = cycle_where(&all, index, forward, |row| match row {
                    Qc2Choice::Volts(voltage) => qc.takes(voltage),
                    Qc2Choice::Off => true,
                })
                .unwrap_or(*preview);
            }
            Page::Open(Open::Qc3) => {
                let max = input.qc.steps_max();
                if let Some(QcTarget::Steps(mv)) = &mut input.trigger {
                    *mv = if forward {
                        mv.saturating_add(STEP_MV).min(max.millivolts())
                    } else {
                        mv.saturating_sub(STEP_MV).max(MIN_MV)
                    };
                }
            }
            // SCP moves by at least the PPS amounts, rounded up to whole
            // advertised steps, so both editors feel the same.
            Page::Open(Open::ScpSet(focus)) => {
                let voltage = *focus == PpsFocus::Voltage;
                let amount = match (voltage, repeat) {
                    (true, false) => PPS_MV_STEP,
                    (true, true) => PPS_MV_REPEAT,
                    (false, false) => PPS_MA_STEP,
                    (false, true) => PPS_MA_REPEAT,
                };
                if let Some(selection) = &mut input.scp_selection {
                    selection.step(voltage, forward, amount.unsigned_abs());
                }
            }
            // The rows are the adapter's voltages, then off.
            Page::Open(Open::FcpSet(row)) => {
                let rows = input
                    .fcp_adapter
                    .map_or(0, |adapter| adapter.levels().len())
                    + 1;
                *row = if forward {
                    (*row + 1) % rows
                } else {
                    (*row + rows - 1) % rows
                };
            }
            // Probes run until a result or Back.
            Page::Open(Open::Probe(_) | Open::Scp | Open::Fcp) => {}
            // A held capture cannot be re-timed: a new timebase runs live.
            Page::Open(Open::Ripple(RippleFocus::Time)) => {
                let all = RippleTimebase::ALL;
                let index = all
                    .iter()
                    .position(|t| *t == input.ripple_timebase)
                    .unwrap_or(0);
                input.ripple_timebase = cycle(&all, index, forward);
                input.ripple_hold = false;
            }
            Page::Open(Open::Ripple(RippleFocus::Scale)) => {
                let all = RippleScale::ALL;
                let index = all
                    .iter()
                    .position(|s| *s == input.ripple_scale)
                    .unwrap_or(0);
                input.ripple_scale = cycle(&all, index, forward);
            }
            Page::Open(Open::Ripple(RippleFocus::Capture)) => {
                input.ripple_hold = !input.ripple_hold;
            }
            Page::Open(Open::Graph(GraphFocus::Trace)) => {
                let all = GraphTrace::ALL;
                let index = all
                    .iter()
                    .position(|t| *t == input.graph_trace)
                    .unwrap_or(0);
                input.graph_trace = cycle(&all, index, forward);
            }
            Page::Open(Open::Graph(GraphFocus::Window)) => {
                let all = GraphWindow::ALL;
                let index = all
                    .iter()
                    .position(|w| *w == input.graph_window)
                    .unwrap_or(0);
                input.graph_window = cycle(&all, index, forward);
            }
            Page::Open(Open::Pps(PpsFocus::Voltage)) => {
                let step = if repeat { PPS_MV_REPEAT } else { PPS_MV_STEP };
                return Some(PdCommand::PpsVoltage(sign * step));
            }
            Page::Open(Open::Pps(PpsFocus::Current)) => {
                let step = if repeat { PPS_MA_REPEAT } else { PPS_MA_STEP };
                return Some(PdCommand::PpsCurrent(sign * step));
            }
            Page::Open(Open::Avs) => {
                let step = if repeat { AVS_MV_REPEAT } else { AVS_MV_STEP };
                return Some(PdCommand::AvsVoltage(sign * step));
            }
        }
        None
    }

    fn ok(&mut self) -> Option<PdCommand> {
        let input = &mut self.input;
        let (page, command) = match input.page {
            Page::Groups(tab) => (Page::Tabs(tab), None),
            page @ Page::Tabs(Tab::Overview | Tab::Lines | Tab::Energy) => (page, None),
            Page::Tabs(Tab::Ripple) => (Page::Open(Open::Ripple(RippleFocus::Time)), None),
            Page::Open(Open::Ripple(focus)) => (Page::Open(Open::Ripple(focus.next())), None),
            Page::Tabs(Tab::Graph) => (Page::Open(Open::Graph(GraphFocus::Trace)), None),
            Page::Open(Open::Graph(GraphFocus::Trace)) => {
                (Page::Open(Open::Graph(GraphFocus::Window)), None)
            }
            // Quick Charge holds the charger: restart into PD, back on this tab.
            page @ Page::Tabs(tab @ (Tab::Fixed | Tab::Pps | Tab::Avs)) if self.silent => {
                input
                    .settings
                    .set_next_boot(NextBoot::Normal, Some(tab.into()));
                (page, Some(PdCommand::Restart))
            }
            page @ Page::Tabs(Tab::Fixed | Tab::Pps | Tab::Avs) if !self.selectable => (page, None),
            Page::Tabs(Tab::Fixed) => (Page::Open(Open::Fixed), Some(PdCommand::StepFixed(0))),
            Page::Tabs(Tab::Pps) => (
                Page::Open(Open::Pps(PpsFocus::Voltage)),
                Some(PdCommand::PpsVoltage(0)),
            ),
            Page::Tabs(Tab::Avs) => (Page::Open(Open::Avs), Some(PdCommand::AvsVoltage(0))),
            Page::Tabs(Tab::Qc2) if !self.selectable => (QcPage::Qc2.open(input), None),
            Page::Tabs(Tab::Qc3) if !self.selectable => (QcPage::Qc3.open(input), None),
            Page::Tabs(Tab::Scp) if !self.selectable => {
                if input.scp_selection.is_some() {
                    (Page::Open(Open::ScpSet(PpsFocus::Voltage)), None)
                } else {
                    input.scp_attempt = input.scp_attempt.wrapping_add(1);
                    (Page::Open(Open::Scp), None)
                }
            }
            // FCP opens its choice at the voltage in force, else the first.
            Page::Tabs(Tab::Fcp) if !self.selectable => match input.fcp_adapter {
                Some(adapter) => {
                    let row = match input.trigger {
                        Some(QcTarget::FcpOutput { mv, .. }) => {
                            adapter.levels().iter().position(|level| *level == mv)
                        }
                        _ => None,
                    };
                    (Page::Open(Open::FcpSet(row.unwrap_or(0))), None)
                }
                None => {
                    input.fcp_attempt = input.fcp_attempt.wrapping_add(1);
                    (Page::Open(Open::Fcp), None)
                }
            },
            // A PD contract hides Quick Charge: restart into a silent boot,
            // back on this tab.
            page @ Page::Tabs(tab @ (Tab::Qc2 | Tab::Qc3 | Tab::Scp | Tab::Fcp)) => {
                input
                    .settings
                    .set_next_boot(NextBoot::QuickCharge, Some(tab.into()));
                (page, Some(PdCommand::Restart))
            }
            Page::Tabs(Tab::Detail { page, top }) => (Page::Open(Open::Detail { page, top }), None),
            Page::Open(Open::Fixed) => (Page::Tabs(Tab::Fixed), Some(PdCommand::Confirm)),
            Page::Open(Open::Pps(PpsFocus::Voltage)) => {
                (Page::Open(Open::Pps(PpsFocus::Current)), None)
            }
            Page::Open(Open::Pps(PpsFocus::Current)) => {
                (Page::Tabs(Tab::Pps), Some(PdCommand::Confirm))
            }
            Page::Open(Open::Avs) => (Page::Tabs(Tab::Avs), Some(PdCommand::Confirm)),
            Page::Open(Open::Qc2(preview)) => {
                input.trigger = preview.target();
                (Page::Tabs(Tab::Qc2), None)
            }
            Page::Open(Open::ScpSet(PpsFocus::Voltage)) => {
                (Page::Open(Open::ScpSet(PpsFocus::Current)), None)
            }
            Page::Open(Open::ScpSet(PpsFocus::Current)) => {
                if let Some(selection) = input.scp_selection {
                    let output = selection.output();
                    input.trigger = Some(QcTarget::ScpOutput {
                        attempt: input.scp_attempt,
                        mv: output.millivolts(),
                        ma: output.milliamps(),
                    });
                }
                (Page::Tabs(Tab::Scp), None)
            }
            // Off is the row past the voltages. A change keeps the running
            // output; a new one is a new attempt, so a failed run still
            // stopping cannot end it.
            Page::Open(Open::FcpSet(row)) => {
                let attempt = match input.trigger {
                    Some(QcTarget::FcpOutput { attempt, .. }) => attempt,
                    _ => input.fcp_attempt.wrapping_add(1),
                };
                input.trigger = input
                    .fcp_adapter
                    .and_then(|adapter| adapter.levels().get(row).copied())
                    .map(|mv| {
                        input.fcp_attempt = attempt;
                        QcTarget::FcpOutput { attempt, mv }
                    });
                (Page::Tabs(Tab::Fcp), None)
            }
            page @ Page::Open(Open::Probe(_) | Open::Scp | Open::Fcp) => (page, None),
            Page::Open(open @ (Open::Qc3 | Open::Detail { .. } | Open::Graph(_))) => {
                (Page::Tabs(open.tab()), None)
            }
            Page::Settings(SettingsFocus::Row(item)) => (
                Page::Settings(match item {
                    Item::Brightness => SettingsFocus::Brightness,
                    Item::Rotation => SettingsFocus::Rotation,
                    Item::EnergyMemory => SettingsFocus::EnergyMemory,
                    Item::Calibration => {
                        SettingsFocus::Calibrate(Wizard::new(input.settings.calibration))
                    }
                }),
                None,
            ),
            // The last step's OK applies the calibration.
            Page::Settings(SettingsFocus::Calibrate(mut wizard)) => {
                let focus = match wizard.ok(self.mean) {
                    Some(calibration) => {
                        input.settings.calibration = calibration;
                        SettingsFocus::Row(Item::Calibration)
                    }
                    None => SettingsFocus::Calibrate(wizard),
                };
                (Page::Settings(focus), None)
            }
            Page::Settings(focus) => (Page::Settings(SettingsFocus::Row(focus.row())), None),
        };
        input.page = page;
        command
    }
}
