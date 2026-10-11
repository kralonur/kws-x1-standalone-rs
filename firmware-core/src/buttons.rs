//! Button sampling and debouncing (pure logic).
//!
//! The original firmware configures four button inputs as active-low with
//! pull-ups: PB2, PB10, PH0 and PC14, each treating a low level as pressed.
//! They are the Left, Right, OK and Back buttons, as identified on
//! the device.
#![allow(dead_code)]

use crate::settings::{Item, NextBoot, QcFindings, Settings};
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

/// Button sampling period in milliseconds; the hold times below are counted
/// in these samples.
pub const POLL_MS: u32 = 10;
/// A long OK press (cable query on the cable page).
const LONG_OK_SAMPLES: u32 = 500 / POLL_MS;
/// A held Left or Right repeats after this delay, at this period.
const REPEAT_DELAY_SAMPLES: u32 = 500 / POLL_MS;
const REPEAT_SAMPLES: u32 = 100 / POLL_MS;
/// PPS steps per press and per repeat: 20 mV is the PPS voltage resolution
/// and 50 mA the PPS current resolution (USB PD Table 6.21).
const PPS_MV_STEP: i16 = 20;
const PPS_MV_REPEAT: i16 = 100;
const PPS_MA_STEP: i16 = 50;
const PPS_MA_REPEAT: i16 = 250;

/// The screen and what its Left, Right and OK act on. Each page carries only
/// its own state, so e.g. a PPS edit outside the PPS page cannot exist.
/// Left/Right cycle the pages in `PAGES` that the PD mode can use; Back on
/// the overview opens the settings.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Page {
    #[default]
    Overview,
    /// D+, D−, CC1 and CC2 line voltages.
    Lines,
    /// Fixed PD profile selector; while `choosing`, Left/Right preview
    /// profiles.
    Fixed { choosing: bool },
    /// PPS voltage and current selector.
    Pps(PpsFocus),
    /// Quick Charge 2.0 and 3.0 trigger.
    Qc(QcFocus),
    /// Opened by Back on the overview.
    Settings(SettingsFocus),
    /// A detail list shown from row `top`; while `scrolling` (OK), Left/Right
    /// move it instead of changing pages.
    Detail {
        page: Detail,
        top: u16,
        scrolling: bool,
    },
}

/// What Left/Right change on the PPS page.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PpsFocus {
    /// Nothing: Left/Right move between pages.
    #[default]
    Browse,
    Voltage,
    Current,
}

/// The Quick Charge page: OK starts the trigger at 5 V, OK again opens a
/// voltage choice that a third OK requests; Back closes the choice, then
/// stops. With Quick Charge 3.0 steps requested, Left/Right move the target.
/// Leaving the page stops the trigger too. Under a PD contract the charger
/// ignores Quick Charge, so OK first restarts it into a silent PD boot
/// (`NextBoot::QuickCharge`); Back on the stopped page there restarts it into
/// PD again.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum QcFocus {
    /// D+/D− undriven.
    #[default]
    Off,
    /// This is asked of the charger.
    Requested(QcTarget),
    /// Left/Right choose `preview` while `request` stays in force.
    Choosing {
        request: QcTarget,
        preview: QcTarget,
    },
}

/// What the Quick Charge page asks of the charger.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QcTarget {
    Volts(Voltage),
    /// Quick Charge 3.0 continuous mode, stepping toward this output in mV.
    Steps(u16),
    /// Try each request in turn and report which the charger followed.
    Detect,
}

impl QcTarget {
    /// The choices, in Left/Right order. Steps start where continuous mode
    /// is entered from, 5 V.
    const ALL: [Self; 6] = [
        Self::Volts(Voltage::V5),
        Self::Volts(Voltage::V9),
        Self::Volts(Voltage::V12),
        Self::Volts(Voltage::V20),
        Self::Steps(Voltage::V5.millivolts()),
        Self::Detect,
    ];

    /// Position in [`Self::ALL`]; any steps target is the steps choice.
    fn index(self) -> usize {
        Self::ALL
            .iter()
            .position(|choice| {
                *choice == self || matches!((choice, self), (Self::Steps(_), Self::Steps(_)))
            })
            .unwrap_or(0)
    }
}

impl QcFocus {
    /// What is asked of the charger; `None` leaves D+/D− undriven.
    pub const fn request(self) -> Option<QcTarget> {
        match self {
            Self::Off => None,
            Self::Requested(request) | Self::Choosing { request, .. } => Some(request),
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
        }
    }
}

/// Detail pages, in Left/Right order after the selectors.
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

/// The pages as Left/Right enter them.
const PAGES: [Page; 9] = [
    Page::Overview,
    Page::Lines,
    Page::Fixed { choosing: false },
    Page::Pps(PpsFocus::Browse),
    Page::Qc(QcFocus::Off),
    Page::detail(Detail::Pd),
    Page::detail(Detail::Charger),
    Page::detail(Detail::Cable),
    Page::detail(Detail::System),
];
/// Index of the first detail page in `PAGES`.
const FIRST_DETAIL: usize = 5;

/// `items[index ± 1]`, wrapping.
fn cycle<T: Copy>(items: &[T], index: usize, forward: bool) -> T {
    let next = if forward {
        index + 1
    } else {
        index + items.len() - 1
    };
    items[next % items.len()]
}

impl Page {
    /// A detail page at the top of its list, not scrolling.
    pub const fn detail(page: Detail) -> Self {
        Self::Detail {
            page,
            top: 0,
            scrolling: false,
        }
    }

    fn is_charger(self) -> bool {
        matches!(
            self,
            Self::Detail {
                page: Detail::Charger,
                ..
            }
        )
    }

    /// Whether the page has anything to show: in passthrough the meter only
    /// listens, so the selectors and the charger and cable pages, which need
    /// a sink session, are left out.
    pub fn available(self, passthrough: bool) -> bool {
        !passthrough
            || matches!(
                self,
                Self::Overview
                    | Self::Lines
                    | Self::Settings(_)
                    | Self::Detail {
                        page: Detail::Pd | Detail::System,
                        ..
                    }
            )
    }

    /// The available page `forward` (Right) or back from this one, wrapping.
    fn next(self, forward: bool, passthrough: bool) -> Self {
        let index = self.index().unwrap_or(0);
        let count = PAGES.len();
        (1..count)
            .map(|offset| {
                if forward {
                    PAGES[(index + offset) % count]
                } else {
                    PAGES[(index + count - offset) % count]
                }
            })
            .find(|page| page.available(passthrough))
            .unwrap_or(Self::Overview)
    }

    /// Index into `PAGES`; `None` for the settings page, which is outside
    /// the cycle.
    fn index(self) -> Option<usize> {
        match self {
            Self::Overview => Some(0),
            Self::Lines => Some(1),
            Self::Fixed { .. } => Some(2),
            Self::Pps(_) => Some(3),
            Self::Qc(_) => Some(4),
            Self::Detail { page, .. } => Some(FIRST_DETAIL + page as usize),
            Self::Settings(_) => None,
        }
    }

    /// One-based position among the available pages and their count;
    /// `None` for the settings page.
    pub fn position(self, passthrough: bool) -> Option<(u8, u8)> {
        let index = self.index()?;
        let shown = |page: &&Page| page.available(passthrough);
        Some((
            PAGES[..=index].iter().filter(shown).count() as u8,
            PAGES.iter().filter(shown).count() as u8,
        ))
    }

    /// Left/Right change a value or scroll here, rather than moving between
    /// pages or rows.
    pub fn editing(self) -> bool {
        matches!(
            self,
            Self::Fixed { choosing: true }
                | Self::Pps(PpsFocus::Voltage | PpsFocus::Current)
                | Self::Qc(QcFocus::Choosing { .. })
                | Self::Settings(SettingsFocus::Brightness | SettingsFocus::Rotation)
                | Self::Detail {
                    scrolling: true,
                    ..
                }
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
    /// Request the previewed profile.
    Confirm,
    /// Ask the cable for its E-marker identity (from fixed 5 V only).
    QueryCable,
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
    pub settings: Settings,
    /// Incremented by each peak reset request; the meter clears its peaks
    /// when this changes.
    pub peaks_reset: u8,
}

/// Navigation and settings, owned by the input task rather than the renderer,
/// so a render backlog never loses taps.
///
/// Left and Right act when released, unless the press already repeated; OK
/// acts when released unless it was held long.
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
    /// A probe this boot found the charger takes 20 V (Class B), so steps
    /// may go up to it; Class A stops at 12 V (NCP4371 Table 4).
    qc_class_b: bool,
    /// Last row the shown detail list can start at, as the renderer laid it
    /// out.
    list_end: u16,
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
        if !selectable && matches!(self.input.page, Page::Fixed { .. } | Page::Pps(_)) {
            self.input.page = PAGES[self.input.page.index().unwrap_or(0)];
        }
    }

    /// Whether the PD controller only listens; a page passthrough cannot
    /// use closes to the overview.
    pub fn set_passthrough(&mut self, passthrough: bool) {
        self.input.passthrough = passthrough;
        if !self.input.page.available(passthrough) {
            self.input.page = Page::Overview;
        }
    }

    /// The meter's plug sits in the charger (`plug_in_charger`), and this
    /// boot runs the 5 A cable test (`testing`). Then the full scan (a long
    /// OK on the charger page) ends in the test, and Back there ends it.
    /// The test keeps the charger page open (`update`).
    pub fn set_cable_test(&mut self, plug_in_charger: bool, testing: bool) {
        self.plug_in_charger = plug_in_charger;
        self.cable_test = testing;
        if testing && !self.input.page.is_charger() {
            self.input.page = Page::detail(Detail::Charger);
        }
    }

    /// Whether this boot keeps PD silent for Quick Charge, and whether the
    /// charger has had time to give PD up (`settled`). Quick Charge mode is
    /// saved only to outlast the charger's own restarts until then, so a
    /// later plug-in starts with PD again.
    pub fn set_silent(&mut self, silent: bool, settled: bool) {
        self.silent = silent;
        if settled && self.input.settings.next_boot == NextBoot::QuickCharge {
            self.input.settings.next_boot = NextBoot::Normal;
        }
    }

    /// A probe found the charger's class: Class B (`class_b`) raises the
    /// steps' ceiling from 12 V to 20 V.
    pub fn set_qc_class_b(&mut self, class_b: bool) {
        self.qc_class_b = class_b;
    }

    /// This boot runs the full scan's Quick Charge part, which keeps the
    /// probe on screen until it ends; Back abandons the scan.
    fn scanning(&self) -> bool {
        self.silent
            && matches!(
                self.input.settings.next_boot,
                NextBoot::ScanQuickCharge { .. }
            )
    }

    /// The scan's Quick Charge part ended with `findings`: keep them for its
    /// PD part and restart into that.
    pub fn finish_scan(&mut self, findings: QcFindings) -> Option<PdCommand> {
        if !self.scanning() {
            return None;
        }
        let settings = &mut self.input.settings;
        let NextBoot::ScanQuickCharge { then_cable_test } = settings.next_boot else {
            return None;
        };
        settings.qc_findings = Some(findings);
        settings.next_boot = if then_cable_test {
            NextBoot::CableTest
        } else {
            NextBoot::ScanPd
        };
        Some(PdCommand::Restart)
    }

    /// The last row the shown detail list can start at; a list that got
    /// shorter scrolls back.
    pub fn set_list_end(&mut self, end: u16) {
        self.list_end = end;
        if let Page::Detail { top, .. } = &mut self.input.page {
            *top = (*top).min(end);
        }
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
                && matches!(
                    self.input.page,
                    Page::Pps(PpsFocus::Voltage | PpsFocus::Current)
                        | Page::Detail {
                            scrolling: true,
                            ..
                        }
                )
            {
                self.repeated = true;
                command = self.step(left_right == RIGHT, true);
            }
        } else if left_right == 0 {
            self.repeated = false;
            self.held = 0;
        }

        if after & OK != 0 {
            self.ok_held += 1;
            if self.ok_held == LONG_OK_SAMPLES
                && let Some(query) = self.long_ok()
            {
                self.ok_long = true;
                command = query;
            }
        } else {
            if falling & OK != 0 && !self.ok_long {
                command = self.ok().or(command);
            }
            self.ok_held = 0;
            self.ok_long = false;
        }
        if rising & BACK != 0 {
            let leave_silent =
                self.scanning() || (self.silent && self.input.page == Page::Qc(QcFocus::Off));
            if self.cable_test && self.input.page.is_charger() {
                command = Some(PdCommand::Restart);
            } else if leave_silent {
                self.input.settings.next_boot = NextBoot::Normal;
                command = Some(PdCommand::Restart);
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
        if self.scanning() {
            self.input.page = Page::Qc(QcFocus::Requested(QcTarget::Detect));
        }
        if self.silent {
            command = command.filter(|c| *c == PdCommand::Restart);
        }
        (self.input, command)
    }

    /// What a long OK does here: `None` when the page has no long OK, else
    /// the PD command it sends, if any.
    fn long_ok(&mut self) -> Option<Option<PdCommand>> {
        let Page::Detail { page, .. } = self.input.page else {
            return None;
        };
        match page {
            Detail::Cable => Some(Some(PdCommand::QueryCable)),
            Detail::Charger if self.cable_test => Some(Some(PdCommand::QueryEpr)),
            // The full scan: Quick Charge first, with PD silent (a charger
            // ignores it under a PD contract), then PD again with its EPR
            // offer, ending here.
            Detail::Charger if self.selectable => {
                self.input.settings.next_boot = NextBoot::ScanQuickCharge {
                    then_cable_test: self.plug_in_charger,
                };
                Some(Some(PdCommand::Restart))
            }
            // Without PD, Quick Charge answers directly.
            Detail::Charger => {
                self.input.page = Page::Qc(QcFocus::Requested(QcTarget::Detect));
                Some(None)
            }
            _ => None,
        }
    }

    fn back(&mut self) {
        let page = &mut self.input.page;
        *page = match *page {
            Page::Overview => Page::Settings(SettingsFocus::default()),
            Page::Fixed { choosing: true } => Page::Fixed { choosing: false },
            Page::Pps(PpsFocus::Voltage | PpsFocus::Current) => Page::Pps(PpsFocus::Browse),
            Page::Qc(QcFocus::Choosing { request, .. }) => Page::Qc(QcFocus::Requested(request)),
            Page::Qc(QcFocus::Requested(_)) => Page::Qc(QcFocus::Off),
            Page::Settings(focus @ (SettingsFocus::Brightness | SettingsFocus::Rotation)) => {
                Page::Settings(SettingsFocus::Row(focus.row()))
            }
            Page::Detail {
                page,
                top,
                scrolling: true,
            } => Page::Detail {
                page,
                top,
                scrolling: false,
            },
            _ => Page::Overview,
        };
    }

    /// Left (`forward` false) or Right; `repeat` for a held button.
    fn step(&mut self, forward: bool, repeat: bool) -> Option<PdCommand> {
        let sign = if forward { 1 } else { -1 };
        let input = &mut self.input;
        match &mut input.page {
            Page::Detail {
                top,
                scrolling: true,
                ..
            } => {
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
            Page::Fixed { choosing: true } => return Some(PdCommand::StepFixed(sign as i8)),
            Page::Qc(QcFocus::Choosing { preview, .. }) => {
                *preview = cycle(&QcTarget::ALL, preview.index(), forward);
            }
            Page::Qc(QcFocus::Requested(QcTarget::Steps(mv))) => {
                let max = if self.qc_class_b {
                    Voltage::V20
                } else {
                    Voltage::V12
                };
                *mv = if forward {
                    mv.saturating_add(STEP_MV).min(max.millivolts())
                } else {
                    mv.saturating_sub(STEP_MV).max(MIN_MV)
                };
            }
            Page::Pps(PpsFocus::Voltage) => {
                let step = if repeat { PPS_MV_REPEAT } else { PPS_MV_STEP };
                return Some(PdCommand::PpsVoltage(sign * step));
            }
            Page::Pps(PpsFocus::Current) => {
                let step = if repeat { PPS_MA_REPEAT } else { PPS_MA_STEP };
                return Some(PdCommand::PpsCurrent(sign * step));
            }
            page => *page = page.next(forward, input.passthrough),
        }
        None
    }

    fn ok(&mut self) -> Option<PdCommand> {
        let input = &mut self.input;
        let (page, command) = match input.page {
            Page::Detail {
                page,
                top,
                scrolling,
            } => (
                Page::Detail {
                    page,
                    top,
                    scrolling: !scrolling,
                },
                None,
            ),
            page @ (Page::Fixed { choosing: false } | Page::Pps(PpsFocus::Browse))
                if !self.selectable =>
            {
                (page, None)
            }
            Page::Fixed { choosing: false } => (
                Page::Fixed { choosing: true },
                Some(PdCommand::StepFixed(0)),
            ),
            Page::Fixed { choosing: true } => {
                (Page::Fixed { choosing: false }, Some(PdCommand::Confirm))
            }
            Page::Pps(PpsFocus::Browse) => {
                (Page::Pps(PpsFocus::Voltage), Some(PdCommand::PpsVoltage(0)))
            }
            Page::Pps(PpsFocus::Voltage) => (Page::Pps(PpsFocus::Current), None),
            Page::Pps(PpsFocus::Current) => (Page::Pps(PpsFocus::Browse), Some(PdCommand::Confirm)),
            Page::Settings(SettingsFocus::Row(item)) => (
                Page::Settings(match item {
                    Item::Brightness => SettingsFocus::Brightness,
                    Item::Rotation => SettingsFocus::Rotation,
                    Item::ResetPeaks => {
                        input.peaks_reset = input.peaks_reset.wrapping_add(1);
                        SettingsFocus::Row(item)
                    }
                }),
                None,
            ),
            Page::Settings(focus) => (Page::Settings(SettingsFocus::Row(focus.row())), None),
            // A PD contract hides Quick Charge: restart into a silent boot.
            page @ Page::Qc(QcFocus::Off) if self.selectable => {
                input.settings.next_boot = NextBoot::QuickCharge;
                (page, Some(PdCommand::Restart))
            }
            Page::Qc(focus) => (
                Page::Qc(match focus {
                    QcFocus::Off => QcFocus::Requested(QcTarget::Volts(Voltage::V5)),
                    QcFocus::Requested(request) => QcFocus::Choosing {
                        request,
                        preview: request,
                    },
                    QcFocus::Choosing { preview, .. } => QcFocus::Requested(preview),
                }),
                None,
            ),
            page @ (Page::Overview | Page::Lines) => (page, None),
        };
        input.page = page;
        command
    }
}

/// Convert active-low pin levels into a pressed mask.
pub fn pressed_from_levels(low_active_levels: u8) -> u8 {
    !low_active_levels & ((1u8 << COUNT) - 1)
}
