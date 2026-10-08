//! Button sampling and debouncing (pure logic).
//!
//! The original firmware configures four button inputs as active-low with
//! pull-ups: PB2, PB10, PH0 and PC14, each treating a low level as pressed.
//! They are the Left, Right, OK and Back buttons, as identified on
//! the device.
#![allow(dead_code)]

use crate::settings::{Item, Settings};

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
const LEFT_RIGHT: u8 = LEFT | RIGHT;

/// Button sampling period in milliseconds; the hold times below are counted
/// in these samples.
pub const POLL_MS: u32 = 10;
/// Left and Right held together this long toggle the advanced pages.
const CHORD_SAMPLES: u32 = 500 / POLL_MS;
/// A long OK press (cable query on the advanced cable page).
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
/// Left/Right cycle the everyday pages; Back on the overview opens the
/// settings.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Page {
    #[default]
    Overview,
    /// Fixed PD profile selector; while `choosing`, Left/Right preview
    /// profiles.
    Fixed { choosing: bool },
    /// PPS voltage and current selector.
    Pps(PpsFocus),
    /// Opened by Back on the overview.
    Settings(SettingsFocus),
    /// Detailed information, opened by holding Left and Right together;
    /// `detail` is the sub-page of the charger and cable pages.
    Advanced { page: Advanced, detail: u32 },
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

/// Detailed information pages, in OK order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Advanced {
    /// PD session state.
    #[default]
    Pd,
    /// What the charger said about itself, on sub-pages.
    Charger,
    /// Cable E-marker details, on sub-pages.
    Cable,
    /// GPIO port levels and frame timing.
    Ports,
    /// Interrupt and I2C fault counters.
    Irq,
}

/// The everyday pages as Left/Right enter them.
const EVERYDAY: [Page; 3] = [
    Page::Overview,
    Page::Fixed { choosing: false },
    Page::Pps(PpsFocus::Browse),
];
const ADVANCED: [Advanced; 5] = [
    Advanced::Pd,
    Advanced::Charger,
    Advanced::Cable,
    Advanced::Ports,
    Advanced::Irq,
];

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
    /// An advanced page at its first sub-page.
    pub const fn advanced(page: Advanced) -> Self {
        Self::Advanced { page, detail: 0 }
    }

    /// Index into `EVERYDAY`.
    fn everyday(self) -> Option<usize> {
        match self {
            Self::Overview => Some(0),
            Self::Fixed { .. } => Some(1),
            Self::Pps(_) => Some(2),
            Self::Settings(_) | Self::Advanced { .. } => None,
        }
    }

    /// One-based position in its group (everyday or advanced) and the group
    /// size; `None` for the settings page, which is outside both.
    pub fn position(self) -> Option<(u8, u8)> {
        let (index, count) = match self {
            Self::Advanced { page, .. } => (page as usize, ADVANCED.len()),
            page => (page.everyday()?, EVERYDAY.len()),
        };
        Some((index as u8 + 1, count as u8))
    }

    /// Left/Right change a value here, rather than moving between pages or
    /// rows.
    pub fn editing(self) -> bool {
        matches!(
            self,
            Self::Fixed { choosing: true }
                | Self::Pps(PpsFocus::Voltage | PpsFocus::Current)
                | Self::Settings(SettingsFocus::Brightness | SettingsFocus::Rotation)
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
}

/// One coherent input state, published by the input task as a unit.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Input {
    /// Stable pressed mask; bit `n` is `PINS[n]`.
    pub pressed: u8,
    pub page: Page,
    pub settings: Settings,
    /// Incremented by each peak reset request; the meter clears its peaks
    /// when this changes.
    pub peaks_reset: u8,
}

/// Navigation and settings, owned by the input task rather than the renderer,
/// so a render backlog never loses taps.
///
/// Left and Right act when released, so a press that becomes a Left+Right
/// chord does nothing else; OK acts when released unless it was held long.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Navigation {
    debounce: Debounced,
    input: Input,
    /// Both Left and Right were down during the current press.
    chord: bool,
    /// The current Left/Right press already repeated.
    repeated: bool,
    /// Samples the current Left/Right press (or chord) has been held.
    held: u32,
    /// Samples OK has been held, and whether that hold already acted.
    ok_held: u32,
    ok_long: bool,
    /// A sink session runs, so the PD and PPS pages can select.
    selectable: bool,
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
            self.input.page = EVERYDAY[self.input.page.everyday().unwrap_or(0)];
        }
    }

    /// Feed one sample; returns the input state and any PD session request.
    pub fn update(&mut self, raw_pressed: u8) -> (Input, Option<PdCommand>) {
        let before = self.debounce.stable();
        let after = self.debounce.update(raw_pressed, DEFAULT_THRESHOLD);
        let rising = after & !before;
        let falling = before & !after;
        self.input.pressed = after;
        let mut command = None;

        // A lone Left/Right press steps when released, unless it repeated.
        if !self.chord && !self.repeated {
            if falling & LEFT != 0 {
                command = self.step(false, false);
            } else if falling & RIGHT != 0 {
                command = self.step(true, false);
            }
        }
        let left_right = after & LEFT_RIGHT;
        if left_right == LEFT_RIGHT {
            if !self.chord {
                self.chord = true;
                self.held = 0;
            }
            self.held += 1;
            if self.held == CHORD_SAMPLES {
                self.toggle_advanced();
            }
        } else if left_right != 0 && !self.chord {
            self.held += 1;
            if self.held >= REPEAT_DELAY_SAMPLES
                && (self.held - REPEAT_DELAY_SAMPLES).is_multiple_of(REPEAT_SAMPLES)
                && matches!(
                    self.input.page,
                    Page::Pps(PpsFocus::Voltage | PpsFocus::Current)
                )
            {
                self.repeated = true;
                command = self.step(left_right == RIGHT, true);
            }
        } else if left_right == 0 {
            self.chord = false;
            self.repeated = false;
            self.held = 0;
        }

        if after & OK != 0 {
            self.ok_held += 1;
            if self.ok_held == LONG_OK_SAMPLES
                && matches!(
                    self.input.page,
                    Page::Advanced {
                        page: Advanced::Cable,
                        ..
                    }
                )
            {
                self.ok_long = true;
                command = Some(PdCommand::QueryCable);
            }
        } else {
            if falling & OK != 0 && !self.ok_long {
                command = self.ok().or(command);
            }
            self.ok_held = 0;
            self.ok_long = false;
        }
        if rising & BACK != 0 {
            self.back();
        }
        (self.input, command)
    }

    fn toggle_advanced(&mut self) {
        self.input.page = match self.input.page {
            Page::Advanced { .. } => Page::Overview,
            _ => Page::advanced(Advanced::default()),
        };
    }

    fn back(&mut self) {
        let page = &mut self.input.page;
        *page = match *page {
            Page::Overview => Page::Settings(SettingsFocus::default()),
            Page::Fixed { choosing: true } => Page::Fixed { choosing: false },
            Page::Pps(PpsFocus::Voltage | PpsFocus::Current) => Page::Pps(PpsFocus::Browse),
            Page::Settings(focus @ (SettingsFocus::Brightness | SettingsFocus::Rotation)) => {
                Page::Settings(SettingsFocus::Row(focus.row()))
            }
            _ => Page::Overview,
        };
    }

    /// Left (`forward` false) or Right; `repeat` for a held button.
    fn step(&mut self, forward: bool, repeat: bool) -> Option<PdCommand> {
        let sign = if forward { 1 } else { -1 };
        let input = &mut self.input;
        match &mut input.page {
            Page::Advanced { detail, .. } => {
                *detail = if forward {
                    detail.wrapping_add(1) & 0x00ff_ffff
                } else {
                    detail.saturating_sub(1)
                };
            }
            Page::Settings(SettingsFocus::Row(item)) => {
                let index = Item::ALL.iter().position(|row| row == item).unwrap_or(0);
                *item = cycle(&Item::ALL, index, forward);
            }
            Page::Settings(SettingsFocus::Brightness) => input.settings.step_brightness(forward),
            Page::Settings(SettingsFocus::Rotation) => input.settings.step_rotation(forward),
            Page::Fixed { choosing: true } => return Some(PdCommand::StepFixed(sign as i8)),
            Page::Pps(PpsFocus::Voltage) => {
                let step = if repeat { PPS_MV_REPEAT } else { PPS_MV_STEP };
                return Some(PdCommand::PpsVoltage(sign * step));
            }
            Page::Pps(PpsFocus::Current) => {
                let step = if repeat { PPS_MA_REPEAT } else { PPS_MA_STEP };
                return Some(PdCommand::PpsCurrent(sign * step));
            }
            page => {
                let index = page.everyday().unwrap_or(0);
                *page = cycle(&EVERYDAY, index, forward);
            }
        }
        None
    }

    fn ok(&mut self) -> Option<PdCommand> {
        let input = &mut self.input;
        let (page, command) = match input.page {
            Page::Advanced { page, .. } => {
                (Page::advanced(cycle(&ADVANCED, page as usize, true)), None)
            }
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
            Page::Overview => (Page::Overview, None),
        };
        input.page = page;
        command
    }
}

/// Convert active-low pin levels into a pressed mask.
pub fn pressed_from_levels(low_active_levels: u8) -> u8 {
    !low_active_levels & ((1u8 << COUNT) - 1)
}
