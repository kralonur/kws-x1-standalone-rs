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

/// Everyday pages, in Left/Right order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Page {
    #[default]
    Overview,
    /// Fixed PD profile selector.
    Fixed,
    /// PPS voltage and current selector.
    Pps,
    Settings,
    /// Detailed information, opened by holding Left and Right together.
    Advanced(Advanced),
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

const EVERYDAY: [Page; 4] = [Page::Overview, Page::Fixed, Page::Pps, Page::Settings];
const ADVANCED: [Advanced; 5] = [
    Advanced::Pd,
    Advanced::Charger,
    Advanced::Cable,
    Advanced::Ports,
    Advanced::Irq,
];

/// `items[index of current ± 1]`, wrapping.
fn cycle<T: Copy + PartialEq>(items: &[T], current: T, forward: bool) -> T {
    let index = items.iter().position(|item| *item == current).unwrap_or(0);
    let next = if forward {
        index + 1
    } else {
        index + items.len() - 1
    };
    items[next % items.len()]
}

impl Page {
    /// One-based position in its group (everyday or advanced) and the group size.
    pub fn position(self) -> (u8, u8) {
        let (index, count) = match self {
            Self::Advanced(page) => (
                ADVANCED.iter().position(|item| *item == page),
                ADVANCED.len(),
            ),
            page => (
                EVERYDAY.iter().position(|item| *item == page),
                EVERYDAY.len(),
            ),
        };
        (index.map_or(0, |index| index as u8 + 1), count as u8)
    }
}

/// What Left, Right and OK are editing on the current page. `None` means
/// Left/Right move between pages.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Edit {
    #[default]
    None,
    /// Choosing a fixed profile.
    Fixed,
    PpsVoltage,
    PpsCurrent,
    /// Moving between settings rows.
    SettingsRows,
    Brightness,
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
    pub edit: Edit,
    /// Sub-page of the advanced charger and cable pages.
    pub detail: u32,
    pub settings: Settings,
    /// Highlighted settings row.
    pub cursor: Item,
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
}

impl Navigation {
    /// Start on `page`, e.g. a diagnostic boot page.
    pub fn at_page(page: Page) -> Self {
        let mut navigation = Self::default();
        navigation.input.page = page;
        navigation
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
                && matches!(self.input.edit, Edit::PpsVoltage | Edit::PpsCurrent)
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
            if self.ok_held == LONG_OK_SAMPLES && self.input.page == Page::Advanced(Advanced::Cable)
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

    fn go(&mut self, page: Page) {
        self.input.page = page;
        self.input.edit = Edit::None;
        self.input.detail = 0;
    }

    fn toggle_advanced(&mut self) {
        self.go(match self.input.page {
            Page::Advanced(_) => Page::Overview,
            _ => Page::Advanced(Advanced::default()),
        });
    }

    fn back(&mut self) {
        let input = &mut self.input;
        match input.edit {
            Edit::Brightness => input.edit = Edit::SettingsRows,
            Edit::None => self.go(Page::Overview),
            _ => input.edit = Edit::None,
        }
    }

    /// Left (`forward` false) or Right; `repeat` for a held button.
    fn step(&mut self, forward: bool, repeat: bool) -> Option<PdCommand> {
        let sign = if forward { 1 } else { -1 };
        let input = &mut self.input;
        match (input.page, input.edit) {
            (Page::Advanced(_), _) => {
                input.detail = if forward {
                    input.detail.wrapping_add(1) & 0x00ff_ffff
                } else {
                    input.detail.saturating_sub(1)
                };
            }
            (page, Edit::None) => self.go(cycle(&EVERYDAY, page, forward)),
            (_, Edit::Fixed) => return Some(PdCommand::StepFixed(sign as i8)),
            (_, Edit::PpsVoltage) => {
                let step = if repeat { PPS_MV_REPEAT } else { PPS_MV_STEP };
                return Some(PdCommand::PpsVoltage(sign * step));
            }
            (_, Edit::PpsCurrent) => {
                let step = if repeat { PPS_MA_REPEAT } else { PPS_MA_STEP };
                return Some(PdCommand::PpsCurrent(sign * step));
            }
            (_, Edit::SettingsRows) => input.cursor = cycle(&Item::ALL, input.cursor, forward),
            (_, Edit::Brightness) => input.settings.step_brightness(forward),
        }
        None
    }

    fn ok(&mut self) -> Option<PdCommand> {
        let input = &mut self.input;
        match (input.page, input.edit) {
            (Page::Advanced(page), _) => self.go(Page::Advanced(cycle(&ADVANCED, page, true))),
            (Page::Fixed, Edit::None) => {
                input.edit = Edit::Fixed;
                return Some(PdCommand::StepFixed(0));
            }
            (Page::Pps, Edit::None) => {
                input.edit = Edit::PpsVoltage;
                return Some(PdCommand::PpsVoltage(0));
            }
            (_, Edit::PpsVoltage) => input.edit = Edit::PpsCurrent,
            (_, Edit::Fixed | Edit::PpsCurrent) => {
                input.edit = Edit::None;
                return Some(PdCommand::Confirm);
            }
            (Page::Settings, Edit::None) => input.edit = Edit::SettingsRows,
            (_, Edit::SettingsRows) => match input.cursor {
                Item::Brightness => input.edit = Edit::Brightness,
                Item::ResetPeaks => input.peaks_reset = input.peaks_reset.wrapping_add(1),
            },
            (_, Edit::Brightness) => input.edit = Edit::SettingsRows,
            (Page::Overview, Edit::None) => {}
        }
        None
    }
}

/// Convert active-low pin levels into a pressed mask.
pub fn pressed_from_levels(low_active_levels: u8) -> u8 {
    !low_active_levels & ((1u8 << COUNT) - 1)
}
