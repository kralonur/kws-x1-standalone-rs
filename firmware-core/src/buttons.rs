//! Button sampling and debouncing (pure logic).
//!
//! The original firmware configures four button inputs as active-low with
//! pull-ups: PB2, PB10, PH0 and PC14, each treating a low level as pressed.
//! Physical labels are still unknown, so this module reports pin-level states
//! rather than names.
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

/// Button roles, as bits of the stable pressed mask (bit `n` is `PINS[n]`).
/// PB2 steps (sub-pages, settings cursor, PD preview), PB10 selects (settings
/// item, peak reset, PD confirm), PH0 advances the page and PC14 returns home.
pub const STEP: u8 = 1 << 0;
pub const SELECT: u8 = 1 << 1;
pub const NEXT_PAGE: u8 = 1 << 2;
pub const HOME: u8 = 1 << 3;

/// Screen pages, in [`NEXT_PAGE`] order. The two diagnostic pages are only
/// in the cycle while [`Settings::debug_pages`] is on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Page {
    #[default]
    Meter,
    /// PD session and the fixed/PPS option selection.
    Pd,
    /// What the charger said about itself, on sub-pages.
    Charger,
    /// Cable E-marker details, on sub-pages.
    Cable,
    Settings,
    /// GPIO port levels and frame timing.
    Ports,
    /// Interrupt and I2C fault counters.
    Irq,
}

impl Page {
    fn next(self, debug_pages: bool) -> Self {
        match self {
            Self::Meter => Self::Pd,
            Self::Pd => Self::Charger,
            Self::Charger => Self::Cable,
            Self::Cable => Self::Settings,
            Self::Settings if debug_pages => Self::Ports,
            Self::Ports => Self::Irq,
            Self::Settings | Self::Irq => Self::Meter,
        }
    }

    /// One-based position in the page cycle and the cycle length; the
    /// position is 0 for a diagnostic page outside the cycle.
    pub fn position(self, debug_pages: bool) -> (u8, u8) {
        let (mut page, mut index, mut count) = (Self::Meter, 0, 0);
        loop {
            count += 1;
            if page == self {
                index = count;
            }
            page = page.next(debug_pages);
            if page == Self::Meter {
                return (index, count);
            }
        }
    }

    /// Pages whose sub-page [`STEP`] and [`SELECT`] move.
    fn has_detail(self) -> bool {
        matches!(self, Self::Charger | Self::Cable)
    }
}

/// One coherent input state, published by the input task as a unit.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Input {
    /// Stable pressed mask; bit `n` is `PINS[n]`.
    pub pressed: u8,
    pub page: Page,
    /// Sub-page of the charger and cable pages.
    pub detail: u32,
    pub settings: Settings,
    /// Highlighted settings row.
    pub cursor: Item,
    /// Incremented by each peak reset request; the meter clears its peaks
    /// when this changes.
    pub peaks_reset: u8,
}

/// Navigation and settings, owned by the input task rather than the renderer,
/// so a render backlog never loses taps. [`HOME`] wins over a simultaneous
/// [`NEXT_PAGE`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Navigation {
    debounce: Debounced,
    input: Input,
}

impl Navigation {
    /// Start on `page`, e.g. a diagnostic boot page.
    pub fn at_page(page: Page) -> Self {
        let mut navigation = Self::default();
        navigation.input.page = page;
        navigation
    }

    /// Feed one sample and return the resulting input state.
    pub fn update(&mut self, raw_pressed: u8) -> Input {
        let before = self.debounce.stable();
        let after = self.debounce.update(raw_pressed, DEFAULT_THRESHOLD);
        let rising = after & !before;
        let input = &mut self.input;
        input.pressed = after;
        if rising & NEXT_PAGE != 0 {
            input.page = input.page.next(input.settings.debug_pages);
            input.detail = 0;
        }
        if rising & HOME != 0 {
            input.page = Page::Meter;
            input.detail = 0;
        }
        match input.page {
            page if page.has_detail() => {
                if rising & STEP != 0 {
                    input.detail = input.detail.wrapping_add(1) & 0x00ff_ffff;
                }
                if rising & SELECT != 0 {
                    input.detail = input.detail.saturating_sub(1);
                }
            }
            Page::Settings => {
                if rising & STEP != 0 {
                    input.cursor = input.cursor.next();
                }
                if rising & SELECT != 0 {
                    match input.cursor {
                        Item::Brightness => input.settings.brighter(),
                        Item::ResetPeaks => input.peaks_reset = input.peaks_reset.wrapping_add(1),
                        Item::DebugPages => {
                            input.settings.debug_pages = !input.settings.debug_pages;
                        }
                    }
                }
            }
            Page::Meter if rising & SELECT != 0 => {
                input.peaks_reset = input.peaks_reset.wrapping_add(1);
            }
            _ => {}
        }
        *input
    }
}

/// Convert active-low pin levels into a pressed mask.
pub fn pressed_from_levels(low_active_levels: u8) -> u8 {
    !low_active_levels & ((1u8 << COUNT) - 1)
}
