//! Button sampling and debouncing (pure logic).
//!
//! The original firmware configures four button inputs as active-low with
//! pull-ups: PB2, PB10, PH0 and PC14, each treating a low level as pressed.
//! Physical labels are still unknown, so this module reports pin-level states
//! rather than names.
#![allow(dead_code)]

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

/// Meter navigation owned by the input task, not the renderer. PH0 advances
/// through six pages; PC14 returns to the meter page (and wins a simultaneous
/// press). Publishing the latest snapshot avoids a render backlog losing taps.
/// Meter, ports, IRQ test, PD, E-marker details, charger info.
pub const PAGES: u8 = 6;

/// One coherent input state, published by the input task as a unit.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Input {
    /// Stable pressed mask; bit `n` is `PINS[n]`.
    pub pressed: u8,
    /// Current page, `0..PAGES`.
    pub page: u8,
    /// Sub-page of the E-marker detail and charger info pages.
    pub detail: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Navigation {
    debounce: Debounced,
    page: u8,
    detail: u32,
}

impl Navigation {
    /// Start on `page` (taken modulo the pages), e.g. a diagnostic boot page.
    pub fn at_page(page: u8) -> Self {
        Self {
            page: page % PAGES,
            ..Self::default()
        }
    }

    /// Feed one sample and return the resulting input state.
    pub fn update(&mut self, raw_pressed: u8) -> Input {
        let before = self.debounce.stable();
        let after = self.debounce.update(raw_pressed, DEFAULT_THRESHOLD);
        let rising = after & !before;
        if rising & 4 != 0 {
            self.page = (self.page + 1) % PAGES;
            self.detail = 0;
        }
        if rising & 8 != 0 {
            self.page = 0;
            self.detail = 0;
        }
        // E-marker detail (4) and charger info (5) pages step with PB2/PB10.
        if self.page >= 4 {
            if rising & 1 != 0 {
                self.detail = self.detail.wrapping_add(1) & 0x00ff_ffff;
            }
            if rising & 2 != 0 {
                self.detail = self.detail.saturating_sub(1);
            }
        }
        Input {
            pressed: after,
            page: self.page,
            detail: self.detail,
        }
    }
}

/// Convert active-low pin levels into a pressed mask.
pub fn pressed_from_levels(low_active_levels: u8) -> u8 {
    !low_active_levels & ((1u8 << COUNT) - 1)
}
