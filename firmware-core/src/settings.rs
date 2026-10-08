//! Display settings changed on the settings page. They live in RAM only and
//! return to the defaults at every boot; nothing is written to flash.

/// Backlight duty levels, percent of the PWM period. The timer needs a
/// compare inside the period, so the top level stays below 100%. 60% is the
/// retained, hardware-tested level.
pub const BRIGHTNESS_PERCENT: [u8; 5] = [20, 40, 60, 80, 95];
/// Index of the 60% level in [`BRIGHTNESS_PERCENT`].
const DEFAULT_BRIGHTNESS: u8 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Settings {
    /// Index into [`BRIGHTNESS_PERCENT`].
    pub brightness: u8,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            brightness: DEFAULT_BRIGHTNESS,
        }
    }
}

impl Settings {
    /// One level brighter (`brighter`) or dimmer, stopping at either end.
    pub fn step_brightness(&mut self, brighter: bool) {
        let top = BRIGHTNESS_PERCENT.len() as u8 - 1;
        self.brightness = if brighter {
            (self.brightness + 1).min(top)
        } else {
            self.brightness.saturating_sub(1)
        };
    }

    pub fn brightness_percent(&self) -> u8 {
        BRIGHTNESS_PERCENT[usize::from(self.brightness) % BRIGHTNESS_PERCENT.len()]
    }
}

/// Settings page rows, in cursor order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Item {
    #[default]
    Brightness,
    ResetPeaks,
}

impl Item {
    pub const ALL: [Self; 2] = [Self::Brightness, Self::ResetPeaks];
}
