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
    /// Add the port and interrupt diagnostic pages to the page cycle.
    pub debug_pages: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            brightness: DEFAULT_BRIGHTNESS,
            debug_pages: false,
        }
    }
}

impl Settings {
    /// The next brightness level, wrapping from the brightest to the dimmest.
    pub fn brighter(&mut self) {
        self.brightness = (self.brightness + 1) % BRIGHTNESS_PERCENT.len() as u8;
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
    DebugPages,
}

impl Item {
    pub const ALL: [Self; 3] = [Self::Brightness, Self::ResetPeaks, Self::DebugPages];

    pub fn next(self) -> Self {
        match self {
            Self::Brightness => Self::ResetPeaks,
            Self::ResetPeaks => Self::DebugPages,
            Self::DebugPages => Self::Brightness,
        }
    }
}
