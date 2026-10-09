//! Display settings changed on the settings page, and their flash record
//! encoding. The firmware stores the record; this module only packs it.

/// Backlight duty levels, percent of the PWM period. The timer needs a
/// compare inside the period, so the top level stays below 100%. 60% is the
/// retained, hardware-tested level.
pub const BRIGHTNESS_PERCENT: [u8; 5] = [20, 40, 60, 80, 95];
/// Index of the 60% level in [`BRIGHTNESS_PERCENT`].
const DEFAULT_BRIGHTNESS: u8 = 2;

/// Screen orientation: follow the tilt sensor, or stay fixed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Rotation {
    #[default]
    Auto,
    Normal,
    Flipped,
}

impl Rotation {
    const ALL: [Self; 3] = [Self::Auto, Self::Normal, Self::Flipped];
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Settings {
    /// Index into [`BRIGHTNESS_PERCENT`].
    pub brightness: u8,
    pub rotation: Rotation,
    /// Start the next boot in the 5 A cable test. Set just before the meter
    /// restarts itself into the test; that boot clears it.
    pub cable_test: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            brightness: DEFAULT_BRIGHTNESS,
            rotation: Rotation::Auto,
            cable_test: false,
        }
    }
}

/// Record tag in the top byte of an encoded settings word, so a blank or
/// foreign word never decodes.
const RECORD_TAG: u32 = 0x5e;

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

    /// The next (`forward`) or previous rotation choice, wrapping.
    pub fn step_rotation(&mut self, forward: bool) {
        let all = Rotation::ALL;
        let index = all.iter().position(|r| *r == self.rotation).unwrap_or(0);
        let next = if forward {
            index + 1
        } else {
            index + all.len() - 1
        };
        self.rotation = all[next % all.len()];
    }

    pub fn brightness_percent(&self) -> u8 {
        BRIGHTNESS_PERCENT[usize::from(self.brightness) % BRIGHTNESS_PERCENT.len()]
    }

    /// Pack into one word: tag in bits 31..24, the cable test in 16,
    /// rotation in 15..8, brightness in 7..0.
    pub fn encode(&self) -> u32 {
        let rotation = Rotation::ALL
            .iter()
            .position(|r| *r == self.rotation)
            .unwrap_or(0) as u32;
        RECORD_TAG << 24
            | u32::from(self.cable_test) << 16
            | rotation << 8
            | u32::from(self.brightness)
    }

    /// Unpack a word from [`Self::encode`]; anything else is `None`.
    pub fn decode(word: u32) -> Option<Self> {
        if word >> 24 != RECORD_TAG || word >> 16 & 0xff > 1 {
            return None;
        }
        let brightness = word as u8;
        let rotation = *Rotation::ALL.get(usize::from((word >> 8) as u8))?;
        (usize::from(brightness) < BRIGHTNESS_PERCENT.len()).then_some(Self {
            brightness,
            rotation,
            cable_test: word >> 16 & 1 != 0,
        })
    }
}

/// Settings page rows, in cursor order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Item {
    #[default]
    Brightness,
    Rotation,
    ResetPeaks,
}

impl Item {
    pub const ALL: [Self; 3] = [Self::Brightness, Self::Rotation, Self::ResetPeaks];
}
