//! Per-unit corrections to the current and bus voltage readings, the
//! calibration steps that find them, and their settings records.
//!
//! `INA` is the INA226 data sheet; the README defines the cited revision.

/// Current register LSB in µA: 0.5 mA, set by the firmware's calibration
/// register value for the 5 mOhm shunt (INA Equation 1).
pub const CURRENT_LSB_UA: i64 = 500;
/// Bus Voltage register LSB in µV (INA Table 7-8).
pub const BUS_LSB_UV: i64 = 1250;
/// Corrections are kept in parts per million.
const PPM: i64 = 1_000_000;
/// Largest correction: ±5%. The INA226's gain errors are at most 0.1% (INA
/// Electrical Characteristics) and a current shunt is typically within 1%,
/// so a reference further off than this is more likely wrong than the meter.
pub const MAX_CORRECTION_PPM: i64 = 50_000;
/// Largest zero offset: ten times the INA226's 10 µV worst-case shunt
/// offset (INA Electrical Characteristics) across the 5 mOhm shunt, 2 mA.
/// More current than this is a load, not an offset.
pub const MAX_ZERO_UA: i64 = 20_000;
/// Least current a current correction is taken at: 0.5 A, where one 0.5 mA
/// register step is 0.1%.
pub const MIN_LOAD_UA: i64 = 500_000;
/// Least bus voltage a voltage correction is taken at: 4 V, below any USB
/// supply's 5 V, where one 1.25 mV register step is 0.03%.
pub const MIN_BUS_UV: i64 = 4_000_000;
/// Readings averaged for a calibration step: 1.6 s of the meter's 100 ms
/// reads, each itself an average of 64 conversions.
pub const AVERAGE_SAMPLES: usize = 16;

/// What the meter adds to its raw readings. The default changes nothing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Calibration {
    /// Current read with nothing drawing it, in µA, subtracted first.
    pub zero_ua: i32,
    /// Current and bus voltage gain corrections, in ppm.
    pub current_ppm: i32,
    pub voltage_ppm: i32,
}

/// `value` scaled by `ppm` and divided by 1000, toward zero.
/// The division is unsigned: the firmware already links unsigned 64-bit
/// division, and signed division would add a routine it has no room for.
fn scaled_milli(value: i64, ppm: i32) -> i64 {
    let gain = (PPM + i64::from(ppm)) as u64;
    let magnitude = (value.unsigned_abs() * gain / (PPM * 1000) as u64) as i64;
    if value < 0 { -magnitude } else { magnitude }
}

/// The correction that turns `measured` into `target` (both positive, in
/// µ-units), rounded to the nearest ppm; unsigned as in [`scaled_milli`].
fn correction(measured: i64, target: i64) -> Result<i32, Refusal> {
    let (measured, target) = (measured as u64, target as u64);
    let ppm = ((target * PPM as u64 + measured / 2) / measured) as i64 - PPM;
    if ppm.abs() > MAX_CORRECTION_PPM {
        return Err(Refusal::OutOfRange);
    }
    Ok(ppm as i32)
}

impl Calibration {
    /// Bus voltage in mV from the Bus Voltage register.
    pub fn millivolts(self, raw: u16) -> u32 {
        scaled_milli(i64::from(raw) * BUS_LSB_UV, self.voltage_ppm) as u32
    }

    /// Current in mA from the two's complement Current register (INA
    /// Table 7-1); negative when it flows from plug to receptacle.
    pub fn milliamps(self, raw: u16) -> i32 {
        let ua = i64::from(raw as i16) * CURRENT_LSB_UA - i64::from(self.zero_ua);
        scaled_milli(ua, self.current_ppm) as i32
    }

    /// The averaged current's magnitude in mA, as this calibration shows it.
    pub fn mean_milliamps(self, mean: Mean) -> u32 {
        scaled_milli(mean.ua - i64::from(self.zero_ua), self.current_ppm).unsigned_abs() as u32
    }

    /// The averaged bus voltage in mV, as this calibration shows it.
    pub fn mean_millivolts(self, mean: Mean) -> u32 {
        scaled_milli(mean.uv, self.voltage_ppm) as u32
    }

    /// Take `mean`, read with nothing drawing current, as the zero.
    pub fn zeroed(self, mean: Mean) -> Result<Self, Refusal> {
        if mean.ua.abs() > MAX_ZERO_UA {
            return Err(Refusal::Load);
        }
        Ok(Self {
            zero_ua: mean.ua as i32,
            ..self
        })
    }

    /// Correct the current so `mean` shows as `target_ma`.
    pub fn current_matched(self, mean: Mean, target_ma: u32) -> Result<Self, Refusal> {
        let measured = (mean.ua - i64::from(self.zero_ua)).abs();
        if measured < MIN_LOAD_UA {
            return Err(Refusal::NoLoad);
        }
        Ok(Self {
            current_ppm: correction(measured, i64::from(target_ma) * 1000)?,
            ..self
        })
    }

    /// Correct the bus voltage so `mean` shows as `target_mv`.
    pub fn voltage_matched(self, mean: Mean, target_mv: u32) -> Result<Self, Refusal> {
        if mean.uv < MIN_BUS_UV {
            return Err(Refusal::NoVoltage);
        }
        Ok(Self {
            voltage_ppm: correction(mean.uv, i64::from(target_mv) * 1000)?,
            ..self
        })
    }

    /// One settings record per field, see [`Found::read`].
    pub fn records(self) -> [u32; FIELDS] {
        let values = [self.zero_ua, self.current_ppm, self.voltage_ppm];
        core::array::from_fn(|field| {
            RECORD_TAG << 24 | (field as u32) << FIELD_SHIFT | values[field] as u32 & VALUE_MASK
        })
    }
}

/// Why a calibration step kept the previous value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// No averaged reading yet, or the bus is not read.
    NoReading,
    /// Zero: more current flows than an offset explains.
    Load,
    /// Current: too little flows to correct it precisely.
    NoLoad,
    /// Voltage: too little on the bus to correct it precisely.
    NoVoltage,
    /// Over [`MAX_CORRECTION_PPM`] off.
    OutOfRange,
}

impl Refusal {
    /// What the calibration page says; the limits it names are
    /// [`MIN_LOAD_UA`], [`MIN_BUS_UV`] and [`MAX_CORRECTION_PPM`].
    pub fn message(self) -> &'static str {
        match self {
            Self::NoReading => "No steady reading yet, try again",
            Self::Load => "Current flows: unplug the load",
            Self::NoLoad => "Needs a steady load of 0.5A or more",
            Self::NoVoltage => "Needs 4V or more",
            Self::OutOfRange => "Over 5% off: check your reference",
        }
    }
}

/// The last [`AVERAGE_SAMPLES`] raw readings, uncalibrated, in µV and µA.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mean {
    pub uv: i64,
    pub ua: i64,
}

/// Running window of raw readings for [`Mean`].
#[derive(Clone, Copy, Debug, Default)]
pub struct Average {
    samples: [(u16, u16); AVERAGE_SAMPLES],
    next: usize,
    filled: usize,
}

impl Average {
    /// Add one pair of raw Bus Voltage and Current registers.
    pub fn add(&mut self, raw_v: u16, raw_i: u16) {
        self.samples[self.next] = (raw_v, raw_i);
        self.next = (self.next + 1) % AVERAGE_SAMPLES;
        self.filled = (self.filled + 1).min(AVERAGE_SAMPLES);
    }

    /// Forget the window, e.g. while the bus is not read.
    pub fn clear(&mut self) {
        self.filled = 0;
    }

    /// The window's mean, once it is full.
    pub fn mean(&self) -> Option<Mean> {
        if self.filled < AVERAGE_SAMPLES {
            return None;
        }
        // In 32 bits, which the core divides in hardware: 16 full-scale
        // registers times their LSB stay below 2^32 (2^31 for the current).
        let count = AVERAGE_SAMPLES as u32;
        let (v, i) = self
            .samples
            .iter()
            .fold((0u32, 0i32), |(v, i), &(raw_v, raw_i)| {
                (v + u32::from(raw_v), i + i32::from(raw_i as i16))
            });
        Some(Mean {
            uv: i64::from(v * BUS_LSB_UV as u32 / count),
            ua: i64::from(i * CURRENT_LSB_UA as i32 / count as i32),
        })
    }
}

/// Calibration fields, each its own record: zero, current, voltage.
pub const FIELDS: usize = 3;
/// Record tag in the top byte, distinct from the settings word's, so
/// firmware without calibration skips these records.
const RECORD_TAG: u32 = 0xca;
/// The field index in bits 23..22, a two's complement value in 21..0.
const FIELD_SHIFT: u32 = 22;
const VALUE_MASK: u32 = (1 << FIELD_SHIFT) - 1;
/// Largest value a record may hold: the zero limit and the correction
/// limit both fit, anything larger is not a record this firmware wrote.
const MAX_VALUE: i64 = if MAX_ZERO_UA > MAX_CORRECTION_PPM {
    MAX_ZERO_UA
} else {
    MAX_CORRECTION_PPM
};

/// The newest record of each field found so far.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Found([Option<i32>; FIELDS]);

impl Found {
    /// Take `word` if it is a calibration record; later records win.
    pub fn read(&mut self, word: u32) -> bool {
        let field = (word >> FIELD_SHIFT & 0b11) as usize;
        // Sign-extend the 22-bit value.
        let value = ((word & VALUE_MASK) << (32 - FIELD_SHIFT)) as i32 >> (32 - FIELD_SHIFT);
        if word >> 24 != RECORD_TAG || field >= FIELDS || i64::from(value).abs() > MAX_VALUE {
            return false;
        }
        self.0[field] = Some(value);
        true
    }

    pub fn any(self) -> bool {
        self.0.iter().any(Option::is_some)
    }

    pub fn calibration(self) -> Calibration {
        let [zero_ua, current_ppm, voltage_ppm] = self.0.map(|field| field.unwrap_or(0));
        Calibration {
            zero_ua,
            current_ppm,
            voltage_ppm,
        }
    }
}

/// What an entry step corrects.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quantity {
    Current,
    Voltage,
}

/// A calibration step, in order; each can be skipped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// Hold OK with nothing drawing current.
    Zero,
    /// Left/Right move this meter's averaged reading at the step's start
    /// (`None` without one) by `adjust`, in mA or mV, to the reference
    /// meter's value.
    Entry {
        quantity: Quantity,
        start: Option<u32>,
        adjust: i32,
    },
}

impl Step {
    /// The value being entered: the start plus the adjustment.
    pub fn entered(self) -> Option<u32> {
        match self {
            Self::Entry {
                start: Some(start),
                adjust,
                ..
            } => Some(start.saturating_add_signed(adjust)),
            _ => None,
        }
    }
}

/// Entry steps per Left/Right press and per repeat, in mA or mV: one
/// display digit, then ten.
const ENTRY_STEP: i32 = 1;
const ENTRY_REPEAT: i32 = 10;

/// The calibration steps in progress: nothing is kept until the last OK.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Wizard {
    pub step: Step,
    /// The calibration so far, applied once the steps finish.
    pub pending: Calibration,
    /// Why the last OK kept the previous value.
    pub refusal: Option<Refusal>,
}

impl Wizard {
    pub fn new(calibration: Calibration) -> Self {
        Self {
            step: Step::Zero,
            pending: calibration,
            refusal: None,
        }
    }

    /// A held OK: on the zero step, take the zero and go on.
    pub fn hold_ok(&mut self, mean: Option<Mean>) {
        if self.step != Step::Zero {
            return;
        }
        match mean
            .ok_or(Refusal::NoReading)
            .and_then(|mean| self.pending.zeroed(mean))
        {
            Ok(zeroed) => {
                self.pending = zeroed;
                self.next(mean);
            }
            Err(refusal) => self.refusal = Some(refusal),
        }
    }

    /// OK: keep an unchanged value, else take the entered one, and go on.
    /// Returns the calibration once the last step is done.
    pub fn ok(&mut self, mean: Option<Mean>) -> Option<Calibration> {
        let Step::Entry {
            quantity, adjust, ..
        } = self.step
        else {
            self.next(mean);
            return None;
        };
        if let Some(target) = self.step.entered().filter(|_| adjust != 0) {
            let matched = mean
                .ok_or(Refusal::NoReading)
                .and_then(|mean| match quantity {
                    Quantity::Current => self.pending.current_matched(mean, target),
                    Quantity::Voltage => self.pending.voltage_matched(mean, target),
                });
            match matched {
                Ok(matched) => self.pending = matched,
                Err(refusal) => {
                    self.refusal = Some(refusal);
                    return None;
                }
            }
        }
        if quantity == Quantity::Voltage {
            return Some(self.pending);
        }
        self.next(mean);
        None
    }

    /// Left (`forward` false) or Right; `repeat` for a held button.
    pub fn step(&mut self, forward: bool, repeat: bool) {
        let size = if repeat { ENTRY_REPEAT } else { ENTRY_STEP };
        if let Step::Entry {
            start: Some(start),
            adjust,
            ..
        } = &mut self.step
        {
            // Never below zero.
            *adjust = (*adjust + if forward { size } else { -size }).max(-(*start as i32));
            self.refusal = None;
        }
    }

    /// The following step, starting from this meter's averaged reading.
    fn next(&mut self, mean: Option<Mean>) {
        let pending = self.pending;
        let quantity = match self.step {
            Step::Zero => Quantity::Current,
            Step::Entry { .. } => Quantity::Voltage,
        };
        self.refusal = None;
        self.step = Step::Entry {
            quantity,
            start: mean.map(|mean| match quantity {
                Quantity::Current => pending.mean_milliamps(mean),
                Quantity::Voltage => pending.mean_millivolts(mean),
            }),
            adjust: 0,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every reading goes through the calibration: the default must give
    /// exactly what the plain register conversions gave.
    #[test]
    fn default_changes_no_reading() {
        let calibration = Calibration::default();
        for raw in [0u16, 1, 2, 3, 4001, 16000, 0x7fff, 0x8000, 0xfffe, 0xffff] {
            assert_eq!(calibration.millivolts(raw), u32::from(raw) * 5 / 4);
            assert_eq!(calibration.milliamps(raw), i32::from(raw as i16) / 2);
        }
    }

    #[test]
    fn steps_correct_a_reading_and_keep_skipped_ones() {
        // 2.000 A and 20.000 V averaged, with a 3 mA offset.
        let loaded = Mean {
            uv: 20_000_000,
            ua: 2_003_000,
        };
        let mut wizard = Wizard::new(Calibration::default());
        wizard.hold_ok(Some(Mean {
            uv: 20_000_000,
            ua: 3_000,
        }));
        assert_eq!(wizard.pending.zero_ua, 3_000);
        // The current step starts at the reading without the offset.
        wizard.step = Step::Entry {
            quantity: Quantity::Current,
            start: Some(wizard.pending.mean_milliamps(loaded)),
            adjust: 0,
        };
        assert_eq!(wizard.step.entered(), Some(2_000));
        for _ in 0..2 {
            wizard.step(true, true);
        }
        assert_eq!(wizard.ok(Some(loaded)), None);
        // Voltage left as it was: the steps finish without a voltage change.
        let calibration = wizard.ok(Some(loaded)).unwrap_or_default();
        assert_eq!(calibration.voltage_ppm, 0);
        assert_eq!(calibration.milliamps(4006), 2_020);
        assert_eq!(calibration.millivolts(16000), 20_000);
    }

    #[test]
    fn refuses_what_a_reading_cannot_support() {
        let calibration = Calibration::default();
        let idle = Mean {
            uv: 5_000_000,
            ua: 100_000,
        };
        assert_eq!(calibration.zeroed(idle), Err(Refusal::Load));
        assert_eq!(calibration.current_matched(idle, 101), Err(Refusal::NoLoad));
        assert_eq!(
            calibration.voltage_matched(idle, 5_300),
            Err(Refusal::OutOfRange)
        );
    }

    #[test]
    fn records_round_trip_and_stay_apart_from_settings() {
        let calibration = Calibration {
            zero_ua: -2_500,
            current_ppm: 8_000,
            voltage_ppm: -1_000,
        };
        let mut found = Found::default();
        for word in calibration.records() {
            assert!(found.read(word));
            assert_eq!(crate::settings::Settings::decode(word), None);
        }
        assert_eq!(found.calibration(), calibration);
        assert!(!Found::default().read(crate::settings::Settings::default().encode()));
    }
}
