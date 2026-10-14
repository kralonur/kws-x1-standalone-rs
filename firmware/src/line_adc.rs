//! D+, D−, CC1 and CC2 line voltages, measured by ADC2 against the internal
//! reference (RM 16.3.2). ADC2 is independent of the INA226 bus monitor and
//! the PD controller; nothing here drives the lines.

use embassy_hc32::adc::{
    Adc2, AdcPin, Average, FULL_SCALE, Input, InternalReference, SampleTime, Source,
};
use embassy_hc32::gpio::Port;
use embassy_time::Duration;

/// The four line voltages in mV, scaled by the measured reference
/// ([`REFERENCE_MV`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LineVoltages {
    pub dp_mv: u16,
    pub dm_mv: u16,
    pub cc1_mv: u16,
    pub cc2_mv: u16,
}

/// The line pins, as the source firmware reads them.
const CC1_PIN: (Port, u8) = (Port::A, 6);
const CC2_PIN: (Port, u8) = (Port::A, 7);
const DM_PIN: (Port, u8) = (Port::B, 0);
const DP_PIN: (Port, u8) = (Port::B, 1);

/// Sampling times in ADC clock cycles (RM 16.4.11), the source firmware's:
/// long for the divided CC inputs, shorter for D+/D− and the reference.
/// Checked when compiled: an out-of-range value fails the build.
const CC_SAMPLE_TIME: SampleTime = SampleTime::new(255).expect("255 cycles is in range");
const DATA_SAMPLE_TIME: SampleTime = SampleTime::new(40).expect("40 cycles is in range");
const REFERENCE_SAMPLE_TIME: SampleTime = SampleTime::new(80).expect("80 cycles is in range");
/// Each result is the mean of four conversions, as the source firmware sets.
const AVERAGE: Average = Average::Four;
/// One averaged scan is about 3000 ADC clocks (four times the sampling
/// cycles above plus 12-bit conversions), 0.1 ms at a 30 MHz PCLK2, so this
/// is a wide bound.
const SCAN_TIMEOUT: Duration = Duration::from_millis(3);

/// Internal reference in mV. The manual gives about 1100 mV (RM 16.3.2),
/// with no tolerance and no per-chip calibration value. The stock firmware
/// uses 1083 mV, which a multimeter check on one unit matched: D+ and D−
/// driven at 0.55-3.23 V fit a gain-only scale (no offset) of 1083.7 mV,
/// every point then within 5 mV. Other units' references may differ by a
/// few percent.
pub const REFERENCE_MV: u16 = 1083;
/// Software scale of each line: the CC inputs read half the line voltage
/// (the source firmware doubles them), D+/D− are direct.
const CC_SCALE: u32 = 2;
const DATA_SCALE: u32 = 1;

/// Owns ADC2, its four analog line pins and the internal reference path.
pub struct LineAdc {
    /// CC1, CC2, D−, D+, then the internal reference.
    adc: Adc2<5>,
}

impl LineAdc {
    /// Make the line pins analog inputs, route the internal reference to
    /// ADC2 and set up one averaged scan of the lines and the reference.
    ///
    /// # Errors
    /// Returns the HAL error if the inherited ADC conversion cannot be stopped
    /// or the scan configuration is invalid.
    ///
    /// # Safety
    /// Call once during startup; route ADC2's end-of-scan event before the
    /// first [`Self::read`]. The caller gives up ADC2, these four pins, PWC
    /// FCG3's ADC2/CMP gates, PWCMR.ADBUFE and CMP RVADC to this owner; no
    /// other code may configure them.
    pub async unsafe fn new(adc: hc32f460_pac::Adc2) -> Result<Self, embassy_hc32::adc::Error> {
        // SAFETY: forwarded from this function's contract.
        unsafe {
            let pin = |(port, pin), sample_time| Input {
                source: Source::Pin(
                    AdcPin::new(port, pin).expect("each line pin is an ADC12 input"),
                ),
                sample_time,
            };
            let reference = Input {
                source: Source::Internal(InternalReference::new().await),
                sample_time: REFERENCE_SAMPLE_TIME,
            };
            let inputs = [
                pin(CC1_PIN, CC_SAMPLE_TIME),
                pin(CC2_PIN, CC_SAMPLE_TIME),
                pin(DM_PIN, DATA_SAMPLE_TIME),
                pin(DP_PIN, DATA_SAMPLE_TIME),
                reference,
            ];
            Ok(Self {
                adc: Adc2::new(adc, inputs, Some(AVERAGE), SCAN_TIMEOUT)?,
            })
        }
    }

    /// Run one scan, returning line voltages and the reference count for
    /// ripple normalization. `None` on timeout or an unusable reference.
    pub async fn read(&mut self) -> Option<(LineVoltages, u16)> {
        let [cc1, cc2, dm, dp, reference] = self.adc.read().await.ok()?;
        let line = |count: u16, scale| millivolts(count, scale, reference);
        Some((
            LineVoltages {
                dp_mv: line(dp, DATA_SCALE)?,
                dm_mv: line(dm, DATA_SCALE)?,
                cc1_mv: line(cc1, CC_SCALE)?,
                cc2_mv: line(cc2, CC_SCALE)?,
            },
            reference,
        ))
    }
}

/// `count * scale` in mV, normalized by the internal reference count and
/// rounded to the nearest mV: `count * scale * REFERENCE_MV / reference`.
/// A zero or saturated reference gives `None`, not a plausible-looking value.
fn millivolts(count: u16, scale: u32, reference: u16) -> Option<u16> {
    if reference == 0 || reference >= FULL_SCALE {
        return None;
    }
    let reference = u32::from(reference);
    let scaled = u32::from(count.min(FULL_SCALE)) * scale * u32::from(REFERENCE_MV);
    u16::try_from((scaled + reference / 2) / reference).ok()
}
