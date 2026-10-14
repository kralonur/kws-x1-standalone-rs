//! Ripple statistics, reference-normalized estimates and a mean-triggered trace.

/// Samples per capture, matching the established ADC DMA acquisition.
pub const SAMPLE_COUNT: usize = 1024;
/// Trace columns: two pixels each inside the display's 224-pixel plot.
pub const TRACE_COLUMNS: usize = 112;
/// Show half the capture, leaving the other half for trigger alignment.
pub const TRACE_SAMPLES: usize = SAMPLE_COUNT / 2;
/// Eight horizontal divisions across the displayed trace's sample bins.
pub const HORIZONTAL_DIVISIONS: u32 = 8;
/// Ten tenths of a microsecond per microsecond, one million µs per second.
const TENTHS_US_PER_SECOND: u64 = 10_000_000;
/// Maximum count from a right-aligned 12-bit ADC result.
pub const FULL_SCALE: u16 = (1 << 12) - 1;
/// Built-in amplifier gain configured by the capture driver.
const PGA_GAIN: u32 = 2;
/// Sixteenth-count RMS precision retains sub-count noise without float state.
const RMS_FRACTION: u64 = 16;
/// Four vertical divisions, with the capture mean on the centre grid line.
pub const VERTICAL_DIVISIONS: u32 = 4;
/// Trigger hysteresis as a fraction of the capture's peak-to-peak: after
/// reaching the mean, the signal must rise an eighth of its range above it.
/// A chosen value, so noise around the mean cannot retrigger the trace.
const TRIGGER_BAND_DIVISOR: u16 = 8;
/// A 4 V plot spans even a one-sided excursion at 3.6 V supply / 2x gain.
const MAX_MV_PER_DIV: u16 = 1000;

/// Volts-per-division choices, in millivolts; transient, never persisted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Scale {
    #[default]
    Auto,
    Mv2,
    Mv5,
    Mv10,
    Mv20,
    Mv50,
    Mv100,
    Mv200,
    Mv500,
    Mv1000,
}

impl Scale {
    pub const ALL: [Self; 10] = [
        Self::Auto,
        Self::Mv2,
        Self::Mv5,
        Self::Mv10,
        Self::Mv20,
        Self::Mv50,
        Self::Mv100,
        Self::Mv200,
        Self::Mv500,
        Self::Mv1000,
    ];

    pub const fn millivolts(self) -> Option<u16> {
        match self {
            Self::Auto => None,
            Self::Mv2 => Some(2),
            Self::Mv5 => Some(5),
            Self::Mv10 => Some(10),
            Self::Mv20 => Some(20),
            Self::Mv50 => Some(50),
            Self::Mv100 => Some(100),
            Self::Mv200 => Some(200),
            Self::Mv500 => Some(500),
            Self::Mv1000 => Some(MAX_MV_PER_DIV),
        }
    }
}

/// Mean-centred plot bounds in ADC counts; may extend beyond the ADC rails.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlotScale {
    pub min: i32,
    pub max: i32,
    /// Missing reference means a raw-count autoscale, never a voltage scale.
    pub mv_per_div: Option<u16>,
    pub limited: bool,
}

/// Preserve the peaks within each displayed bin rather than decimating them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Envelope {
    pub min: u16,
    pub max: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reading {
    pub min: u16,
    pub max: u16,
    pub mean: u16,
    /// RMS after exact DC removal, in sixteenths of an ADC count.
    rms_counts_x16: u32,
    pub triggered: bool,
    pub trace: [Envelope; TRACE_COLUMNS],
}

impl Reading {
    /// Peak-to-peak counts over the entire capture, not only the trace window.
    pub fn peak_to_peak(&self) -> u16 {
        self.max - self.min
    }

    /// Estimated mV from the shared ADC reference, rounding only the final
    /// input-equivalent ratio rather than truncating an intermediate voltage.
    /// `reference_mv` is the application's internal-reference calibration.
    /// No estimate is available with a zero/saturated reference or overflow.
    pub fn peak_to_peak_mv(&self, reference_count: u16, reference_mv: u16) -> Option<u16> {
        if reference_count == 0 || reference_count >= FULL_SCALE || reference_mv == 0 {
            return None;
        }
        let numerator = u64::from(self.peak_to_peak()) * u64::from(reference_mv);
        let denominator = u64::from(reference_count) * u64::from(PGA_GAIN);
        u16::try_from((numerator + denominator / 2) / denominator).ok()
    }

    /// DC-removed ripple RMS in tenths of a millivolt, preserving fractional
    /// counts before rounding the reference-normalized ratio.
    pub fn rms_tenths_mv(&self, reference_count: u16, reference_mv: u16) -> Option<u16> {
        if reference_count == 0 || reference_count >= FULL_SCALE || reference_mv == 0 {
            return None;
        }
        // Ten tenths per mV; u64 also covers the largest u16 reference value.
        let numerator = u64::from(self.rms_counts_x16) * u64::from(reference_mv) * 10;
        let denominator = RMS_FRACTION * u64::from(reference_count) * u64::from(PGA_GAIN);
        u16::try_from((numerator + denominator / 2) / denominator).ok()
    }

    pub fn plot_scale(&self, scale: Scale, reference: Option<u16>, reference_mv: u16) -> PlotScale {
        let excursion = u32::from((self.max - self.mean).max(self.mean - self.min)).max(1);
        let mv_per_div = reference
            .filter(|count| *count > 0 && *count < FULL_SCALE)
            .filter(|_| reference_mv > 0)
            .map(|reference| {
                let needed = (excursion * u32::from(reference_mv))
                    .div_ceil(PGA_GAIN * u32::from(reference) * (VERTICAL_DIVISIONS / 2));
                scale.millivolts().unwrap_or_else(|| {
                    Scale::ALL
                        .into_iter()
                        .filter_map(Scale::millivolts)
                        .find(|mv| u32::from(*mv) >= needed)
                        .unwrap_or(MAX_MV_PER_DIV)
                })
            });
        let half_span = match (mv_per_div, reference) {
            (Some(mv), Some(reference)) => {
                (u32::from(mv) * (VERTICAL_DIVISIONS / 2) * PGA_GAIN * u32::from(reference))
                    .div_ceil(u32::from(reference_mv))
                    .max(1)
            }
            _ => excursion,
        } as i32;
        let min = i32::from(self.mean) - half_span;
        let max = i32::from(self.mean) + half_span;
        PlotScale {
            min,
            max,
            mv_per_div,
            limited: i32::from(self.min) < min || i32::from(self.max) > max,
        }
    }

    /// A rail hit means peak-to-peak may underestimate the actual input.
    pub fn clipped(&self) -> bool {
        self.min == 0 || self.max >= FULL_SCALE
    }
}

/// Nominal time per horizontal division, in tenths of a microsecond.
/// Uses the trace window, not the full statistics capture or UI update period.
/// Clock accuracy, acquisition overhead and lost samples are not measured here.
pub fn nominal_tenths_us_per_div(adc_clock_hz: u32, cycles_per_sample: u16) -> Option<u32> {
    if adc_clock_hz == 0 || cycles_per_sample == 0 {
        return None;
    }
    let numerator = TRACE_SAMPLES as u64 * u64::from(cycles_per_sample) * TENTHS_US_PER_SECOND;
    let denominator = u64::from(adc_clock_hz) * u64::from(HORIZONTAL_DIVISIONS);
    u32::try_from((numerator + denominator / 2) / denominator).ok()
}

pub fn analyze(samples: &[u16; SAMPLE_COUNT]) -> Reading {
    let mut min = u16::MAX;
    let mut max = 0;
    let mut sum = 0u32;
    let mut square_sum = 0u64;
    for &sample in samples {
        min = min.min(sample);
        max = max.max(sample);
        sum += u32::from(sample);
        square_sum += u64::from(sample).pow(2);
    }
    // N*sum(x²) - sum(x)² removes the exact mean before taking the root.
    // Cauchy-Schwarz guarantees nonnegative variance. Even u16 samples at
    // this fixed capture length fit u64, including the fractional scale.
    let variance = SAMPLE_COUNT as u64 * square_sum - u64::from(sum).pow(2);
    let rms_counts_x16 = ((variance * RMS_FRACTION.pow(2)).isqrt() / SAMPLE_COUNT as u64) as u32;
    let mean = (sum / SAMPLE_COUNT as u32) as u16;
    // Rising crossing only, and only where a whole trace window remains.
    // Never wrap the capture: its end and start are not adjacent in time.
    // Arm at or below the mean, fire above it plus the hysteresis band.
    let level = mean + (max - min) / TRIGGER_BAND_DIVISOR;
    let mut armed = false;
    let trigger = samples[..=SAMPLE_COUNT - TRACE_SAMPLES]
        .iter()
        .position(|&sample| {
            let fired = armed && sample > level;
            armed |= sample <= mean;
            fired
        });
    let start = trigger.unwrap_or(0);
    let trace = core::array::from_fn(|column| {
        let first = start + column * TRACE_SAMPLES / TRACE_COLUMNS;
        let end = start + (column + 1) * TRACE_SAMPLES / TRACE_COLUMNS;
        let mut bin = Envelope {
            min: u16::MAX,
            max: 0,
        };
        for &sample in &samples[first..end] {
            bin.min = bin.min.min(sample);
            bin.max = bin.max.max(sample);
        }
        bin
    });
    Reading {
        min,
        max,
        mean,
        rms_counts_x16,
        triggered: trigger.is_some(),
        trace,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nominal_timebase_uses_trace_span_clock_and_conversion_cycles() {
        // Protect both clock profiles and the half-capture/eight-division
        // mapping; copying full-capture timing would double these labels.
        assert_eq!(nominal_tenths_us_per_div(30_000_000, 33), Some(704));
        assert_eq!(nominal_tenths_us_per_div(37_500_000, 33), Some(563));
        assert_eq!(nominal_tenths_us_per_div(30_000_000, 24), Some(512));
        assert_eq!(nominal_tenths_us_per_div(0, 33), None);
        assert_eq!(nominal_tenths_us_per_div(30_000_000, 0), None);
        assert_eq!(nominal_tenths_us_per_div(1, u16::MAX), None);
    }

    #[test]
    fn voltage_estimate_rounds_reference_ratio_and_rejects_bad_reference() {
        // Protect small-signal precision and final half-up rounding: early
        // truncation used to discard nonzero ranges of up to four counts.
        const REFERENCE_MV: u16 = 1083;
        let mut samples = [0; SAMPLE_COUNT];
        for (counts, reference, expected) in [
            (0, 1344, 0),
            (1, 1344, 0),
            (1, 1083, 1),
            (4, 1344, 2),
            (5, 1344, 2),
            (159, 1200, 72),
            (159, 1344, 64),
            (159, 1500, 57),
            (1024, 1344, 413),
            (FULL_SCALE, 1344, 1650),
        ] {
            samples[0] = counts;
            let reading = analyze(&samples);
            assert_eq!(
                reading.peak_to_peak_mv(reference, REFERENCE_MV),
                Some(expected)
            );
            assert_eq!(reading.peak_to_peak_mv(0, REFERENCE_MV), None);
            assert_eq!(reading.peak_to_peak_mv(FULL_SCALE, REFERENCE_MV), None);
            assert_eq!(reading.peak_to_peak_mv(reference, 0), None);
        }
        let reading = analyze(&samples);
        assert_eq!(reading.peak_to_peak_mv(1, REFERENCE_MV), None);
    }

    #[test]
    fn rms_removes_dc_and_preserves_fractional_mean() {
        for baseline in [1000, 2000] {
            let mut samples = [baseline; SAMPLE_COUNT];
            samples[SAMPLE_COUNT / 2..].fill(baseline + 20);
            let reading = analyze(&samples);
            assert_eq!(reading.rms_tenths_mv(1344, 1083), Some(40));
            assert_eq!(reading.rms_tenths_mv(1500, 1083), Some(36));
            assert_eq!(reading.rms_tenths_mv(0, 1083), None);
            assert_eq!(reading.rms_tenths_mv(FULL_SCALE, 1083), None);
            assert_eq!(reading.rms_tenths_mv(1344, 0), None);
        }
        let mut samples = [2047; SAMPLE_COUNT];
        samples[SAMPLE_COUNT / 2..].fill(2048);
        assert_eq!(analyze(&samples).rms_tenths_mv(1344, 1083), Some(2));
        samples[..SAMPLE_COUNT / 2].fill(0);
        samples[SAMPLE_COUNT / 2..].fill(FULL_SCALE);
        let reading = analyze(&samples);
        assert_eq!(reading.rms_tenths_mv(1344, 1083), Some(8249));
        assert_eq!(reading.rms_tenths_mv(1, 1083), None);
    }

    #[test]
    fn plot_scales_preserve_spikes_and_flag_locked_range_limits() {
        let mut samples = [1000; SAMPLE_COUNT];
        samples[SAMPLE_COUNT - 1] = 1100;
        let reading = analyze(&samples);
        let auto = reading.plot_scale(Scale::Auto, Some(1344), 1083);
        assert_eq!(auto.mv_per_div, Some(50));
        assert!(auto.min <= i32::from(reading.min));
        assert!(auto.max >= i32::from(reading.max));
        assert!(!auto.limited);
        let locked = reading.plot_scale(Scale::Mv2, Some(1344), 1083);
        assert_eq!(locked.mv_per_div, Some(2));
        assert!(locked.limited);
        // A missing/invalid reference cannot leave a voltage label on a raw
        // plot, even when the user selected a fixed mV range.
        for reference in [None, Some(0), Some(FULL_SCALE)] {
            let raw = reading.plot_scale(Scale::Mv2, reference, 1083);
            assert_eq!(raw.mv_per_div, None);
            assert!(!raw.limited);
        }
        assert_eq!(
            reading.plot_scale(Scale::Mv2, Some(1344), 0).mv_per_div,
            None
        );
        samples.fill(0);
        samples[0] = FULL_SCALE;
        let reading = analyze(&samples);
        let auto = reading.plot_scale(Scale::Auto, Some(1344), 1083);
        assert_eq!(auto.mv_per_div, Some(1000));
        assert!(auto.min < 0);
        assert!(!auto.limited);
    }

    #[test]
    fn flat_capture_has_no_trigger_or_invented_ripple() {
        let reading = analyze(&[2048; SAMPLE_COUNT]);
        assert_eq!((reading.min, reading.max, reading.mean), (2048, 2048, 2048));
        assert_eq!(reading.peak_to_peak(), 0);
        assert_eq!(reading.rms_tenths_mv(1344, 1083), Some(0));
        let plot = reading.plot_scale(Scale::Auto, Some(1344), 1083);
        assert!(plot.min < i32::from(reading.mean) && plot.max > i32::from(reading.mean));
        assert!(!plot.limited);
        assert!(!reading.triggered);
        assert!(!reading.clipped());
        assert!(reading.trace.iter().all(|bin| *bin
            == Envelope {
                min: 2048,
                max: 2048
            }));
    }

    #[test]
    fn trigger_window_preserves_peaks_without_wrapping() {
        let mut samples = [1000; SAMPLE_COUNT];
        samples[10..20].fill(3000);
        samples[SAMPLE_COUNT - 1] = FULL_SCALE;
        let reading = analyze(&samples);
        assert_eq!(
            reading.mean,
            (samples.iter().map(|&v| u32::from(v)).sum::<u32>() / SAMPLE_COUNT as u32) as u16
        );
        assert!(reading.triggered);
        assert_eq!(reading.trace[0].max, 3000);
        assert_eq!(reading.trace.last().map(|bin| bin.max), Some(1000));
        assert!(reading.trace.iter().all(|bin| bin.max <= 3000));
        assert_eq!(reading.peak_to_peak(), FULL_SCALE - 1000);
        assert!(reading.clipped());
    }

    #[test]
    fn noise_at_the_mean_does_not_trigger_before_the_real_edge() {
        // Without hysteresis the noise below crossed the mean at sample 1
        // and the trace jumped between captures.
        let mut samples = [1000; SAMPLE_COUNT];
        samples[TRACE_SAMPLES..].fill(1100);
        for (index, sample) in samples[..100].iter_mut().enumerate() {
            *sample = if index % 2 == 0 { 1053 } else { 1056 };
        }
        let reading = analyze(&samples);
        assert_eq!(reading.mean, 1055);
        assert!(reading.triggered);
        assert_eq!(
            reading.trace[0],
            Envelope {
                min: 1100,
                max: 1100
            }
        );
    }

    #[test]
    fn late_crossing_uses_untriggered_window_and_bins_keep_narrow_peaks() {
        let mut samples = [2000; SAMPLE_COUNT];
        samples[TRACE_SAMPLES + 1] = 4000;
        let reading = analyze(&samples);
        assert!(!reading.triggered);
        assert!(reading.trace.iter().all(|bin| bin.max == 2000));
        // A one-sample peak must survive binning when it is in the window.
        samples[2] = 4000;
        let reading = analyze(&samples);
        assert!(reading.triggered);
        assert_eq!(
            reading.trace[0],
            Envelope {
                min: 2000,
                max: 4000
            }
        );
    }
}
