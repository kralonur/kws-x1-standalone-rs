//! Ripple spectrum: a windowed FFT of the whole capture, drawn as bars behind
//! the trace, and its strongest peaks. Heights are relative to the strongest
//! bin, not calibrated voltages.

use core::f32::consts::TAU;

use crate::ripple::SAMPLE_COUNT;

/// Bar columns: one pixel each across the display's 224-pixel plot.
pub const COLUMNS: usize = 224;
/// One-sided bins, from DC up to just below Nyquist.
const BINS: usize = SAMPLE_COUNT / 2;
/// Peaks labelled, as many as the reference firmware labels.
pub const PEAKS: usize = 3;
/// A bar at the strongest bin's height.
pub const FULL_HEIGHT: u8 = u8::MAX;
/// Peak positions are kept in sixteenths of a bin.
pub const BIN_FRACTION: u32 = 16;
/// First bin searched: the window spreads leftover DC into bin 1.
const FIRST_BIN: usize = 2;
/// Smallest peak shown, in windowed magnitude: a sine of amplitude A counts
/// gives A * N / 4 under a Hann window, so this is one ADC count (about
/// 0.4 mV at the 2x amplifier), above the quantization noise floor.
const MIN_PEAK: f32 = (SAMPLE_COUNT / 4) as f32;
/// Peaks below 1/16 (-24 dB) of the strongest are not labelled: a Hann
/// window's first sidelobe is at -31 dB, so a strong tone's sidelobes never
/// pass as peaks of their own.
const PEAK_RATIO: f32 = 16.0;
/// Hann window phase step per sample, in radians.
const WINDOW_STEP: f32 = TAU / SAMPLE_COUNT as f32;
/// `cos` and `sin` of [`WINDOW_STEP`] from their Taylor series, exact to f32
/// at this small angle, so the window rotates a phasor instead of calling
/// libm's `cosf` (which links software f64 helpers).
const STEP_COS: f32 = 1.0 - WINDOW_STEP * WINDOW_STEP / 2.0
    + WINDOW_STEP * WINDOW_STEP * WINDOW_STEP * WINDOW_STEP / 24.0;
const STEP_SIN: f32 = WINDOW_STEP - WINDOW_STEP * WINDOW_STEP * WINDOW_STEP / 6.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Spectrum {
    /// Per column, the strongest bin it covers, relative to the strongest
    /// bin overall (or [`MIN_PEAK`] when nothing reaches it).
    pub bars: [u8; COLUMNS],
    /// Peak positions in [`BIN_FRACTION`]s of a bin, strongest first.
    pub peaks: [Option<u16>; PEAKS],
}

impl Spectrum {
    pub const EMPTY: Self = Self {
        bars: [0; COLUMNS],
        peaks: [None; PEAKS],
    };

    /// Remove the capture's mean, apply a Hann window and transform it.
    /// `scratch` holds the windowed capture, then the transform.
    pub fn new(samples: &[u16; SAMPLE_COUNT], scratch: &mut [f32; SAMPLE_COUNT]) -> Self {
        let sum: u32 = samples.iter().map(|&sample| u32::from(sample)).sum();
        let mean = sum as f32 / SAMPLE_COUNT as f32;
        // `(cos, sin)` of sample n's window phase.
        let (mut cos, mut sin) = (1.0_f32, 0.0_f32);
        for (value, &sample) in scratch.iter_mut().zip(samples) {
            *value = (f32::from(sample) - mean) * (0.5 - 0.5 * cos);
            (cos, sin) = (
                cos * STEP_COS - sin * STEP_SIN,
                sin * STEP_COS + cos * STEP_SIN,
            );
        }
        let bins = microfft::real::rfft_1024(scratch);
        // Magnitudes in place of the real parts. Bin 0 packs DC (removed
        // above) and Nyquist, neither of which is ripple to show.
        for bin in bins.iter_mut() {
            bin.re = libm::sqrtf(bin.norm_sqr());
        }
        let magnitude = |bin: usize| if bin < FIRST_BIN { 0.0 } else { bins[bin].re };
        let strongest = (FIRST_BIN..BINS).map(magnitude).fold(0.0, f32::max);
        let scale = strongest.max(MIN_PEAK);
        let bars = core::array::from_fn(|column| {
            let highest = (column * BINS / COLUMNS..(column + 1) * BINS / COLUMNS)
                .map(magnitude)
                .fold(0.0, f32::max);
            (highest / scale * f32::from(FULL_HEIGHT)) as u8
        });
        // Local maxima with both neighbours, so each can be interpolated.
        let floor = MIN_PEAK.max(strongest / PEAK_RATIO);
        let mut found = [(0.0f32, 0usize); PEAKS];
        for bin in FIRST_BIN..BINS - 1 {
            let (before, here, after) = (magnitude(bin - 1), magnitude(bin), magnitude(bin + 1));
            if here < floor || here <= before || here < after {
                continue;
            }
            if let Some(slot) = found.iter().position(|&(other, _)| here > other) {
                found.copy_within(slot..PEAKS - 1, slot + 1);
                found[slot] = (here, bin);
            }
        }
        let peaks = found.map(|(here, bin)| {
            (here > 0.0).then(|| {
                let (before, after) = (magnitude(bin - 1), magnitude(bin + 1));
                // Exact for a Hann-windowed tone: offset from the peak bin,
                // 2 (after - before) / (before + 2 here + after).
                let offset = 2.0 * (after - before) / (before + 2.0 * here + after);
                ((bin as f32 + offset) * BIN_FRACTION as f32 + 0.5) as u16
            })
        });
        Self { bars, peaks }
    }
}

/// A peak position's frequency in Hz at `sample_rate_hz`.
pub fn frequency_hz(position: u16, sample_rate_hz: u32) -> u32 {
    let denominator = u64::from(BIN_FRACTION) * SAMPLE_COUNT as u64;
    ((u64::from(position) * u64::from(sample_rate_hz) + denominator / 2) / denominator) as u32
}

/// The bar column a peak position falls in.
pub fn column(position: u16) -> usize {
    (usize::from(position) * COLUMNS / (BINS * BIN_FRACTION as usize)).min(COLUMNS - 1)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tones(tones: &[(f32, f32)]) -> [u16; SAMPLE_COUNT] {
        core::array::from_fn(|n| {
            let wave: f32 = tones
                .iter()
                .map(|&(bin, amplitude)| {
                    amplitude * libm::sinf(TAU * bin * n as f32 / SAMPLE_COUNT as f32)
                })
                .sum();
            libm::roundf(2048.0 + wave) as u16
        })
    }

    #[test]
    fn peaks_rank_tones_and_interpolate_between_bins() {
        // Protect ranking, sub-bin accuracy (27.7 kHz at 0.91 MS/s is bin
        // 31.2) and the frequency conversion.
        let samples = tones(&[(31.2, 40.0), (62.4, 20.0), (93.6, 13.0)]);
        let spectrum = Spectrum::new(&samples, &mut [0.0; SAMPLE_COUNT]);
        let [Some(first), Some(second), Some(third)] = spectrum.peaks else {
            panic!("three peaks expected: {:?}", spectrum.peaks);
        };
        for (position, bin) in [(first, 31.2), (second, 62.4), (third, 93.6)] {
            let error = f32::from(position) / BIN_FRACTION as f32 - bin;
            assert!(error.abs() <= 0.1, "{position} vs {bin}");
        }
        let rate = 30_000_000 / 33;
        assert!(frequency_hz(first, rate).abs_diff(27_700) < 100);
        assert_eq!(spectrum.bars[column(first)], FULL_HEIGHT);
    }

    #[test]
    fn pure_tone_has_no_sidelobe_peaks_and_flat_capture_none() {
        // A strong tone's window sidelobes and quantization noise must not
        // be labelled as harmonics.
        let spectrum = Spectrum::new(&tones(&[(40.5, 1500.0)]), &mut [0.0; SAMPLE_COUNT]);
        assert!(spectrum.peaks[0].is_some());
        assert_eq!(spectrum.peaks[1..], [None, None]);
        let flat = Spectrum::new(&[2048; SAMPLE_COUNT], &mut [0.0; SAMPLE_COUNT]);
        assert_eq!(flat, Spectrum::EMPTY);
    }
}
