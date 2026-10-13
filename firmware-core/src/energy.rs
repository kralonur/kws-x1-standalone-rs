//! Charge and energy integration from fresh, timestamped meter samples.

/// A gap beyond ten nominal 100 ms samples is unknown usage, not a reading
/// to extrapolate indefinitely across a failed bus or a stalled task.
const MAX_GAP_MS: u64 = 1000;
/// mA*ms per mAh, and uW*ms per mWh (3600 seconds per hour).
const MA_MS_PER_MAH: u64 = 3_600_000;
const UW_MS_PER_MWH: u64 = 3_600_000_000;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Totals {
    pub ma_ms: u64,
    pub uw_ms: u64,
}

impl Totals {
    pub fn milliamp_hours(self) -> u64 {
        self.ma_ms / MA_MS_PER_MAH
    }

    pub fn milliwatt_hours(self) -> u64 {
        self.uw_ms / UW_MS_PER_MWH
    }
}

pub struct Tracker {
    pub totals: Totals,
    previous: Option<(u64, u32, u32)>,
}

impl Tracker {
    pub fn new(totals: Totals) -> Self {
        Self {
            totals,
            previous: None,
        }
    }

    /// Integrate the trapezoid between fresh samples, counting current in
    /// either direction as throughput. Missing samples break the interval.
    pub fn sample(&mut self, now_ms: u64, reading: Option<(u32, i32)>) {
        let next = reading.map(|(mv, ma)| (now_ms, mv, ma.unsigned_abs()));
        if let (Some((then, mv, ma)), Some((now, next_mv, next_ma))) = (self.previous, next)
            && let Some(dt) = now.checked_sub(then).filter(|dt| *dt <= MAX_GAP_MS)
        {
            let charge = (u64::from(ma) + u64::from(next_ma)) * dt / 2;
            let power = u64::from(mv) * u64::from(ma);
            let next_power = u64::from(next_mv) * u64::from(next_ma);
            let energy = power.saturating_add(next_power).saturating_mul(dt) / 2;
            self.totals.ma_ms = self.totals.ma_ms.saturating_add(charge);
            self.totals.uw_ms = self.totals.uw_ms.saturating_add(energy);
        }
        self.previous = next;
    }

    pub fn reset(&mut self) {
        self.totals = Totals::default();
        self.previous = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn integrates_both_directions_and_variable_sample_spacing() {
        let mut tracker = Tracker::new(Totals::default());
        tracker.sample(0, Some((5000, -2000)));
        for time in [100, 350, 1000] {
            tracker.sample(time, Some((5000, 2000)));
        }
        assert_eq!(tracker.totals.ma_ms, 2_000_000);
        assert_eq!(tracker.totals.uw_ms, 10_000_000_000);
        // Repeat one second of 5 V / 2 A for a full hour.
        for time in 2..=3600 {
            tracker.sample(time * 1000, Some((5000, 2000)));
        }
        assert_eq!(tracker.totals.milliamp_hours(), 2000);
        assert_eq!(tracker.totals.milliwatt_hours(), 10_000);
    }

    #[test]
    fn missing_samples_and_long_gaps_are_not_counted_and_reset_breaks_interval() {
        let mut tracker = Tracker::new(Totals::default());
        tracker.sample(0, Some((5000, 1000)));
        tracker.sample(100, None);
        tracker.sample(200, Some((5000, 1000)));
        tracker.sample(2000, Some((5000, 1000)));
        assert_eq!(tracker.totals, Totals::default());
        tracker.sample(2100, Some((5000, 1000)));
        assert_eq!(tracker.totals.ma_ms, 100_000);
        tracker.reset();
        tracker.sample(2200, Some((5000, 1000)));
        assert_eq!(tracker.totals, Totals::default());
    }
}
