//! Application retry policy for acknowledged AFC requests whose voltage did not follow.

use single_wire_protocols::afc::Voltage;

/// Maximum acknowledged request groups per setpoint, including the first.
/// Based on `retry_cnt_max = 5` in the pinned reference's `afc_set_ta_vchr`;
/// each group retains the protocol library's own wire-error retries.
const MAX_REQUESTS: u8 = 5;
/// Application spacing in ms between request groups. The reference waits
/// 38 ms, but our meter averages 64 × (1.1 ms bus + 1.1 ms shunt) = 140.8 ms.
/// Use roughly two averaging periods so readings can update before resending,
/// rather than repeatedly acting on a stale reading. This is not an AFC limit.
const RETRY_MS: u64 = 300;

/// Bounded retries after the first acknowledged request. The caller keeps
/// its original output-settle deadline across retries and fails immediately
/// on missing/unsafe measurements or loss of an already applied voltage.
#[derive(Default)]
pub struct OutputRetry {
    retries: u8,
}

impl OutputRetry {
    /// Reserve another group only for a measured, still-unmatched output.
    /// Matching VBUS, a lost read or a voltage outside the application envelope
    /// must never cause more traffic here. `elapsed_ms` is time since the last
    /// request group, taken from the caller's existing communication timestamp.
    pub fn retry(&mut self, elapsed_ms: u64, target: Voltage, measured_mv: Option<u16>) -> bool {
        let Some(mv) = measured_mv else {
            return false;
        };
        if !(crate::single_wire::MIN_MV..=crate::single_wire::MAX_MV).contains(&mv)
            || crate::single_wire::followed(target.millivolts(), mv)
            || self.retries >= MAX_REQUESTS - 1
            || elapsed_ms < RETRY_MS
        {
            return false;
        }
        self.retries += 1;
        true
    }
}
