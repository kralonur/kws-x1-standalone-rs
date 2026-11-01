//! Guard against unbounded AFC retries and traffic after voltage verification.

use firmware_core::afc::OutputRetry;
use single_wire_protocols::afc::Voltage;

// Application policy: 300 ms between at most five acknowledged request groups.
const RETRY_MS: u64 = 300;
const MAX_REQUESTS: u64 = 5;

#[test]
fn acknowledged_9v_request_is_repeated_when_vbus_stays_at_5v_but_is_bounded() {
    let mut retry = OutputRetry::default();
    for _ in 1..MAX_REQUESTS {
        assert!(!retry.retry(RETRY_MS - 1, Voltage::V9, Some(5_000)));
        assert!(retry.retry(RETRY_MS, Voltage::V9, Some(5_000)));
        // The caller's communication timestamp restarts after each group.
        assert!(!retry.retry(0, Voltage::V9, Some(5_000)));
    }
    assert!(!retry.retry(RETRY_MS, Voltage::V9, Some(5_000)));
    assert!(!retry.retry(u64::MAX, Voltage::V9, Some(5_000)));
}

#[test]
fn verified_voltage_missing_read_and_unsafe_voltage_do_not_resend_or_consume_budget() {
    for voltage in Voltage::ALL {
        let mut retry = OutputRetry::default();
        for measured in [None, Some(0), Some(30_001), Some(voltage.millivolts())] {
            assert!(!retry.retry(RETRY_MS, voltage, measured));
        }
        // Just outside the existing +5% acceptance boundary.
        let unmatched = voltage.millivolts() + voltage.millivolts() / 20 + 1;
        assert!(retry.retry(RETRY_MS, voltage, Some(unmatched)));
    }
}

#[test]
fn new_explicit_setpoint_gets_a_fresh_budget() {
    let mut retry = OutputRetry::default();
    for _ in 1..MAX_REQUESTS {
        assert!(retry.retry(RETRY_MS, Voltage::V9, Some(5_000)));
    }
    assert!(!retry.retry(RETRY_MS, Voltage::V9, Some(5_000)));
    let mut retry = OutputRetry::default();
    assert!(!retry.retry(RETRY_MS - 1, Voltage::V5, Some(9_000)));
    assert!(retry.retry(RETRY_MS, Voltage::V5, Some(9_000)));
}
