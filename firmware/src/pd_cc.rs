//! Board power-on baseline and factories for the reusable FUSB startup adapters.
pub use fusb302_pd::{config::Error, startup::Report};

/// Verified power-on configuration in `snapshot::REGISTERS` order; dynamic
/// status bytes are excluded. All but DEVICE_ID are reset values (DS Table 16).
pub(crate) const EXPECTED: [u8; 11] = [
    0x91, // DEVICE_ID: Version B, default product, revision B (DS Table 17)
    0x03, // SWITCHES0: PDWN1 | PDWN2, Rd on both CCs
    0x20, // SWITCHES1: SPECREV 2.0, transmit drivers off
    0x24, // CONTROL0: INT_MASK, HOST_CUR 80 uA
    0x00, // CONTROL1: SOP'/SOP'' reception off
    0x02, // CONTROL2: MODE DRP, toggle off
    0x06, // CONTROL3: N_RETRIES 3, AUTO_RETRY off
    0x01, // POWER: bandgap and wake only
    0x00, // MASK
    0x00, // MASKA
    0x00, // MASKB
];
/// The original firmware's boot releases both controller Rd terminations.
pub(crate) const PASSIVE_SWITCHES0: u8 = 0x00;
const fn passive_expected() -> [u8; 11] {
    let mut values = EXPECTED;
    values[1] = PASSIVE_SWITCHES0;
    values
}
pub(crate) const PASSIVE_EXPECTED: [u8; 11] = passive_expected();

/// The boot's one passive release, gated on the power-on baseline.
pub const fn passive() -> fusb302_pd::startup::Passive {
    fusb302_pd::startup::Passive::new(EXPECTED, PASSIVE_SWITCHES0)
}
/// The boot's one CC measurement, gated on the power-on baseline.
pub const fn diagnostic() -> fusb302_pd::startup::Diagnostic {
    fusb302_pd::startup::Diagnostic::once(EXPECTED, crate::pd_sniff::MEASURE)
}
