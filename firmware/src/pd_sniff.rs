//! X1 passive baseline/CC threshold for the reusable receive-only sniffer.
#[allow(unused_imports)]
pub use fusb302_pd::sniffer::Error;
/// Validated power-on MDAC/reference value, matching stock CC measurement.
pub(crate) const MEASURE: u8 = 0x31;
/// The boot's one receive-only sniffer, gated on the passive baseline.
pub const fn once() -> fusb302_pd::sniffer::Sniffer {
    fusb302_pd::sniffer::Sniffer::once(crate::pd_cc::PASSIVE_EXPECTED, MEASURE)
}
