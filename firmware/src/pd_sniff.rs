//! Board passive baseline/CC threshold for the reusable receive-only sniffer.
#[allow(unused_imports)]
pub use fusb302_pd::sniffer::Error;
/// MEASURE with MEAS_VBUS clear and MDAC = 0b11_0001: a 2.100 V CC comparator
/// threshold (DS Table 20). This is also the register's reset value.
pub(crate) const MEASURE: u8 = 0x31;
/// The boot's one receive-only sniffer, gated on the passive baseline.
pub const fn once() -> fusb302_pd::sniffer::Sniffer {
    fusb302_pd::sniffer::Sniffer::once(crate::pd_cc::PASSIVE_EXPECTED, MEASURE)
}
