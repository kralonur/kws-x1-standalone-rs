//! X1 passive baseline/CC threshold for the reusable receive-only sniffer.
#[allow(unused_imports)]
pub use fusb302_pd::sniffer::Error;
/// Validated power-on MDAC/reference value, matching stock CC measurement.
pub(crate) const MEASURE: u8 = 0x31;
pub struct Sniffer(fusb302_pd::sniffer::Sniffer);
impl Sniffer {
    pub const fn once() -> Self {
        Self(fusb302_pd::sniffer::Sniffer::once(
            crate::pd_cc::PASSIVE_EXPECTED,
            MEASURE,
        ))
    }
}
impl core::ops::Deref for Sniffer {
    type Target = fusb302_pd::sniffer::Sniffer;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl core::ops::DerefMut for Sniffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
