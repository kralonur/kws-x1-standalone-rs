//! X1 power-on baseline and factories for the reusable FUSB startup adapters.
#[allow(unused_imports)]
pub use fusb302_pd::{config::Error, startup::Report};

// Verified power-on configuration; dynamic status bytes are excluded.
pub(crate) const EXPECTED: [u8; 11] = [0x91, 0x03, 0x20, 0x24, 0, 0x02, 0x06, 0x01, 0, 0, 0];
/// Stock boot releases both controller Rd terminations (0x15ea0..0x15f16).
pub(crate) const PASSIVE_SWITCHES0: u8 = 0x00;
const fn passive_expected() -> [u8; 11] {
    let mut values = EXPECTED;
    values[1] = PASSIVE_SWITCHES0;
    values
}
pub(crate) const PASSIVE_EXPECTED: [u8; 11] = passive_expected();

pub struct Passive(fusb302_pd::startup::Passive);
impl Passive {
    pub const fn new() -> Self {
        Self(fusb302_pd::startup::Passive::new(
            EXPECTED,
            PASSIVE_SWITCHES0,
        ))
    }
}
impl core::ops::Deref for Passive {
    type Target = fusb302_pd::startup::Passive;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl core::ops::DerefMut for Passive {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
pub struct Diagnostic(fusb302_pd::startup::Diagnostic);
impl Diagnostic {
    pub const fn once() -> Self {
        Self(fusb302_pd::startup::Diagnostic::once(EXPECTED, 0x31))
    }
}
impl core::ops::Deref for Diagnostic {
    type Target = fusb302_pd::startup::Diagnostic;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl core::ops::DerefMut for Diagnostic {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
