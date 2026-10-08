//! Board policy and INA226 provider for the reusable FUSB302 PHY.
//! All wire authorization remains in the shared PHY and portable policy gates.
use core::sync::atomic::{AtomicBool, Ordering};
use embedded_hal_async::i2c::I2c;
pub use fusb302_pd::phy::{Clock, Config, Error, GatedBus, Waiter};

pub struct BoardPlatform;
static RX_IDLE: AtomicBool = AtomicBool::new(false);
impl fusb302_pd::phy::Platform for BoardPlatform {
    // Preserve the previous discovery receive/no-progress bounds (milliseconds).
    const CABLE_RESPONSE_TIMEOUT_MS: u64 = 51;
    const CABLE_SVID_NO_PROGRESS_TIMEOUT_MS: u64 = 1_000;
    type Limits = firmware_core::pd_spr::BoardLimits;
    const BASELINE: [u8; 11] = crate::pd_cc::EXPECTED;
    const MEASURE: u8 = crate::pd_sniff::MEASURE;
    const INSPECTION_CC: u8 = 1;
    // Owner-approved direct CC2 VCONN diagnostic; no role negotiation added.
    // VCONN on CC2 while the measure block watches CC1 (DS Table 18).
    const INSPECTION_SWITCHES0: u8 =
        fusb302_pd::register::switches0::VCONN_CC2 | fusb302_pd::register::switches0::MEAS_CC1;
    const ALLOW_STALE_SINK_TX_NG: bool = true;
    const VBUS_ADDRESS: u8 = 0x40;
    const VBUS_REGISTER: u8 = 2;
    fn voltage_mv(raw: u16) -> u32 {
        u32::from(raw) * 5 / 4
    }
    fn set_rx_idle(idle: bool) {
        RX_IDLE.store(idle, Ordering::Relaxed);
    }
    async fn vbus_raw<B: I2c>(bus: &mut B) -> Result<u16, Error<B::Error>> {
        let mut raw = [0; 2];
        bus.write_read(Self::VBUS_ADDRESS, &[Self::VBUS_REGISTER], &mut raw)
            .await
            .map_err(|source| Error::Bus {
                address: Self::VBUS_ADDRESS,
                register: Self::VBUS_REGISTER,
                source,
            })?;
        Ok(u16::from_be_bytes(raw))
    }
}
pub type Session<'a, B, D, C> = fusb302_pd::phy::Session<'a, B, D, C, BoardPlatform>;

pub async fn wait_for_rx_idle(delay: &mut impl Waiter) {
    while !RX_IDLE.load(Ordering::Relaxed) {
        delay.wait_event().await;
    }
}
