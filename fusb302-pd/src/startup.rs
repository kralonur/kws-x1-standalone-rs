//! Caller-baseline-gated FUSB startup measurements and passive release.
use crate::config::{ConfigRegister, Error, write_checked};
use crate::register::{self, interrupt, power, status0, switches0};
use crate::snapshot::{self as pd, Snapshot};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};

/// Why a one-shot startup step failed.
#[derive(Debug, PartialEq, Eq)]
pub struct Failure<E> {
    pub error: Error<E>,
    /// A write was attempted and the configuration was not verified afterwards:
    /// an error, NACK or cancellation can still have changed hardware.
    pub unverified: bool,
}
impl<E> Failure<E> {
    fn before_write(error: Error<E>) -> Self {
        Self {
            error,
            unverified: false,
        }
    }
    fn after_write(error: Error<E>) -> Self {
        Self {
            error,
            unverified: true,
        }
    }
}

/// Passthrough boot only: power-on baseline -> SWITCHES0 = 0x00, verified.
/// `release` consumes it, so it runs at most once; any error halts without
/// retry or cleanup. There is no way back to a sink session in the same boot.
pub struct Passive {
    expected: [u8; 11],
    release_switches: u8,
}
impl Passive {
    pub const fn new(expected: [u8; 11], release_switches: u8) -> Self {
        Self {
            expected,
            release_switches,
        }
    }
    pub async fn release<B: I2c>(
        self,
        bus: &mut B,
        before: Snapshot,
    ) -> Result<Snapshot, Failure<B::Error>> {
        check(&before, &self.expected, |register, value| Error::Baseline {
            register,
            value,
        })
        .map_err(Failure::before_write)?;
        self.write_and_verify(bus)
            .await
            .map_err(Failure::after_write)
    }
    async fn write_and_verify<B: I2c>(&self, bus: &mut B) -> Result<Snapshot, Error<B::Error>> {
        write_checked(bus, ConfigRegister::Switches0, self.release_switches).await?;
        let after = pd::snapshot(bus).await.map_err(|f| Error::Bus {
            register: f.register,
            source: f.source,
        })?;
        let mut released = self.expected;
        released[1] = self.release_switches;
        check(&after, &released, |register, value| Error::Verify {
            register,
            value,
        })?;
        Ok(after)
    }
}
fn check<E>(
    snapshot: &Snapshot,
    expected: &[u8; 11],
    error: impl Fn(u8, u8) -> Error<E>,
) -> Result<(), Error<E>> {
    for (i, &expected) in expected.iter().enumerate() {
        if snapshot.values[i] != expected {
            return Err(error(pd::REGISTERS[i], snapshot.values[i]));
        }
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Report {
    pub status: [u8; 2],
    /// When both CCs carry Rp (the port's plug sits in the source's
    /// receptacle, which drives Rp on both pins): the one CC (1 or 2) that
    /// showed BMC activity while listening, if exactly one did.
    pub traffic: Option<u8>,
    pub after: Snapshot,
}

/// How long to listen on each CC for traffic: a source that gets no GoodCRC
/// resends Source_Capabilities every tTypeCSendSourceCap (100-200 ms, USB PD
/// section 6.6.4.1), so one window covers at least one resend.
const TRAFFIC_LISTEN_MS: u32 = 300;

/// One-shot measurement: `run` consumes it, so no second run is possible, even
/// after a mismatch, error or cancellation. Cancellation does not restore hardware.
pub struct Diagnostic {
    expected: [u8; 11],
    measure: u8,
}
impl Diagnostic {
    pub const fn once(expected: [u8; 11], measure: u8) -> Self {
        Self { expected, measure }
    }
    pub async fn run<B: I2c, D: DelayNs>(
        self,
        bus: &mut B,
        delay: &mut D,
        before: Snapshot,
    ) -> Result<Report, Failure<B::Error>> {
        self.check_baseline(bus, before)
            .await
            .map_err(Failure::before_write)?;
        self.measure_and_restore(bus, delay)
            .await
            .map_err(Failure::after_write)
    }
    async fn check_baseline<B: I2c>(
        &self,
        bus: &mut B,
        before: Snapshot,
    ) -> Result<(), Error<B::Error>> {
        for (i, expected) in self.expected.into_iter().enumerate() {
            if before.values[i] != expected {
                return Err(Error::Baseline {
                    register: pd::REGISTERS[i],
                    value: before.values[i],
                });
            }
        }
        // Also gate MEASURE (not in the old snapshot): no VBUS selection,
        // reserved-bit writes or guesses about a different retained MDAC setup.
        let measure = read(bus, register::MEASURE).await?;
        if measure != self.measure {
            return Err(Error::Baseline {
                register: register::MEASURE,
                value: measure,
            });
        }
        Ok(())
    }
    async fn measure_and_restore<B: I2c, D: DelayNs>(
        &self,
        bus: &mut B,
        delay: &mut D,
    ) -> Result<Report, Error<B::Error>> {
        write_checked(
            bus,
            ConfigRegister::Power,
            power::BANDGAP_WAKE | power::RECEIVER | power::MEASURE,
        )
        .await?;
        // Keep Rd on both CCs; measure one CC at a time.
        write_checked(
            bus,
            ConfigRegister::Switches0,
            switches0::PDWN1 | switches0::PDWN2 | switches0::MEAS_CC1,
        )
        .await?;
        delay.delay_ms(10).await; // nominal settling interval; no calibrated voltage result
        let cc1 = read(bus, register::STATUS0).await?;
        write_checked(
            bus,
            ConfigRegister::Switches0,
            switches0::PDWN1 | switches0::PDWN2 | switches0::MEAS_CC2,
        )
        .await?;
        delay.delay_ms(10).await;
        let cc2 = read(bus, register::STATUS0).await?;
        let traffic = if cc1 & status0::BC_LVL != 0 && cc2 & status0::BC_LVL != 0 {
            listen_for_traffic(bus, delay).await?
        } else {
            None
        };
        // Normal successful sequence restores the known originals. On any
        // preceding error/cancellation: abort, no retries or blind cleanup writes.
        write_checked(
            bus,
            ConfigRegister::Switches0,
            switches0::PDWN1 | switches0::PDWN2,
        )
        .await?;
        write_checked(bus, ConfigRegister::Power, power::BANDGAP_WAKE).await?;
        let after = pd::snapshot(bus).await.map_err(|f| Error::Bus {
            register: f.register,
            source: f.source,
        })?;
        for (i, expected) in self.expected.into_iter().enumerate() {
            if after.values[i] != expected {
                return Err(Error::Verify {
                    register: pd::REGISTERS[i],
                    value: after.values[i],
                });
            }
        }
        Ok(Report {
            status: [cc1, cc2],
            traffic,
            after,
        })
    }
}

/// Receive only, Rd kept on both CCs: listen on each CC for
/// `TRAFFIC_LISTEN_MS` and return the one CC whose latched I_ACTIVITY was set.
/// Reading INTERRUPT clears it, so each window starts clean.
async fn listen_for_traffic<B: I2c, D: DelayNs>(
    bus: &mut B,
    delay: &mut D,
) -> Result<Option<u8>, Error<B::Error>> {
    let mut active = [false; 2];
    for (seen, measure) in active
        .iter_mut()
        .zip([switches0::MEAS_CC1, switches0::MEAS_CC2])
    {
        write_checked(
            bus,
            ConfigRegister::Switches0,
            switches0::PDWN1 | switches0::PDWN2 | measure,
        )
        .await?;
        read(bus, register::INTERRUPT).await?;
        delay.delay_ms(TRAFFIC_LISTEN_MS).await;
        *seen = read(bus, register::INTERRUPT).await? & interrupt::I_ACTIVITY != 0;
    }
    Ok(match active {
        [true, false] => Some(1),
        [false, true] => Some(2),
        _ => None,
    })
}
async fn read<B: I2c>(bus: &mut B, register: u8) -> Result<u8, Error<B::Error>> {
    let mut value = [0];
    bus.write_read(pd::ADDRESS, &[register], &mut value)
        .await
        .map_err(|source| Error::Bus { register, source })?;
    Ok(value[0])
}
