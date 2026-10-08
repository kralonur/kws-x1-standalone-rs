//! Caller-baseline-gated FUSB startup measurements and passive release.
use crate::config::{ConfigRegister, Error, write_checked};
use crate::register;
use crate::snapshot::{self as pd, Snapshot};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};

/// Passthrough boot only: power-on baseline -> SWITCHES0 = 0x00, verified.
/// Runs at most once; any error halts without retry or cleanup. There is no
/// way back to a sink session in the same boot.
pub struct Passive {
    started: bool,
    unverified: bool,
    expected: [u8; 11],
    release_switches: u8,
}
impl Passive {
    pub const fn new(expected: [u8; 11], release_switches: u8) -> Self {
        Self {
            started: false,
            unverified: false,
            expected,
            release_switches,
        }
    }
    /// The SWITCHES0 write was attempted and its result not yet verified.
    pub fn unverified(&self) -> bool {
        self.unverified
    }
    pub async fn release<B: I2c>(
        &mut self,
        bus: &mut B,
        before: Snapshot,
    ) -> Result<Snapshot, Error<B::Error>> {
        if self.started {
            return Err(Error::Stopped);
        }
        self.started = true;
        check(&before, &self.expected, |register, value| Error::Baseline {
            register,
            value,
        })?;
        self.unverified = true; // BEFORE the write await.
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
        self.unverified = false;
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
    pub after: Snapshot,
}

/// Persistent one-shot state. Cancellation does not restore hardware. Once
/// polled, no second run is allowed, even on a mismatch/error/cancellation.
pub struct Diagnostic {
    started: bool,
    unverified: bool,
    expected: [u8; 11],
    measure: u8,
}
impl Diagnostic {
    pub const fn once(expected: [u8; 11], measure: u8) -> Self {
        Self {
            started: false,
            unverified: false,
            expected,
            measure,
        }
    }
    /// A write was attempted and full original configuration not yet verified.
    /// An error/NACK/cancellation can still have changed hardware.
    pub fn unverified(&self) -> bool {
        self.unverified
    }
    pub async fn run<B: I2c, D: DelayNs>(
        &mut self,
        bus: &mut B,
        delay: &mut D,
        before: Snapshot,
    ) -> Result<Report, Error<B::Error>> {
        if self.started {
            return Err(Error::Stopped);
        }
        self.started = true;
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
        self.unverified = true; // BEFORE first write await.
        write_checked(bus, ConfigRegister::Power, 0x07).await?; // bandgap + receiver/references + measure
        write_checked(bus, ConfigRegister::Switches0, 0x07).await?; // preserve Rd1/Rd2, measure CC1 only
        delay.delay_ms(10).await; // nominal settling interval; no calibrated voltage result
        let cc1 = read(bus, register::STATUS0).await?;
        write_checked(bus, ConfigRegister::Switches0, 0x0b).await?; // preserve Rd1/Rd2, measure CC2 only
        delay.delay_ms(10).await;
        let cc2 = read(bus, register::STATUS0).await?;
        // Normal successful sequence restores the known originals. On any
        // preceding error/cancellation: abort, no retries or blind cleanup writes.
        write_checked(bus, ConfigRegister::Switches0, 0x03).await?;
        write_checked(bus, ConfigRegister::Power, 0x01).await?;
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
        self.unverified = false;
        Ok(Report {
            status: [cc1, cc2],
            after,
        })
    }
}
async fn read<B: I2c>(bus: &mut B, register: u8) -> Result<u8, Error<B::Error>> {
    let mut value = [0];
    bus.write_read(pd::ADDRESS, &[register], &mut value)
        .await
        .map_err(|source| Error::Bus { register, source })?;
    Ok(value[0])
}
