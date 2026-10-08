//! Receive-only passthrough sniffer (`docs/SAFETY.md`). Starts only from the
//! verified passive release (SWITCHES0=00, `pd_cc::Passive`).
//!
//! Write allowlist: POWER 07/09, SWITCHES0 04/08 (measure CC1/CC2; no Rd, Rp or
//! VCONN), one CONTROL1 RX_FLUSH. SWITCHES1 (AUTO_CRC, TXCC), CONTROL0
//! (TX_START), CONTROL2 (TOGGLE), CONTROL3 (Hard Reset), RESET, MEASURE, the masks
//! and the FIFO are never written, so the tester never transmits or acknowledges
//! (no GoodCRC). Errors/cancellation halt without retry or cleanup writes.
use crate::pd_sniff;
use crate::register::{self, STATUS0};
use crate::{config as pd_cc, service as pd_service, snapshot as pd};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use pd_cc::ConfigRegister;

#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Stopped,
    /// Both quiet STATUS0 samples, CC1 then CC2; no unambiguous active pin.
    Orientation([u8; 2]),
    /// STATUS1 after the flush: not RX/TX empty, or full/fault bits set.
    Boundary {
        status: u8,
    },
    Config(pd_cc::Error<E>),
    /// Periodic read-only check found a configuration byte changed.
    Drift {
        register: u8,
        value: u8,
    },
    Rx(pd_service::Error<E>),
}

/// Configuration while listening: passive values plus measure/receiver power.
pub const fn listening(baseline: [u8; 11], cc: u8) -> [u8; 11] {
    let mut values = baseline;
    values[1] = if cc == 1 { 0x04 } else { 0x08 };
    values[7] = 0x07;
    values
}

/// One sniffer per boot: `start` consumes it, so it cannot be started twice.
pub struct Sniffer {
    baseline: [u8; 11],
    measure: u8,
}

/// Why `start` failed, with what it observed before failing.
#[derive(Debug, PartialEq, Eq)]
pub struct StartFailure<E> {
    pub error: Error<E>,
    /// A write was attempted and the full listening configuration is not verified.
    pub unverified: bool,
    pub cc_status: Option<[u8; 2]>,
    /// Read-clear flags acknowledged during start, OR-latched.
    pub flags: [u8; 3],
}

/// What `start` has observed so far, kept whether or not it succeeds.
#[derive(Default)]
struct Evidence {
    unverified: bool,
    cc_status: Option<[u8; 2]>,
    acknowledged: [u8; 3],
}
impl Evidence {
    async fn write<B: I2c>(
        &mut self,
        bus: &mut B,
        register: ConfigRegister,
        value: u8,
    ) -> Result<(), Error<B::Error>> {
        self.unverified = true; // before the write await, even if NACKed/cancelled
        pd_cc::write_checked(bus, register, value)
            .await
            .map_err(Error::Config)
    }
}

async fn read<B: I2c>(bus: &mut B, register: u8) -> Result<u8, Error<B::Error>> {
    let mut byte = [0];
    bus.write_read(pd::ADDRESS, &[register], &mut byte)
        .await
        .map_err(|source| Error::Config(pd_cc::Error::Bus { register, source }))?;
    Ok(byte[0])
}

/// The first snapshot register that differs from `expected`, with its value.
async fn mismatch<B: I2c>(
    bus: &mut B,
    expected: [u8; 11],
) -> Result<Option<(u8, u8)>, Error<B::Error>> {
    let snapshot = pd::snapshot(bus).await.map_err(|f| {
        Error::Config(pd_cc::Error::Bus {
            register: f.register,
            source: f.source,
        })
    })?;
    Ok(expected
        .iter()
        .zip(snapshot.values)
        .zip(pd::REGISTERS)
        .find(|((expected, value), _)| *expected != value)
        .map(|((_, value), register)| (register, value)))
}

impl Sniffer {
    pub const fn once(baseline: [u8; 11], measure: u8) -> Self {
        Self { baseline, measure }
    }
    /// Orientation, clean FIFO boundary, then listen on the active CC.
    /// `released` is the verified post-release snapshot.
    pub async fn start<B: I2c, D: DelayNs>(
        self,
        bus: &mut B,
        delay: &mut D,
        released: pd::Snapshot,
    ) -> Result<ActiveSniffer, StartFailure<B::Error>> {
        let mut evidence = Evidence::default();
        match self.listen(&mut evidence, bus, delay, released).await {
            Ok((selected, cc_status)) => Ok(ActiveSniffer {
                receiver: pd_service::Receiver::for_sniffer(),
                acknowledged: evidence.acknowledged,
                cc_status,
                selected,
                baseline: self.baseline,
                stopped: false,
            }),
            Err(error) => Err(StartFailure {
                error,
                unverified: evidence.unverified,
                cc_status: evidence.cc_status,
                flags: evidence.acknowledged,
            }),
        }
    }
    /// The verified start sequence; returns the selected CC and both CC samples.
    async fn listen<B: I2c, D: DelayNs>(
        &self,
        evidence: &mut Evidence,
        bus: &mut B,
        delay: &mut D,
        released: pd::Snapshot,
    ) -> Result<(u8, [u8; 2]), Error<B::Error>> {
        for (i, expected) in self.baseline.into_iter().enumerate() {
            if released.values[i] != expected {
                return Err(Error::Config(pd_cc::Error::Baseline {
                    register: pd::REGISTERS[i],
                    value: released.values[i],
                }));
            }
        }
        let measure = read(bus, register::MEASURE).await?;
        if measure != self.measure {
            return Err(Error::Config(pd_cc::Error::Baseline {
                register: register::MEASURE,
                value: measure,
            }));
        }
        // Measure block + references (also powers the receiver; flushed below).
        evidence.write(bus, ConfigRegister::Power, 0x07).await?;
        let mut cc = [0; 2];
        for (i, switches) in [0x04, 0x08].into_iter().enumerate() {
            evidence
                .write(bus, ConfigRegister::Switches0, switches)
                .await?;
            delay.delay_ms(10).await; // nominal settling; comparator levels only
            cc[i] = read(bus, STATUS0).await?;
        }
        evidence.cc_status = Some(cc);
        let selected = pd_sniff::orientation(cc).ok_or(Error::Orientation(cc))?;
        // Keep receiver/references OFF while clocking the flush.
        evidence.write(bus, ConfigRegister::Power, 0x09).await?;
        evidence
            .write(bus, ConfigRegister::Control1Flush, 0x04)
            .await?;
        let status = read(bus, register::STATUS1).await?;
        if status & 0x3f != 0x28 {
            return Err(Error::Boundary { status });
        }
        for (i, register) in register::INTERRUPTS.into_iter().enumerate() {
            evidence.acknowledged[i] |= read(bus, register).await?;
        }
        let acknowledged = evidence.acknowledged;
        if acknowledged[0] & 1 != 0 {
            return Err(Error::Rx(pd_service::Error::HardReset));
        }
        if acknowledged[0] & 0x80 != 0 {
            return Err(Error::Rx(pd_service::Error::PhyFault));
        }
        if acknowledged[0] & 0x7c != 0 || acknowledged[1] & 1 != 0 || acknowledged[2] & 2 != 0 {
            return Err(Error::Rx(pd_service::Error::UnexpectedTransmission));
        }
        evidence
            .write(
                bus,
                ConfigRegister::Switches0,
                if selected == 1 { 0x04 } else { 0x08 },
            )
            .await?;
        evidence.write(bus, ConfigRegister::Power, 0x07).await?;
        if let Some((register, value)) = mismatch(bus, listening(self.baseline, selected)).await? {
            return Err(Error::Config(pd_cc::Error::Verify { register, value }));
        }
        Ok((selected, cc))
    }
}

/// A started sniffer, listening on `selected`. Any error or cancellation of
/// `poll`/`check` stops it for good.
pub struct ActiveSniffer {
    receiver: pd_service::Receiver,
    acknowledged: [u8; 3],
    cc_status: [u8; 2],
    selected: u8,
    baseline: [u8; 11],
    stopped: bool,
}
impl ActiveSniffer {
    pub fn selected(&self) -> u8 {
        self.selected
    }
    pub fn cc_status(&self) -> [u8; 2] {
        self.cc_status
    }
    /// Read-clear flags observed so far, OR-latched (not event counts).
    pub fn flags(&mut self) -> [u8; 3] {
        for (known, received) in self
            .acknowledged
            .iter_mut()
            .zip(self.receiver.take_acknowledged())
        {
            *known |= received;
        }
        self.acknowledged
    }
    /// One receive poll, then one read-only STATUS0 sample for the Rp indicator.
    pub async fn poll<B: I2c>(
        &mut self,
        bus: &mut B,
    ) -> Result<(pd_service::Outcome, u8), Error<B::Error>> {
        if self.stopped {
            return Err(Error::Stopped);
        }
        self.stopped = true; // persistent halt if this future is cancelled
        let result = self.receiver.poll(bus).await;
        self.flags(); // account observed flags even if a later operation failed
        let outcome = result.map_err(Error::Rx)?;
        let status0 = read(bus, STATUS0).await?;
        self.stopped = false;
        Ok((outcome, status0))
    }
    /// Read-only re-check of the 11 configuration bytes against the listening set.
    pub async fn check<B: I2c>(&mut self, bus: &mut B) -> Result<(), Error<B::Error>> {
        if self.stopped {
            return Err(Error::Stopped);
        }
        self.stopped = true;
        if let Some((register, value)) =
            mismatch(bus, listening(self.baseline, self.selected)).await?
        {
            return Err(Error::Drift { register, value });
        }
        self.stopped = false;
        Ok(())
    }
}
