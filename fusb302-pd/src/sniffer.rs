//! Receive-only passthrough sniffer (`docs/PD-SNIFFER.md`). Starts only from the
//! verified passive release (SWITCHES0=00, `pd_cc::Passive`).
//!
//! Write allowlist: POWER 07/09, SWITCHES0 04/08 (measure CC1/CC2; no Rd, Rp or
//! VCONN), one CONTROL1 RX_FLUSH. SWITCHES1 (AUTO_CRC, TXCC), CONTROL0
//! (TX_START), CONTROL2 (TOGGLE), CONTROL3 (Hard Reset), RESET, MEASURE, the masks
//! and the FIFO are never written, so the tester never transmits or acknowledges
//! (no GoodCRC). Errors/cancellation halt without retry or cleanup writes.
use crate::pd_sniff;
use crate::{config as pd_cc, service as pd_service, snapshot as pd};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use pd_cc::ConfigRegister;

const STATUS0: u8 = 0x40;

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

/// One sniffer per boot. Once started, any error or cancellation stops it for good.
pub struct Sniffer {
    started: bool,
    stopped: bool,
    unverified: bool,
    receiver: Option<pd_service::Receiver>,
    acknowledged: [u8; 3],
    cc_status: Option<[u8; 2]>,
    selected: Option<u8>,
    baseline: [u8; 11],
    measure: u8,
}
impl Sniffer {
    pub const fn once(baseline: [u8; 11], measure: u8) -> Self {
        Self {
            started: false,
            stopped: true,
            unverified: false,
            receiver: None,
            acknowledged: [0; 3],
            cc_status: None,
            selected: None,
            baseline,
            measure,
        }
    }
    /// A write was attempted and the full listening configuration is not verified.
    pub fn unverified(&self) -> bool {
        self.unverified
    }
    pub fn selected(&self) -> Option<u8> {
        self.selected
    }
    pub fn cc_status(&self) -> Option<[u8; 2]> {
        self.cc_status
    }
    /// Read-clear flags observed so far, OR-latched (not event counts).
    pub fn flags(&mut self) -> [u8; 3] {
        if let Some(receiver) = &mut self.receiver {
            for (known, received) in self
                .acknowledged
                .iter_mut()
                .zip(receiver.take_acknowledged())
            {
                *known |= received;
            }
        }
        self.acknowledged
    }
    async fn read<B: I2c>(&mut self, bus: &mut B, register: u8) -> Result<u8, Error<B::Error>> {
        let mut byte = [0];
        bus.write_read(pd::ADDRESS, &[register], &mut byte)
            .await
            .map_err(|source| Error::Config(pd_cc::Error::Bus { register, source }))?;
        Ok(byte[0])
    }
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
    /// Orientation, clean FIFO boundary, then listen on the active CC.
    /// `released` is the verified post-release snapshot.
    pub async fn start<B: I2c, D: DelayNs>(
        &mut self,
        bus: &mut B,
        delay: &mut D,
        released: pd::Snapshot,
    ) -> Result<(), Error<B::Error>> {
        if self.started {
            return Err(Error::Stopped);
        }
        self.started = true;
        for (i, expected) in self.baseline.into_iter().enumerate() {
            if released.values[i] != expected {
                return Err(Error::Config(pd_cc::Error::Baseline {
                    register: pd::REGISTERS[i],
                    value: released.values[i],
                }));
            }
        }
        let measure = self.read(bus, 0x04).await?;
        if measure != self.measure {
            return Err(Error::Config(pd_cc::Error::Baseline {
                register: 0x04,
                value: measure,
            }));
        }
        // Measure block + references (also powers the receiver; flushed below).
        self.write(bus, ConfigRegister::Power, 0x07).await?;
        let mut cc = [0; 2];
        for (i, switches) in [0x04, 0x08].into_iter().enumerate() {
            self.write(bus, ConfigRegister::Switches0, switches).await?;
            delay.delay_ms(10).await; // nominal settling; comparator levels only
            cc[i] = self.read(bus, STATUS0).await?;
        }
        self.cc_status = Some(cc);
        let selected = pd_sniff::orientation(cc).ok_or(Error::Orientation(cc))?;
        // Receiver/references OFF while clocking the flush (attempt31 sequence).
        self.write(bus, ConfigRegister::Power, 0x09).await?;
        self.write(bus, ConfigRegister::Control1Flush, 0x04).await?;
        let status = self.read(bus, 0x41).await?;
        if status & 0x3f != 0x28 {
            return Err(Error::Boundary { status });
        }
        for (i, register) in [0x3e, 0x3f, 0x42].into_iter().enumerate() {
            self.acknowledged[i] |= self.read(bus, register).await?;
        }
        if self.acknowledged[0] & 1 != 0 {
            return Err(Error::Rx(pd_service::Error::HardReset));
        }
        if self.acknowledged[0] & 0x80 != 0 {
            return Err(Error::Rx(pd_service::Error::PhyFault));
        }
        if self.acknowledged[0] & 0x7c != 0
            || self.acknowledged[1] & 1 != 0
            || self.acknowledged[2] & 2 != 0
        {
            return Err(Error::Rx(pd_service::Error::UnexpectedTransmission));
        }
        self.write(
            bus,
            ConfigRegister::Switches0,
            if selected == 1 { 0x04 } else { 0x08 },
        )
        .await?;
        self.write(bus, ConfigRegister::Power, 0x07).await?;
        self.check_config(bus, selected).await?;
        self.unverified = false;
        self.receiver = Some(pd_service::Receiver::for_sniffer());
        self.selected = Some(selected);
        self.stopped = false;
        Ok(())
    }
    async fn check_config<B: I2c>(&mut self, bus: &mut B, cc: u8) -> Result<(), Error<B::Error>> {
        let expected = listening(self.baseline, cc);
        let snapshot = pd::snapshot(bus).await.map_err(|f| {
            Error::Config(pd_cc::Error::Bus {
                register: f.register,
                source: f.source,
            })
        })?;
        for (i, &value) in expected.iter().enumerate() {
            if snapshot.values[i] != value {
                return Err(if self.receiver.is_some() {
                    Error::Drift {
                        register: pd::REGISTERS[i],
                        value: snapshot.values[i],
                    }
                } else {
                    Error::Config(pd_cc::Error::Verify {
                        register: pd::REGISTERS[i],
                        value: snapshot.values[i],
                    })
                });
            }
        }
        Ok(())
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
        let receiver = self
            .receiver
            .as_mut()
            .expect("started at a verified boundary");
        let result = receiver.poll(bus).await;
        self.flags(); // account observed flags even if a later operation failed
        let outcome = result.map_err(Error::Rx)?;
        let status0 = self.read(bus, STATUS0).await?;
        self.stopped = false;
        Ok((outcome, status0))
    }
    /// Read-only re-check of the 11 configuration bytes against the listening set.
    pub async fn check<B: I2c>(&mut self, bus: &mut B) -> Result<(), Error<B::Error>> {
        if self.stopped {
            return Err(Error::Stopped);
        }
        self.stopped = true;
        let cc = self.selected.expect("started");
        self.check_config(bus, cc).await?;
        self.stopped = false;
        Ok(())
    }
}
