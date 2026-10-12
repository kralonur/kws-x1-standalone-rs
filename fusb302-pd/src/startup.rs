//! Caller-baseline-gated FUSB startup measurements and passive release.
use crate::config::{ConfigRegister, Error, write_checked};
use crate::pd_rx::{self, Sop};
use crate::register::{self, control1, interrupt, power, status0, status1, switches0};
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
    /// Only when both CCs carry Rp (the port's plug sits in the source's
    /// receptacle, which drives Rp on both pins): what listening heard.
    pub listen: Option<Listen>,
    pub after: Snapshot,
}

/// What the receive-only listen on both CCs heard.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Listen {
    /// The CC (1 or 2) the source talks on: the one CC that received
    /// CRC-valid packets or, when neither or both did, the one CC that showed
    /// BMC activity or, when both received packets, the one with the higher
    /// Rp level (`by_level`).
    pub traffic: Option<u8>,
    /// `traffic` was chosen by Rp level: both CCs received packets.
    pub by_level: bool,
    /// Per CC: CRC-valid packets addressed to the port partner (SOP) and to a
    /// cable plug (SOP' or SOP''), counted from the RX FIFO after its window.
    pub partner: [u8; 2],
    pub cable: [u8; 2],
    /// The CC (1 or 2) above the MDAC level while the other is not: the
    /// source powers VCONN there, so it talks on the other one and the
    /// listen is skipped (`vconn_line`).
    pub vconn: Option<u8>,
}

/// The CC carrying VCONN: with the caller's MDAC at 2.1 V (above the 2.04 V
/// vRd-3.0 maximum, USB Type-C Table 4-36), a quiet CC whose comparator is
/// set is not an Rd line. Deciding here, as the source firmware did from its
/// CC voltages, skips the listen windows, in which nothing acknowledges the
/// source's early SOP' Discover Identity requests.
fn vconn_line(cc: [u8; 2]) -> Option<u8> {
    let high = cc.map(|status| status & (status0::COMP | status0::ACTIVITY) == status0::COMP);
    only_one(high)
}

/// The CC with the higher Rp level (STATUS0 BC_LVL) when both received the
/// source's packets. A source heard on both pins of its receptacle carries
/// its signal on both (a power bank that did worked on either pin), so this
/// only picks one; a source heard on one pin never gets here. Equal levels
/// give no answer.
fn stronger_line(cc: [u8; 2]) -> Option<u8> {
    let [cc1, cc2] = cc.map(|status| status & status0::BC_LVL);
    match cc1.cmp(&cc2) {
        core::cmp::Ordering::Greater => Some(1),
        core::cmp::Ordering::Less => Some(2),
        core::cmp::Ordering::Equal => None,
    }
}

/// Largest FIFO frame the listen counts: token, header, seven data objects
/// and CRC (USB PD section 6.2.1). Extended frames end the count.
const MAX_FRAME: usize = 1 + 2 + 7 * 4 + 4;

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
        let listen = if cc1 & status0::BC_LVL != 0 && cc2 & status0::BC_LVL != 0 {
            Some(match vconn_line([cc1, cc2]) {
                Some(vconn) => Listen {
                    traffic: Some(3 - vconn),
                    vconn: Some(vconn),
                    ..Listen::default()
                },
                None => {
                    let mut heard = listen(bus, delay).await?;
                    if heard.traffic.is_none() && heard.partner.iter().all(|&count| count > 0) {
                        heard.traffic = stronger_line([cc1, cc2]);
                        heard.by_level = heard.traffic.is_some();
                    }
                    heard
                }
            })
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
            listen,
            after,
        })
    }
}

/// Receive only, Rd kept on both CCs and AUTO_CRC off (the baseline), so
/// nothing is acknowledged: listen on each CC for `TRAFFIC_LISTEN_MS`, with
/// SOP'/SOP'' reception on, note whose latched I_ACTIVITY was set (reading
/// INTERRUPT clears it) and count the packets the FIFO holds. SOP'/SOP''
/// reception is switched off again and the FIFO flushed before returning.
async fn listen<B: I2c, D: DelayNs>(bus: &mut B, delay: &mut D) -> Result<Listen, Error<B::Error>> {
    let mut heard = Listen::default();
    let mut active = [false; 2];
    write_checked(
        bus,
        ConfigRegister::Control1,
        control1::ENSOP1 | control1::ENSOP2,
    )
    .await?;
    for (index, measure) in [switches0::MEAS_CC1, switches0::MEAS_CC2]
        .into_iter()
        .enumerate()
    {
        write_checked(
            bus,
            ConfigRegister::Switches0,
            switches0::PDWN1 | switches0::PDWN2 | measure,
        )
        .await?;
        write_checked(bus, ConfigRegister::Control1Flush, control1::RX_FLUSH).await?;
        read(bus, register::INTERRUPT).await?;
        delay.delay_ms(TRAFFIC_LISTEN_MS).await;
        active[index] = read(bus, register::INTERRUPT).await? & interrupt::I_ACTIVITY != 0;
        let (partner, cable) = count_frames(bus).await?;
        heard.partner[index] = partner;
        heard.cable[index] = cable;
    }
    write_checked(bus, ConfigRegister::Control1, 0).await?;
    write_checked(bus, ConfigRegister::Control1Flush, control1::RX_FLUSH).await?;
    let received = heard.partner.map(|count| count > 0);
    heard.traffic = only_one(received).or(only_one(active));
    Ok(heard)
}

/// The CC (1 or 2) whose flag alone is set.
fn only_one(flags: [bool; 2]) -> Option<u8> {
    match flags {
        [true, false] => Some(1),
        [false, true] => Some(2),
        _ => None,
    }
}

/// Count the complete, CRC-valid frames in the RX FIFO addressed to the
/// partner and to a cable plug. The first frame that does not decode (an
/// extended or overrun one) ends the count; the caller flushes the rest.
async fn count_frames<B: I2c>(bus: &mut B) -> Result<(u8, u8), Error<B::Error>> {
    let (mut partner, mut cable) = (0u8, 0u8);
    let mut frame = [0; MAX_FRAME];
    while read(bus, register::STATUS1).await? & status1::RX_EMPTY == 0 {
        let mut prefix = [0; 3];
        read_fifo(bus, &mut prefix).await?;
        let Ok(length) = pd_rx::fifo_frame_len(&prefix) else {
            break;
        };
        let Some(rest) = frame.get_mut(3..length) else {
            break;
        };
        read_fifo(bus, rest).await?;
        frame[..3].copy_from_slice(&prefix);
        let Ok((decoded, _)) = pd_rx::decode(&frame[..length]) else {
            break;
        };
        let count = match decoded.sop() {
            Sop::Partner => &mut partner,
            Sop::Cable | Sop::CableDoublePrime => &mut cable,
            Sop::Debug | Sop::DebugDoublePrime => continue,
        };
        *count = count.saturating_add(1);
    }
    Ok((partner, cable))
}

async fn read_fifo<B: I2c>(bus: &mut B, bytes: &mut [u8]) -> Result<(), Error<B::Error>> {
    bus.write_read(pd::ADDRESS, &[register::FIFOS], bytes)
        .await
        .map_err(|source| Error::Bus {
            register: register::FIFOS,
            source,
        })
}
async fn read<B: I2c>(bus: &mut B, register: u8) -> Result<u8, Error<B::Error>> {
    let mut value = [0];
    bus.write_read(pd::ADDRESS, &[register], &mut value)
        .await
        .map_err(|source| Error::Bus { register, source })?;
    Ok(value[0])
}
