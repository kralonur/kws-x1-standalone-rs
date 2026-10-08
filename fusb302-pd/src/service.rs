//! Bounded receive transport for guarded RX capture and the checked SPR PHY.
//! Interrupt reads acknowledge hardware; FIFO reads consume bytes. No writes.
//! RX-only mode rejects transmission flags; SPR mode exposes them to its owner.
//! Sniffer mode is RX-only and also rejects retry/toggle/collision/TX-FIFO evidence.

use crate::pd_rx;
use crate::register::{FIFOS, INTERRUPTS, STATUS1};
use crate::snapshot::ADDRESS;
use embedded_hal_async::i2c::I2c;

#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Halted,
    Bus { register: u8, source: E },
    HardReset,
    PhyFault,
    UnexpectedTransmission,
    TransmissionFault,
    RxFull,
    Frame(pd_rx::Error),
}

/// Owned, CRC-checked packet. No public constructor or mutable byte access.
#[derive(Debug, PartialEq, Eq)]
pub struct Packet {
    bytes: [u8; 35],
    length: usize,
}
impl Packet {
    pub fn bytes(&self) -> &[u8] {
        &self.bytes[..self.length]
    }
    /// Invariant: a `Packet` is only built after its bytes decoded. Admitting
    /// extended frames on every address cannot reject them again: the flags
    /// only gate frames whose Extended bit was already admitted.
    pub fn frame(&self) -> pd_rx::Frame<'_> {
        pd_rx::decode_with(self.bytes(), true, true)
            .expect("packet bytes were decoded before the packet was built")
            .0
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum Outcome {
    Idle,
    AwaitingCompletion,
    Packet(Packet),
}

/// One persistent owner per known-clean FIFO session; not a recovery mechanism.
/// At most one packet / seven transactions per poll (six for ordinary frames).
/// Caller owns the bus mutex,
/// deadlines and bounded scheduling/queue; never invoke from ISR.
/// Any error or cancellation AFTER first poll permanently halts this instance.
pub struct Receiver {
    halted: bool,
    draining: bool,
    acknowledged: [u8; 3],
    attempted_register: Option<u8>,
    active_spr: bool,
    cable_extended: bool,
    partner_extended: bool,
    sniffer: bool,
    status: u8,
}
impl Receiver {
    /// Caller must establish a clean FIFO boundary with separately reviewed PHY
    /// setup. Creating a fresh instance does not reset/flush hardware or a bus.
    pub const fn at_clean_boundary() -> Self {
        Self {
            halted: false,
            draining: false,
            acknowledged: [0; 3],
            attempted_register: None,
            active_spr: false,
            cable_extended: false,
            partner_extended: false,
            sniffer: false,
            status: 0,
        }
    }
    /// Only the reviewed SPR owner may tolerate TX/GoodCRC flags and use their
    /// completion hints. It must validate pending TX, matching ACK and faults.
    /// It frames single partner Chunks so policy can answer them (Table 7.1).
    pub const fn for_spr_boundary() -> Self {
        Self {
            active_spr: true,
            partner_extended: true,
            ..Self::at_clean_boundary()
        }
    }
    /// Passthrough sniffer (`docs/SAFETY.md`): RX-only like
    /// `at_clean_boundary`, with single partner Chunks framed for the log. Any
    /// sign that the PHY transmitted, retried, toggled, saw a TX collision or
    /// holds TX FIFO bytes halts it: the tester must never transmit.
    pub const fn for_sniffer() -> Self {
        Self {
            partner_extended: true,
            sniffer: true,
            ..Self::at_clean_boundary()
        }
    }
    pub fn allow_cable_extended(&mut self, allow: bool) {
        self.cable_extended = allow && self.active_spr;
    }
    pub fn status(&self) -> u8 {
        self.status
    }
    pub fn halted(&self) -> bool {
        self.halted
    }
    /// Last attempted address, including a cancelled operation whose hardware
    /// progress/acknowledged bits are unknowable through the I2C trait.
    pub fn attempted_register(&self) -> Option<u8> {
        self.attempted_register
    }
    /// Observed read-clear flags, OR-latched even on later error/cancellation.
    /// Coalesced bits are NOT event/packet counters. Unknown in-flight results
    /// cannot be reconstructed. Caller must record these before discarding state.
    pub fn take_acknowledged(&mut self) -> [u8; 3] {
        core::mem::take(&mut self.acknowledged)
    }
    async fn read<B: I2c>(
        &mut self,
        bus: &mut B,
        register: u8,
        bytes: &mut [u8],
    ) -> Result<(), Error<B::Error>> {
        self.attempted_register = Some(register);
        bus.write_read(ADDRESS, &[register], bytes)
            .await
            .map_err(|source| Error::Bus { register, source })
    }
    pub async fn poll<B: I2c>(&mut self, bus: &mut B) -> Result<Outcome, Error<B::Error>> {
        if self.halted {
            return Err(Error::Halted);
        }
        // Set before the first await: dropping any pending operation cannot
        // silently reuse a potentially acknowledged/partially consumed session.
        self.halted = true;
        let mut interrupts = [0; 3];
        for (i, register) in INTERRUPTS.into_iter().enumerate() {
            self.read(bus, register, &mut interrupts[i..i + 1]).await?;
            self.acknowledged[i] |= interrupts[i];
        }
        if interrupts[0] & 1 != 0 {
            return Err(Error::HardReset);
        }
        if interrupts[0] & 0x80 != 0 {
            return Err(Error::PhyFault);
        }
        if !self.active_spr && (interrupts[0] & 0x0c != 0 || interrupts[1] & 1 != 0) {
            return Err(Error::UnexpectedTransmission);
        }
        // INTERRUPTA TXSENT/HARDSENT/RETRYFAIL/SOFTFAIL/TOGDONE, INTERRUPT I_COLLISION.
        if self.sniffer && (interrupts[0] & 0x7c != 0 || interrupts[2] & 2 != 0) {
            return Err(Error::UnexpectedTransmission);
        }
        // SPR: HARDSENT/SOFTFAIL (no Hard Reset or automatic Soft Reset is ever
        // requested) and I_COLLISION are faults. I_SOFTRST (a Soft_Reset was
        // received) and I_RETRYFAIL (no GoodCRC after retries) are reported to
        // the owner through `take_acknowledged`, which must validate them.
        if self.active_spr && (interrupts[0] & 0x28 != 0 || interrupts[2] & 2 != 0) {
            return Err(Error::TransmissionFault);
        }
        self.draining |= self.active_spr && (interrupts[0] & 4 != 0 || interrupts[1] & 1 != 0);
        self.draining |= interrupts[2] & 0x10 != 0; // I_CRC_CHK: completion hint, not CRC proof.
        let mut status = [0];
        self.read(bus, STATUS1, &mut status).await?;
        self.status = status[0];
        if status[0] & 3 != 0 || (self.active_spr && status[0] & 4 != 0) {
            return Err(Error::PhyFault);
        }
        // STATUS1 TX_EMPTY set, TX_FULL clear: nothing was ever queued to send.
        if self.sniffer && status[0] & 0x0c != 0x08 {
            return Err(Error::UnexpectedTransmission);
        }
        if status[0] & 0x10 != 0 {
            return Err(Error::RxFull);
        }
        let outcome = if status[0] & 0x20 != 0 {
            self.draining = false;
            Outcome::Idle
        } else if !self.draining {
            // RX_EMPTY alone does not establish a complete packet boundary.
            Outcome::AwaitingCompletion
        } else {
            let mut packet = Packet {
                bytes: [0; 35],
                length: 0,
            };
            self.read(bus, FIFOS, &mut packet.bytes[..3]).await?;
            let [b0, b1, b2, ..] = packet.bytes;
            let prefix = [b0, b1, b2];
            let extended = prefix[2] & 0x80 != 0
                && (self.cable_extended && matches!(prefix[0] >> 5, 5 | 6)
                    || self.partner_extended && prefix[0] >> 5 == 7);
            let body_start = if extended {
                self.read(bus, FIFOS, &mut packet.bytes[3..5]).await?;
                let [_, _, _, b3, b4, ..] = packet.bytes;
                packet.length = pd_rx::fifo_extended_frame_len(
                    &[b0, b1, b2, b3, b4],
                    self.cable_extended,
                    self.partner_extended,
                )
                .map_err(Error::Frame)?;
                5
            } else {
                packet.length = pd_rx::fifo_frame_len(&prefix).map_err(Error::Frame)?;
                3
            };
            self.read(bus, FIFOS, &mut packet.bytes[body_start..packet.length])
                .await?;
            if extended {
                pd_rx::decode_with(packet.bytes(), self.cable_extended, self.partner_extended)
            } else {
                pd_rx::decode(packet.bytes())
            }
            .map_err(Error::Frame)?;
            Outcome::Packet(packet)
        };
        // Keep draining across coalesced events until observed empty; if a later
        // partial/malformed tail exists, software CRC/error handling halts rather
        // than scanning for tokens, retrying FIFO reads or resetting hardware.
        self.halted = false;
        Ok(outcome)
    }
}
