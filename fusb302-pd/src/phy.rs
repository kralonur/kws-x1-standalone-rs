//! Checked native FUSB302 fixed/PPS SPR adapter. One bus owner, one Sink session.
//! Read-Clear registers are never written. No Rp/power-role swaps.
//! Explicit PD3 fixed5V inspection uses caller-authorized VCONN routing and SOP' reads.
//! SOP'' alone uses an authorized synchronization Soft_Reset, then Get_Status.
//! SOP Soft Reset (`docs/SAFETY.md`): a partner Soft_Reset is accepted in
//! any state; one Soft_Reset is sent after a Protocol Error or a Message without
//! GoodCRC. Hard Reset (`docs/SAFETY.md`): sent only in the mandatory
//! cases, never for SinkWaitCapTimer; after one is sent or received the session
//! ends and `transition_to_default` resets the PHY to its boot state, Rd
//! unchanged. No VCONN_Swap or error recovery/retry.
//! Sink-initiated AMS starts pass the SinkTxOK gate (`docs/SAFETY.md`), with
//! an explicit caller policy option for the nonstandard stale-SinkTxNG fallback
//! (see `stale_sink_tx_ng`); no default opt-in is provided.
//! Errors/cancellation halt without hardware cleanup; only normal completion restores.
use crate::header;
use crate::register::{
    self, control0, control1, control3, interrupt, interrupta, interruptb, power, reset, status0,
    status1, switches0, switches1, tx_token,
};
use crate::snapshot::{self, Snapshot};
use crate::{config as pd_cc, observation as pd, service as pd_service};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use pd_cc::ConfigRegister;
use pd_protocol::pd_rx::Sop;
use pd_protocol::pd_spr::Limits;
use usbpd_traits::{Driver, DriverRxError, DriverTxError};

mod tx;
use tx::PendingTx;

/// Board policy and physical observations supplied by the caller. No sensor,
/// power-on baseline, pin topology or nonstandard AMS override is assumed.
pub trait Platform {
    type Limits: Limits;
    /// Caller-selected cable receive deadline and no-progress bound, in ms.
    const CABLE_RESPONSE_TIMEOUT_MS: u64;
    const CABLE_SVID_NO_PROGRESS_TIMEOUT_MS: u64;
    const BASELINE: [u8; 11];
    const MEASURE: u8;
    const INSPECTION_CC: u8;
    const INSPECTION_SWITCHES0: u8;
    const ALLOW_STALE_SINK_TX_NG: bool;
    const VBUS_ADDRESS: u8;
    const VBUS_REGISTER: u8;
    fn voltage_mv(raw: u16) -> u32;
    fn set_rx_idle(idle: bool);
    fn vbus_raw<B: I2c>(
        bus: &mut B,
    ) -> impl core::future::Future<Output = Result<u16, Error<B::Error>>>;
}

// Covers the 10-second window at 1 ms/poll, with handshake margin.
const POLL_BUDGET: u16 = 16_384;
// Software bound on waiting for SinkTxOK before the AMS start is Deferred. Longer
// than tSinkTx max (20 ms, Table 6.68): a Source that asserts SinkTxNG to start
// its own AMS is normally seen as a received Message first. Not a spec timer.
const SINK_TX_WAIT_MS: u64 = 100;
// BIST Carrier Mode length: nominal tBISTContMode (30-60 ms, Table 6.68); the
// carrier must stop within tBISTContMode of starting (section 7.31.6.2).
const BIST_CARRIER_MS: u32 = 45;
// tVCONNStable wait before the first cable Message.
const VCONN_SETTLE_MS: u64 = 50;
/// Window after the Get_PPS_Status GoodCRC in which a PPS_Status or
/// Not_Supported is its answer: the same selected tSenderResponse (Table 7.9)
/// and GoodCRC origin as the engine's SenderResponseTimer.
const PPS_STATUS_ANSWER_MS: u64 = usbpd_traits::SENDER_RESPONSE_MS;
/// The same window for the answer to an information request (`pd_partner`).
const QUERY_ANSWER_MS: u64 = usbpd_traits::SENDER_RESPONSE_MS;
/// Hard Reset timer cases are re-checked against the minimum of each timer
/// (Table 6.68): tSenderResponse 27 ms, tPSTransition (SPR) 450 ms.
const SENDER_RESPONSE_MIN_MS: u64 = 27;
const PS_TRANSITION_MIN_MS: u64 = 450;
/// tHardResetComplete max (Table 6.68): without I_HARDSENT by then, the
/// Hard Reset counts as sent (section 7.31.10.1).
const HARD_RESET_COMPLETE_MS: u64 = 5;
/// Section 4.4.1: observed VBUS below vSafe0V (0.8 V) within tPSHardReset
/// max + tSafe0V (35 + 650 ms), then back in 4.5–5.5 V within tSrcRecover max +
/// tSrcTurnOn (1000 + 275 ms); each with margin.
const VBUS_SAFE0_MV: u32 = 800;
const VBUS_OFF_MS: u64 = 1_000;
const VBUS_ON_MS: u64 = 1_500;

/// Bus access for the first Message of a Sink-initiated AMS (section 7.2): read
/// `registers`, then write `bytes` only if `allow` accepts the two samples. The
/// board implementation holds one I2C lease and gives no other task a turn from
/// the first sample to the last written byte. Returns the samples and whether
/// `bytes` were written.
pub trait GatedBus: I2c {
    fn gated_write(
        &mut self,
        address: u8,
        registers: [u8; 2],
        allow: impl FnOnce(u8, u8) -> bool,
        bytes: &[u8],
    ) -> impl core::future::Future<Output = Result<([u8; 2], bool), Self::Error>> {
        async move {
            let mut samples = [0; 2];
            for (sample, register) in samples.iter_mut().zip(registers) {
                self.write_read(address, &[register], core::slice::from_mut(sample))
                    .await?;
            }
            if !allow(samples[0], samples[1]) {
                return Ok((samples, false));
            }
            self.write(address, bytes).await?;
            Ok((samples, true))
        }
    }
}

/// FUSB302 STATUS0/STATUS1 gate for an AMS start. Always: no BMC activity and
/// an empty RX FIFO, so no SOP Message arrived first (section 7.3). Under a PD3
/// Explicit Contract also Rp = SinkTxOK, i.e. BC_LVL 3 (3 A) with COMP clear
/// (sections 5.2.2, 7.2, Table 5.1); SinkTxNG, Rp default or an open line refuse.
/// PD2 has no Rp signalling (PD2 V1.3 section 2.6.3).
pub fn sink_tx_allowed(status0: u8, status1: u8, rp_required: bool) -> bool {
    use crate::pd_sniff::{Rp, quiet, rp};
    quiet(status0)
        && status1 & status1::RX_EMPTY != 0
        && (!rp_required || rp(status0) == Some(Rp::Current3A0))
}

/// A refused sample that is only SinkTxNG: quiet line, empty RX FIFO and Rp
/// 1.5 A.
///
/// **NOT STANDARD — deliberate deviation from PD R3.2 section 7.2** ("the Sink
/// Shall Not initiate an AMS" while Rp is SinkTxNG). This requires explicit
/// caller policy; see `docs/SAFETY.md`.
///
/// Why: a tested PD3 charger holds SinkTxNG for the whole 5 V/100 mA
/// contract (measured BC_LVL 2 with Rd on one or both CCs, so not a board
/// artefact), while it shows SinkTxOK to a phone. Obeying it strictly blocks
/// every voltage change, which commercial testers do not do.
///
/// Why it is safe enough: SinkTxNG exists so a Source can start its own AMS
/// without a collision, and the Source sends within tSinkTx (16-20 ms, Table
/// 6.68) of asserting it. SinkTxNG that stays quiet, with nothing received, for
/// the whole `SINK_TX_WAIT_MS` (100 ms) window is stale. The AMS start is then
/// sent only if one more atomic sample still shows a quiet line and an empty RX
/// FIFO. A collision anyway would show up as a missing GoodCRC, handled as a
/// Protocol Error (Soft_Reset, `docs/SAFETY.md`).
pub fn stale_sink_tx_ng(status0: u8, status1: u8) -> bool {
    use crate::pd_sniff::{Rp, quiet, rp};
    quiet(status0) && status1 & status1::RX_EMPTY != 0 && rp(status0) == Some(Rp::Current1A5)
}

pub trait Clock {
    fn now_ms(&self) -> u64;
}
pub trait Waiter {
    fn delay_ns(&mut self, ns: u32) -> impl core::future::Future<Output = ()>;
    fn delay_ms(&mut self, ms: u32) -> impl core::future::Future<Output = ()> {
        self.delay_ns(ms * 1_000_000)
    }
    fn wait_event(&mut self) -> impl core::future::Future<Output = ()> {
        self.delay_ms(1)
    }
}
impl<T: DelayNs> Waiter for T {
    async fn delay_ns(&mut self, ns: u32) {
        DelayNs::delay_ns(self, ns).await;
    }
    async fn delay_ms(&mut self, ms: u32) {
        DelayNs::delay_ms(self, ms).await;
    }
}
#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Stopped,
    StopRequested,
    Orientation,
    InputVoltage {
        raw: u16,
    },
    Config(pd_cc::Error<E>),
    Bus {
        address: u8,
        register: u8,
        source: E,
    },
    Rx(pd_service::Error<E>),
    Boundary {
        status: u8,
    },
    UnexpectedAck,
    Request,
    QueueFull,
    Budget,
    TxTimeout,
    PpsDeadline,
    Completion,
    Peer,
    /// AMS start not sent (sections 7.2, 7.3); not a fault.
    Deferred,
    /// No GoodCRC after the hardware retries (I_RETRYFAIL): a Protocol Error
    /// answered with Soft_Reset (section 7.1.1); not a fault.
    NotAcknowledged,
    /// The spec requires Hard Reset here but this is not a maintained session
    /// (one-shot trial); not sent.
    HardResetNeeded,
    /// After a Hard Reset, VBUS did not fall to vSafe0V or did not return to
    /// vSafe5V in time (section 4.4.1).
    HardResetVbus,
    /// After BIST Carrier Mode, VBUS was not at vSafe5V (4.5-5.5 V) for the
    /// transition to default without Hard Reset (section 9.2.26.4.1.1).
    BistCarrierVbus,
}

use pd_protocol::pd_discovery::{self, Phase as CablePhase};

pub type Control<'a> = &'a dyn Fn(&mut pd::Observation, Option<&dyn pd_protocol::pd_svids::Store>);

/// The Soft Reset or post-reset step in progress (sections 6.8.1, 7.7). While
/// one is set it gates every transmission and the accepted received Messages.
#[derive(Clone, Copy, PartialEq, Eq)]
enum ResetStep {
    /// A Protocol Error was seen: the next transmission is our Soft_Reset.
    SendSoftReset,
    /// Our Soft_Reset was acknowledged: only Accept (or the partner's Soft_Reset).
    AwaitAccept,
    /// The partner's Soft_Reset was received: the next transmission is Accept.
    AcceptPartner,
    /// A Soft Reset completed: only fresh Source_Capabilities.
    WaitCapsAfterReset,
    /// After the BIST carrier (PE_SNK_Transition_to_default without Hard Reset):
    /// only fresh Source_Capabilities; anything else fails closed.
    WaitCapsAfterCarrier,
}

/// The stage of an acknowledged Request's AMS (section 8.3.3.3).
#[derive(Clone, Copy, PartialEq, Eq)]
enum RequestStage {
    /// Accept, Reject or Wait not yet received (PE_SNK_Select_Capability).
    AwaitReply,
    /// Accept received: power transitions until PS_RDY (PE_SNK_Transition_Sink).
    TransitionSink,
}

/// Caller policy for one session, fixed when the session is created.
pub struct Config<'a, L: Limits> {
    /// A long-lived session that stays in Ready after the first contract; only
    /// it may use Soft Reset, Hard Reset and Ready-state messages.
    pub persistent: bool,
    pub selection: Option<&'a pd_protocol::pd_spr::Selection<L>>,
    pub control: Option<Control<'a>>,
    /// Optional caller UI notification when a fresh identity inspection begins.
    /// It grants no transport permission; absence means no UI to invalidate.
    pub identity_started: Option<&'a dyn Fn()>,
    pub publish: Option<&'a dyn Fn(pd::Observation)>,
    /// `pd-boot-diagnostic`: record session steps in `observation.boot_step`
    /// and publish each new step at once (`docs/SAFETY.md`).
    pub trace_steps: bool,
    /// `cable-log`: record only the cable breadcrumbs (`cable_mark`).
    pub trace_cable: bool,
    pub stop_requested: Option<fn() -> bool>,
    pub svids: Option<&'a dyn pd_protocol::pd_svids::Store>,
}

/// Whether the session may perform I/O. `Halted` is the latch held across
/// every bus operation, so an error or a cancelled future leaves the session
/// stopped; `Finished` is terminal and also closes `transition_to_default`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Lifecycle {
    Running,
    Halted,
    Finished,
}

pub struct Session<'a, B: I2c, D, C, P: Platform> {
    bus: &'a mut B,
    delay: &'a mut D,
    clock: &'a C,
    lifecycle: Lifecycle,
    unverified: bool,
    register: Option<u8>,
    receiver: pd_service::Receiver,
    queue: [Option<pd_service::Packet>; 2],
    pending_tx: Option<PendingTx>,
    reset: Option<ResetStep>,
    protocol_started: bool,
    unsupported_reply: Option<u16>,
    // Object Position of the last acknowledged Request, and of the contract it
    // established at PS_RDY: object 1 is the vSafe5V Fixed PDO (section 6.4.1).
    requested_object: Option<u8>,
    contract_object: Option<u8>,
    // BIST Test Data Mode (section 6.4.3.1): CONTROL3 BIST_TMODE set, every
    // frame discarded after the hardware GoodCRC, no transmission until Hard Reset.
    bist_test_data: bool,
    // A BIST Carrier Mode classified in Ready at vSafe5V: one carrier allowed.
    bist_carrier_armed: bool,
    revision_reply: bool,
    sink_caps_reply: bool,
    skedb_reply: bool,
    status_reply: Option<[u8; 7]>,
    sink_alert_until: Option<u64>,
    // Received Source_Capabilities not yet answered: that Request is a response.
    caps_reply: bool,
    // Get_PPS_Status acknowledged: its answer window ends at this time.
    pps_status_until: Option<u64>,
    // An information request acknowledged: its answer window and which request.
    query_until: Option<(u64, pd_protocol::pd_partner::PartnerQuery)>,
    // INTERRUPTA bits seen since the last transmission was written.
    tx_wait_flags: u8,
    request: Option<RequestStage>,
    // Time of the Protocol Error that armed our Soft_Reset: it bounds
    // tProtErrSoftReset/tSoftReset (Table 6.68).
    protocol_error_at: Option<u64>,
    // From a completed Soft Reset until the renegotiated Request ends.
    recovering: bool,
    // A mandatory Hard Reset is due (`docs/SAFETY.md`); the time of
    // the Protocol Error bounds tHardReset/tProtErrHardReset (Table 6.68).
    hard_reset_armed: bool,
    hard_reset_error_at: Option<u64>,
    // Hard Reset Signaling written / received (I_HARDRST): the session is over.
    hard_reset_sent: bool,
    hard_reset_received: bool,
    // I_SOFTRST must match a received SOP Soft_Reset (or our own Soft_Reset TX).
    polls: u32,
    softrst_flag_at: Option<u32>,
    softrst_frame_at: Option<u32>,
    cable_phase: CablePhase,
    cable_rx_id: u8,
    tx_revision: u8,
    t_send: Option<u64>,
    t_ack: Option<u64>,
    t_accept: Option<u64>,
    error: Option<Error<B::Error>>,
    pub observation: pd::Observation,
    config: Config<'a, P::Limits>,
    steps_t0: Option<u64>,
    last_publish_ms: u64,
    last_poll_ms: Option<u64>,
    /// `pd-boot-diagnostic`: end of the last cable inspection, until Ready
    /// has run 2 s after it (steps 30/31, `cable_step`).
    cable_done_ms: Option<u64>,
    rp_probing: bool,
    stop_since_ms: Option<u64>,
}
impl<'a, B: GatedBus, D: Waiter, C: Clock, P: Platform> Session<'a, B, D, C, P> {
    /// Create a session and run its checked startup. The session is returned
    /// either way: after an error it is halted and still carries its evidence.
    pub async fn start(
        bus: &'a mut B,
        delay: &'a mut D,
        clock: &'a C,
        config: Config<'a, P::Limits>,
        observation: pd::Observation,
        before: Snapshot,
        cc: [u8; 2],
    ) -> (Self, Result<(), Error<B::Error>>) {
        let mut session = Self::new(bus, delay, clock, config, observation);
        let result = session.begin(before, cc).await;
        (session, result)
    }
    fn new(
        bus: &'a mut B,
        delay: &'a mut D,
        clock: &'a C,
        config: Config<'a, P::Limits>,
        observation: pd::Observation,
    ) -> Self {
        P::set_rx_idle(false);
        Self {
            bus,
            delay,
            clock,
            lifecycle: Lifecycle::Halted,
            unverified: false,
            register: None,
            receiver: pd_service::Receiver::for_spr_boundary(),
            queue: [None, None],
            pending_tx: None,
            reset: None,
            protocol_started: false,
            unsupported_reply: None,
            requested_object: None,
            contract_object: None,
            bist_test_data: false,
            bist_carrier_armed: false,
            revision_reply: false,
            sink_caps_reply: false,
            skedb_reply: false,
            status_reply: None,
            sink_alert_until: None,
            caps_reply: false,
            pps_status_until: None,
            query_until: None,
            tx_wait_flags: 0,
            request: None,
            protocol_error_at: None,
            recovering: false,
            hard_reset_armed: false,
            hard_reset_error_at: None,
            hard_reset_sent: false,
            hard_reset_received: false,
            polls: 0,
            softrst_flag_at: None,
            softrst_frame_at: None,
            cable_phase: CablePhase::Idle,
            cable_rx_id: 0,
            tx_revision: 2,
            t_send: None,
            t_ack: None,
            t_accept: None,
            error: None,
            observation,
            config,
            steps_t0: None,
            last_publish_ms: 0,
            last_poll_ms: None,
            cable_done_ms: None,
            rp_probing: false,
            stop_since_ms: None,
        }
    }
    pub fn stopped(&self) -> bool {
        self.lifecycle != Lifecycle::Running
    }
    pub fn unverified(&self) -> bool {
        self.unverified
    }
    pub fn register(&self) -> Option<u8> {
        if self.receiver.halted() {
            self.receiver.attempted_register().or(self.register)
        } else {
            self.register
        }
    }
    pub fn stop(&mut self) {
        P::set_rx_idle(false);
        self.lifecycle = Lifecycle::Finished;
    }
    pub fn take_error(&mut self) -> Option<Error<B::Error>> {
        self.error.take()
    }
    /// Hard Reset Signaling was written by `transmit_hard_reset`.
    pub fn hard_reset_sent(&self) -> bool {
        self.hard_reset_sent
    }
    /// The receiver saw Hard Reset Signaling (I_HARDRST).
    pub fn hard_reset_received(&self) -> bool {
        self.hard_reset_received
    }
    fn trace(&mut self) -> &mut pd::SprObservation {
        // `start` installs a pending trace, but the caller's observation may lack
        // one before it; a fresh pending trace fails every gate closed.
        self.observation
            .spr
            .get_or_insert_with(pd::SprObservation::pending)
    }
    async fn read(
        &mut self,
        address: u8,
        register: u8,
        bytes: &mut [u8],
    ) -> Result<(), Error<B::Error>> {
        self.register = Some(register);
        self.trace().failed_address = address;
        self.bus
            .write_read(address, &[register], bytes)
            .await
            .map_err(|source| Error::Bus {
                address,
                register,
                source,
            })
    }
    async fn byte(&mut self, register: u8) -> Result<u8, Error<B::Error>> {
        let mut bytes = [0];
        self.read(snapshot::ADDRESS, register, &mut bytes).await?;
        Ok(bytes[0])
    }
    async fn checked(
        &mut self,
        register: ConfigRegister,
        value: u8,
    ) -> Result<(), Error<B::Error>> {
        self.register = Some(register.address());
        self.trace().failed_address = snapshot::ADDRESS;
        self.unverified = true;
        pd_cc::write_checked(self.bus, register, value)
            .await
            .map_err(Error::Config)
    }
    async fn begin(&mut self, before: Snapshot, cc: [u8; 2]) -> Result<(), Error<B::Error>> {
        if self.observation.restore_unverified {
            self.unverified = true;
            return Err(Error::Stopped);
        }
        // Preserve fresh CC/baseline evidence, not an earlier port's protocol
        // history. This is not a reconnect loop or hardware recovery operation.
        self.observation = pd::Observation {
            snapshot: self.observation.snapshot,
            phase: self.observation.phase,
            irq_high: self.observation.irq_high,
            irq_count: self.observation.irq_count,
            cc_status: self.observation.cc_status,
            post_snapshot: self.observation.post_snapshot,
            spr: Some(pd::SprObservation::pending()),
            ..pd::Observation::new()
        };
        if self.config.selection.is_some() {
            self.trace().target_mv = 5000;
        }
        if let Some(svids) = self.config.svids {
            svids.clear();
        }
        for (index, expected) in P::BASELINE.into_iter().enumerate() {
            if before.values[index] != expected {
                return Err(Error::Config(pd_cc::Error::Baseline {
                    register: snapshot::REGISTERS[index],
                    value: before.values[index],
                }));
            }
        }
        if cc.iter().any(|status| status & status0::ACTIVITY != 0) {
            return Err(Error::Orientation);
        }
        let selected = match (cc[0] & 3 != 0, cc[1] & 3 != 0) {
            (true, false) => 1,
            (false, true) => 2,
            _ => return Err(Error::Orientation),
        };
        self.trace().tx_off_verified = true;
        let measure = self.byte(register::MEASURE).await?;
        if measure != P::MEASURE {
            return Err(Error::Config(pd_cc::Error::Baseline {
                register: 4,
                value: measure,
            }));
        }
        // Use the board's qualified voltage provider, never assume controller
        // VBUSOK connectivity or silently trust a sensor error value.
        let raw = self.vbus_raw().await?;
        let mv = P::voltage_mv(raw);
        if !(4500..=5500).contains(&mv) {
            return Err(Error::InputVoltage { raw });
        }
        self.trace().vbus_pre_mv = Some(mv as u16);
        self.checked(
            ConfigRegister::Power,
            power::OSCILLATOR | power::BANDGAP_WAKE,
        )
        .await?;
        self.checked(ConfigRegister::Control1Flush, control1::RX_FLUSH)
            .await?;
        let status = self.byte(register::STATUS1).await?;
        if status & status1::FIFOS_AND_FAULTS != status1::IDLE {
            return Err(Error::Boundary { status });
        }
        let mut flags = [0; 3];
        for (i, register) in register::INTERRUPTS.into_iter().enumerate() {
            flags[i] = self.byte(register).await?;
        }
        self.observation.rx_flags = flags;
        // A stale CRC flag never authorizes FIFO draining. Any reset/TX fault
        // at the clean boundary is fatal rather than explained away.
        if flags[0] & !interrupta::I_TOGDONE != 0
            || flags[1] & interruptb::I_GCRCSENT != 0
            || flags[2] & interrupt::I_COLLISION != 0
        {
            return Err(Error::Rx(pd_service::Error::TransmissionFault));
        }
        self.checked(
            ConfigRegister::Switches0,
            switches0::PDWN1
                | switches0::PDWN2
                | if selected == 1 {
                    switches0::MEAS_CC1
                } else {
                    switches0::MEAS_CC2
                },
        )
        .await?;
        // Keep GoodCRC SPECREV=01 and sink/UFP roles; do not write reserved
        // SPECREV=10. Ordinary packet revision is managed by upstream software.
        self.trace().tx_off_verified = false; // before enabling TX await, even on NACK
        self.checked(
            ConfigRegister::Switches1,
            switches1::SPECREV_2_0
                | switches1::AUTO_CRC
                | if selected == 1 {
                    switches1::TXCC1
                } else {
                    switches1::TXCC2
                },
        )
        .await?;
        self.steps_t0 = Some(self.clock.now_ms());
        self.step(3);
        self.checked(
            ConfigRegister::Power,
            power::BANDGAP_WAKE | power::RECEIVER | power::MEASURE | power::OSCILLATOR,
        )
        .await?;
        self.observation.selected_cc = Some(selected);
        self.lifecycle = Lifecycle::Running;
        Ok(())
    }
    async fn poll(&mut self) -> Result<Option<pd_service::Packet>, Error<B::Error>> {
        if let Some(selection) = self.config.selection {
            selection.set_now_ms(self.clock.now_ms());
            if selection.pps_expired() {
                return Err(Error::PpsDeadline);
            }
            if let Some(control) = self.config.control {
                control(&mut self.observation, self.config.svids);
            }
            self.trace().selected_mv = Some(selection.candidate_mv());
            self.trace().selected_ma = Some(selection.candidate_ma());
            self.trace().selected_pps = selection.candidate_is_pps();
            self.trace().adjust_current = selection.adjusting_current();
            if !selection.request_in_flight() && selection.established_target().is_some() {
                self.trace().target_mv = selection.target_mv();
                self.trace().target_ma = selection.target_ma();
                self.trace().pps = selection.target_pps_object() != 0;
            }
            if let Some(report) = selection.cable_result() {
                self.trace().cable_report = Some(report);
            }
            self.trace().pps_status = selection.pps_status();
            self.trace().partner = selection.partner_info();
        }
        if self.config.trace_steps
            && self.observation.rp_probe.is_none()
            && !self.rp_probing
            && self.trace().contract_seen
            && self.pending_tx.is_none()
            && !self.reset_in_progress()
            && self.cable_phase == CablePhase::Idle
            && self.queue.iter().all(Option::is_none)
            && self.config.selection.is_none_or(|s| !s.request_in_flight())
        {
            self.probe_rp().await?;
        }
        let live = self.config.persistent && self.trace().contract_seen;
        if !live && (self.observation.rx_polls >= POLL_BUDGET || self.observation.rx_packets >= 32)
        {
            return Err(Error::Budget);
        }
        let now = self.clock.now_ms();
        if self.config.trace_steps {
            if let Some(last) = self.last_poll_ms {
                let gap = now.saturating_sub(last).min(255) as u8;
                if gap > self.observation.poll_gap_ms {
                    self.observation.poll_gap_ms = gap;
                }
            }
            self.last_poll_ms = Some(now);
        }
        if let Some(done) = self.cable_done_ms {
            self.cable_mark(30); // Ready polls again after the inspection
            if now.saturating_sub(done) >= 2000 {
                self.cable_mark(31); // and is still running 2 s later
                self.cable_done_ms = None;
            }
        }
        if now.saturating_sub(self.last_publish_ms) >= 100 {
            self.last_publish_ms = now;
            if let Some(publish) = self.config.publish {
                let mut observation = self.observation;
                observation.restore_unverified = self.unverified;
                publish(observation);
            }
        }
        self.observation.rx_polls = self.observation.rx_polls.saturating_add(1);
        self.trace().failed_address = snapshot::ADDRESS;
        let outcome = self.receiver.poll(self.bus).await;
        self.register = self.receiver.attempted_register();
        let flags = self.receiver.take_acknowledged();
        self.tx_wait_flags |= flags[0];
        for (total, bits) in self.observation.rx_flags.iter_mut().zip(flags) {
            *total |= bits;
        }
        if matches!(outcome, Err(pd_service::Error::HardReset)) {
            self.hard_reset_received = true;
        }
        let outcome = outcome.map_err(Error::Rx)?;
        if flags[0] & interrupta::I_TXSENT != 0 {
            if self.pending_tx.is_none() {
                return Err(Error::UnexpectedAck);
            }
            self.trace().tx_sent_irq = true;
        }
        // I_RETRYFAIL: only for the pending transmission, never with TXSENT.
        if flags[0] & interrupta::I_RETRYFAIL != 0 {
            if self.trace().tx_sent_irq {
                return Err(Error::Rx(pd_service::Error::TransmissionFault));
            }
            let Some(tx) = self.pending_tx.as_mut() else {
                return Err(Error::Rx(pd_service::Error::TransmissionFault));
            };
            tx.retry_failed = true;
        }
        // I_SOFTRST: matched by a SOP Soft_Reset frame drained in this or an
        // adjacent poll, or by our own Soft_Reset transmission.
        self.polls = self.polls.wrapping_add(1);
        // BIST_TMODE also clears a received Soft_Reset from the FIFO: in BIST
        // Test Data Mode it is not acted on (section 6.4.3.1).
        if flags[0] & interrupta::I_SOFTRST != 0 && !self.bist_test_data {
            let own = self
                .pending_tx
                .as_ref()
                .is_some_and(PendingTx::reset_with_zero_id);
            if own
                || self
                    .softrst_frame_at
                    .take()
                    .is_some_and(|at| self.polls.wrapping_sub(at) <= 1)
            {
                self.softrst_flag_at = None;
            } else {
                self.softrst_flag_at = Some(self.polls);
            }
        }
        if matches!(outcome, pd_service::Outcome::Idle) && self.softrst_flag_at.take().is_some() {
            return Err(Error::Peer); // a Soft_Reset was signalled but never delivered
        }
        if let pd_service::Outcome::Packet(packet) = outcome {
            self.observation.rx_packets = self.observation.rx_packets.saturating_add(1);
            let frame = packet.frame();
            self.observation.rx_last_header = Some(frame.header());
            let cable_allowed = matches!(
                (frame.sop(), self.cable_phase),
                (Sop::Cable, CablePhase::Reset | CablePhase::Identity)
                    | (
                        Sop::CableDoublePrime,
                        CablePhase::SecondReset | CablePhase::SecondStatus
                    )
            );
            if (!cable_allowed && frame.sop() != Sop::Partner)
                || (cable_allowed && frame.header() & header::POWER_ROLE == 0)
            {
                return Err(Error::Peer);
            }
            if frame.header() & header::KIND_COUNT == 1 {
                let acknowledged = self.pending_tx.as_mut().is_some_and(|tx| {
                    tx.acknowledge(frame.sop(), frame.header(), || self.clock.now_ms())
                });
                if !acknowledged {
                    return Err(Error::UnexpectedAck);
                }
                // Earliest software observation of GoodCRC; do not move the
                // section 7.31.2 timer origin while waiting for I_TXSENT.
                self.trace().tx_goodcrc = true;
                if self.trace().request_id.is_some() && !self.trace().contract_seen {
                    self.step(7);
                }
                return Ok(None);
            }
            if frame.sop() == Sop::Partner
                && frame.header() & header::KIND_COUNT == 13
                && self.softrst_flag_at.take().is_none()
            {
                self.softrst_frame_at = Some(self.polls);
            }
            if let Ok(capabilities) = frame.advertisements() {
                self.observation.capabilities = Some(capabilities);
                self.step(5);
            }
            self.step(4);
            return Ok(Some(packet));
        }
        Ok(None)
    }
    fn enqueue(&mut self, packet: pd_service::Packet) -> Result<(), Error<B::Error>> {
        let Some(slot) = self.queue.iter_mut().find(|slot| slot.is_none()) else {
            return Err(Error::QueueFull);
        };
        *slot = Some(packet);
        Ok(())
    }
    fn dequeue(&mut self) -> Option<pd_service::Packet> {
        let packet = self.queue[0].take();
        self.queue[0] = self.queue[1].take();
        packet
    }
    /// Independent re-check of an outgoing Request against the parsed
    /// advertisement: exact fixed/PPS target and reviewed current, valid APDO
    /// range/type, and no dangerous or reserved RDO/APDO flags.
    fn request_is_reviewed(&self, rdo: u32) -> bool {
        let Some(capabilities) = self.observation.capabilities else {
            return false;
        };
        let target = self
            .config
            .selection
            .map(|selection| pd_protocol::pd_spr::Target {
                mv: selection.target_mv(),
                ma: selection.target_ma(),
                pps_object: selection.target_pps_object(),
            });
        pd_protocol::pd_spr::review_rdo::<P::Limits>(&capabilities, rdo, target)
    }
    async fn send(&mut self, bytes: &[u8]) -> Result<(), Error<B::Error>> {
        self.send_sop(bytes, Sop::Partner).await
    }
    /// A Deferred Message was never sent: keep the trace of the live contract.
    async fn send_sop(&mut self, bytes: &[u8], sop: Sop) -> Result<(), Error<B::Error>> {
        let before = *self.trace();
        let timers = (self.t_ack, self.t_accept);
        let result = self.send_sop_once(bytes, sop).await;
        if matches!(result, Err(Error::Deferred)) {
            (self.t_ack, self.t_accept) = timers;
            let trace = self.trace();
            trace.accept_seen = before.accept_seen;
            trace.psrdy_seen = before.psrdy_seen;
            trace.elapsed_ack_ms = before.elapsed_ack_ms;
            trace.elapsed_accept_ms = before.elapsed_accept_ms;
            trace.elapsed_psrdy_ms = before.elapsed_psrdy_ms;
            trace.pps = before.pps;
            trace.target_mv = before.target_mv;
            trace.target_ma = before.target_ma;
        }
        result
    }
    async fn send_sop_once(&mut self, bytes: &[u8], sop: Sop) -> Result<(), Error<B::Error>> {
        P::set_rx_idle(false);
        // A due Hard Reset is the only permitted next step; BIST Test Data
        // Mode permits no Message but the hardware GoodCRC (section 6.4.3.1).
        if self.hard_reset_armed || self.bist_test_data {
            return Err(Error::Request);
        }
        let rdo = first_data_object(bytes).filter(|_| bytes.len() == 6);
        let is_request = rdo.is_some_and(|rdo| self.request_is_reviewed(rdo));
        // The TX FIFO image below holds at most one 30-byte message.
        if bytes.len() < 2 || bytes.len() > 30 {
            return Err(Error::Request);
        }
        let header = u16::from_le_bytes([bytes[0], bytes[1]]);
        // Sink_Capabilities once per received Get_Sink_Cap: Data, Sink/UFP, the
        // negotiated revision and exactly the objects for it (section 6.4.1.2).
        let is_sink_caps = sop == Sop::Partner
            && self.sink_caps_reply
            && pd_protocol::pd_spr::sink_capabilities::<P::Limits>(self.tx_revision == 2)
                .is_some_and(|(objects, count)| {
                    bytes.len() == 2 + 4 * count
                        && header & header::WITHOUT_ID
                            == (count as u16) << 12 | u16::from(self.tx_revision) << 6 | 4
                        && bytes[2..]
                            .as_chunks::<4>()
                            .0
                            .iter()
                            .zip(&objects[..count])
                            .all(|(raw, object)| *raw == object.to_le_bytes())
                });
        // PD3 Sink_Capabilities_Extended: one Chunked Chunk 0 of the exact
        // 24-byte SKEDB padded with 00h to 7 Data Objects (Tables 6.48, 6.61).
        let skedb_reply = sop == Sop::Partner
            && self.skedb_reply
            && self.tx_revision == 2
            && bytes.len() == 30
            && header & header::WITHOUT_ID == 0xf08f
            && bytes[2..4] == [0x18, 0x80]
            && pd_protocol::pd_spr::sink_capabilities_extended::<P::Limits>()
                .is_some_and(|block| bytes[4..28] == block)
            && bytes[28..] == [0, 0];
        let status_reply = sop == Sop::Partner
            && self.tx_revision == 2
            && bytes.len() == 14
            && self.config.persistent
            && self.trace().contract_seen
            && self.request.is_none()
            && self.cable_phase == CablePhase::Idle
            && self.config.selection.is_none_or(|s| !s.request_in_flight())
            && header & header::WITHOUT_ID == 0xb082
            && bytes[2..4] == [7, 0x80]
            && bytes[11..] == [0, 0, 0]
            && self.status_reply.is_some_and(|block| {
                bytes[4..11] == block && pd_protocol::pd_spr::sink_status_valid(block)
            });
        let unsupported_reply =
            bytes.len() == 2 && self.unsupported_reply == Some(header & header::WITHOUT_ID);
        let revision_reply = sop == Sop::Partner
            && self.revision_reply
            && header & header::WITHOUT_ID == 0x100c | (u16::from(self.tx_revision) << 6)
            && rdo == Some(pd_protocol::pd_spr::REVISION_DATA_OBJECT);
        let reset_accept = sop == Sop::Partner
            && self.reset == Some(ResetStep::AcceptPartner)
            && bytes.len() == 2
            && header == 3 | (u16::from(self.tx_revision) << 6);
        // PE_SNK_Send_Soft_Reset: Control, MessageID 0, Sink/UFP, negotiated
        // revision, once per Protocol Error; never through SinkTxOK (section 7.1.1).
        let soft_reset = sop == Sop::Partner
            && self.reset == Some(ResetStep::SendSoftReset)
            && !self.recovering
            && bytes.len() == 2
            && header == 13 | (u16::from(self.tx_revision) << 6);
        // While a reset step is due it is the only permitted transmission, and
        // nothing is sent while capabilities are awaited after a reset.
        if self.reset.is_some() && !(soft_reset || reset_accept) {
            return Err(Error::Request);
        }
        // One Request at a time: none while its Accept/Reject/Wait is awaited
        // (PE_SNK_Select_Capability) or power transitions (PE_SNK_Transition_Sink).
        if is_request && self.request.is_some() {
            return Err(Error::Request);
        }
        let cable_optional = sop == Sop::Cable
            && self.cable_phase == CablePhase::Identity
            && (header >> 6) & 3 == 2
            && self.trace().cable_report.is_some_and(|r| {
                r.status == usbpd_traits::CableStatus::Identity && r.revision == 2
            })
            && (header & header::WITHOUT_ID_REVISION == 0x9006
                && bytes.len() == 6
                && bytes[2..] == [2, 0x80, 0, 0]
                || header & header::WITHOUT_ID_REVISION == 0x0012
                    && bytes.len() == 2
                    && self
                        .trace()
                        .cable_report
                        .is_some_and(|r| (r.objects[1] >> 27) & 7 == 4));
        let cable_second = sop == Sop::CableDoublePrime
            && self.trace().cable_report.is_some_and(|report| {
                report.revision == 2
                    && pd_protocol::pd_cable::second_controller(&report) == Some(true)
            })
            && bytes.len() == 2
            && match self.cable_phase {
                CablePhase::SecondReset => header == 0x008d,
                CablePhase::SecondStatus => header == 0x0292,
                _ => false,
            };
        // SOP' Soft_Reset before Discover Identity: Control, MessageID 0, PD3
        // (pd_discovery::synchronize; sections 6.1.3.2, 7.1.1, 7.32.3).
        let cable_reset =
            sop == Sop::Cable && self.cable_phase == CablePhase::Reset && bytes == [0x8d, 0];
        let cable_identity = cable_reset
            || cable_second
            || cable_optional
            || (sop == Sop::Cable
                && self.cable_phase == CablePhase::Identity
                && header & header::WITHOUT_ID_REVISION == 0x100f
                && rdo.is_some_and(|vdm| {
                    let command = vdm & 0x1f;
                    vdm & 0x87ff == 0x8000 | command
                        && matches!(vdm & 0x7800, 0 | 0x2000 | 0x2800)
                        && match command {
                            1 | 2 => vdm >> 16 == 0xff00,
                            3 => self
                                .config
                                .svids
                                .is_some_and(|store| store.contains((vdm >> 16) as u16)),
                            _ => false,
                        }
                }));
        if sop != Sop::Partner && !cable_identity {
            return Err(Error::Request);
        }
        // Get_PPS_Status (sections 6.3.20, 7.16): PD3 only, from Ready with an
        // established PPS contract that no Request or reset is changing.
        let pps_status_query = sop == Sop::Partner
            && bytes.len() == 2
            && header & header::WITHOUT_ID == pd_protocol::pd_spr::GET_PPS_STATUS_HEADER
            && self.tx_revision == 2
            && self.config.persistent
            && self.trace().contract_seen
            && !self.caps_reply
            && !self.reset_in_progress()
            && self.cable_phase == CablePhase::Idle
            && self.config.selection.is_some_and(|s| {
                s.target_pps_object() != 0
                    && !s.request_in_flight()
                    && s.established_target()
                        == Some(pd_protocol::pd_spr::Target {
                            mv: s.target_mv(),
                            ma: s.target_ma(),
                            pps_object: s.target_pps_object(),
                        })
            });
        // Information requests (`pd_partner`): exactly one of the three, PD3
        // only, from Ready with an established contract that no Request, reset
        // or cable inspection is changing, and no other request awaiting its answer.
        // An answer window that ended without an answer (SenderResponseTimer
        // timeout) stays recorded until the next packet: compare it with now.
        let now = self.clock.now_ms();
        if self.sink_alert_until.is_some_and(|until| now < until) && !soft_reset && !reset_accept {
            return Err(Error::Request); // PE_SNK_Wait_for_Get_Status is not Ready
        }
        let answer_window_open = self.query_until.is_some_and(|(until, _)| now < until)
            || self.pps_status_until.is_some_and(|until| now < until);
        let query_ready = sop == Sop::Partner
            && self.tx_revision == 2
            && self.config.persistent
            && self.trace().contract_seen
            && !self.caps_reply
            && !self.reset_in_progress()
            && self.cable_phase == CablePhase::Idle
            && !answer_window_open
            && self
                .config
                .selection
                .is_some_and(|s| !s.request_in_flight() && s.established_target().is_some());
        let query = query_ready
            .then(|| pd_protocol::pd_partner::query_request(bytes))
            .flatten();
        let sink_alert = query_ready
            && bytes.len() == 6
            && header & header::WITHOUT_ID == 0x1086
            && self
                .config
                .selection
                .is_some_and(|s| s.armed_sink_alert() == rdo)
            && rdo
                .is_some_and(|ado| ado != 0 && ado & !pd_protocol::pd_spr::SINK_ALERT_EVENTS == 0)
            && <P::Limits as pd_protocol::pd_spr::Limits>::status()
                .is_some_and(pd_protocol::pd_spr::sink_status_valid);
        // First Message of a Sink-initiated AMS (section 7.2): every cable
        // Message, Get_PPS_Status, an information request, and a Request from
        // Ready that answers no Source_Capabilities. Responses belong to the
        // Source's AMS and are never gated.
        let ams_start = sink_alert
            || cable_identity
            || pps_status_query
            || query.is_some()
            || (is_request
                && self.config.persistent
                && self.trace().contract_seen
                && !self.caps_reply);
        if !is_request
            && !unsupported_reply
            && !is_sink_caps
            && !cable_identity
            && !reset_accept
            && !revision_reply
            && !skedb_reply
            && !status_reply
            && !sink_alert
            && !pps_status_query
            && query.is_none()
            && !soft_reset
        {
            return Err(Error::Request);
        }
        let expected_type = if cable_identity {
            header & header::WITHOUT_ID_REVISION
        } else if is_request {
            0x1002
        } else if reset_accept {
            0x0003
        } else if soft_reset {
            0x000d
        } else if unsupported_reply {
            header & header::WITHOUT_ID_REVISION
        } else if revision_reply {
            0x100c
        } else if skedb_reply {
            0xf00f
        } else if pps_status_query {
            0x0014
        } else {
            // Sink_Capabilities, or an information request (`query_request`), checked whole above.
            header & header::WITHOUT_ID_REVISION
        };
        if header & header::WITHOUT_ID_REVISION != expected_type {
            return Err(Error::Request);
        }
        let revision = (header >> 6) & 3;
        if sop == Sop::Partner
            && self.trace().peer_revision.is_some()
            && revision as u8 != self.tx_revision
        {
            return Err(Error::Request);
        }
        if is_request
            && self
                .config
                .selection
                .is_some_and(|s| s.target_pps_object() != 0)
            && revision != 2
        {
            return Err(Error::Request);
        }
        // nRetryCount: 3 for PD 2.0 (revision 1), 2 for PD 3.x (revision 2).
        let retries = match revision {
            1 => control3::AUTO_RETRY | control3::n_retries(3),
            2 => control3::AUTO_RETRY | control3::n_retries(2),
            _ => return Err(Error::Request),
        };
        if is_request {
            self.trace().accept_seen = false;
            self.trace().psrdy_seen = false;
            self.trace().elapsed_ack_ms = None;
            self.trace().elapsed_accept_ms = None;
            self.trace().elapsed_psrdy_ms = None;
            self.t_ack = None;
            self.t_accept = None;
            self.trace().pps = self
                .config
                .selection
                .is_some_and(|s| s.target_pps_object() != 0);
            self.trace().target_mv = self
                .config
                .selection
                .map_or(P::Limits::TARGET_MV as u16, |selection| {
                    selection.target_mv()
                });
            self.trace().target_ma = self
                .config
                .selection
                .map_or(P::Limits::REQUEST_MA as u16, |selection| {
                    selection.target_ma()
                });
        }
        // Account fresh flags before TX; never consume stale success as ours.
        if let Some(packet) = self.poll().await? {
            let header = packet.frame().header();
            // Section 7.3, Table 7.3: a received SOP Message discards it.
            if ams_start && packet.frame().sop() == Sop::Partner {
                self.enqueue(packet)?;
                return self.defer(b'M');
            }
            // Partner traffic before a reset Message: uncertain, fail closed.
            if reset_accept || soft_reset {
                return Err(Error::Peer);
            }
            if cable_second && packet.frame().sop() != Sop::CableDoublePrime {
                return Err(Error::Peer);
            }
            if is_request
                && self
                    .config
                    .selection
                    .is_some_and(|s| s.target_pps_object() != 0)
                && header & header::OBJECT_COUNT != 0
                && header & header::MESSAGE_TYPE == 1
                && (header >> 6) & 3 != 2
            {
                return Err(Error::Request);
            }
            self.enqueue(packet)?;
        }
        // Fresh Source_Capabilities may have changed the indexed object/range.
        if is_request && !rdo.is_some_and(|rdo| self.request_is_reviewed(rdo)) {
            return Err(Error::Request);
        }
        if self.receiver.status()
            & (status1::TX_EMPTY | status1::TX_FULL | status1::OVRTEMP | status1::OCP)
            != status1::TX_EMPTY
            || ((self.cable_phase != CablePhase::Idle || reset_accept || soft_reset)
                && (self.receiver.status() & status1::FIFOS_AND_FAULTS != status1::IDLE
                    || self.queue.iter().any(Option::is_some)))
        {
            return Err(Error::Boundary {
                status: self.receiver.status(),
            });
        }
        self.checked(ConfigRegister::Control3, retries).await?;
        use tx_token::{EOP, JAM_CRC, PACKSYM, SOP1, SOP2, SOP3, TXOFF, TXON};
        let token_len = PACKSYM | (bytes.len() as u8);
        let mut fifo = [0u8; 40];
        fifo[0] = register::FIFOS;
        // SOP* ordered sets as Sync symbols (USB PD R3.2 §5.4).
        fifo[1..5].copy_from_slice(match sop {
            Sop::Cable => &[SOP1, SOP1, SOP3, SOP3],
            Sop::CableDoublePrime => &[SOP1, SOP3, SOP1, SOP3],
            _ => &[SOP1, SOP1, SOP1, SOP2],
        });
        fifo[5] = token_len;
        fifo[6..6 + bytes.len()].copy_from_slice(bytes);
        let end = 6 + bytes.len();
        let total_end = if matches!(sop, Sop::Cable | Sop::CableDoublePrime) {
            fifo[end..end + 3].copy_from_slice(&[JAM_CRC, EOP, TXON]);
            end + 3
        } else {
            fifo[end..end + 4].copy_from_slice(&[JAM_CRC, EOP, TXOFF, TXON]);
            end + 4
        };
        self.register = Some(register::FIFOS);
        let t_send = if ams_start {
            self.gated_send(&fifo[..total_end]).await?
        } else {
            self.trace().tx_written = true;
            let t_send = self.clock.now_ms();
            self.bus
                .write(snapshot::ADDRESS, &fifo[..total_end])
                .await
                .map_err(|source| Error::Bus {
                    address: snapshot::ADDRESS,
                    register: register::FIFOS,
                    source,
                })?;
            t_send
        };
        // No await since the write: nothing can have polled these yet.
        self.t_send = Some(t_send);
        if sop == Sop::Partner {
            self.tx_revision = revision as u8;
            // Only the first Request locks SOP revision (section 6.1.3.1).
            // A startup reset Accept uses a temporary revision, not a contract.
            if is_request || self.trace().peer_revision.is_some() {
                self.trace().peer_revision = Some(if revision == 2 { 3 } else { 2 });
            }
            self.trace().request_id = Some(PendingTx::header_id(header) as u8);
        }
        if is_request {
            self.caps_reply = false;
            self.step(6);
        }
        self.pending_tx = Some(PendingTx::new(sop, header));
        self.trace().tx_written = true;
        self.trace().tx_sent_irq = false;
        self.trace().tx_goodcrc = false;
        self.trace().tx_ack_goodcrc_only = false;
        let began = self.clock.now_ms();
        let mut wait_packets = 0u8;
        self.tx_wait_flags = 0;
        let mut wait_header = 0u16;
        for _ in 0..32 {
            if let Some(packet) = self.poll().await? {
                // Evidence for a timeout: partner Messages that arrived while ours waited.
                if packet.frame().sop() == Sop::Partner {
                    if wait_packets == 0 {
                        wait_header = packet.frame().header();
                    }
                    wait_packets = wait_packets.saturating_add(1);
                }
                self.enqueue(packet)?;
            }
            if self.pending_tx.as_ref().is_some_and(|tx| tx.retry_failed) {
                self.pending_tx = None;
                // A cable that does not acknowledge simply does not answer.
                if sop != Sop::Partner {
                    return Err(Error::TxTimeout);
                }
                // Section 7.1.1: a failed Soft_Reset or reset Accept needs Hard
                // Reset; a partner Message alongside the failure is uncertain.
                if !self.config.persistent {
                    return Err(Error::HardResetNeeded);
                }
                if reset_accept || soft_reset {
                    self.arm_hard_reset();
                    return Err(Error::NotAcknowledged);
                }
                if self.queue.iter().any(Option::is_some) {
                    return Err(Error::Peer);
                }
                self.protocol_error()?;
                return Err(Error::NotAcknowledged);
            }
            // Section 6.7.1: the GoodCRC with our MessageID is the acknowledgement;
            // the FUSB302's I_TXSENT is only its own hint. On tested hardware,
            // a Source that answers at once left I_TXSENT unset
            // for the whole window. At the window's end the matching GoodCRC frame,
            // no I_RETRYFAIL and an empty TX FIFO (nothing left to retry) also count.
            let window_over = self.clock.now_ms().saturating_sub(began) >= 15;
            let goodcrc_only = window_over
                && self.trace().tx_goodcrc
                && !self.trace().tx_sent_irq
                && self.tx_wait_flags & interrupta::I_RETRYFAIL == 0
                && self.receiver.status() & (status1::TX_EMPTY | status1::TX_FULL)
                    == status1::TX_EMPTY;
            if self.trace().tx_sent_irq && self.trace().tx_goodcrc || goodcrc_only {
                self.trace().tx_ack_goodcrc_only = goodcrc_only;
                if goodcrc_only {
                    self.trace().txsent_missing = self.trace().txsent_missing.saturating_add(1);
                }
                let Some(t_ack) = self
                    .pending_tx
                    .as_ref()
                    .and_then(PendingTx::acknowledged_at)
                else {
                    return Err(Error::UnexpectedAck);
                };
                self.trace().elapsed_ack_ms = Some(t_ack.saturating_sub(t_send).min(999) as u16);
                self.t_ack = Some(t_ack);
                self.pending_tx = None;
                if unsupported_reply {
                    self.unsupported_reply = None;
                }
                if revision_reply {
                    self.revision_reply = false;
                }
                if is_sink_caps {
                    self.sink_caps_reply = false;
                }
                if skedb_reply {
                    self.skedb_reply = false;
                }
                if status_reply {
                    self.status_reply = None;
                }
                if sink_alert {
                    self.sink_alert_until = Some(t_ack + QUERY_ANSWER_MS);
                }
                if pps_status_query {
                    self.pps_status_until = Some(t_ack + PPS_STATUS_ANSWER_MS);
                }
                if let Some(query) = query {
                    self.query_until = Some((t_ack + QUERY_ANSWER_MS, query));
                }
                if is_request {
                    self.requested_object = rdo.map(|rdo| (rdo >> 28) as u8);
                    self.trace().request_ma = rdo.map_or(0, |rdo| {
                        if self
                            .config
                            .selection
                            .is_some_and(|s| s.target_pps_object() != 0)
                        {
                            (rdo & 0x7f) as u16 * 50
                        } else {
                            ((rdo >> 10) & 0x3ff) as u16 * 10
                        }
                    });
                }
                if reset_accept {
                    self.reset = Some(ResetStep::WaitCapsAfterReset);
                    self.recovering = true;
                    self.trace().soft_resets_accepted =
                        self.trace().soft_resets_accepted.saturating_add(1);
                }
                if soft_reset {
                    self.reset = Some(ResetStep::AwaitAccept);
                    self.recovering = true;
                    self.trace().soft_resets_sent = self.trace().soft_resets_sent.saturating_add(1);
                    if let Some(at) = self.protocol_error_at.take() {
                        let ms = t_send.saturating_sub(at).min(255) as u8;
                        if ms > self.trace().soft_reset_ms {
                            self.trace().soft_reset_ms = ms;
                        }
                    }
                }
                if is_request {
                    self.request = Some(RequestStage::AwaitReply);
                    if let Some(selection) = self.config.selection {
                        selection.note_request_sent(t_ack);
                    }
                }
                return Ok(());
            }
            if window_over {
                break;
            }
            self.delay.delay_ms(1).await;
        }
        // No GoodCRC and no I_RETRYFAIL: keep what the PHY showed, for the PD page.
        let lost_to_partner = ams_start
            && wait_packets > 0
            && !self.trace().tx_goodcrc
            && self.tx_wait_flags & (interrupta::I_TXSENT | interrupta::I_RETRYFAIL) == 0
            && self.receiver.status() & (status1::TX_EMPTY | status1::TX_FULL) == status1::TX_EMPTY;
        self.trace().tx_timeout = Some(pd::TxTimeout {
            status1: self.receiver.status(),
            flags: self.tx_wait_flags,
            packets: wait_packets,
            header: wait_header,
            cause: if ams_start {
                self.trace().sink_tx_cause
            } else {
                0
            },
        });
        // Section 7.3: a Source Message received before ours was sent discards
        // ours. The FUSB302 GoodCRCs the received Message and drops the pending
        // AMS start without TXSENT or RETRYFAIL; with the TX FIFO empty
        // (STATUS1 TX_EMPTY set, TX_FULL clear) nothing of ours can still go
        // out, so it is Deferred (MessageID unused), never a fault. Any other
        // timeout stays uncertain and halts.
        if lost_to_partner {
            self.pending_tx = None;
            return self.defer(b'C');
        }
        Err(Error::TxTimeout)
    }
    /// A Soft Reset step is due or capabilities are awaited after one.
    fn reset_in_progress(&self) -> bool {
        self.reset.is_some()
    }
    /// Abandon the AMS in progress for a Soft Reset (sections 7.1.1, 9.2.5.2).
    fn begin_reset(&mut self) {
        self.contract_object = None;
        self.request = None;
        self.pps_status_until = None;
        self.query_until = None;
        self.status_reply = None;
        self.sink_alert_until = None;
        self.unsupported_reply = None;
        self.revision_reply = false;
        self.sink_caps_reply = false;
        self.skedb_reply = false;
        self.caps_reply = false;
        self.trace().accept_seen = false;
        self.trace().psrdy_seen = false;
    }
    /// A Protocol Error (Table 7.1, section 9.2.5.2.1): permit one Soft_Reset in
    /// a persistent session after the first Request fixed the revision. While
    /// recovering it means the Soft Reset did not correct the error.
    fn protocol_error(&mut self) -> Result<(), Error<B::Error>> {
        if !self.config.persistent || self.trace().peer_revision.is_none() {
            return Err(Error::Peer);
        }
        if self.recovering
            || matches!(
                self.reset,
                Some(
                    ResetStep::SendSoftReset
                        | ResetStep::AwaitAccept
                        | ResetStep::AcceptPartner
                        | ResetStep::WaitCapsAfterReset
                )
            )
        {
            self.arm_hard_reset();
            return Ok(());
        }
        self.begin_reset();
        self.reset = Some(ResetStep::SendSoftReset);
        self.protocol_error_at = Some(self.clock.now_ms());
        Ok(())
    }
    /// A mandatory Hard Reset case (`docs/SAFETY.md`) was detected:
    /// it is the only permitted next step.
    fn arm_hard_reset(&mut self) {
        self.hard_reset_armed = true;
        self.hard_reset_error_at = Some(self.clock.now_ms());
    }
    /// Record a session step (only with `trace_steps`); a new furthest step is
    /// published at once so it is on screen before a possible VBUS loss.
    fn step(&mut self, step: u8) {
        if !self.config.trace_steps || step <= self.observation.boot_step {
            return;
        }
        self.observation.boot_step = step;
        let since = self
            .steps_t0
            .map_or(0, |t0| self.clock.now_ms().saturating_sub(t0));
        self.observation.boot_step_ms = since.min(9999) as u16;
        if let Some(publish) = self.config.publish {
            let mut observation = self.observation;
            observation.restore_unverified = self.unverified;
            publish(observation);
        }
    }
    /// Cable breadcrumb (`cable_step`), published at once so the flash log
    /// holds the last one reached even if the tester stops right after it.
    fn cable_mark(&mut self, step: u8) {
        if !(self.config.trace_steps || self.config.trace_cable)
            || step <= self.observation.cable_step
        {
            return;
        }
        self.observation.cable_step = step;
        if let Some(publish) = self.config.publish {
            let mut observation = self.observation;
            observation.restore_unverified = self.unverified;
            publish(observation);
        }
    }
    /// `pd-boot-diagnostic` only, once, idle after the first contract
    /// (`docs/SAFETY.md`): STATUS0 with Rd on both CCs, then with Rd on the
    /// active CC only, then the original SWITCHES0,
    /// each write read back. Rd on the active CC never changes.
    async fn probe_rp(&mut self) -> Result<(), Error<B::Error>> {
        let (both, single) = match self.observation.selected_cc {
            Some(1) => (
                switches0::PDWN1 | switches0::PDWN2 | switches0::MEAS_CC1,
                switches0::PDWN1 | switches0::MEAS_CC1,
            ),
            Some(2) => (
                switches0::PDWN1 | switches0::PDWN2 | switches0::MEAS_CC2,
                switches0::PDWN2 | switches0::MEAS_CC2,
            ),
            _ => return Ok(()),
        };
        self.rp_probing = true;
        let with_both = self.byte(register::STATUS0).await?;
        self.checked(ConfigRegister::Switches0, single).await?;
        self.delay.delay_ms(2).await;
        let with_single = self.byte(register::STATUS0).await?;
        self.checked(ConfigRegister::Switches0, both).await?;
        self.observation.rp_probe = Some([with_both, with_single]);
        Ok(())
    }
    /// `cause`: b'G' SinkTxOK not seen in time, b'M' a SOP Message came first,
    /// b'C' a SOP Message arrived and ours was dropped unsent (section 7.3).
    /// (`gated_send` records b'Q' when it sent through the stale-SinkTxNG fallback.)
    fn defer<T>(&mut self, cause: u8) -> Result<T, Error<B::Error>> {
        self.trace().sink_tx_deferred = self.trace().sink_tx_deferred.saturating_add(1);
        self.trace().sink_tx_cause = cause;
        Err(Error::Deferred)
    }
    /// Sections 7.2/7.3: write the AMS start only through the atomic gate.
    /// While refused, keep servicing RX; a SOP Message discards ours. Returns
    /// the time of the accepted sample. Non-standard exception: a stale SinkTxNG
    /// window ends in one quiet-line send (`stale_sink_tx_ng`, owner decision).
    async fn gated_send(&mut self, fifo: &[u8]) -> Result<u64, Error<B::Error>> {
        // AMS starts exist only under an Explicit Contract; PD3 is code 2.
        let rp_required = self.tx_revision == 2;
        let began = self.clock.now_ms();
        // Every refusal so far was a quiet SinkTxNG (see `stale_sink_tx_ng`).
        let mut stale = rp_required;
        let mut fallback = false;
        loop {
            let sampled = self.clock.now_ms();
            self.register = Some(register::STATUS0);
            let need_rp = rp_required && !fallback;
            let (samples, sent) = self
                .bus
                .gated_write(
                    snapshot::ADDRESS,
                    [0x40, 0x41],
                    |status0, status1| sink_tx_allowed(status0, status1, need_rp),
                    fifo,
                )
                .await
                .map_err(|source| {
                    self.trace().tx_written = true; // the FIFO write may have started
                    Error::Bus {
                        address: snapshot::ADDRESS,
                        register: register::FIFOS,
                        source,
                    }
                })?;
            let now = self.clock.now_ms();
            if sent {
                self.register = Some(register::FIFOS);
                self.trace().sink_tx_cause = if fallback { b'Q' } else { 0 };
                let gate = now.saturating_sub(sampled).min(255) as u8;
                if gate > self.trace().sink_tx_gate_ms {
                    self.trace().sink_tx_gate_ms = gate;
                }
                return Ok(sampled);
            }
            self.trace().sink_tx_refused = Some(samples);
            stale &= P::ALLOW_STALE_SINK_TX_NG && stale_sink_tx_ng(samples[0], samples[1]);
            if fallback {
                return self.defer(b'G'); // activity or RX at the fallback sample
            }
            if now.saturating_sub(began) >= SINK_TX_WAIT_MS {
                if stale {
                    // NOT STANDARD (section 7.2), owner-approved: the whole window
                    // was a quiet SinkTxNG, see `stale_sink_tx_ng` for why.
                    fallback = true;
                    continue; // one immediate atomic re-sample, no Rp condition
                }
                return self.defer(b'G');
            }
            self.delay.delay_ms(1).await;
            if let Some(packet) = self.poll().await? {
                let partner = packet.frame().sop() == Sop::Partner;
                self.enqueue(packet)?;
                if partner {
                    return self.defer(b'M');
                }
            }
        }
    }
    /// tVCONNStable wait that keeps servicing RX: Port Partner traffic has
    /// priority over the cable AMS (section 7.3), so it ends the inspection.
    async fn vconn_settle(&mut self) -> Result<(), Error<B::Error>> {
        let began = self.clock.now_ms();
        while self.clock.now_ms().saturating_sub(began) < VCONN_SETTLE_MS {
            self.delay.delay_ms(1).await;
            if let Some(packet) = self.poll().await? {
                if packet.frame().sop() != Sop::Partner {
                    return Err(Error::Peer);
                }
                self.enqueue(packet)?;
                return self.defer(b'M');
            }
        }
        Ok(())
    }
    async fn receive_packet(&mut self, buffer: &mut [u8]) -> Result<usize, Error<B::Error>> {
        if self.hard_reset_armed {
            return Err(Error::Request); // the engine never waits before a due Hard Reset
        }
        loop {
            P::set_rx_idle(false);
            let packet = match self.dequeue() {
                Some(packet) => Some(packet),
                None => self.poll().await?,
            };
            if let Some(packet) = packet {
                // BIST Test Data Mode: a frame queued before BIST_TMODE took
                // effect got its GoodCRC and is not acted on (section 6.4.3.1).
                if self.bist_test_data {
                    continue;
                }
                let length = packet.bytes().len() - 5; // exclude SOP token and CRC
                if buffer.len() < length {
                    return Err(Error::Peer);
                }
                buffer[..length].copy_from_slice(&packet.bytes()[1..1 + length]);
                let header = packet.frame().header();
                // Normalize only the policy-engine copy, after CRC validation.
                // Packet/observation headers retain the actual wire bits.
                if header & header::SPEC_REVISION == 0 {
                    buffer[0] |= 0x40;
                }
                let revision = (buffer[0] >> 6) & 3;
                self.unsupported_reply = None;
                self.revision_reply = false;
                self.sink_caps_reply = false;
                self.skedb_reply = false;
                self.status_reply = None;
                self.bist_carrier_armed = false;
                let now = self.clock.now_ms();
                let revision_known = self.trace().peer_revision.is_some();
                let same_revision = revision == self.tx_revision;
                if header & header::KIND_COUNT == 13 {
                    // PE_SNK_Soft_Reset from any state (section 9.2.5.2.2):
                    // MessageID 0 (section 7.7), no pending TX, cable work or
                    // queued traffic. Startup Accept does not lock revision.
                    if !self.config.persistent
                        || !matches!(revision, 1 | 2)
                        || self.pending_tx.is_some()
                        || matches!(
                            self.reset,
                            Some(ResetStep::AcceptPartner | ResetStep::SendSoftReset)
                        )
                        || self.cable_phase != CablePhase::Idle
                        || header & header::MESSAGE_ID != 0
                        || (revision_known && !same_revision)
                        || self.queue.iter().any(Option::is_some)
                    {
                        return Err(Error::Peer);
                    }
                    if !revision_known {
                        self.tx_revision = revision;
                    }
                    self.begin_reset();
                    self.reset = Some(ResetStep::AcceptPartner);
                    return Ok(length);
                }
                if self.reset == Some(ResetStep::AwaitAccept) {
                    // Our Soft_Reset: Accept with MessageID 0. Another Message at
                    // this revision is a Protocol Error during the Soft Reset:
                    // Hard Reset (section 7.7). A malformed Accept fails closed.
                    if header & header::KIND_COUNT != 3 && same_revision {
                        self.arm_hard_reset();
                        return Ok(length);
                    }
                    if header & header::KIND_COUNT != 3
                        || header & header::MESSAGE_ID != 0
                        || !same_revision
                    {
                        return Err(Error::Peer);
                    }
                    self.reset = Some(ResetStep::WaitCapsAfterReset);
                    return Ok(length);
                }
                if self.reset == Some(ResetStep::WaitCapsAfterCarrier) {
                    // PE_SNK_Wait_for_Capabilities after BIST Carrier Mode: only fresh
                    // Source_Capabilities. No Shall Hard Reset covers anything else.
                    if header & header::KIND != 1
                        || header & header::OBJECT_COUNT == 0
                        || !same_revision
                    {
                        return Err(Error::Peer);
                    }
                    self.reset = None;
                } else if self.reset == Some(ResetStep::WaitCapsAfterReset) {
                    // Section 7.7: only fresh Source_Capabilities; another Message
                    // at this revision means Hard Reset.
                    if header & header::KIND != 1 && same_revision {
                        self.arm_hard_reset();
                        return Ok(length);
                    }
                    if header & header::KIND != 1
                        || header & header::OBJECT_COUNT == 0
                        || (revision_known && !same_revision)
                    {
                        return Err(Error::Peer);
                    }
                    self.reset = None;
                } else if self
                    .pps_status_until
                    .take()
                    .is_some_and(|until| now < until)
                {
                    // PE_SNK_Get_PPS_Status: only PD3 PPS_Status as one Chunk
                    // (Chunked, Chunk 0, no Chunk request, Data Size 4, two Data
                    // Objects; Tables 6.48, 6.58; Extended Header bit 9 Reserved)
                    // or Not_Supported. It is never answered. Another well-formed
                    // Message at this revision is a Protocol Error; a malformed
                    // PPS_Status fails closed.
                    let pps_status = header & header::KIND_COUNT == 0xa00c
                        && length == 10
                        && u16::from_le_bytes([buffer[2], buffer[3]]) & 0xfdff == 0x8004;
                    let not_supported = header & header::KIND_COUNT == 0x0010 && length == 2;
                    if !(pps_status || not_supported) || revision != 2 {
                        if header & header::KIND == 0x800c || !same_revision {
                            return Err(Error::Peer);
                        }
                        self.protocol_error()?;
                        return Ok(length);
                    }
                } else if let Some(query) = self
                    .query_until
                    .take()
                    .and_then(|(until, query)| (now < until).then_some(query))
                {
                    // PE_SNK_Get_Source_Cap_Ext / PE_Get_Revision /
                    // PE_Get_Manufacturer_Info: only the exact PD3 answer or
                    // Not_Supported (`pd_partner::query_answer`); it is never
                    // answered. Another well-formed Message at this revision is a
                    // Protocol Error; a malformed answer fails closed.
                    use pd_protocol::pd_partner::{Reply, query_answer};
                    match query_answer(query, &buffer[..length]) {
                        Reply::Answer | Reply::NotSupported => {}
                        Reply::Other if same_revision => {
                            self.protocol_error()?;
                            return Ok(length);
                        }
                        _ => return Err(Error::Peer),
                    }
                } else if self
                    .sink_alert_until
                    .take()
                    .is_some_and(|until| now < until)
                {
                    // Figure 9.27: only Get_Status in this bounded Alert AMS.
                    if length == 2 && header & header::KIND_COUNT == 18 && same_revision {
                        self.status_reply = <P::Limits as pd_protocol::pd_spr::Limits>::status()
                            .filter(|&block| pd_protocol::pd_spr::sink_status_valid(block));
                        if self.status_reply.is_none() {
                            return Err(Error::Peer);
                        }
                    } else if same_revision {
                        self.protocol_error()?;
                        return Ok(length);
                    } else {
                        return Err(Error::Peer);
                    }
                } else if self.request == Some(RequestStage::TransitionSink) {
                    // Table 7.1, section 9.2.4.6: only PS_RDY while power transitions;
                    // another Message at this revision is answered with Hard Reset.
                    if header & header::KIND_COUNT != 6 {
                        if !same_revision {
                            return Err(Error::Peer);
                        }
                        self.arm_hard_reset();
                        return Ok(length);
                    }
                    self.request = None;
                    self.recovering = false;
                    self.contract_object = self.requested_object;
                } else if self.request == Some(RequestStage::AwaitReply) {
                    // PE_SNK_Select_Capability: Accept, Reject or Wait; anything
                    // else is a Protocol Error (Figure 9.18 note 1).
                    self.request = match header & header::KIND_COUNT {
                        3 => Some(RequestStage::TransitionSink),
                        4 | 12 => {
                            self.recovering = false;
                            None
                        }
                        _ if same_revision => {
                            self.protocol_error()?;
                            return Ok(length);
                        }
                        _ => return Err(Error::Peer),
                    };
                } else if self.config.persistent
                    && self.trace().contract_seen
                    && self.reset != Some(ResetStep::AcceptPartner)
                    && self.config.selection.is_none_or(|s| !s.request_in_flight())
                {
                    use pd_protocol::pd_spr::{ReadyResponse, ready_response};
                    let extended = (header & header::EXTENDED != 0 && length >= 4)
                        .then(|| u16::from_le_bytes([buffer[2], buffer[3]]));
                    // PE_BIST_Test_Mode (section 9.2.26.4.2.1), only at vSafe5V: the
                    // FUSB302 BIST_TMODE keeps sending GoodCRC and clears the RX
                    // FIFO; Hard Reset restores CONTROL3 (restore_baseline).
                    use pd_protocol::pd_spr::{BistRequest, bist_request};
                    let bdo = first_data_object(buffer).filter(|_| length >= 6);
                    match bdo
                        .map(|bdo| bist_request(header, bdo, self.contract_object.unwrap_or(0)))
                    {
                        Some(BistRequest::TestData) => {
                            let control3 = self.byte(register::CONTROL3).await?;
                            self.checked(ConfigRegister::Control3, control3 | control3::BIST_TMODE)
                                .await?;
                            self.bist_test_data = true;
                            return Ok(length);
                        }
                        // PE_BIST_Carrier_Mode: the policy asks for one carrier next.
                        Some(BistRequest::CarrierMode) => {
                            self.bist_carrier_armed = true;
                            return Ok(length);
                        }
                        _ => {}
                    }
                    match ready_response(self.tx_revision, header, extended) {
                        ReadyResponse::Unsupported | ReadyResponse::UnsupportedAfterChunk => {
                            self.unsupported_reply = Some(
                                (u16::from(self.tx_revision) << 6)
                                    | if self.tx_revision == 1 { 4 } else { 16 },
                            );
                        }
                        // DR_Swap (section 7.12.1) and VCONN_Swap (section 7.13) get
                        // Reject. The VCONN Source Shall Not Reject VCONN_Swap: if VCONN
                        // were on here, fail closed rather than send a forbidden reply.
                        ReadyResponse::Reject => {
                            if header & header::MESSAGE_TYPE == 11 && self.trace().vconn_on {
                                return Err(Error::Peer);
                            }
                            self.unsupported_reply = Some((u16::from(self.tx_revision) << 6) | 4);
                        }
                        ReadyResponse::Revision => self.revision_reply = true,
                        ReadyResponse::SinkCap => self.sink_caps_reply = true,
                        ReadyResponse::SinkCapExtended => self.skedb_reply = true,
                        ReadyResponse::Status => {
                            if !same_revision {
                                return Err(Error::Peer);
                            }
                            self.status_reply =
                                <P::Limits as pd_protocol::pd_spr::Limits>::status()
                                    .filter(|&block| pd_protocol::pd_spr::sink_status_valid(block));
                            if self.status_reply.is_none() {
                                return Err(Error::Peer);
                            }
                        }
                        // Table 7.1: Unexpected in PE_SNK_Ready.
                        ReadyResponse::SoftReset => {
                            self.protocol_error()?;
                            return Ok(length);
                        }
                        _ => {}
                    }
                }
                if header & header::KIND == 1 && header & header::OBJECT_COUNT != 0 {
                    self.caps_reply = true;
                }
                if header & header::COUNT_TYPE == 3 {
                    self.step(8);
                    self.trace().accept_seen = true;
                    let t_accept = self.clock.now_ms();
                    if let Some(t_ack) = self.t_ack {
                        self.trace().elapsed_accept_ms =
                            Some(t_accept.saturating_sub(t_ack).min(999) as u16);
                    }
                    self.t_accept = Some(t_accept);
                }
                if header & header::COUNT_TYPE == 6 {
                    self.step(9);
                    self.trace().psrdy_seen = true;
                    if self.config.persistent {
                        self.trace().contract_seen = true;
                    }
                    let t_psrdy = self.clock.now_ms();
                    if let Some(t_accept) = self.t_accept {
                        self.trace().elapsed_psrdy_ms =
                            Some(t_psrdy.saturating_sub(t_accept).min(999) as u16);
                    }
                }
                return Ok(length);
            }
            if self.config.persistent
                && self.trace().contract_seen
                && !self.reset_in_progress()
                && self.config.selection.is_none_or(|s| !s.request_in_flight())
            {
                if self
                    .config
                    .stop_requested
                    .is_some_and(|requested| requested())
                {
                    let now = self.clock.now_ms();
                    let since = *self.stop_since_ms.get_or_insert(now);
                    if now.saturating_sub(since) >= 1000 {
                        return Err(Error::StopRequested);
                    }
                } else {
                    self.stop_since_ms = None;
                }
            }
            if let Some(selection) = self.config.selection {
                // Only this idle point is safe for the upstream Ready select to
                // cancel receive: no transaction/FIFO read/TX is in flight.
                self.lifecycle = Lifecycle::Running;
                selection.set_rx_idle(true);
                P::set_rx_idle(true);
                if selection.has_pending()
                    || selection.refresh_due()
                    || selection.pps_status_due()
                    || selection.query_due()
                    || selection.sink_alert_due()
                {
                    core::future::pending::<()>().await;
                }
                self.delay.wait_event().await;
                selection.set_rx_idle(false);
                P::set_rx_idle(false);
                self.lifecycle = Lifecycle::Halted;
            } else {
                self.delay.wait_event().await;
            }
        }
    }
    async fn cable_wait(
        &mut self,
        timeout_ms: u64,
    ) -> Result<Option<pd_service::Packet>, Error<B::Error>> {
        let began = self.clock.now_ms();
        for _ in 0..512 {
            let packet = match self.dequeue() {
                Some(packet) => Some(packet),
                None => self.poll().await?,
            };
            if self.clock.now_ms().saturating_sub(began) >= timeout_ms {
                return if packet.is_some() {
                    Err(Error::Peer)
                } else {
                    Ok(None)
                };
            }
            if let Some(packet) = packet {
                // Section 7.3: the Port Partner's AMS takes priority.
                if packet.frame().sop() == Sop::Partner {
                    self.enqueue(packet)?;
                    return self.defer(b'M');
                }
                return Ok(Some(packet));
            }
            self.delay.delay_ms(1).await;
        }
        Err(Error::Budget)
    }
    async fn cable_safe5(&mut self) -> Result<(), Error<B::Error>> {
        let raw = self.vbus_raw().await?;
        if !(4750..=5500).contains(&(P::voltage_mv(raw))) {
            return Err(Error::InputVoltage { raw });
        }
        Ok(())
    }
    async fn cable_operation(
        &mut self,
        next_id: u8,
        last_id: Option<u8>,
    ) -> Result<usbpd_traits::CableReport, Error<B::Error>> {
        use usbpd_traits::{CableReport, CableStatus};
        if self.cable_phase != CablePhase::Idle
            || next_id > 7
            || last_id.is_some_and(|id| id > 7)
            || !self.trace().contract_seen
            || self.trace().target_mv != 5000
            || self.trace().pps
            || !self.config.selection.is_some_and(|s| {
                s.cable_active() && s.target_mv() == 5000 && s.target_pps_object() == 0
            })
            || self.trace().peer_revision != Some(3)
            || self.pending_tx.is_some()
            || self.reset_in_progress()
            || self.queue.iter().any(Option::is_some)
        {
            return Err(Error::Request);
        }
        let original_switch = match self.observation.selected_cc {
            Some(1) => switches0::PDWN1 | switches0::PDWN2 | switches0::MEAS_CC1,
            Some(2) => switches0::PDWN1 | switches0::PDWN2 | switches0::MEAS_CC2,
            _ => return Err(Error::Orientation),
        };
        let mut report = CableReport {
            status: CableStatus::Timeout,
            objects: [0; 7],
            count: 0,
            revision: 0,
            next_tx_id: next_id,
            last_rx_id: last_id,
        };
        // Inspection topology is authorized by the board profile, not inferred.
        if self.observation.selected_cc != Some(P::INSPECTION_CC) {
            // Nothing was sent: no stale reason from an earlier inspection.
            self.trace().cable_modes.sync = pd_protocol::pd_cable_modes::Sync::NotSent;
            self.trace().cable_active = false;
            self.trace().cable_report = Some(report);
            return Ok(report);
        }
        self.cable_safe5().await?;
        if self.byte(register::SWITCHES0).await? != original_switch
            || self.byte(register::CONTROL1).await? != 0
        {
            return Err(Error::Request);
        }
        let requested_mode = self.config.selection.and_then(|s| s.cable_mode());
        let previous_report = self.trace().cable_report;
        pd_discovery::begin(self, requested_mode, previous_report)?;
        if requested_mode.is_none()
            && let Some(notify) = self.config.identity_started
        {
            notify();
        }
        self.trace().cable_active = true;
        self.trace().cable_report = None;
        self.cable_phase = CablePhase::Identity;
        // Breadcrumbs restart for every query; the count tells them apart.
        self.observation.cable_step = 0;
        self.cable_done_ms = None;
        self.observation.cable_queries = self.observation.cable_queries.saturating_add(1);
        self.cable_mark(10);

        // Apply the board-authorized inspection routing; no role negotiation is
        // added by this transport. Any direct-VCONN deviation belongs to its profile.
        self.checked(ConfigRegister::Switches0, P::INSPECTION_SWITCHES0)
            .await?;
        self.trace().vconn_on = true;
        // Wait 50 ms for cable controller to power up and stabilize (tVCONNStable)
        let inspected = match self.vconn_settle().await {
            Ok(()) => {
                self.checked(ConfigRegister::Control1, control1::ENSOP1)
                    .await?;
                pd_discovery::inspect(self, next_id, last_id, requested_mode, previous_report).await
            }
            Err(error) => Err(error),
        };
        self.cable_mark(27);
        report = match inspected {
            Ok(report) => report,
            // Nothing pending on the wire: end through the normal restore, with
            // no partial results kept, and let Ready receive the queued Message.
            Err(Error::Deferred) => {
                self.receiver.allow_cable_extended(false);
                self.trace().cable_modes = pd_protocol::pd_cable_modes::Discovery::EMPTY;
                if let Some(store) = self.config.svids {
                    store.clear();
                }
                CableReport {
                    status: CableStatus::Deferred,
                    ..report
                }
            }
            Err(error) => return Err(error),
        };

        self.checked(ConfigRegister::Control1, 0).await?;
        self.checked(ConfigRegister::Switches0, original_switch)
            .await?;
        self.trace().vconn_on = false;
        self.cable_phase = CablePhase::Idle;
        self.trace().cable_active = false;
        report.next_tx_id = next_id;
        report.last_rx_id = last_id;
        self.trace().cable_report = Some(report);
        self.cable_mark(28);
        if self.config.trace_steps || self.config.trace_cable {
            self.cable_done_ms = Some(self.clock.now_ms());
        }
        Ok(report)
    }
    pub async fn finish(
        &mut self,
        contract: &pd_protocol::pd_spr::Contract,
    ) -> Result<Snapshot, Error<B::Error>> {
        if self.stopped() {
            return Err(Error::Stopped);
        }
        P::set_rx_idle(false);
        self.lifecycle = Lifecycle::Finished;
        let established = self.config.persistent
            && self.trace().contract_seen
            && self
                .config
                .selection
                .is_some_and(|s| !s.request_in_flight() && s.established_target().is_some());
        if !self.request_is_reviewed(contract.request.0)
            || self.pending_tx.is_some()
            || self.reset_in_progress()
            || self.recovering
            || self.request.is_some()
            || !self.trace().tx_goodcrc
            || (!established && (!self.trace().accept_seen || !self.trace().psrdy_seen))
            || self.queue.iter().any(Option::is_some)
        {
            return Err(Error::Completion);
        }
        self.trace().contract_seen = true;
        // Let the final automatic GoodCRC finish before normal shutdown. This
        // is a nominal quiet interval, not electrical waveform qualification.
        self.delay.delay_ms(1).await;
        let raw_rdy = self.vbus_raw().await?;
        self.trace().vbus_rdy_mv = Some((P::voltage_mv(raw_rdy)) as u16);
        let status = self.byte(register::STATUS1).await?;
        if status & status1::FIFOS_AND_FAULTS != status1::IDLE {
            return Err(Error::Boundary { status });
        }
        let after = self.restore_baseline().await?;
        let raw_post = self.vbus_raw().await?;
        self.trace().vbus_post_mv = Some((P::voltage_mv(raw_post)) as u16);
        let measure = self.byte(register::MEASURE).await?;
        if measure != P::MEASURE {
            return Err(Error::Config(pd_cc::Error::Verify {
                register: 4,
                value: measure,
            }));
        }
        self.unverified = false;
        self.trace().tx_off_verified = true;
        Ok(after)
    }
}
impl<'a, B: GatedBus, D: Waiter, C: Clock, P: Platform> Session<'a, B, D, C, P> {
    /// The power-on register baseline the boot path checks: AUTO_CRC and the
    /// receiver off, MEAS_CC cleared. SWITCHES0 keeps both PDWN bits (Rd).
    async fn restore_baseline(&mut self) -> Result<Snapshot, Error<B::Error>> {
        self.checked(ConfigRegister::Switches1, switches1::SPECREV_2_0)
            .await?;
        self.checked(ConfigRegister::Power, power::BANDGAP_WAKE)
            .await?;
        self.checked(
            ConfigRegister::Switches0,
            switches0::PDWN1 | switches0::PDWN2,
        )
        .await?;
        self.checked(ConfigRegister::Control3, control3::n_retries(3))
            .await?;
        self.register = None; // snapshot cancellation has unknown in-flight address
        let after = snapshot::snapshot(self.bus).await.map_err(|f| {
            self.register = Some(f.register);
            Error::Bus {
                address: snapshot::ADDRESS,
                register: f.register,
                source: f.source,
            }
        })?;
        for (i, expected) in P::BASELINE.into_iter().enumerate() {
            if after.values[i] != expected {
                return Err(Error::Config(pd_cc::Error::Verify {
                    register: snapshot::REGISTERS[i],
                    value: after.values[i],
                }));
            }
        }
        Ok(after)
    }
    /// PE_SNK_Hard_Reset (section 9.2.4.8), only in a mandatory case
    /// (`docs/SAFETY.md`) re-checked here: one armed by a Protocol
    /// Error, or a response timer past its minimum (SenderResponseTimer after the
    /// GoodCRC of our Request or Soft_Reset, PSTransitionTimer after Accept).
    /// Never for SinkWaitCapTimer, never through the SinkTxOK gate (section 7.1.3).
    async fn send_hard_reset(&mut self) -> Result<(), Error<B::Error>> {
        P::set_rx_idle(false);
        let now = self.clock.now_ms();
        let since = |at: Option<u64>, min: u64| at.is_some_and(|at| now.saturating_sub(at) >= min);
        let timed_out = ((self.request == Some(RequestStage::AwaitReply)
            || self.reset == Some(ResetStep::AwaitAccept))
            && since(self.t_ack, SENDER_RESPONSE_MIN_MS))
            || (self.request == Some(RequestStage::TransitionSink)
                && since(self.t_accept, PS_TRANSITION_MIN_MS));
        let startup_reset = matches!(
            self.reset,
            Some(ResetStep::AcceptPartner | ResetStep::WaitCapsAfterReset)
        );
        if !self.config.persistent
            || (self.trace().peer_revision.is_none() && !startup_reset)
            || self.pending_tx.is_some()
            || self.cable_phase != CablePhase::Idle
            || !(self.hard_reset_armed || timed_out)
        {
            return Err(Error::Request);
        }
        if let Some(at) = self.hard_reset_error_at {
            self.trace().hard_reset_ms = now.saturating_sub(at).min(255) as u8;
        }
        self.trace().hard_resets_sent = self.trace().hard_resets_sent.saturating_add(1);
        // Only seen if the tester stays powered through the VBUS drop.
        if let Some(publish) = self.config.publish {
            let mut observation = self.observation;
            observation.restore_unverified = self.unverified;
            publish(observation);
        }
        let control3 = self.byte(register::CONTROL3).await?;
        // SEND_HARD_RESET clears itself, so this write cannot be read back.
        self.unverified = true;
        self.hard_reset_sent = true; // before the await: it may have gone out
        self.register = Some(register::CONTROL3);
        self.bus
            .write(
                snapshot::ADDRESS,
                &[register::CONTROL3, control3 | control3::SEND_HARD_RESET],
            )
            .await
            .map_err(|source| Error::Bus {
                address: snapshot::ADDRESS,
                register: register::CONTROL3,
                source,
            })?;
        // Section 7.31.10.1: without I_HARDSENT within tHardResetComplete, the
        // Hard Reset counts as sent.
        let began = self.clock.now_ms();
        loop {
            let flags = self.byte(register::INTERRUPTA).await?; // read-clear
            self.observation.rx_flags[0] |= flags;
            if flags & interrupta::I_HARDRST != 0 {
                self.hard_reset_received = true; // the Source's crossed ours
            }
            if flags & interrupta::I_HARDSENT != 0 {
                self.trace().hard_reset_confirmed = true;
                return Ok(());
            }
            if self.clock.now_ms().saturating_sub(began) >= HARD_RESET_COMPLETE_MS {
                return Ok(());
            }
            self.delay.delay_ms(1).await;
        }
    }
    /// PE_SNK_Transition_to_default (section 9.2.4.9) after Hard Reset Signaling
    /// was sent or received: reset the FUSB302 PD logic (section 7.1.3, the
    /// protocol layer), restore the boot baseline without touching Rd, then see
    /// VBUS fall to vSafe0V and return to vSafe5V (section 4.4.1). The caller then
    /// resets the MCU, so the next session starts like a power-up at 5 V. On VBUS
    /// power the tester loses power during the wait instead.
    /// BIST Carrier Mode (sections 5.4.1, 9.2.26.4.1.1), once per classified
    /// request: CONTROL1 BIST_MODE2 then CONTROL0 TX_START (as Chromium EC's
    /// FUSB302 driver), BIST_CARRIER_MS, BIST_MODE2 cleared (checked). Then
    /// PE_SNK_Transition_to_default without Hard Reset: PD_RESET, read-clear the
    /// flags (I_HARDRST or a fault ends here), a fresh receiver, VBUS still at
    /// vSafe5V, and only fresh Source_Capabilities may follow.
    async fn send_bist_carrier(&mut self) -> Result<(), Error<B::Error>> {
        if !core::mem::take(&mut self.bist_carrier_armed)
            || self.pending_tx.is_some()
            || self.reset_in_progress()
            || self.cable_phase != CablePhase::Idle
            || self.queue.iter().any(Option::is_some)
            || self.contract_object != Some(1)
            || self.bist_test_data
        {
            return Err(Error::Request);
        }
        let control1 = self.byte(register::CONTROL1).await?;
        self.checked(ConfigRegister::Control1, control1 | control1::BIST_MODE2)
            .await?;
        let control0 = self.byte(register::CONTROL0).await?;
        self.register = Some(register::CONTROL0);
        self.bus
            .write(
                snapshot::ADDRESS,
                &[register::CONTROL0, control0 | control0::TX_START],
            )
            .await
            .map_err(|source| Error::Bus {
                address: snapshot::ADDRESS,
                register: register::CONTROL0,
                source,
            })?;
        self.delay.delay_ms(BIST_CARRIER_MS).await;
        self.checked(ConfigRegister::Control1, control1 & !control1::BIST_MODE2)
            .await?;
        self.trace().bist_carriers = self.trace().bist_carriers.saturating_add(1);
        self.register = Some(register::RESET);
        self.bus
            .write(snapshot::ADDRESS, &[register::RESET, reset::PD_RESET])
            .await
            .map_err(|source| Error::Bus {
                address: snapshot::ADDRESS,
                register: register::RESET,
                source,
            })?;
        let mut flags = [0; 3];
        for (flag, register) in flags.iter_mut().zip(register::INTERRUPTS) {
            *flag = self.byte(register).await?;
        }
        if flags[0] & interrupta::I_HARDRST != 0 {
            self.hard_reset_received = true;
            return Err(Error::Rx(pd_service::Error::HardReset));
        }
        // I_OCP_TEMP; TX completion flags from the carrier are not documented.
        if flags[0] & interrupta::I_OCP_TEMP != 0 {
            return Err(Error::Rx(pd_service::Error::PhyFault));
        }
        self.receiver = pd_service::Receiver::for_spr_boundary();
        if !(4500..=5500).contains(&self.vbus_mv().await?) {
            return Err(Error::BistCarrierVbus);
        }
        self.begin_reset();
        self.recovering = false;
        self.reset = Some(ResetStep::WaitCapsAfterCarrier);
        Ok(())
    }
    pub async fn transition_to_default(&mut self) -> Result<(), Error<B::Error>> {
        if self.lifecycle == Lifecycle::Finished
            || !(self.hard_reset_sent || self.hard_reset_received)
        {
            return Err(Error::Stopped);
        }
        P::set_rx_idle(false);
        self.lifecycle = Lifecycle::Finished;
        self.unverified = true;
        self.register = Some(register::RESET);
        self.bus
            .write(snapshot::ADDRESS, &[register::RESET, reset::PD_RESET])
            .await
            .map_err(|source| Error::Bus {
                address: snapshot::ADDRESS,
                register: register::RESET,
                source,
            })?;
        self.restore_baseline().await?;
        let began = self.clock.now_ms();
        while self.vbus_mv().await? >= VBUS_SAFE0_MV {
            if self.clock.now_ms().saturating_sub(began) >= VBUS_OFF_MS {
                return Err(Error::HardResetVbus);
            }
            self.delay.delay_ms(5).await;
        }
        let off = self.clock.now_ms();
        while !(4500..=5500).contains(&self.vbus_mv().await?) {
            if self.clock.now_ms().saturating_sub(off) >= VBUS_ON_MS {
                return Err(Error::HardResetVbus);
            }
            self.delay.delay_ms(5).await;
        }
        // Read-clear the latched flags so the next start sees a clean boundary.
        // Only the Hard Reset's own (I_HARDRST, I_HARDSENT) and TOGDONE may be set.
        let mut flags = [0; 3];
        for (i, register) in register::INTERRUPTS.into_iter().enumerate() {
            flags[i] = self.byte(register).await?;
            self.observation.rx_flags[i] |= flags[i];
        }
        if flags[0] & !(interrupta::I_TOGDONE | interrupta::I_HARDSENT | interrupta::I_HARDRST) != 0
            || flags[1] & interruptb::I_GCRCSENT != 0
            || flags[2] & interrupt::I_COLLISION != 0
        {
            return Err(Error::Rx(pd_service::Error::TransmissionFault));
        }
        self.unverified = false;
        Ok(())
    }
    /// Physical voltage observations and conversion are supplied by the board.
    async fn vbus_raw(&mut self) -> Result<u16, Error<B::Error>> {
        self.register = Some(P::VBUS_REGISTER);
        self.trace().failed_address = P::VBUS_ADDRESS;
        P::vbus_raw(self.bus).await
    }
    async fn vbus_mv(&mut self) -> Result<u32, Error<B::Error>> {
        let raw = self.vbus_raw().await?;
        Ok(P::voltage_mv(raw))
    }
}
impl pd_discovery::Packet for pd_service::Packet {
    fn frame(&self) -> pd_protocol::pd_rx::Frame<'_> {
        pd_service::Packet::frame(self)
    }
}
/// The first 32-bit Data Object, which follows the 16-bit Message Header
/// (sections 6.2.1.1, 6.4), if the message is long enough to hold one.
fn first_data_object(message: &[u8]) -> Option<u32> {
    let &[_, _, a, b, c, d, ..] = message else {
        return None;
    };
    Some(u32::from_le_bytes([a, b, c, d]))
}
/// `pd-boot-diagnostic` breadcrumbs for an acknowledged cable request (odd
/// steps 11-25; its reply is the next even step). 10 = VCONN on, 27 = the
/// inspection returned, 28 = VCONN off and restored, 30/31 = Ready after it.
fn cable_step(bytes: &[u8], sop: Sop) -> u8 {
    let header = u16::from_le_bytes([bytes[0], bytes[1]]);
    let command = bytes.get(2).copied().unwrap_or(0) & 0x1f;
    match (sop, header & header::KIND) {
        (Sop::Cable, 0x000d) => 11,                 // SOP' Soft_Reset
        (Sop::Cable, 0x000f) if command == 1 => 13, // Discover Identity
        (Sop::Cable, 0x000f) if command == 2 => 15, // Discover SVIDs
        (Sop::Cable, 0x000f) => 17,                 // Discover Modes
        (Sop::Cable, 0x8006) => 19,                 // Get_Manufacturer_Info
        (Sop::Cable, 0x0012) => 21,                 // Get_Status
        (Sop::CableDoublePrime, 0x000d) => 23,      // SOP'' Soft_Reset
        (Sop::CableDoublePrime, _) => 25,           // SOP'' Get_Status
        _ => 0,
    }
}
impl<B: GatedBus, D: Waiter, C: Clock, P: Platform> pd_discovery::Transport
    for Session<'_, B, D, C, P>
{
    const RESPONSE_TIMEOUT_MS: u64 = P::CABLE_RESPONSE_TIMEOUT_MS;
    const SVID_NO_PROGRESS_TIMEOUT_MS: u64 = P::CABLE_SVID_NO_PROGRESS_TIMEOUT_MS;
    type Error = Error<B::Error>;
    type Packet = pd_service::Packet;
    fn fault(fault: pd_discovery::Fault) -> Self::Error {
        match fault {
            pd_discovery::Fault::Peer => Error::Peer,
            pd_discovery::Fault::Request => Error::Request,
        }
    }
    fn is_tx_timeout(error: &Self::Error) -> bool {
        matches!(error, Error::TxTimeout)
    }
    fn now_ms(&self) -> u64 {
        self.clock.now_ms()
    }
    fn supports_second(&self) -> bool {
        true
    } // owner-authorized scope on this board
    async fn send(&mut self, bytes: &[u8], sop: Sop) -> Result<(), Self::Error> {
        self.send_sop(bytes, sop).await?;
        self.cable_mark(cable_step(bytes, sop));
        Ok(())
    }
    async fn wait(&mut self, timeout_ms: u64) -> Result<Option<Self::Packet>, Self::Error> {
        let packet = self.cable_wait(timeout_ms).await?;
        // A reply to the request just acknowledged: its step plus one.
        let sent = self.observation.cable_step;
        if packet.is_some() && (11..=25).contains(&sent) && sent % 2 == 1 {
            self.cable_mark(sent + 1);
        }
        Ok(packet)
    }
    fn discovery(&mut self) -> &mut pd_protocol::pd_cable_modes::Discovery {
        &mut self.trace().cable_modes
    }
    fn store(&self) -> Option<&dyn pd_protocol::pd_svids::Store> {
        self.config.svids
    }
    fn receive_id(&self) -> u8 {
        self.cable_rx_id
    }
    fn set_receive_id(&mut self, id: u8) {
        self.cable_rx_id = id;
    }
    fn phase(&self) -> CablePhase {
        self.cable_phase
    }
    fn set_phase(&mut self, phase: CablePhase) {
        self.cable_phase = phase;
    }
    fn allow_extended(&mut self, enabled: bool) {
        self.receiver.allow_cable_extended(enabled);
    }
    fn set_report(&mut self, report: Option<usbpd_traits::CableReport>) {
        self.trace().cable_report = report;
    }
    async fn recheck_power(&mut self) -> Result<(), Self::Error> {
        self.cable_safe5().await
    }
    async fn select_second(&mut self) -> Result<(), Self::Error> {
        self.checked(ConfigRegister::Control1, control1::ENSOP2)
            .await
    }
}
impl<B: GatedBus, D: Waiter, C: Clock, P: Platform> Driver for Session<'_, B, D, C, P> {
    const HAS_AUTO_GOOD_CRC: bool = true;
    const HAS_AUTO_RETRY: bool = true;
    fn sender_response_elapsed_ms(&self) -> u64 {
        self.t_ack
            .map_or(0, |at| self.clock.now_ms().saturating_sub(at))
    }
    async fn wait_for_vbus(&mut self) {
        // A fresh Trial cannot reuse this PHY owner's counters/configuration.
        // Return to the guarded receiver, which reports Discarded without I/O.
        if self.protocol_started {
            self.lifecycle = Lifecycle::Halted;
            self.error = Some(Error::Stopped);
            return;
        }
        self.protocol_started = true;
        // External voltage/CC/baseline gates already ran before enabling RX/TX.
        if self.stopped() {
            self.error = Some(Error::Stopped);
            core::future::pending::<()>().await;
        }
    }
    async fn discover_cable(
        &mut self,
        next_tx_id: u8,
        last_rx_id: Option<u8>,
    ) -> Result<usbpd_traits::CableReport, DriverRxError> {
        if self.stopped() {
            return Err(DriverRxError::Discarded);
        }
        self.lifecycle = Lifecycle::Halted;
        match self.cable_operation(next_tx_id, last_rx_id).await {
            Ok(report) => {
                self.lifecycle = Lifecycle::Running;
                Ok(report)
            }
            Err(error) => {
                self.error = Some(error);
                Err(DriverRxError::Discarded)
            }
        }
    }
    async fn transmit_bist_carrier(&mut self) -> Result<(), DriverTxError> {
        if self.stopped() {
            return Err(DriverTxError::Discarded);
        }
        self.lifecycle = Lifecycle::Halted;
        match self.send_bist_carrier().await {
            Ok(()) => {
                self.lifecycle = Lifecycle::Running;
                Ok(())
            }
            Err(Error::Rx(pd_service::Error::HardReset)) => {
                self.error = Some(Error::Rx(pd_service::Error::HardReset));
                Err(DriverTxError::HardReset)
            }
            Err(error) => {
                self.error = Some(error);
                Err(DriverTxError::Discarded)
            }
        }
    }
    async fn transmit_hard_reset(&mut self) -> Result<(), DriverTxError> {
        if self.stopped() {
            return Err(DriverTxError::Discarded);
        }
        // The session ends here either way; `transition_to_default` follows.
        self.lifecycle = Lifecycle::Halted;
        match self.send_hard_reset().await {
            Ok(()) => Ok(()),
            Err(error) => {
                self.error = Some(error);
                Err(DriverTxError::Discarded)
            }
        }
    }
    async fn transmit(&mut self, bytes: &[u8]) -> Result<(), DriverTxError> {
        if self.stopped() {
            return Err(DriverTxError::Discarded);
        }
        self.lifecycle = Lifecycle::Halted;
        match self.send(bytes).await {
            Ok(()) => {
                self.lifecycle = Lifecycle::Running;
                Ok(())
            }
            Err(Error::Deferred) => {
                self.lifecycle = Lifecycle::Running;
                Err(DriverTxError::Deferred)
            }
            Err(Error::NotAcknowledged) => {
                self.lifecycle = Lifecycle::Running;
                Err(DriverTxError::NotAcknowledged)
            }
            Err(error) => {
                let hard_reset = matches!(error, Error::Rx(pd_service::Error::HardReset));
                self.error = Some(error);
                Err(if hard_reset {
                    DriverTxError::HardReset
                } else {
                    DriverTxError::Discarded
                })
            }
        }
    }
    async fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, DriverRxError> {
        if self.stopped() {
            return Err(DriverRxError::Discarded);
        }
        self.lifecycle = Lifecycle::Halted;
        match self.receive_packet(buffer).await {
            Ok(length) => {
                self.lifecycle = Lifecycle::Running;
                Ok(length)
            }
            Err(error) => {
                if matches!(error, Error::StopRequested) {
                    self.lifecycle = Lifecycle::Running;
                }
                let hard_reset = matches!(error, Error::Rx(pd_service::Error::HardReset));
                self.error = Some(error);
                Err(if hard_reset {
                    DriverRxError::HardReset
                } else {
                    DriverRxError::Discarded
                })
            }
        }
    }
}
