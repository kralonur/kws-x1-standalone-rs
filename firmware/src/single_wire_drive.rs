//! Hardware adapter for SCP, FCP and AFC detection and explicit output
//! control.
//! Table numbers refer to Appendix B of the SCP draft standard cited in
//! `single-wire-protocols`.

use core::convert::Infallible;
use core::ops::RangeInclusive;
use cortex_m::peripheral::{DWT, SYST};
use qc_protocol::qc2::{Handshake, Level, Progress};
use single_wire_protocols::{
    Transport, afc,
    control::{ControlError, RequestError},
    fcp,
};

use crate::{
    line_adc::{DmInput, LineAdc, LineVoltages},
    qc_drive::{DataDrive, WireDp},
};
mod status;
pub use status::{Adapter, DEFAULT_VBUS_MV, Failure, Output, Protocol, Status};

/// Time in ms both lines stay released before entry. Application choice,
/// well past the charger's 20 ms minimum glitch filter (CHY100
/// TGLITCH(V)CHANGE), so the charger sees the lines idle.
pub(crate) const RELEASE_MS: u64 = 100;
/// Time in ms after release for VBUS to reach the default window before the
/// probe gives up. Application choice.
const SETTLE_MS: u64 = 1_000;
/// Wait in ms after raising D+: the 40–80 ms `Tvdp_src_on` (Table B.4).
const DP_SETTLE_MS: u64 = 80;
/// D+ detection-source level in mV, `VDP_SRC_SCP` (Table B.2).
const SCP_DP_MV: RangeInclusive<u16> = 2_600..=3_000;
/// D+ level in mV AFC keeps after entry: BC1.2 `VDP_SRC`, 0.5–0.7 V (USB
/// Battery Charging 1.2, Table 5-2).
const AFC_DP_MV: RangeInclusive<u16> = 500..=700;
/// Highest idle-low D− in mV, the terminal `VIL` (Table B.2).
const IDLE_MAX_MV: u16 = 600;
/// Application bounds in ms: verify a new setpoint within 2 s, and refresh
/// protocol communication every 500 ms, ahead of the adapter's 2 s watchdog.
const OUTPUT_SETTLE_MS: u64 = 2_000;
const REFRESH_MS: u64 = 500;
/// Longest stop in ms before the released lines are given up without a
/// default VBUS reading: the adapter's 2 s watchdog (single-wire-protocols
/// `scp::control`), then `OUTPUT_SETTLE_MS` for the output to fall. Application
/// bound, so a lost VBUS read cannot hold the lines or a PD restart.
const STOP_TIMEOUT_MS: u64 = 2_000 + OUTPUT_SETTLE_MS;

/// An output request: mV and, for SCP, the current limit in mA (0 for FCP
/// and AFC).
pub type Request = (u16, u16);

/// D+ while D− carries `protocol`'s data, and the window it must measure
/// in: SCP and FCP raise it to the detection source, AFC keeps 0.6 V.
fn data_dp(protocol: Protocol) -> (WireDp, RangeInclusive<u16>) {
    match protocol {
        Protocol::Scp | Protocol::Fcp => (WireDp::Scp, SCP_DP_MV),
        Protocol::Afc => (WireDp::Mid, AFC_DP_MV),
    }
}

/// A protocol's identification, one transaction per poll.
enum Detection {
    Scp(single_wire_protocols::scp::Detection),
    Fcp(fcp::Detection),
    Afc(afc::Detection),
}

impl Detection {
    fn new(protocol: Protocol) -> Self {
        match protocol {
            Protocol::Scp => Self::Scp(single_wire_protocols::scp::Detection::new()),
            Protocol::Fcp => Self::Fcp(fcp::Detection::new()),
            Protocol::Afc => Self::Afc(afc::Detection::new()),
        }
    }

    /// The adapter answered, so a later timeout is not silence.
    fn answered(&self) -> bool {
        match self {
            Self::Scp(detection) => detection.answered(),
            Self::Fcp(detection) => detection.answered(),
            Self::Afc(detection) => detection.answered(),
        }
    }

    fn poll(
        &mut self,
        phy: &mut Phy<'_>,
    ) -> Result<Option<Adapter>, single_wire_protocols::Error<Infallible>> {
        Ok(match self {
            Self::Scp(detection) => detection.poll(phy)?.map(Adapter::Scp),
            Self::Fcp(detection) => detection.poll(phy)?.map(Adapter::Fcp),
            Self::Afc(detection) => detection.poll(phy)?.map(Adapter::Afc),
        })
    }
}

/// A protocol's output session.
enum Session {
    Scp(single_wire_protocols::scp::control::Session),
    Fcp(fcp::Session),
    Afc(afc::Session),
}

impl Session {
    fn start(adapter: Adapter, (mv, ma): Request) -> Result<Self, RequestError> {
        match adapter {
            Adapter::Scp(adapter) => firmware_core::scp::request(adapter, mv, ma)
                .and_then(|output| {
                    single_wire_protocols::scp::control::Session::start(adapter, output)
                })
                .map(Self::Scp),
            Adapter::Fcp(adapter) => fcp::Output::new(adapter, mv)
                .and_then(|output| fcp::Session::start(adapter, output))
                .map(Self::Fcp),
            Adapter::Afc(adapter) => afc::Voltage::from_millivolts(mv)
                .map(|voltage| Self::Afc(afc::Session::start(adapter, voltage)))
                .ok_or(RequestError::Range),
        }
    }

    fn output(&self) -> Output {
        match self {
            Self::Scp(session) => Output::Scp(session.output()),
            Self::Fcp(session) => Output::Fcp(session.output()),
            Self::Afc(session) => Output::Afc(session.output()),
        }
    }

    fn busy(&self) -> bool {
        match self {
            Self::Scp(session) => session.busy(),
            Self::Fcp(session) => session.busy(),
            Self::Afc(session) => session.busy(),
        }
    }

    fn set(&mut self, (mv, ma): Request) -> Result<(), RequestError> {
        match self {
            Self::Scp(session) => session
                .request(mv, ma)
                .and_then(|output| session.set(output)),
            Self::Fcp(session) => session.request(mv).and_then(|output| session.set(output)),
            Self::Afc(session) => afc::Voltage::from_millivolts(mv)
                .ok_or(RequestError::Range)
                .and_then(|voltage| session.set(voltage)),
        }
    }

    /// No AFC status/watchdog refresh is implemented; VBUS is still checked.
    fn refresh(&mut self) {
        match self {
            Self::Scp(session) => session.refresh(),
            Self::Fcp(session) => session.refresh(),
            Self::Afc(_) => {}
        }
    }

    fn stop(&mut self) {
        match self {
            Self::Scp(session) => session.stop(),
            Self::Fcp(session) => session.stop(),
            Self::Afc(session) => session.stop(),
        }
    }

    fn poll(&mut self, phy: &mut Phy<'_>) -> Result<(), ControlError<Infallible>> {
        match self {
            Self::Scp(session) => session.poll(phy),
            Self::Fcp(session) => session.poll(phy),
            Self::Afc(session) => session.poll(phy),
        }
    }
}

/// The request `output` answers, to compare with the request in force.
fn requested(output: Output) -> Request {
    (output.millivolts(), output.milliamps().unwrap_or(0))
}

/// Whether `request` is inside the application's power-path envelope.
fn allowed((mv, ma): Request) -> bool {
    (firmware_core::single_wire::MIN_MV..=firmware_core::single_wire::MAX_MV).contains(&mv)
        && ma <= firmware_core::single_wire::MAX_MA
}

/// SCP, FCP or AFC over the shared entry, PHY and checks: one copy of
/// this logic, the protocol chosen at run time.
pub struct Trigger {
    protocol: Protocol,
    state: State,
}
enum State {
    Releasing(u64),
    Entering(Handshake),
    Promoting(u64),
    /// Identification and ratings, one register read per step.
    Detecting(Detection),
    Checking(Adapter),
    Detected(Adapter),
    Output {
        session: Session,
        /// First completed setpoint request; retries keep this deadline.
        /// `None` while the initial request is being written.
        written_ms: Option<u64>,
        /// AFC voltage-mismatch retries; never resets `written_ms`.
        afc_retry: Option<firmware_core::afc::OutputRetry>,
        refresh_ms: u64,
        applied: bool,
    },
    /// The stop's transactions run, one per step.
    Closing(Session),
    Stopping(u64, Option<Failure>),
    Failed(Failure),
}

impl Trigger {
    pub fn start(protocol: Protocol, now_ms: u64, drive: &mut DataDrive) -> Self {
        drive.set(None, None);
        Self {
            protocol,
            state: State::Releasing(now_ms),
        }
    }

    pub fn protocol(&self) -> Protocol {
        self.protocol
    }

    pub fn update(
        &mut self,
        drive: &mut DataDrive,
        adc: &mut LineAdc,
        now_ms: u64,
        lines: Option<LineVoltages>,
        vbus_mv: Option<u16>,
        request: Option<Request>,
    ) -> Status {
        let at_default = vbus_mv.is_some_and(|mv| DEFAULT_VBUS_MV.contains(&mv));
        let (dp, dp_mv) = data_dp(self.protocol);
        let valid_lines =
            lines.is_some_and(|lines| dp_mv.contains(&lines.dp_mv) && lines.dm_mv <= IDLE_MAX_MV);
        // A failed AFC request may already have raised VBUS. This AFC path
        // configures no watchdog to undo an ambiguous exchange.
        let afc_output =
            self.protocol == Protocol::Afc && matches!(self.state, State::Output { .. });
        let state = core::mem::replace(&mut self.state, State::Failed(Failure::Vbus));
        self.state = match state {
            State::Releasing(since) if now_ms.saturating_sub(since) < RELEASE_MS => {
                State::Releasing(since)
            }
            State::Releasing(_) if at_default => {
                drive.set(Some(Level::Mid), None);
                State::Entering(Handshake::start(now_ms))
            }
            State::Releasing(since) if now_ms.saturating_sub(since) < SETTLE_MS => {
                State::Releasing(since)
            }
            State::Releasing(_) => State::Failed(Failure::Vbus),
            State::Entering(handshake) => match lines {
                None => State::Failed(Failure::DriveLevel),
                Some(lines) => match handshake.poll(now_ms, lines.dm_mv) {
                    Progress::Waiting(handshake) => State::Entering(handshake),
                    Progress::Failed(_) => State::Failed(Failure::Entry),
                    Progress::Ready(_) => {
                        drive.wire(dp, None);
                        State::Promoting(now_ms)
                    }
                },
            },
            State::Promoting(since) if now_ms.saturating_sub(since) < DP_SETTLE_MS => {
                State::Promoting(since)
            }
            State::Promoting(_) if !at_default => State::Failed(Failure::Vbus),
            State::Promoting(_) if !valid_lines => State::Failed(Failure::DriveLevel),
            State::Promoting(_) => State::Detecting(Detection::new(self.protocol)),
            State::Detecting(mut detection) => {
                let result = match Phy::new(drive, adc.dm_input(), dp) {
                    Some(mut phy) => detection.poll(&mut phy).map_err(|error| match error {
                        single_wire_protocols::Error::Timeout if !detection.answered() => {
                            Failure::Silent
                        }
                        error => Failure::Wire(error),
                    }),
                    None => Err(Failure::Clock),
                };
                match result {
                    Ok(None) => State::Detecting(detection),
                    Ok(Some(adapter)) => {
                        // Automatic detection remains read-only. An explicit
                        // output request keeps D+ asserted into the session.
                        if request.is_none() {
                            drive.set(None, None);
                        }
                        State::Checking(adapter)
                    }
                    Err(error) => State::Failed(error),
                }
            }
            State::Checking(adapter) if at_default => match request {
                None => State::Detected(adapter),
                // Only queued here: the transactions run one per step.
                Some(request) => match Session::start(adapter, request) {
                    Ok(session) => State::Output {
                        session,
                        written_ms: None,
                        afc_retry: None,
                        refresh_ms: now_ms,
                        applied: false,
                    },
                    Err(error) => State::Failed(Failure::Control(ControlError::Request(error))),
                },
            },
            State::Checking(_) => State::Failed(Failure::Vbus),
            State::Detected(_) if request.is_some() => {
                drive.set(None, None);
                State::Releasing(now_ms)
            }
            State::Detected(adapter) => State::Detected(adapter),
            State::Output {
                mut session,
                mut written_ms,
                mut afc_retry,
                mut refresh_ms,
                mut applied,
            } => {
                let result = (|| {
                    let vbus = vbus_mv.ok_or(Failure::Vbus)?;
                    if vbus > firmware_core::single_wire::MAX_MV {
                        return Err(Failure::Vbus);
                    }
                    let request = request.ok_or(Failure::Vbus)?;
                    // Revalidate against freshly read ratings, not cached UI evidence.
                    if !allowed(request) {
                        return Err(Failure::Vbus);
                    }
                    let target = session.output();
                    // A changed request waits for the running procedure.
                    if !session.busy() && requested(target) != request {
                        session
                            .set(request)
                            .map_err(|error| Failure::Control(ControlError::Request(error)))?;
                        written_ms = None;
                        afc_retry = None;
                        applied = false;
                    }
                    if !session.busy() {
                        if let Some(written) = written_ms {
                            let followed =
                                firmware_core::single_wire::followed(target.millivolts(), vbus);
                            if (applied && !followed)
                                || (!followed && now_ms.saturating_sub(written) >= OUTPUT_SETTLE_MS)
                            {
                                return Err(Failure::Vbus);
                            }
                            applied |= followed;
                            if let (Some(retry), Output::Afc(voltage)) = (&mut afc_retry, target)
                                && retry.retry(
                                    now_ms.saturating_sub(refresh_ms),
                                    voltage,
                                    Some(vbus),
                                )
                            {
                                // Re-send the same setpoint without extending
                                // its original voltage-verification deadline.
                                session.set(request).map_err(|error| {
                                    Failure::Control(ControlError::Request(error))
                                })?;
                            }
                        }
                        if !session.busy() {
                            if now_ms.saturating_sub(refresh_ms) < REFRESH_MS {
                                return Ok(());
                            }
                            session.refresh();
                        }
                    }
                    // One transaction per step; any traffic refreshes the
                    // watchdog.
                    communicate(drive, adc, dp, |phy| session.poll(phy))?;
                    refresh_ms = now_ms;
                    if !session.busy() && written_ms.is_none() {
                        written_ms = Some(now_ms);
                        if self.protocol == Protocol::Afc {
                            afc_retry = Some(firmware_core::afc::OutputRetry::default());
                        }
                    }
                    Ok(())
                })();
                match result {
                    Ok(()) => State::Output {
                        session,
                        written_ms,
                        afc_retry,
                        refresh_ms,
                        applied,
                    },
                    Err(error) => State::Failed(error),
                }
            }
            State::Closing(session) => State::Closing(session),
            State::Stopping(since, error) => State::Stopping(since, error),
            State::Failed(error) => State::Failed(error),
        };
        match self.state {
            State::Releasing(_) | State::Entering(_) | State::Promoting(_) => Status::Entering,
            State::Detecting(_) | State::Checking(_) => Status::Reading,
            State::Detected(adapter) => Status::Detected(adapter),
            State::Output {
                ref session,
                applied,
                ..
            } => Status::Output {
                output: session.output(),
                applied,
            },
            State::Closing(_) | State::Stopping(_, _) => Status::Stopping,
            State::Failed(error) => {
                if afc_output {
                    // Best effort: retain the original failure if reset also
                    // fails. Shutdown still measures VBUS before handoff.
                    let _ = reset_afc(drive, adc);
                }
                drive.set(None, None);
                Status::Failed(error)
            }
        }
    }

    /// A stop in progress, which must finish before this run can restart.
    pub fn stopping(&self) -> bool {
        matches!(self.state, State::Closing(_) | State::Stopping(..))
    }

    /// Work a faster step rate finishes sooner: entry, detection, a
    /// setpoint being written or not yet followed, a refresh, or a stop.
    pub fn busy(&self) -> bool {
        match &self.state {
            State::Detected(_) | State::Failed(_) => false,
            State::Output {
                session, applied, ..
            } => session.busy() || !applied,
            _ => true,
        }
    }

    /// On an orderly stop, run the protocol's stop (SCP leaves protocol
    /// output mode, FCP sets its lowest voltage, AFC sends its reset pulse),
    /// then release D+. SCP errors use D+ protection and its watchdog;
    /// AFC output errors get a best-effort reset, not another setpoint.
    /// Do not hand the lines to QC until measured VBUS is back at default,
    /// or `STOP_TIMEOUT_MS` has passed.
    pub fn stop(
        &mut self,
        now_ms: u64,
        drive: &mut DataDrive,
        adc: &mut LineAdc,
        vbus_mv: Option<u16>,
    ) -> Status {
        let state = core::mem::replace(&mut self.state, State::Failed(Failure::Vbus));
        let (since, error) = match state {
            State::Stopping(since, error) => (since, error),
            State::Output {
                session: Session::Afc(_),
                ..
            } => {
                // The reference driver's leave path uses a 100-UI reset,
                // independent of whether the last setpoint was acknowledged.
                let error = reset_afc(drive, adc).err();
                return self.settle(now_ms, error, now_ms, drive, vbus_mv);
            }
            State::Output { mut session, .. } => {
                // Queued work is replaced: a half-written setpoint ends here.
                session.stop();
                return self.close(session, now_ms, drive, adc, vbus_mv);
            }
            State::Closing(session) => return self.close(session, now_ms, drive, adc, vbus_mv),
            State::Failed(error) => (now_ms, Some(error)),
            _ => (now_ms, None),
        };
        self.settle(since, error, now_ms, drive, vbus_mv)
    }

    /// Run one stop transaction; release once they are all done or one fails.
    fn close(
        &mut self,
        mut session: Session,
        now_ms: u64,
        drive: &mut DataDrive,
        adc: &mut LineAdc,
        vbus_mv: Option<u16>,
    ) -> Status {
        let (dp, _) = data_dp(self.protocol);
        match communicate(drive, adc, dp, |phy| session.poll(phy)) {
            Ok(()) if session.busy() => {
                self.state = State::Closing(session);
                Status::Stopping
            }
            result => self.settle(now_ms, result.err(), now_ms, drive, vbus_mv),
        }
    }

    /// Release the lines and wait for default VBUS, at most
    /// `STOP_TIMEOUT_MS` from `since`.
    fn settle(
        &mut self,
        since: u64,
        error: Option<Failure>,
        now_ms: u64,
        drive: &mut DataDrive,
        vbus_mv: Option<u16>,
    ) -> Status {
        drive.set(None, None);
        self.state = State::Stopping(since, error);
        let settled = vbus_mv.is_some_and(|mv| DEFAULT_VBUS_MV.contains(&mv)) && now_ms > since;
        if settled || now_ms.saturating_sub(since) >= STOP_TIMEOUT_MS {
            return Status::Stopped;
        }
        error.map_or(Status::Stopping, Status::Failed)
    }
}

/// Samsung's `afc_reset`: a 100-UI D− high pulse, not a register write.
fn reset_afc(drive: &mut DataDrive, adc: &mut LineAdc) -> Result<(), Failure> {
    communicate(drive, adc, WireDp::Mid, |phy| {
        afc::reset(phy).map_err(ControlError::Wire)
    })
}

fn communicate<R>(
    drive: &mut DataDrive,
    adc: &mut LineAdc,
    dp: WireDp,
    operation: impl FnOnce(&mut Phy<'_>) -> Result<R, ControlError<Infallible>>,
) -> Result<R, Failure> {
    let mut phy = Phy::new(drive, adc.dm_input(), dp).ok_or(Failure::Clock)?;
    operation(&mut phy).map_err(Failure::Control)
}

struct Phy<'a> {
    drive: &'a mut DataDrive,
    dp: WireDp,
    receiver: DmInput<'a>,
    start: u32,
    cycles_per_us: u32,
}

/// Start the DWT cycle counter and return HCLK cycles per µs, or `None`
/// when that clock cannot time microseconds.
pub fn cycle_clock() -> Option<u32> {
    // The HAL's SysTick reload is HCLK cycles per one millisecond tick.
    const MICROS_PER_TICK: u32 = 1_000;
    let period = SYST::get_reload() + 1;
    let cycles_per_us = period / MICROS_PER_TICK;
    if cycles_per_us == 0 || !period.is_multiple_of(MICROS_PER_TICK) {
        return None;
    }
    // SAFETY: only enable the shared read-only cycle clock; never reset it.
    // The energy-store adapter also enables/reads DWT without resetting.
    let mut core = unsafe { cortex_m::Peripherals::steal() };
    core.DCB.enable_trace();
    core.DWT.enable_cycle_counter();
    let start = DWT::cycle_count();
    cortex_m::asm::delay(cycles_per_us);
    if DWT::cycle_count() == start {
        return None;
    }
    Some(cycles_per_us)
}

impl<'a> Phy<'a> {
    fn new(drive: &'a mut DataDrive, receiver: DmInput<'a>, dp: WireDp) -> Option<Self> {
        let cycles_per_us = cycle_clock()?;
        let start = DWT::cycle_count();
        Some(Self {
            drive,
            dp,
            receiver,
            start,
            cycles_per_us,
        })
    }
}

impl Transport for Phy<'_> {
    type Error = Infallible;
    fn drive(&mut self, high: Option<bool>) -> Result<(), Infallible> {
        self.drive.wire(self.dp, high);
        Ok(())
    }
    fn sample(&mut self) -> Result<bool, Infallible> {
        Ok(self.receiver.high())
    }
    fn now_us(&self) -> u32 {
        // A bounded detection finishes well before one 32-bit counter wrap.
        DWT::cycle_count().wrapping_sub(self.start) / self.cycles_per_us
    }
    fn delay_us(&mut self, us: u32) {
        let start = DWT::cycle_count();
        while DWT::cycle_count().wrapping_sub(start) < us * self.cycles_per_us {
            core::hint::spin_loop();
        }
    }
}
