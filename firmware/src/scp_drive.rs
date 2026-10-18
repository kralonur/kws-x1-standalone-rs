//! Hardware adapter for SCP and FCP detection and explicit output control.
//! Table numbers refer to Appendix B of the SCP draft standard cited in
//! `scp-protocol`.

use core::convert::Infallible;
use cortex_m::peripheral::{DWT, SYST};
use qc_protocol::qc2::{Handshake, Level, Progress};
use scp_protocol::{
    Transport,
    control::{ControlError, RequestError},
    fcp,
};

use crate::{
    line_adc::{DmInput, LineAdc, LineVoltages},
    qc_drive::DataDrive,
};
mod status;
pub use status::{DEFAULT_VBUS_MV, Failure, FcpStatus, Status};

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
const DP_MIN_MV: u16 = 2_600;
const DP_MAX_MV: u16 = 3_000;
/// Highest idle-low D− in mV, the terminal `VIL` (Table B.2).
const IDLE_MAX_MV: u16 = 600;
/// Application bounds in ms: verify a new setpoint within 2 s, and refresh
/// protocol communication every 500 ms, ahead of the adapter's 2 s watchdog.
const OUTPUT_SETTLE_MS: u64 = 2_000;
const REFRESH_MS: u64 = 500;
/// Longest stop in ms before the released lines are given up without a
/// default VBUS reading: the adapter's 2 s watchdog (scp-protocol
/// `control`), then `OUTPUT_SETTLE_MS` for the output to fall. Application
/// bound, so a lost VBUS read cannot hold the lines or a PD restart.
const STOP_TIMEOUT_MS: u64 = 2_000 + OUTPUT_SETTLE_MS;

/// What runs over the single-wire PHY once entry is done: SCP or FCP,
/// which share the entry, the PHY and the trigger's checks.
pub trait Link {
    type Detection;
    type Adapter: Copy;
    type Output: Copy;
    type Session;
    /// An output request: SCP's (mV, mA), FCP's mV.
    type Request: Copy + PartialEq;
    fn detection() -> Self::Detection;
    /// The adapter answered a read, so a later timeout is not silence.
    fn answered(detection: &Self::Detection) -> bool;
    fn detect<T: Transport>(
        detection: &mut Self::Detection,
        phy: &mut T,
    ) -> Result<Option<Self::Adapter>, scp_protocol::Error<T::Error>>;
    /// Whether `request` is inside the application's power-path envelope.
    fn allowed(request: Self::Request) -> bool;
    fn start(adapter: Self::Adapter, request: Self::Request)
    -> Result<Self::Session, RequestError>;
    fn output(session: &Self::Session) -> Self::Output;
    fn requested(output: Self::Output) -> Self::Request;
    fn millivolts(output: Self::Output) -> u16;
    fn busy(session: &Self::Session) -> bool;
    fn set(session: &mut Self::Session, request: Self::Request) -> Result<(), RequestError>;
    fn refresh(session: &mut Self::Session);
    fn stop(session: &mut Self::Session);
    fn poll<T: Transport>(
        session: &mut Self::Session,
        phy: &mut T,
    ) -> Result<(), ControlError<T::Error>>;
}

pub enum Scp {}

impl Link for Scp {
    type Detection = scp_protocol::Detection;
    type Adapter = scp_protocol::Adapter;
    type Output = scp_protocol::control::Output;
    type Session = scp_protocol::control::Session;
    type Request = (u16, u16);
    fn detection() -> Self::Detection {
        scp_protocol::Detection::new()
    }
    fn answered(detection: &Self::Detection) -> bool {
        detection.answered()
    }
    fn detect<T: Transport>(
        detection: &mut Self::Detection,
        phy: &mut T,
    ) -> Result<Option<Self::Adapter>, scp_protocol::Error<T::Error>> {
        detection.poll(phy)
    }
    fn allowed((mv, ma): Self::Request) -> bool {
        (firmware_core::scp::MIN_MV..=firmware_core::scp::MAX_MV).contains(&mv)
            && ma <= firmware_core::scp::MAX_MA
    }
    fn start(
        adapter: Self::Adapter,
        (mv, ma): Self::Request,
    ) -> Result<Self::Session, RequestError> {
        firmware_core::scp::request(adapter, mv, ma)
            .and_then(|output| Self::Session::start(adapter, output))
    }
    fn output(session: &Self::Session) -> Self::Output {
        session.output()
    }
    fn requested(output: Self::Output) -> Self::Request {
        (output.millivolts(), output.milliamps())
    }
    fn millivolts(output: Self::Output) -> u16 {
        output.millivolts()
    }
    fn busy(session: &Self::Session) -> bool {
        session.busy()
    }
    fn set(session: &mut Self::Session, (mv, ma): Self::Request) -> Result<(), RequestError> {
        session
            .request(mv, ma)
            .and_then(|output| session.set(output))
    }
    fn refresh(session: &mut Self::Session) {
        session.refresh();
    }
    fn stop(session: &mut Self::Session) {
        session.stop();
    }
    fn poll<T: Transport>(
        session: &mut Self::Session,
        phy: &mut T,
    ) -> Result<(), ControlError<T::Error>> {
        session.poll(phy)
    }
}

pub enum Fcp {}

impl Link for Fcp {
    type Detection = fcp::Detection;
    type Adapter = fcp::Adapter;
    type Output = fcp::Output;
    type Session = fcp::Session;
    type Request = u16;
    fn detection() -> Self::Detection {
        fcp::Detection::new()
    }
    fn answered(detection: &Self::Detection) -> bool {
        detection.answered()
    }
    fn detect<T: Transport>(
        detection: &mut Self::Detection,
        phy: &mut T,
    ) -> Result<Option<Self::Adapter>, scp_protocol::Error<T::Error>> {
        detection.poll(phy)
    }
    fn allowed(mv: Self::Request) -> bool {
        (firmware_core::scp::MIN_MV..=firmware_core::scp::MAX_MV).contains(&mv)
    }
    fn start(adapter: Self::Adapter, mv: Self::Request) -> Result<Self::Session, RequestError> {
        fcp::Output::new(adapter, mv).and_then(|output| Self::Session::start(adapter, output))
    }
    fn output(session: &Self::Session) -> Self::Output {
        session.output()
    }
    fn requested(output: Self::Output) -> Self::Request {
        output.millivolts()
    }
    fn millivolts(output: Self::Output) -> u16 {
        output.millivolts()
    }
    fn busy(session: &Self::Session) -> bool {
        session.busy()
    }
    fn set(session: &mut Self::Session, mv: Self::Request) -> Result<(), RequestError> {
        session.request(mv).and_then(|output| session.set(output))
    }
    fn refresh(session: &mut Self::Session) {
        session.refresh();
    }
    fn stop(session: &mut Self::Session) {
        session.stop();
    }
    fn poll<T: Transport>(
        session: &mut Self::Session,
        phy: &mut T,
    ) -> Result<(), ControlError<T::Error>> {
        session.poll(phy)
    }
}

pub struct Trigger<L: Link> {
    state: State<L>,
}
enum State<L: Link> {
    Releasing(u64),
    Entering(Handshake),
    Promoting(u64),
    /// Identification and ratings, one register read per step.
    Detecting(L::Detection),
    Checking(L::Adapter),
    Detected(L::Adapter),
    Output {
        session: L::Session,
        /// When the setpoint finished writing; `None` while it is written.
        written_ms: Option<u64>,
        refresh_ms: u64,
        applied: bool,
    },
    /// The stop's transactions run, one per step.
    Closing(L::Session),
    Stopping(u64, Option<Failure>),
    Failed(Failure),
}

impl<L: Link> Trigger<L> {
    pub fn start(now_ms: u64, drive: &mut DataDrive) -> Self {
        drive.set(None, None);
        Self {
            state: State::Releasing(now_ms),
        }
    }

    pub fn update(
        &mut self,
        drive: &mut DataDrive,
        adc: &mut LineAdc,
        now_ms: u64,
        lines: Option<LineVoltages>,
        vbus_mv: Option<u16>,
        request: Option<L::Request>,
    ) -> Status<L::Adapter, L::Output> {
        let at_default = vbus_mv.is_some_and(|mv| DEFAULT_VBUS_MV.contains(&mv));
        let valid_lines = lines.is_some_and(|lines| {
            (DP_MIN_MV..=DP_MAX_MV).contains(&lines.dp_mv) && lines.dm_mv <= IDLE_MAX_MV
        });
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
                        drive.scp(None);
                        State::Promoting(now_ms)
                    }
                },
            },
            State::Promoting(since) if now_ms.saturating_sub(since) < DP_SETTLE_MS => {
                State::Promoting(since)
            }
            State::Promoting(_) if !at_default => State::Failed(Failure::Vbus),
            State::Promoting(_) if !valid_lines => State::Failed(Failure::DriveLevel),
            State::Promoting(_) => State::Detecting(L::detection()),
            State::Detecting(mut detection) => {
                let result = match Phy::new(drive, adc.dm_input()) {
                    Some(mut phy) => {
                        L::detect(&mut detection, &mut phy).map_err(|error| match error {
                            scp_protocol::Error::Timeout if !L::answered(&detection) => {
                                Failure::Silent
                            }
                            error => Failure::Wire(error),
                        })
                    }
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
                Some(request) => match L::start(adapter, request) {
                    Ok(session) => State::Output {
                        session,
                        written_ms: None,
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
                mut refresh_ms,
                mut applied,
            } => {
                let result = (|| {
                    let vbus = vbus_mv.ok_or(Failure::Vbus)?;
                    if vbus > firmware_core::scp::MAX_MV {
                        return Err(Failure::Vbus);
                    }
                    let request = request.ok_or(Failure::Vbus)?;
                    // Revalidate against freshly read ratings, not cached UI evidence.
                    if !L::allowed(request) {
                        return Err(Failure::Vbus);
                    }
                    let target = L::output(&session);
                    // A changed request waits for the running procedure.
                    if !L::busy(&session) && L::requested(target) != request {
                        L::set(&mut session, request)
                            .map_err(|error| Failure::Control(ControlError::Request(error)))?;
                        written_ms = None;
                        applied = false;
                    }
                    if !L::busy(&session) {
                        if let Some(written) = written_ms {
                            let followed =
                                firmware_core::scp::followed(L::millivolts(target), vbus);
                            if (applied && !followed)
                                || (!followed && now_ms.saturating_sub(written) >= OUTPUT_SETTLE_MS)
                            {
                                return Err(Failure::Vbus);
                            }
                            applied |= followed;
                        }
                        if now_ms.saturating_sub(refresh_ms) < REFRESH_MS {
                            return Ok(());
                        }
                        L::refresh(&mut session);
                    }
                    // One transaction per step; any traffic refreshes the
                    // watchdog.
                    communicate(drive, adc, |phy| L::poll(&mut session, phy))?;
                    refresh_ms = now_ms;
                    if !L::busy(&session) && written_ms.is_none() {
                        written_ms = Some(now_ms);
                    }
                    Ok(())
                })();
                match result {
                    Ok(()) => State::Output {
                        session,
                        written_ms,
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
                output: L::output(session),
                applied,
            },
            State::Closing(_) | State::Stopping(_, _) => Status::Stopping,
            State::Failed(error) => {
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
            } => L::busy(session) || !applied,
            _ => true,
        }
    }

    /// On an orderly stop, run the link's stop (SCP leaves protocol output
    /// mode, FCP sets its lowest voltage), then release D+. Errors use D+
    /// protection and SCP's watchdog rather than blind cleanup writes.
    /// Do not hand the lines to QC until measured VBUS is back at default,
    /// or `STOP_TIMEOUT_MS` has passed.
    pub fn stop(
        &mut self,
        now_ms: u64,
        drive: &mut DataDrive,
        adc: &mut LineAdc,
        vbus_mv: Option<u16>,
    ) -> Status<L::Adapter, L::Output> {
        let state = core::mem::replace(&mut self.state, State::Failed(Failure::Vbus));
        let (since, error) = match state {
            State::Stopping(since, error) => (since, error),
            State::Output { mut session, .. } => {
                // Queued work is replaced: a half-written setpoint ends here.
                L::stop(&mut session);
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
        mut session: L::Session,
        now_ms: u64,
        drive: &mut DataDrive,
        adc: &mut LineAdc,
        vbus_mv: Option<u16>,
    ) -> Status<L::Adapter, L::Output> {
        match communicate(drive, adc, |phy| L::poll(&mut session, phy)) {
            Ok(()) if L::busy(&session) => {
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
    ) -> Status<L::Adapter, L::Output> {
        drive.set(None, None);
        self.state = State::Stopping(since, error);
        let settled = vbus_mv.is_some_and(|mv| DEFAULT_VBUS_MV.contains(&mv)) && now_ms > since;
        if settled || now_ms.saturating_sub(since) >= STOP_TIMEOUT_MS {
            return Status::Stopped;
        }
        error.map_or(Status::Stopping, Status::Failed)
    }
}

fn communicate<R>(
    drive: &mut DataDrive,
    adc: &mut LineAdc,
    operation: impl FnOnce(&mut Phy<'_>) -> Result<R, ControlError<Infallible>>,
) -> Result<R, Failure> {
    let mut phy = Phy::new(drive, adc.dm_input()).ok_or(Failure::Clock)?;
    operation(&mut phy).map_err(Failure::Control)
}

struct Phy<'a> {
    drive: &'a mut DataDrive,
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
    fn new(drive: &'a mut DataDrive, receiver: DmInput<'a>) -> Option<Self> {
        let cycles_per_us = cycle_clock()?;
        let start = DWT::cycle_count();
        Some(Self {
            drive,
            receiver,
            start,
            cycles_per_us,
        })
    }
}

impl Transport for Phy<'_> {
    type Error = Infallible;
    fn drive(&mut self, high: Option<bool>) -> Result<(), Infallible> {
        self.drive.scp(high);
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
