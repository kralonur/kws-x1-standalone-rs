//! Hardware adapter for SCP detection and explicit output control. Table numbers
//! refer to Appendix B of the SCP draft standard cited in `scp-protocol`.

use core::convert::Infallible;
use cortex_m::peripheral::{DWT, SYST};
use qc_protocol::qc2::{Handshake, Level, Progress};
use scp_protocol::{
    Adapter, Detection, Transport,
    control::{ControlError, Session},
};

use crate::{
    line_adc::{DmInput, LineAdc, LineVoltages},
    qc_drive::DataDrive,
};
mod status;
pub use status::{DEFAULT_VBUS_MV, Failure, Status};

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

pub struct Trigger {
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
        /// When the setpoint finished writing; `None` while it is written.
        written_ms: Option<u64>,
        refresh_ms: u64,
        applied: bool,
    },
    /// The stop's transactions run, one per step.
    Closing(Session),
    Stopping(u64, Option<Failure>),
    Failed(Failure),
}

impl Trigger {
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
        request: Option<(u16, u16)>,
    ) -> Status {
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
            State::Promoting(_) => State::Detecting(Detection::new()),
            State::Detecting(mut detection) => {
                let result = match Phy::new(drive, adc.dm_input()) {
                    Some(mut phy) => detection.poll(&mut phy).map_err(|error| match error {
                        scp_protocol::Error::Timeout if !detection.answered() => Failure::Silent,
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
                Some((mv, ma)) => match firmware_core::scp::request(adapter, mv, ma)
                    .and_then(|output| Session::start(adapter, output))
                {
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
                    let (mv, ma) = request.ok_or(Failure::Vbus)?;
                    // Revalidate against freshly read ratings, not cached UI evidence.
                    if !(firmware_core::scp::MIN_MV..=firmware_core::scp::MAX_MV).contains(&mv)
                        || ma > firmware_core::scp::MAX_MA
                    {
                        return Err(Failure::Vbus);
                    }
                    let target = session.output();
                    // A changed request waits for the running procedure.
                    if !session.busy() && (target.millivolts(), target.milliamps()) != (mv, ma) {
                        session
                            .request(mv, ma)
                            .and_then(|output| session.set(output))
                            .map_err(|error| Failure::Control(ControlError::Request(error)))?;
                        written_ms = None;
                        applied = false;
                    }
                    if !session.busy() {
                        if let Some(written) = written_ms {
                            let followed = firmware_core::scp::followed(target, vbus);
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
                        session.refresh();
                    }
                    // One transaction per step; any traffic refreshes the
                    // watchdog.
                    communicate(drive, adc, |phy| session.poll(phy))?;
                    refresh_ms = now_ms;
                    if !session.busy() && written_ms.is_none() {
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
                output: session.output(),
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
            } => session.busy() || !applied,
            _ => true,
        }
    }

    /// Disable protocol output mode on an orderly stop, then release D+.
    /// Errors use D+ protection/the watchdog rather than blind cleanup writes.
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
        match communicate(drive, adc, |phy| session.poll(phy)) {
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
