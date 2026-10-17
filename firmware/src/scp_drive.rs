//! Hardware adapter for the portable, read-only SCP detector.

use core::convert::Infallible;
use cortex_m::peripheral::{DWT, SYST};
use qc_protocol::qc2::{Handshake, Level, Progress};
use scp_protocol::{Adapter, Transport};

use crate::{
    line_adc::{DmInput, LineAdc, LineVoltages},
    qc_drive::DataDrive,
};
mod status;
pub use status::{Failure, Status};

/// Application release/settle bounds in ms; start only after measured 5 V.
pub(crate) const RELEASE_MS: u64 = 100;
const SETTLE_MS: u64 = 1_000;
/// Appendix B Table B.4: allow the 40–80 ms VDP_SRC turn-on interval.
const DP_SETTLE_MS: u64 = 80;
/// Appendix B Table B.2: SCP D+ detection-source window in mV.
const DP_MIN_MV: u16 = 2_600;
const DP_MAX_MV: u16 = 3_000;
/// Digital idle-low limit from Table B.2, in mV.
const IDLE_MAX_MV: u16 = 600;
/// Application policy: read-only detection may not raise VBUS above 5 V.
const DEFAULT_MV: u16 = 5_000;
const VBUS_TOLERANCE_MV: u16 = 500;

pub struct Trigger {
    state: State,
}
enum State {
    Releasing(u64),
    Entering(Handshake),
    Promoting(u64),
    Checking(Adapter),
    Detected(Adapter),
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
    ) -> Status {
        let at_default = vbus_mv.is_some_and(|mv| mv.abs_diff(DEFAULT_MV) <= VBUS_TOLERANCE_MV);
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
            State::Promoting(_) => {
                let result = match Phy::new(drive, adc.dm_input()) {
                    Some(mut phy) => scp_protocol::detect(&mut phy).map_err(Failure::Wire),
                    None => Err(Failure::Clock),
                };
                // No register write or negotiated output is held after detection.
                drive.set(None, None);
                match result {
                    Ok(adapter) => State::Checking(adapter),
                    Err(error) => State::Failed(error),
                }
            }
            State::Checking(adapter) if at_default => State::Detected(adapter),
            State::Checking(_) => State::Failed(Failure::Vbus),
            State::Detected(adapter) => State::Detected(adapter),
            State::Failed(error) => State::Failed(error),
        };
        match self.state {
            State::Releasing(_) | State::Entering(_) | State::Promoting(_) => Status::Entering,
            State::Checking(_) => Status::Reading,
            State::Detected(adapter) => Status::Detected(adapter),
            State::Failed(error) => {
                drive.set(None, None);
                Status::Failed(error)
            }
        }
    }
}

struct Phy<'a> {
    drive: &'a mut DataDrive,
    receiver: DmInput<'a>,
    start: u32,
    cycles_per_us: u32,
}

impl<'a> Phy<'a> {
    fn new(drive: &'a mut DataDrive, receiver: DmInput<'a>) -> Option<Self> {
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
