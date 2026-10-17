//! Quick Charge trigger: the D+/D− drive, and the entry handshake, requests
//! and probe the Quick Charge page asks for, each checked against the
//! measured output.
//!
//! Each line has two port B outputs; the source firmware's encoding of a level
//! on a pair is: GND drives both low, 0.6 V drives the first high and the
//! second low, 3.3 V drives both high. The resistor network between the pins
//! and the lines is not documented, so the line ADC reads the driven levels
//! back.

use embassy_hc32::gpio::{Level as GpioLevel, OutputGroup, Port};
use embassy_time::Duration;
use firmware_core::buttons::QcTarget;
use qc_protocol::detect::{Action, Follow, Outcome, Probe, Report, Stage};
use qc_protocol::qc2::{Failure, Handshake, Level, Progress, REQUEST_SETTLE_MS, Session, Voltage};
use qc_protocol::qc3::{Continuous, PULSE_US};

/// What the Quick Charge trigger is doing, for the page.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Status {
    /// D+ and D− undriven.
    #[default]
    Off,
    /// D+ held at 0.6 V, waiting for the charger to release D−.
    Entering,
    Failed(Failure),
    /// This voltage is requested; whether the output followed, `None`
    /// without an output measurement to judge from.
    Active(Voltage, Option<Outcome>),
    /// Quick Charge 3.0 continuous mode; the output the steps sent so far
    /// set, in mV.
    Stepping(u16),
    Detecting(Stage),
    /// The probe's findings; 5 V is requested again.
    Detected(Report),
    Scp(u8, crate::scp_drive::Status),
}

/// Port B pins driving D+ and D−, each line through a pair, first then
/// second, as the source firmware drives them.
const PINS: [u8; 4] = [8, 3, 9, 4];

/// Each pin's level putting `level` on its pair.
fn encode(level: Level) -> [GpioLevel; 2] {
    match level {
        Level::Low => [GpioLevel::Low, GpioLevel::Low],
        Level::Mid => [GpioLevel::High, GpioLevel::Low],
        Level::High => [GpioLevel::High, GpioLevel::High],
    }
}

/// Owns PB3, PB4, PB8 and PB9.
pub(crate) struct DataDrive {
    pins: OutputGroup<4>,
}

impl DataDrive {
    /// # Safety
    /// See [`Trigger::new`].
    unsafe fn new() -> Self {
        Self {
            // SAFETY: forwarded from this function's contract.
            pins: unsafe { OutputGroup::new_high_drive(Port::B, PINS) },
        }
    }

    /// Drive each line to its level, or leave it undriven (`None`). Both
    /// lines change together, well inside the charger's 20 ms minimum
    /// Output Voltage Glitch Filter Time (CHY100 TGLITCH(V)CHANGE).
    pub(crate) fn set(&mut self, dp: Option<Level>, dm: Option<Level>) {
        let [dp1, dp2] = dp.map_or([None; 2], |level| encode(level).map(Some));
        let [dm1, dm2] = dm.map_or([None; 2], |level| encode(level).map(Some));
        self.pins.drive([dp1, dp2, dm1, dm2]);
    }

    /// SCP uses the opposite D+ resistor-pair encoding from the 0.6 V
    /// entry level. The adapter MUST verify 2.6–3.0 V with ADC before sending
    /// anything: the resistor topology itself is not established. D− uses
    /// 0/3.3 V digital signaling, released for source replies.
    pub(crate) fn scp(&mut self, high: Option<bool>) {
        let dm = high.map(|high| if high { Level::High } else { Level::Low });
        let [dm1, dm2] = dm.map_or([None; 2], |level| encode(level).map(Some));
        self.pins
            .drive([Some(GpioLevel::Low), Some(GpioLevel::High), dm1, dm2]);
    }

    /// Drive `(D+, D−)` levels.
    fn levels(&mut self, (dp, dm): (Level, Level)) {
        self.set(Some(dp), Some(dm));
    }

    /// Drive `pulse` for `PULSE_US`, then `hold`. The wait is busy, so no
    /// other thread-mode task can stretch it; an interrupt can only lengthen
    /// it by its own run time, still far inside the 20 ms a pulse must stay
    /// under (`PULSE_US`).
    fn pulse(&mut self, pulse: (Level, Level), hold: (Level, Level)) {
        self.levels(pulse);
        embassy_time::block_for(Duration::from_micros(u64::from(PULSE_US)));
        self.levels(hold);
    }

    fn apply(&mut self, action: Action) {
        match action {
            Action::Drive(levels) => self.levels(levels),
            Action::Pulse { pulse, hold } => self.pulse(pulse, hold),
            Action::Finish(_) => {}
        }
    }
}

/// Trigger progress; the handshake must complete before any request.
enum State {
    Off,
    Releasing(u64),
    Scp(u8, crate::scp_drive::Trigger),
    Entering(Handshake),
    Failed(Failure),
    Active {
        session: Session,
        voltage: Voltage,
        /// Watches the output until it decides; `None` after that, or
        /// without a measurement from before the request.
        follow: Option<Follow>,
        outcome: Option<Outcome>,
    },
    /// 5 V requested at `since_ms` before `target`: the way into continuous
    /// mode from a known output, and out of it (a charger was seen to ignore
    /// a fixed request sent straight from it).
    Resetting {
        session: Session,
        target: QcTarget,
        since_ms: u64,
    },
    /// Continuous mode requested at `since_ms`; steps wait for the charger
    /// to take it.
    Stepping {
        session: Session,
        continuous: Continuous,
        since_ms: u64,
    },
    Detecting(Probe),
    Detected(Session, Report),
}

/// The Quick Charge trigger, stepped with each line scan.
pub struct Trigger {
    drive: DataDrive,
    state: State,
}

impl Trigger {
    /// Select GPIO with high drive on the four pins, outputs disabled so the
    /// lines stay undriven.
    ///
    /// # Safety
    /// Call once, after JTAG is released (PB3/PB4 are JTAG pins at reset,
    /// RM 9.3.1, 9.3.2). The caller gives these four pins to this owner.
    pub unsafe fn new() -> Self {
        Self {
            // SAFETY: forwarded from this function's contract.
            drive: unsafe { DataDrive::new() },
            state: State::Off,
        }
    }

    /// Move toward `target`; `None` releases D+/D−. `dm_mv` is the latest D−
    /// measurement and `vbus_mv` the output's, `None` after a failed read.
    pub fn update(
        &mut self,
        now_ms: u64,
        target: Option<QcTarget>,
        lines: Option<crate::line_adc::LineVoltages>,
        vbus_mv: Option<u16>,
        adc: &mut crate::line_adc::LineAdc,
    ) -> Status {
        if let Some(QcTarget::Scp(attempt)) = target {
            if !matches!(self.state, State::Scp(current, _) if current == attempt) {
                self.state = State::Scp(
                    attempt,
                    crate::scp_drive::Trigger::start(now_ms, &mut self.drive),
                );
            }
            if let State::Scp(_, scp) = &mut self.state {
                return Status::Scp(
                    attempt,
                    scp.update(&mut self.drive, adc, now_ms, lines, vbus_mv),
                );
            }
        }
        if matches!(self.state, State::Scp(..)) {
            self.drive.set(None, None);
            self.state = State::Releasing(now_ms);
        }
        let dm_mv = lines.map(|lines| lines.dm_mv);
        let state = core::mem::replace(&mut self.state, State::Off);
        self.state = match (state, target) {
            (State::Off, None) => State::Off,
            // Releasing D+ lets it fall below VDAT(REF), which returns the
            // charger to 5 V (CHY100).
            (_, None) => {
                self.drive.set(None, None);
                State::Off
            }
            (State::Releasing(since), Some(_))
                if now_ms.saturating_sub(since) < crate::scp_drive::RELEASE_MS =>
            {
                State::Releasing(since)
            }
            (State::Off | State::Releasing(_) | State::Scp(..), Some(_)) => {
                self.drive.set(Some(Level::Mid), None);
                State::Entering(Handshake::start(now_ms))
            }
            (State::Entering(handshake), Some(target)) => match dm_mv {
                // A failed scan waits for the next one.
                None => State::Entering(handshake),
                Some(dm_mv) => match handshake.poll(now_ms, dm_mv) {
                    Progress::Waiting(handshake) => State::Entering(handshake),
                    Progress::Ready(session) => self.begin(session, target, now_ms, vbus_mv),
                    Progress::Failed(failure) => {
                        self.drive.set(None, None);
                        State::Failed(failure)
                    }
                },
            },
            (State::Failed(failure), Some(_)) => State::Failed(failure),
            (
                State::Active {
                    session,
                    voltage,
                    follow,
                    outcome,
                },
                Some(QcTarget::Volts(requested)),
            ) if requested == voltage => {
                let decided = follow.zip(vbus_mv).map(|(f, mv)| f.poll(now_ms, mv));
                State::Active {
                    session,
                    voltage,
                    follow: follow.filter(|_| decided == Some(Outcome::Waiting)),
                    outcome: decided.or(outcome),
                }
            }
            (State::Active { session, .. }, Some(target)) => {
                self.begin(session, target, now_ms, vbus_mv)
            }
            (
                State::Resetting {
                    session, since_ms, ..
                },
                Some(target),
            ) if now_ms.saturating_sub(since_ms) < REQUEST_SETTLE_MS => State::Resetting {
                session,
                target,
                since_ms,
            },
            (State::Resetting { session, .. }, Some(QcTarget::Steps(_))) => {
                let (continuous, levels) = Continuous::request(&session, Voltage::V5);
                self.drive.levels(levels);
                State::Stepping {
                    session,
                    continuous,
                    since_ms: now_ms,
                }
            }
            (State::Resetting { session, .. }, Some(target)) => {
                self.begin(session, target, now_ms, vbus_mv)
            }
            (
                State::Stepping {
                    session,
                    mut continuous,
                    since_ms,
                },
                Some(QcTarget::Steps(target_mv)),
            ) => {
                // One step per scan, once the charger took the mode.
                if now_ms.saturating_sub(since_ms) >= REQUEST_SETTLE_MS
                    && let Some(pulse) = continuous.step_toward(target_mv)
                {
                    self.drive.pulse(pulse, continuous.hold());
                }
                State::Stepping {
                    session,
                    continuous,
                    since_ms,
                }
            }
            (State::Stepping { session, .. }, Some(target)) => self.reset(session, target, now_ms),
            (State::Detecting(mut probe), Some(QcTarget::Detect)) => {
                match vbus_mv.and_then(|mv| probe.poll(now_ms, mv)) {
                    Some(Action::Finish(report)) => {
                        let session = probe.into_session();
                        self.drive.levels(session.request(Voltage::V5));
                        State::Detected(session, report)
                    }
                    Some(action) => {
                        self.drive.apply(action);
                        State::Detecting(probe)
                    }
                    None => State::Detecting(probe),
                }
            }
            (State::Detecting(probe), Some(target)) => {
                self.begin(probe.into_session(), target, now_ms, vbus_mv)
            }
            (State::Detected(session, report), Some(QcTarget::Detect)) => {
                State::Detected(session, report)
            }
            (State::Detected(session, _), Some(target)) => {
                self.begin(session, target, now_ms, vbus_mv)
            }
        };
        match &self.state {
            State::Off => Status::Off,
            State::Entering(_) | State::Releasing(_) => Status::Entering,
            State::Scp(attempt, _) => Status::Scp(*attempt, crate::scp_drive::Status::Entering),
            State::Failed(failure) => Status::Failed(*failure),
            State::Active {
                voltage, outcome, ..
            } => Status::Active(*voltage, *outcome),
            State::Resetting {
                target: QcTarget::Steps(_),
                ..
            } => Status::Stepping(Voltage::V5.millivolts()),
            State::Resetting { .. } => Status::Active(Voltage::V5, Some(Outcome::Waiting)),
            State::Stepping { continuous, .. } => Status::Stepping(continuous.set_mv()),
            State::Detecting(probe) => Status::Detecting(probe.stage()),
            State::Detected(_, report) => Status::Detected(*report),
        }
    }

    /// Ask the charger for `target` in an entered session. A probe needs an
    /// output measurement to start from; without one 5 V is requested and the
    /// next scan tries again.
    fn begin(
        &mut self,
        session: Session,
        target: QcTarget,
        now_ms: u64,
        vbus_mv: Option<u16>,
    ) -> State {
        let voltage = match (target, vbus_mv) {
            (QcTarget::Steps(_), _) => return self.reset(session, target, now_ms),
            (QcTarget::Detect, Some(mv)) => {
                let (probe, action) = Probe::start(session, now_ms, mv);
                self.drive.apply(action);
                return State::Detecting(probe);
            }
            (QcTarget::Detect, None) => Voltage::V5,
            (QcTarget::Volts(voltage), _) => voltage,
            // SCP is dispatched before the QC state machine is entered.
            (QcTarget::Scp(_), _) => return State::Off,
        };
        self.drive.levels(session.request(voltage));
        let follow = vbus_mv.map(|mv| Follow::start(now_ms, mv, voltage.millivolts()));
        State::Active {
            session,
            voltage,
            follow,
            outcome: follow.map(|_| Outcome::Waiting),
        }
    }

    /// Request 5 V, then `target` once the charger took it.
    fn reset(&mut self, session: Session, target: QcTarget, now_ms: u64) -> State {
        self.drive.levels(session.request(Voltage::V5));
        State::Resetting {
            session,
            target,
            since_ms: now_ms,
        }
    }
}
