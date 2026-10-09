//! Persistent female-port fixed-target SPR session; one bus lease per transaction.
use crate::bus::{BUS_LATCH, bus_failed, note_bus_failure};
use crate::{pd, pd_cc, pd_service, pd_spr_phy};
use embedded_hal_async::i2c::{ErrorType, I2c, Operation};

struct SharedBus;

/// `pd-boot-diagnostic` (`docs/SAFETY.md`): the longest I2C3 lock wait,
/// I2C transfer and 1 ms delay overshoot in this session, ms. A receive-poll gap
/// (`Observation::poll_gap_ms`) is made of these plus engine CPU time.
pub static LOCK_WAIT_MAX: AtomicU32 = AtomicU32::new(0);
pub static TRANSFER_MAX: AtomicU32 = AtomicU32::new(0);
pub static DELAY_OVER_MAX: AtomicU32 = AtomicU32::new(0);
use core::sync::atomic::{AtomicU32, Ordering as AtomicOrdering};

fn note_max(max: &AtomicU32, since: embassy_time::Instant) -> embassy_time::Instant {
    let now = embassy_time::Instant::now();
    if cfg!(feature = "pd-boot-diagnostic") {
        max.fetch_max(
            now.duration_since(since).as_millis() as u32,
            AtomicOrdering::Relaxed,
        );
    }
    now
}

/// `embassy_time::Delay` that records how late it wakes (diagnostic only).
struct TimedDelay;
impl embedded_hal_async::delay::DelayNs for TimedDelay {
    async fn delay_ns(&mut self, ns: u32) {
        let began = embassy_time::Instant::now();
        embassy_time::Timer::after_nanos(u64::from(ns)).await;
        if cfg!(feature = "pd-boot-diagnostic") {
            let late = began
                .elapsed()
                .as_millis()
                .saturating_sub(u64::from(ns) / 1_000_000);
            DELAY_OVER_MAX.fetch_max(late as u32, AtomicOrdering::Relaxed);
        }
    }
}
impl ErrorType for SharedBus {
    type Error = embassy_hc32::i2c::Error;
}
impl I2c for SharedBus {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let asked = embassy_time::Instant::now();
        let mut guard = crate::bus::I2C3.lock().await;
        let locked = note_max(&LOCK_WAIT_MAX, asked);
        if bus_failed() {
            return Err(embassy_hc32::i2c::Error::NotReady);
        }
        let bus = guard.as_mut().ok_or(embassy_hc32::i2c::Error::NotReady)?;
        // Run the transfer to completion here (see `spin`). Awaiting it costs
        // one executor turn per byte: a 31-byte FIFO read took 14 ms on hardware
        // and the Request missed the Source's SenderResponseTimer (at least
        // 27 ms, Table 7.9; 24 ms under PD 2.0), which then Hard Reset. At
        // 400 kHz a transfer takes under 1 ms; its 50 ms timeout still bounds it.
        let result = spin(bus.transaction(address, operations));
        note_max(&TRANSFER_MAX, locked);
        if let Err(error) = result {
            note_bus_failure(bus, error);
        }
        result
    }
}

/// Run `future` to completion here, never returning to the executor. The HAL
/// waits re-read the I2C status on every poll and the generic timer queue
/// accepts any waker, so only interrupt handlers run until it completes.
fn spin<F: core::future::Future>(future: F) -> F::Output {
    let mut future = core::pin::pin!(future);
    let mut cx = core::task::Context::from_waker(core::task::Waker::noop());
    loop {
        if let core::task::Poll::Ready(output) = future.as_mut().poll(&mut cx) {
            return output;
        }
    }
}

/// SinkTxOK gate (section 7.2, `docs/SAFETY.md`): one I2C3 lease and no
/// executor turn from the STATUS0 sample to the TXON byte. At 400 kHz the two
/// reads and a FIFO write of at most 39 bytes take about 1.3 ms; tSinkDelay is
/// 5 ms. Each HAL operation keeps its own 50 ms timeout as the hard ceiling.
impl pd_spr_phy::GatedBus for SharedBus {
    async fn gated_write(
        &mut self,
        address: u8,
        registers: [u8; 2],
        allow: impl FnOnce(u8, u8) -> bool,
        bytes: &[u8],
    ) -> Result<([u8; 2], bool), Self::Error> {
        let asked = embassy_time::Instant::now();
        let mut guard = crate::bus::I2C3.lock().await;
        note_max(&LOCK_WAIT_MAX, asked);
        if bus_failed() {
            return Err(embassy_hc32::i2c::Error::NotReady);
        }
        let bus = guard.as_mut().ok_or(embassy_hc32::i2c::Error::NotReady)?;
        let result = spin(async {
            let mut samples = [0; 2];
            for (sample, register) in samples.iter_mut().zip(registers) {
                bus.transaction(
                    address,
                    &mut [
                        Operation::Write(&[register]),
                        Operation::Read(core::slice::from_mut(sample)),
                    ],
                )
                .await?;
            }
            if !allow(samples[0], samples[1]) {
                return Ok((samples, false));
            }
            bus.transaction(address, &mut [Operation::Write(bytes)])
                .await?;
            Ok((samples, true))
        });
        if let Err(error) = result {
            note_bus_failure(bus, error);
        }
        result
    }
}

/// Latest input state from `button_task`; nothing pressed on the meter page
/// until its first poll.
fn input() -> firmware_core::buttons::Input {
    crate::BUTTON_STATE.try_get().unwrap_or_default()
}

/// The cable list section at the top of the screen, while it is shown.
fn detail_page() -> Option<u32> {
    use firmware_core::buttons::{Detail, Page};
    let shown = matches!(
        input().page,
        Page::Detail {
            page: Detail::Cable,
            ..
        }
    );
    let (page, view) = crate::DETAIL_VIEW.try_get()?;
    (shown && page == Detail::Cable).then_some(u32::from(view.section))
}

fn control(
    observation: &mut fusb302_pd::observation::Observation,
    selection: &firmware_core::pd_spr::Selection,
    store: Option<&dyn firmware_core::pd_svids::Store>,
) {
    while let Ok(command) = crate::PD_COMMANDS.try_receive() {
        selection.apply(command);
    }
    pd::control_cable_selection(observation, selection, detail_page(), store);
}

/// Back held (for the session's stop delay) ends the session.
fn stop_requested() -> bool {
    input().pressed & firmware_core::buttons::BACK != 0
}

use firmware_core::pd_spr;

struct Clock;
impl pd_spr_phy::Clock for Clock {
    fn now_ms(&self) -> u64 {
        embassy_time::Instant::now().as_millis()
    }
}
impl pd_spr::Timer for Clock {
    async fn after_millis(ms: u64) {
        embassy_time::Timer::after_millis(ms).await;
        // In this pinned upstream revision only SinkPPSPeriodic uses5000ms.
        // Its Ready select must not drop an active native I2C receive future.
        if ms == 5_000 {
            pd_spr_phy::wait_for_rx_idle(&mut embassy_time::Delay).await;
        }
    }
}

pub async fn run(observation: &mut pd::Observation, report: pd_cc::Report) {
    let selection = pd_spr::Selection::new();
    // Presses made before this session (e.g. an OK with no charger) must not
    // request anything now: only commands sent during the session apply.
    crate::PD_COMMANDS.clear();
    let mut bus = SharedBus;
    let clock = Clock;
    let mut delay = TimedDelay;
    let mut error;
    {
        let control = |observation: &mut fusb302_pd::observation::Observation,
                       store: Option<&dyn firmware_core::pd_svids::Store>| {
            control(observation, &selection, store)
        };
        let identity_started = || selection.set_mode_cursor(None);
        let board_observation = *observation;
        let publish = |phy| crate::store_observation(board_observation.with_phy(phy));
        let config = pd_spr_phy::Config {
            persistent: true,
            selection: Some(&selection),
            control: Some(&control),
            identity_started: Some(&identity_started),
            publish: Some(&publish),
            trace_steps: cfg!(feature = "pd-boot-diagnostic"),
            trace_cable: cfg!(feature = "cable-log"),
            stop_requested: Some(stop_requested),
            svids: Some(&crate::CABLE_SVID_STORE),
        };
        let mut phy_observation = observation.phy;
        phy_observation.phase = pd::Phase::Negotiating;
        let (mut phy, started) = pd_spr_phy::Session::start(
            &mut bus,
            &mut delay,
            &clock,
            config,
            phy_observation,
            report,
        )
        .await;
        if let Err(failure) = started {
            error = Some(failure);
        } else {
            let trace = core::cell::RefCell::new(pd_spr::MaintainedTrace::default());
            let mut trial = pd_spr::Trial::maintained();
            let mut sent_hard_reset = false;
            match trial
                .run_selectable::<_, Clock>(&mut phy, &trace, &selection)
                .await
            {
                Ok(()) => {
                    phy.observation.phase = pd::Phase::Failed;
                    phy.observation.rx_error = 43;
                }
                Err(failure) => {
                    phy.observation.phase = pd::Phase::Failed;
                    phy.observation.rx_error = match failure {
                        pd_spr::Error::Stopped => 32,
                        pd_spr::Error::UnsupportedPhy => 33,
                        pd_spr::Error::UnsafeCapabilities => 34,
                        pd_spr::Error::Peer => 35,
                        pd_spr::Error::ReceiveDiscarded => 36,
                        pd_spr::Error::HardResetReceived => 37,
                        pd_spr::Error::HardResetSent => {
                            sent_hard_reset = true;
                            44
                        }
                        pd_spr::Error::Transmit(_) => 38,
                        pd_spr::Error::Declined(_) => 39,
                        pd_spr::Error::RecoveryBlocked => {
                            if !phy.observation.spr.is_some_and(|spr| spr.tx_written)
                                && phy.observation.rx_packets == 0
                            {
                                phy.observation.phase = pd::Phase::NoCapabilities;
                            }
                            40
                        }
                        pd_spr::Error::TransmissionBlocked => 41,
                        pd_spr::Error::Engine(_) => 42,
                    };
                }
            }
            error = phy.take_error();
            // PE_SNK_Transition_to_default (`docs/SAFETY.md`) when the
            // PHY saw Hard Reset Signaling and, for ours, the wrapper sent it.
            if phy.hard_reset_received() || (phy.hard_reset_sent() && sent_hard_reset) {
                phy.observation.phase = pd::Phase::Failed;
                phy.observation.rx_error = if phy.hard_reset_sent() { 44 } else { 37 };
                publish(phy.observation);
                match phy.transition_to_default().await {
                    // VBUS is back at vSafe5V and the PHY at its boot baseline
                    // with Rd unchanged: restart like a power-up (5 V default).
                    Ok(()) => cortex_m::peripheral::SCB::sys_reset(),
                    Err(failure) => error = Some(failure),
                }
            }
            if matches!(error, Some(pd_spr_phy::Error::StopRequested)) {
                let contract = trace.borrow().contract.clone();
                error = None;
                if let Some(contract) = contract {
                    match phy.finish(&contract).await {
                        Ok(after) => {
                            phy.observation.post_snapshot = Some(after);
                            phy.observation.phase = pd::Phase::Contracted;
                            phy.observation.rx_error = 0;
                        }
                        Err(failure) => error = Some(failure),
                    }
                } else {
                    error = Some(pd_spr_phy::Error::Completion);
                }
            }
            if phy.observation.phase != pd::Phase::Contracted && !phy.stopped() {
                phy.stop();
            }
        }
        if error.is_none() {
            error = phy.take_error();
        }
        *observation = observation.with_phy(phy.observation);
        observation.restore_unverified = phy.unverified();
        observation.failed_register = phy.register().unwrap_or(0);
    }
    if let Some(failure) = error {
        failure_report(observation, failure);
    }
    // An uncertain/changed PHY and cancelled HAL cannot safely share this bus.
    // Nothing attempts to turn TX off, reset, restore or request again on error.
    if observation.phase != pd::Phase::Contracted && observation.restore_unverified {
        BUS_LATCH.fail();
    }
}
fn failure_report(
    observation: &mut pd::Observation,
    failure: pd_spr_phy::Error<embassy_hc32::i2c::Error>,
) {
    use pd_spr_phy::Error;
    observation.phase = pd::Phase::Failed;
    observation.rx_error = 1;
    match failure {
        Error::Orientation | Error::InputVoltage { .. } => {
            observation.phase = pd::Phase::Guarded;
            observation.rx_error = 2;
        }
        Error::Config(pd_cc::Error::Bus { register, .. })
        | Error::Rx(pd_service::Error::Bus { register, .. }) => {
            observation.failed_register = register;
            BUS_LATCH.fail();
        }
        Error::Bus {
            address, register, ..
        } => {
            observation
                .spr
                .get_or_insert_with(pd::SprObservation::pending)
                .failed_address = address;
            observation.failed_register = register;
            BUS_LATCH.fail();
        }
        Error::Config(pd_cc::Error::Baseline { register, value })
        | Error::Config(pd_cc::Error::Verify { register, value }) => {
            observation.phase = pd::Phase::Verify;
            observation.failed_register = register;
            observation.failed_value = Some(value);
        }
        Error::Boundary { status } => {
            observation.failed_register = 0x41;
            observation.failed_value = Some(status);
            observation.rx_error = 3;
        }
        Error::Rx(pd_service::Error::HardReset) => observation.rx_error = 4,
        Error::Rx(pd_service::Error::PhyFault) => observation.rx_error = 5,
        Error::Rx(
            pd_service::Error::TransmissionFault | pd_service::Error::UnexpectedTransmission,
        ) => observation.rx_error = 6,
        Error::Rx(pd_service::Error::RxFull) => observation.rx_error = 7,
        Error::Rx(pd_service::Error::Frame(_)) => observation.rx_error = 8,
        Error::UnexpectedAck => observation.rx_error = 9,
        Error::Request => observation.rx_error = 10,
        Error::QueueFull => observation.rx_error = 11,
        Error::Budget => observation.rx_error = 12,
        Error::TxTimeout => observation.rx_error = 13,
        Error::Completion => observation.rx_error = 14,
        Error::PpsDeadline => observation.rx_error = 16,
        Error::HardResetNeeded => observation.rx_error = 17,
        Error::HardResetVbus => observation.rx_error = 19,
        Error::BistCarrierVbus => observation.rx_error = 20,
        Error::NotAcknowledged => observation.rx_error = 18,
        Error::Peer => observation.rx_error = 15,
        _ => {}
    }
}
