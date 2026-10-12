//! The PD task: selects sink or passthrough mode at boot, then runs the sink
//! session or the receive-only sniffer and keeps publishing the observation.

use core::sync::atomic::Ordering;

use embassy_hc32::gpio::InputPin;
use embassy_time::{Duration, Instant, Timer};
use firmware_core::buttons::PdCommand;
use firmware_core::settings::NextBoot;

use crate::bus::{BUS_LATCH, I2C3, bus_failed, note_bus_failure};
use crate::{
    PD_IRQ_COUNT, pd, pd_cc, pd_service, pd_sniff, pd_trial, publish_observation, store_observation,
};

/// Slider-selected boot: one persistent fixed-target SPR session, or passive.
///
/// PC13 (slider sense, latched once at boot as the original firmware does) is read
/// before any PHY access. Low (sink mode): the session starts at boot, inside
/// the source's Source_Capabilities window (nCapsCount, USB PD §7.32.6); a
/// later start must not use the optional SinkWaitCapTimer Hard Reset (§7.31.3.2).
/// High (passthrough): read-only snapshot plus one verified SWITCHES0 = 0x00
/// write (FUSB302 Rd off, as the original firmware does), then the receive-only sniffer
/// (`docs/SAFETY.md`): never GoodCRC, TX or reset signaling.
/// A silent `boot` (`NextBoot::silent`) stops the sink after the CC check, on
/// its verified baseline: Rd on, automatic GoodCRC off, nothing sent, until
/// the input task asks to restart the charger (`docs/SAFETY.md`). PHY
/// interrupts remain globally masked. Errors halt without reset, retry or
/// cleanup.
#[embassy_executor::task]
pub async fn run(
    mut input: embassy_hc32::exti::ExtiInput,
    pc13: InputPin,
    boot: NextBoot,
    cable_test_skipped: bool,
) {
    let mut observation = pd::Observation::new();
    observation.cable_test = boot == NextBoot::CableTest;
    observation.scan_pd = boot == NextBoot::ScanPd;
    observation.cable_test_skipped = cable_test_skipped;
    observation.reset_flags = crate::reset_flags();
    // Three equal samples or passthrough: an unsettled level never starts a sink.
    let mut sink_mode = true;
    for _ in 0..3 {
        sink_mode &= pc13.is_low();
        Timer::after_millis(1).await;
    }
    observation.pc13_high = Some(!sink_mode);
    observation.irq_high = input.is_high();
    observation.irq_count = PD_IRQ_COUNT.load(Ordering::Relaxed);
    observation.quick_charge = sink_mode && boot.silent();
    if sink_mode && !observation.quick_charge {
        let mut spr = pd::SprObservation::pending();
        spr.target_mv = 5_000;
        observation.spr = Some(spr);
    }
    store_observation(observation);
    if sink_mode {
        // `pd-boot-diagnostic`: step 1 = boot, before the CC check.
        let steps = cfg!(feature = "pd-boot-diagnostic");
        if steps {
            observation.boot_step = 1;
            store_observation(observation);
        }
        if let Some(report) = sink_session(&mut observation).await {
            if steps {
                observation.boot_step = 2;
                store_observation(observation);
            }
            if observation.quick_charge || cfg!(feature = "pd-hold-diagnostic") {
                // CC check restored the power-on baseline: Rd on, AUTO_CRC off.
                observation.spr = None;
                observation.phase = pd::Phase::Hold;
                if observation.quick_charge {
                    silent_hold(&mut input, &mut observation, report).await;
                }
            } else {
                pd_trial::run(&mut observation, report).await;
            }
        }
    } else if let Some(released) = passthrough(&mut observation).await {
        sniff(&mut observation, released).await;
    }
    loop {
        observation.irq_high = input.is_high();
        observation.irq_count = PD_IRQ_COUNT.load(Ordering::Relaxed);
        store_observation(observation);
        // Already-low input is still published without depending on an edge.
        // Timer updates level even if the PHY holds INT_N low indefinitely.
        let _ = embassy_time::with_timeout(
            embassy_time::Duration::from_millis(1000),
            input.wait_for_edge(),
        )
        .await;
    }
}

/// Longest a source keeps trying PD with an unacknowledged sink, from its
/// attach or Hard Reset: nCapsCount (50) Source_Capabilities, each
/// tTypeCSendSourceCap (at most 200 ms) apart (USB PD R3.2, Counter
/// Parameters and Time Values). Its NoResponseTimer Hard Resets (at most
/// 5.5 s) come sooner. The meter boots after the source restarted VBUS, so
/// time since boot is shorter than time since then.
const SILENT_SETTLED: Duration = Duration::from_secs(10);

/// A silent boot: hold until the input task asks for the restart, then
/// restart the charger into PD. A supply still falling from a Quick Charge
/// request is outside the session's start window; that start is retried each
/// read period, nothing having been written.
async fn silent_hold(
    input: &mut embassy_hc32::exti::ExtiInput,
    observation: &mut pd::Observation,
    report: pd_cc::Report,
) {
    loop {
        observation.irq_high = input.is_high();
        observation.irq_count = PD_IRQ_COUNT.load(Ordering::Relaxed);
        observation.silent_settled = Instant::now().as_millis() >= SILENT_SETTLED.as_millis();
        store_observation(*observation);
        let command =
            embassy_time::with_timeout(crate::METER_READ_PERIOD, crate::PD_COMMANDS.receive())
                .await;
        if command == Ok(PdCommand::Restart) {
            break;
        }
    }
    while pd_trial::restart_unheard(observation, report).await {
        store_observation(*observation);
        Timer::after(crate::METER_READ_PERIOD).await;
    }
    store_observation(*observation);
}

/// Passthrough: release the FUSB302 Rd once; no GoodCRC or TX. Returns the
/// verified released snapshot, the only state the sniffer may start from.
async fn passthrough(observation: &mut pd::Observation) -> Option<pd::Snapshot> {
    let mut guard = I2C3.lock().await;
    if bus_failed() {
        observation.phase = pd::Phase::Skipped;
        return None;
    }
    let Some(bus) = guard.as_mut() else {
        observation.phase = pd::Phase::Skipped;
        return None;
    };
    let mut released = None;
    match pd::snapshot(bus).await {
        Ok(snapshot) => {
            observation.snapshot = Some(snapshot);
            match pd_cc::passive().release(bus, snapshot).await {
                Ok(after) => {
                    observation.snapshot = Some(after);
                    observation.phase = pd::Phase::Passive;
                    released = Some(after);
                }
                Err(failure) => {
                    observation.restore_unverified |= failure.unverified;
                    cc_failure(observation, bus, failure.error);
                }
            }
        }
        Err(failure) => {
            observation.failed_register = failure.register;
            observation.phase = pd::Phase::Failed;
            note_bus_failure(bus, failure.source);
        }
    }
    released
}

/// Longest gap between two full sniffer polls. In between, a 1 ms STATUS1
/// sample starts one as soon as a message arrives, so this only bounds how
/// late the latched interrupt flags (Hard Reset, collision) and the Rp level
/// are read. Each full poll is five I2C reads, and each read wakes the PD
/// executor several times: polling them every 1 ms measured a 63% PD
/// executor load with the screen starved, against 6% in the I2C handlers.
const SNIFF_FULL_POLL: embassy_time::Duration = embassy_time::Duration::from_millis(10);

/// Receive-only passthrough sniffer: verified start (orientation, flush,
/// listen), then per I2C3 lock every nominal 1 ms a STATUS1 sample or a full
/// poll (see [`SNIFF_FULL_POLL`]) and a read-only configuration check every
/// second. Any error stops it for good, without
/// cleanup writes; bus errors and configuration mismatches also latch the bus.
async fn sniff(observation: &mut pd::Observation, released: pd::Snapshot) {
    let mut view = pd::SniffObservation::new();
    observation.sniff = Some(view);
    publish_observation(*observation);
    let started = {
        let mut guard = I2C3.lock().await;
        if bus_failed() {
            view.stop = Some(pd::SniffStop::Halted);
            None
        } else if let Some(bus) = guard.as_mut() {
            let mut delay = embassy_time::Delay;
            // Sole bus ownership through both nominal 10 ms CC settles.
            match pd_sniff::once().start(bus, &mut delay, released).await {
                Ok(mut sniffer) => {
                    view.cc_status = Some(sniffer.cc_status());
                    view.cc = Some(sniffer.selected());
                    view.flags = sniffer.flags();
                    Some(sniffer)
                }
                Err(failure) => {
                    view.cc_status = failure.cc_status;
                    view.flags = failure.flags;
                    observation.restore_unverified |= failure.unverified;
                    sniff_stop(&mut view, bus, failure.error);
                    None
                }
            }
        } else {
            view.stop = Some(pd::SniffStop::Halted);
            None
        }
    };
    observation.sniff = Some(view);
    publish_observation(*observation);
    let Some(mut sniffer) = started else {
        return;
    };
    let mut next_check = Instant::now() + embassy_time::Duration::from_secs(1);
    let mut next_poll = Instant::now();
    while view.stop.is_none() {
        Timer::after_millis(1).await;
        let mut changed = false;
        {
            let mut guard = I2C3.lock().await;
            let Some(bus) = guard.as_mut().filter(|_| !bus_failed()) else {
                view.stop = Some(pd::SniffStop::Halted);
                break;
            };
            let now = Instant::now();
            let result = if now >= next_check {
                next_check += embassy_time::Duration::from_secs(1);
                sniffer.check(bus).await.map(|()| None)
            } else {
                let due = if now >= next_poll {
                    Ok(true)
                } else {
                    sniffer.pending(bus).await
                };
                match due {
                    Ok(true) => {
                        next_poll = now + SNIFF_FULL_POLL;
                        sniffer.poll(bus).await.map(Some)
                    }
                    Ok(false) => Ok(None),
                    Err(error) => Err(error),
                }
            };
            view.flags = sniffer.flags();
            match result {
                Ok(Some((outcome, status0))) => {
                    // BC_LVL is only meaningful between BMC transmissions.
                    let rp = firmware_core::pd_sniff::rp(status0);
                    if firmware_core::pd_sniff::quiet(status0) && rp != view.rp {
                        view.rp = rp;
                        changed = true;
                    }
                    if let pd_service::Outcome::Packet(packet) = outcome {
                        view.record(&packet.frame());
                        changed = true;
                    }
                }
                Ok(None) => {}
                Err(error) => {
                    sniff_stop(&mut view, bus, error);
                    changed = true;
                }
            }
        }
        if changed {
            observation.sniff = Some(view);
            publish_observation(*observation);
        }
    }
    observation.sniff = Some(view);
    publish_observation(*observation);
}

/// Record why the sniffer stopped. Bus errors and configuration mismatches latch
/// the bus; wire events (Hard Reset, overflow, bad frame) stop only the sniffer.
fn sniff_stop(
    view: &mut pd::SniffObservation,
    bus: &mut embassy_hc32::i2c::I2c3,
    error: pd_sniff::Error<embassy_hc32::i2c::Error>,
) {
    use pd::SniffStop as Stop;
    view.stop = Some(match error {
        pd_sniff::Error::Stopped => Stop::Halted,
        pd_sniff::Error::Orientation(_) => Stop::Orientation,
        pd_sniff::Error::Boundary { status } => Stop::Boundary(status),
        pd_sniff::Error::Config(pd_cc::Error::Baseline { register, value }) => {
            Stop::Guard { register, value }
        }
        pd_sniff::Error::Config(pd_cc::Error::Verify { register, value }) => {
            BUS_LATCH.fail();
            Stop::Verify { register, value }
        }
        pd_sniff::Error::Drift { register, value } => {
            BUS_LATCH.fail();
            Stop::Drift { register, value }
        }
        pd_sniff::Error::Config(pd_cc::Error::Bus { register, source })
        | pd_sniff::Error::Rx(pd_service::Error::Bus { register, source }) => {
            note_bus_failure(bus, source);
            Stop::Bus(register)
        }
        pd_sniff::Error::Rx(pd_service::Error::HardReset) => Stop::HardReset,
        pd_sniff::Error::Rx(pd_service::Error::RxFull) => Stop::Overflow,
        pd_sniff::Error::Rx(pd_service::Error::Frame(_)) => Stop::Frame,
        pd_sniff::Error::Rx(
            pd_service::Error::UnexpectedTransmission | pd_service::Error::TransmissionFault,
        ) => Stop::UnexpectedTx,
        pd_sniff::Error::Rx(pd_service::Error::PhyFault) => Stop::PhyFault,
        pd_sniff::Error::Rx(pd_service::Error::Halted) => Stop::Halted,
    });
}

/// Record a configuration failure; only a bus error latches the bus here.
fn cc_failure(
    observation: &mut pd::Observation,
    bus: &mut embassy_hc32::i2c::I2c3,
    error: pd_cc::Error<embassy_hc32::i2c::Error>,
) {
    match error {
        pd_cc::Error::Baseline { register, value } => {
            observation.failed_register = register;
            observation.failed_value = Some(value);
            observation.phase = pd::Phase::Guarded;
        }
        pd_cc::Error::Bus { register, source } => {
            observation.failed_register = register;
            observation.phase = pd::Phase::Failed;
            note_bus_failure(bus, source);
        }
        pd_cc::Error::Verify { register, value } => {
            observation.failed_register = register;
            observation.failed_value = Some(value);
            observation.phase = pd::Phase::Verify;
            BUS_LATCH.fail();
        }
    }
}

/// Sink mode: the one-shot CC diagnostic on the power-on configuration.
async fn sink_session(observation: &mut pd::Observation) -> Option<pd_cc::Report> {
    let mut guard = I2C3.lock().await;
    if bus_failed() {
        observation.phase = pd::Phase::Skipped;
        return None;
    }
    let Some(bus) = guard.as_mut() else {
        observation.phase = pd::Phase::Skipped;
        return None;
    };
    let snapshot = match pd::snapshot(bus).await {
        Ok(snapshot) => snapshot,
        Err(failure) => {
            observation.failed_register = failure.register;
            observation.phase = pd::Phase::Failed;
            note_bus_failure(bus, failure.source);
            return None;
        }
    };
    observation.snapshot = Some(snapshot);
    observation.phase = pd::Phase::Measuring;
    store_observation(*observation);
    let mut delay = embassy_time::Delay;
    // Deliberately retain sole bus ownership through the two nominal 10 ms
    // settles: one coherent, non-repeating session.
    // A successful report requires full restored baseline verification; no
    // failed/cancelled session claims restore.
    match pd_cc::diagnostic().run(bus, &mut delay, snapshot).await {
        Ok(report) => {
            observation.cc_status = Some(report.status);
            observation.listen = report.listen;
            observation.post_snapshot = Some(report.after);
            Some(report)
        }
        Err(failure) => {
            observation.restore_unverified |= failure.unverified;
            cc_failure(observation, bus, failure.error);
            None
        }
    }
}
