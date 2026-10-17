//! Read-only detection must exclude PD/passthrough, ignore stale completions,
//! preserve QC evidence across restarts and release the trigger on completion.
use firmware_core::buttons::{
    BACK, DEFAULT_THRESHOLD, Input, Navigation, OK, Open, Page, QcTarget, Tab,
};
use firmware_core::settings::{NextBoot, QcFindings, ScpFindings, Screen, Settings};
use qc_protocol::qc2::Failure;

fn tap(nav: &mut Navigation, mask: u8) -> Input {
    let mut input = Input::default();
    for mask in [mask, 0] {
        for _ in 0..DEFAULT_THRESHOLD {
            let (update, command) = nav.update(mask);
            assert_eq!(command, None);
            input = update;
        }
    }
    input
}

#[test]
fn scp_is_explicit_and_repeated_attempts_ignore_old_results() {
    let mut nav = Navigation::new(Page::Tabs(Tab::Scp), Settings::default());
    assert_eq!(nav.update(0).0.qc_request(), None);
    let first = tap(&mut nav, OK);
    assert_eq!(first.qc_request(), Some(QcTarget::Scp(first.scp_attempt)));
    assert_eq!(nav.finish_scp(first.scp_attempt, true), None);
    let done = nav.update(0).0;
    assert_eq!(done.page, Page::Tabs(Tab::Scp));
    assert_eq!(done.qc_request(), None);
    assert_eq!(done.settings.scp_findings, Some(ScpFindings::Confirmed));
    let second = tap(&mut nav, OK);
    assert_ne!(first.scp_attempt, second.scp_attempt);
    nav.finish_scp(first.scp_attempt, false);
    assert_eq!(nav.update(0).0.page, Page::Open(Open::Scp));
    nav.finish_scp(second.scp_attempt, false);
    assert_eq!(
        nav.update(0).0.settings.scp_findings,
        Some(ScpFindings::Unverified)
    );
    tap(&mut nav, OK);
    let cancelled = tap(&mut nav, BACK);
    assert_eq!(cancelled.qc_request(), None);
    assert_eq!(cancelled.page, Page::Tabs(Tab::Scp));
}

#[test]
fn live_pd_and_passthrough_exclude_digital_detection() {
    let mut nav = Navigation::new(Page::Tabs(Tab::Scp), Settings::default());
    tap(&mut nav, OK);
    nav.set_selectable(true);
    assert_eq!(nav.update(0).0.qc_request(), None);
    nav.set_passthrough(true);
    let input = nav.update(0).0;
    assert_eq!(input.qc_request(), None);
    assert_eq!(input.page, Page::default());
}

#[test]
fn a_source_restart_resumes_scp_without_discarding_qc_evidence() {
    let saved = Settings {
        next_boot: NextBoot::ScanQuickCharge {
            then_cable_test: false,
        },
        resume: Some(Screen::Scp),
        qc_findings: Some(QcFindings::Failed(Failure::NoEntry)),
        ..Settings::default()
    };
    let restored = Settings::decode(saved.encode()).unwrap();
    let mut nav = Navigation::new(Page::resume(Screen::Scp, true), restored);
    nav.set_silent(true, false);
    let input = nav.update(0).0;
    assert_eq!(input.page, Page::Open(Open::Scp));
    assert_eq!(input.qc_request(), Some(QcTarget::Scp(input.scp_attempt)));
    assert_eq!(input.settings.qc_findings, saved.qc_findings);
}

fn output_adapter() -> scp_protocol::Adapter {
    scp_protocol::Adapter {
        modes: scp_protocol::Modes::HighVoltage,
        type0: Some(0x1c),
        type1: None,
        capabilities: Some(scp_protocol::Capabilities {
            min_mv: 5_500,
            max_mv: 11_000,
            min_ma: 300,
            max_ma: 3_200,
            vstep_mv: Some(20),
            istep_ma: Some(50),
            max_power_dw: Some(350),
        }),
    }
}

// Protect against accidental output enabling from detection, stale results,
// preview taps, scan completion, passthrough or PD ownership changes.
#[test]
fn scp_output_requires_confirmation_and_stops_on_back_pd_or_passthrough() {
    use firmware_core::buttons::{PpsFocus, RIGHT};
    let mut nav = Navigation::new(Page::Tabs(Tab::Scp), Settings::default());
    let scanning = tap(&mut nav, OK);
    nav.set_scp_adapter(scanning.scp_attempt.wrapping_sub(1), output_adapter());
    assert!(nav.update(0).0.scp_selection.is_none());
    nav.set_scp_adapter(scanning.scp_attempt, output_adapter());
    nav.finish_scp(scanning.scp_attempt, true);
    assert_eq!(nav.update(0).0.qc_request(), None);
    let preview = tap(&mut nav, OK);
    assert_eq!(preview.page, Page::Open(Open::ScpSet(PpsFocus::Voltage)));
    assert_eq!(preview.qc_request(), None);
    let preview = tap(&mut nav, RIGHT);
    assert_eq!(preview.scp_selection.unwrap().output().millivolts(), 5_520);
    assert_eq!(tap(&mut nav, OK).qc_request(), None);
    let active = tap(&mut nav, OK);
    assert_eq!(
        active.qc_request(),
        Some(QcTarget::ScpOutput {
            attempt: scanning.scp_attempt,
            mv: 5_520,
            ma: 1_000
        })
    );
    assert_eq!(tap(&mut nav, BACK).qc_request(), None);
    // A failure must end the request, never retry the write on its own.
    tap(&mut nav, OK);
    tap(&mut nav, OK);
    assert!(matches!(
        tap(&mut nav, OK).qc_request(),
        Some(QcTarget::ScpOutput { .. })
    ));
    nav.fail_scp_output(scanning.scp_attempt);
    assert_eq!(nav.update(0).0.qc_request(), None);
    // Reopen and apply; live PD must revoke the legacy owner even while editing.
    tap(&mut nav, OK);
    tap(&mut nav, OK);
    assert!(matches!(
        tap(&mut nav, OK).qc_request(),
        Some(QcTarget::ScpOutput { .. })
    ));
    tap(&mut nav, OK);
    nav.set_selectable(true);
    let input = nav.update(0).0;
    assert_eq!(input.page, Page::Tabs(Tab::Scp));
    assert_eq!(input.qc_request(), None);
    nav.set_selectable(false);
    tap(&mut nav, OK);
    tap(&mut nav, OK);
    tap(&mut nav, OK);
    nav.set_passthrough(true);
    assert_eq!(nav.update(0).0.qc_request(), None);
}

#[test]
fn full_scan_scp_completion_cannot_enable_output() {
    let mut nav = Navigation::new(
        Page::Open(Open::Probe(firmware_core::buttons::QcPage::Charger)),
        Settings::default(),
    );
    nav.finish_scan(QcFindings::Failed(Failure::NoEntry));
    let input = nav.update(0).0;
    nav.set_scp_adapter(input.scp_attempt, output_adapter());
    nav.finish_scp(input.scp_attempt, true);
    assert_eq!(nav.update(0).0.qc_request(), None);
}

// Advertised maxima are not necessarily available together. Exercise the
// selector through the power corner and ensure no preview bypasses limits.
#[test]
fn scp_selection_honors_steps_power_and_platform_envelope() {
    use firmware_core::scp::{MAX_MA, MAX_MV, Selection, followed, request};
    let adapter = output_adapter();
    let mut selection = Selection::new(adapter).unwrap();
    for _ in 0..100 {
        selection.step(true, true, 100);
    }
    assert_eq!(selection.output().millivolts(), 11_000);
    for _ in 0..100 {
        selection.step(false, true, 50);
    }
    assert_eq!(selection.output().milliamps(), 3_150);
    assert!(followed(selection.output(), 11_200));
    assert!(!followed(selection.output(), 9_000));
    assert!(request(adapter, 12_000, 1_000).is_err());
    let mut wide = adapter;
    let cap = wide.capabilities.as_mut().unwrap();
    cap.min_mv = 3_000;
    cap.max_mv = 63_000;
    cap.max_ma = 10_000;
    cap.max_power_dw = Some(1_270);
    let mut selection = Selection::new(wide).unwrap();
    for _ in 0..300 {
        selection.step(true, true, 100);
        selection.step(false, true, 250);
        let output = selection.output();
        assert!(output.millivolts() <= MAX_MV);
        assert!(output.milliamps() <= MAX_MA);
        assert!(u32::from(output.millivolts()) * u32::from(output.milliamps()) <= 127_000_000);
    }
    assert!(request(wide, 30_020, 1_000).is_err());
    assert!(request(wide, 9_000, 5_050).is_err());
    wide.capabilities.as_mut().unwrap().vstep_mv = None;
    assert!(Selection::new(wide).is_none());
}

#[test]
fn scp_evidence_roundtrips_without_relocating_legacy_fields() {
    for evidence in [
        None,
        Some(ScpFindings::Unverified),
        Some(ScpFindings::Confirmed),
    ] {
        let settings = Settings {
            scp_findings: evidence,
            ..Settings::default()
        };
        assert_eq!(Settings::decode(settings.encode()), Some(settings));
        // Only the formerly reserved settings bits 7..6 may differ.
        assert_eq!(settings.encode() & !0xc0, Settings::default().encode());
    }
    assert_eq!(Settings::decode(0x5e00_0002), Some(Settings::default()));
    // Deferred AFC uses the old tag with these evidence bits / screen ID.
    // Neither may be accepted as an SCP confirmation or resume destination.
    assert!(Settings::decode(0x5e00_0082).is_none());
    assert!(Settings::decode(0x5e00_3802).is_none());
}

// These synthetic traces protect classifier state transitions, not physical
// capture reliability: the firmware's 100 ms cadence cannot resolve symbols.
#[test]
fn passive_static_scp_candidate_is_not_traffic() {
    use firmware_core::dcp::{Driven, Passive};
    let mut tracker = Passive::default();
    for now in [0, 100, 400, 500, 1_000] {
        assert_eq!(tracker.update(now, 2_700, 0, Some(5_000)), None);
    }
    // Nominal QC9 D− is 0.6 V (Mid), not GND (Low).
    assert_eq!(tracker.update(1_100, 2_700, 600, Some(9_000)), None);
    assert_eq!(
        tracker.update(1_500, 2_700, 600, Some(9_000)),
        Some(Driven::Qc2 { mv: 9_000 })
    );
}

#[test]
fn passive_divider_like_pulse_survives_but_stable_divider_clears_traffic() {
    use firmware_core::dcp::{Driven, Passive};
    let mut tracker = Passive::default();
    assert_eq!(tracker.update(0, 2_700, 0, Some(9_000)), None);
    assert_eq!(tracker.update(100, 2_700, 2_700, Some(9_000)), None);
    assert_eq!(
        tracker.update(200, 2_700, 0, Some(9_000)),
        Some(Driven::SingleWire)
    );
    for now in [300, 500] {
        assert_eq!(
            tracker.update(now, 2_700, 2_700, Some(9_000)),
            Some(Driven::SingleWire)
        );
    }
    assert_eq!(tracker.update(700, 2_700, 2_700, Some(9_000)), None);
    // Reset must discard the unfinished pulse as well as the latch.
    assert_eq!(tracker.update(800, 2_700, 0, Some(9_000)), None);
}

#[test]
fn passive_pulse_edges_expire_without_restarting_on_steady_high() {
    use firmware_core::dcp::Passive;
    for (low, high, end) in [(0, 400, 500), (0, 100, 500)] {
        let mut tracker = Passive::default();
        assert_eq!(tracker.update(low, 2_700, 0, Some(5_000)), None);
        assert_eq!(tracker.update(high, 2_700, 3_200, Some(5_000)), None);
        // Continued high must not restart an expired pulse from the old low.
        assert_eq!(tracker.update(end, 2_700, 3_200, Some(5_000)), None);
        assert_eq!(tracker.update(end + 100, 2_700, 0, Some(5_000)), None);
    }
}

#[test]
fn passive_single_wire_pulses_do_not_flicker_qc3_and_steady_qc_still_settles() {
    use firmware_core::dcp::{Driven, Passive};
    let mut tracker = Passive::default();
    // Simulate a tablet charging over FCP/SCP: D+ held at 0.6 V, D− idling
    // at 0 V and pulsing to 3.2 V for 100 ms twice a second while VBUS is 9.5 V.
    for cycle in 0..4u64 {
        let base = cycle * 500;
        for step in [0u64, 100, 200] {
            let out = tracker.update(base + step, 600, 0, Some(9_500));
            assert_ne!(out, Some(Driven::Qc3));
            if cycle > 0 {
                assert_eq!(out, Some(Driven::SingleWire));
            }
        }
        // Single-wire burst on D− (looks like `(Mid, High)` = QC3 HOLD for 100-200 ms).
        for step in [300u64, 400] {
            let out = tracker.update(base + step, 600, 3_200, Some(9_500));
            assert_ne!(out, Some(Driven::Qc3));
            if cycle > 0 {
                assert_eq!(out, Some(Driven::SingleWire));
            }
        }
    }
    // Unplugging drops D+ to 0 V and clears the latched state immediately.
    assert_eq!(tracker.update(2_100, 0, 0, Some(5_000)), None);

    // Steady QC 3.0 continuous mode (`(Mid, High)` held >= 400 ms without D−
    // dropping to 0 V) still settles cleanly to `Driven::Qc3`.
    assert_eq!(tracker.update(3_000, 600, 3_200, Some(7_000)), None);
    assert_eq!(tracker.update(3_200, 600, 3_200, Some(7_000)), None);
    assert_eq!(
        tracker.update(3_400, 600, 3_200, Some(7_000)),
        Some(Driven::Qc3)
    );
}

// A held arrow must repeat on the SCP editor as it does on PPS; it once
// moved only on release, one step per press.
#[test]
fn held_arrow_repeats_on_scp_voltage() {
    use firmware_core::buttons::{POLL_MS, RIGHT};
    let mut nav = Navigation::new(Page::Tabs(Tab::Scp), Settings::default());
    let scanning = tap(&mut nav, OK);
    nav.set_scp_adapter(scanning.scp_attempt, output_adapter());
    nav.finish_scp(scanning.scp_attempt, true);
    tap(&mut nav, OK);
    for _ in 0..1_000 / POLL_MS {
        nav.update(RIGHT);
    }
    let held = tap(&mut nav, 0);
    // Six 100 mV repeats from 0.5 s, none on release.
    assert!(held.scp_selection.unwrap().output().millivolts() >= 6_000);
}
