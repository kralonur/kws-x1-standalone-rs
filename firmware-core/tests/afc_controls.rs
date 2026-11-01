//! AFC voltage changes require explicit confirmation; cached findings and
//! stale completions must never authorize a voltage request on their own.
use firmware_core::buttons::{
    BACK, DEFAULT_THRESHOLD, Input, Navigation, OK, Open, Page, PdCommand, QcTarget, RIGHT, Tab,
};
use firmware_core::settings::{AfcFindings, NextBoot, Screen, Settings};
use single_wire_protocols::afc::Adapter;

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

fn detected() -> Navigation {
    let mut nav = Navigation::new(Page::Tabs(Tab::Afc), Settings::default());
    let probing = tap(&mut nav, OK);
    assert_eq!(
        probing.qc_request(),
        Some(QcTarget::Afc(probing.afc_attempt))
    );
    nav.finish_afc(probing.afc_attempt, AfcFindings::Detected, Some(Adapter));
    assert_eq!(nav.update(0).0.qc_request(), None);
    nav
}

// The first OK must survive the PD-silent restart as a detection request,
// never as an implicit voltage selection.
#[test]
fn pd_restart_resumes_afc_detection_without_a_second_ok() {
    let mut nav = Navigation::new(Page::Tabs(Tab::Afc), Settings::default());
    nav.set_selectable(true);
    let mut restart = None;
    let mut input = Input::default();
    for mask in [OK, 0] {
        for _ in 0..DEFAULT_THRESHOLD {
            let (update, command) = nav.update(mask);
            input = update;
            restart = command.or(restart);
        }
    }
    assert_eq!(restart, Some(PdCommand::Restart));
    assert_eq!(input.settings.next_boot, NextBoot::QuickCharge);
    assert_eq!(input.settings.resume, Some(Screen::Afc));
    assert_eq!(input.qc_request(), None);

    let settings = input.settings;
    let page = Page::resume(Screen::Afc, settings.next_boot.silent());
    let mut resumed = Navigation::new(page, settings);
    resumed.set_silent(true, false);
    let (input, command) = resumed.update(0);
    assert_eq!(command, None);
    assert_eq!(input.page, Page::Open(Open::Afc));
    assert_eq!(input.qc_request(), Some(QcTarget::Afc(input.afc_attempt)));
    resumed.finish_afc(input.afc_attempt, AfcFindings::Detected, Some(Adapter));
    let input = resumed.update(0).0;
    assert_eq!(input.page, Page::Tabs(Tab::Afc));
    assert_eq!(input.qc_request(), None);
    assert_eq!(tap(&mut resumed, OK).page, Page::Open(Open::AfcSet(0)));
    assert_eq!(resumed.update(0).0.qc_request(), None);
}

#[test]
fn normal_resume_only_shows_the_afc_tab_without_probing() {
    let page = Page::resume(Screen::Afc, false);
    assert_eq!(page, Page::Tabs(Tab::Afc));
    let mut nav = Navigation::new(page, Settings::default());
    assert_eq!(nav.update(0).0.qc_request(), None);
}

#[test]
fn cached_findings_and_stale_completions_cannot_enable_afc_output() {
    let settings = Settings {
        afc_findings: Some(AfcFindings::Detected),
        ..Settings::default()
    };
    let mut nav = Navigation::new(Page::Tabs(Tab::Afc), settings);
    assert_eq!(nav.update(0).0.qc_request(), None);
    let first = tap(&mut nav, OK);
    assert_eq!(first.page, Page::Open(Open::Afc));
    tap(&mut nav, BACK);
    let second = tap(&mut nav, OK);
    assert_ne!(first.afc_attempt, second.afc_attempt);
    nav.finish_afc(first.afc_attempt, AfcFindings::Detected, Some(Adapter));
    assert_eq!(nav.update(0).0.afc_adapter, None);
    assert_eq!(nav.update(0).0.page, Page::Open(Open::Afc));
    nav.finish_afc(second.afc_attempt, AfcFindings::Detected, Some(Adapter));
    assert_eq!(nav.update(0).0.qc_request(), None);
}

#[test]
fn afc_preview_cancel_change_and_off_require_confirmation() {
    let mut nav = detected();
    assert_eq!(tap(&mut nav, OK).page, Page::Open(Open::AfcSet(0)));
    let preview = tap(&mut nav, RIGHT);
    assert_eq!(preview.page, Page::Open(Open::AfcSet(1)));
    assert_eq!(preview.qc_request(), None);
    assert_eq!(tap(&mut nav, BACK).qc_request(), None);
    tap(&mut nav, OK);
    tap(&mut nav, RIGHT);
    let active = tap(&mut nav, OK);
    let request = Some(QcTarget::AfcOutput {
        attempt: active.afc_attempt,
        mv: 9_000,
    });
    assert_eq!(active.qc_request(), request);
    // Opening and canceling the OFF preview keeps the voltage in force.
    tap(&mut nav, OK);
    let preview = tap(&mut nav, RIGHT);
    assert_eq!(preview.page, Page::Open(Open::AfcSet(2)));
    assert_eq!(preview.qc_request(), request);
    assert_eq!(tap(&mut nav, BACK).qc_request(), request);
    // A changed voltage stays in the same session, but only after OK.
    tap(&mut nav, OK);
    tap(&mut nav, RIGHT);
    assert_eq!(tap(&mut nav, RIGHT).qc_request(), request);
    let changed = tap(&mut nav, OK);
    assert_eq!(
        changed.qc_request(),
        Some(QcTarget::AfcOutput {
            attempt: active.afc_attempt,
            mv: 5_000,
        })
    );
    tap(&mut nav, OK);
    tap(&mut nav, RIGHT);
    tap(&mut nav, RIGHT);
    assert_eq!(tap(&mut nav, OK).qc_request(), None);
}

#[test]
fn failure_pd_and_passthrough_revoke_afc_even_while_editing() {
    for cause in 0..3 {
        let mut nav = detected();
        tap(&mut nav, OK);
        tap(&mut nav, RIGHT);
        let active = tap(&mut nav, OK);
        assert!(matches!(
            active.qc_request(),
            Some(QcTarget::AfcOutput { .. })
        ));
        tap(&mut nav, OK);
        match cause {
            0 => {
                nav.fail_afc_output(active.afc_attempt.wrapping_sub(1));
                assert_eq!(nav.update(0).0.qc_request(), active.qc_request());
                nav.fail_afc_output(active.afc_attempt);
            }
            1 => nav.set_selectable(true),
            _ => nav.set_passthrough(true),
        }
        assert_eq!(nav.update(0).0.qc_request(), None);
        // Ownership returning is not a new user confirmation.
        nav.set_selectable(false);
        nav.set_passthrough(false);
        assert_eq!(nav.update(0).0.qc_request(), None);
    }
}
