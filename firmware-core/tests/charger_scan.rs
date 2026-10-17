//! Regression: a Charger-page test on a non-PD source must return to Charger
//! info on success or failure, not open the QC trigger's voltage selector.
use firmware_core::buttons::{
    BACK, DEFAULT_THRESHOLD, Detail, Input, Navigation, OK, Open, POLL_MS, Page, PdCommand, QcPage,
    QcTarget, Tab,
};
use firmware_core::settings::{NextBoot, QcFindings, ScpFindings, Screen, Settings};
use qc_protocol::detect::Report;
use qc_protocol::qc2::Failure;

const CHARGER: Page = Page::Tabs(Tab::detail(Detail::Charger));
const REPORT: Report = Report {
    v9: true,
    v12: true,
    v20: false,
    steps: false,
};

fn hold_ok(nav: &mut Navigation) -> (Input, Option<PdCommand>) {
    // The public UI's long-press gesture, including debounce and release.
    const HOLD_MS: u32 = 600;
    let mut input = Input::default();
    let mut command = None;
    for (pressed, samples) in [(OK, HOLD_MS / POLL_MS), (0, u32::from(DEFAULT_THRESHOLD))] {
        for _ in 0..samples {
            let (update, next) = nav.update(pressed);
            input = update;
            command = next.or(command);
        }
    }
    (input, command)
}

fn local_scan() -> Navigation {
    let mut nav = Navigation::new(CHARGER, Settings::default());
    let (input, command) = hold_ok(&mut nav);
    assert_eq!(command, None);
    assert_eq!(input.page, Page::Open(Open::Probe(QcPage::Charger)));
    assert_eq!(input.qc_request(), Some(QcTarget::Detect));
    nav
}

#[test]
fn non_pd_success_returns_to_charger_without_a_restart_or_voltage_selection() {
    let mut nav = local_scan();
    // Match the application's result delivery order.
    nav.finish_probe(REPORT);
    assert_eq!(nav.finish_scan(QcFindings::Detected(REPORT)), None);
    let input = nav.update(0).0;
    assert_eq!(input.page, Page::Open(Open::Scp));
    assert_eq!(
        nav.finish_scp(input.scp_attempt, ScpFindings::Confirmed),
        None
    );
    let (input, command) = nav.update(0);
    assert_eq!(command, None);
    assert_eq!(input.page, CHARGER);
    assert_eq!(input.qc_request(), None);
    assert_eq!(input.qc.report(), Some(REPORT));
    // The next plug-in opens on the default page, not back on Charger.
    assert_eq!(input.settings.next_boot, NextBoot::Normal);
    assert_eq!(input.settings.resume, None);
}

#[test]
fn non_pd_failure_also_returns_to_charger_and_releases_the_probe() {
    for failure in [Failure::NotDcp, Failure::NoEntry] {
        let mut nav = local_scan();
        nav.fail_probe();
        assert_eq!(nav.finish_scan(QcFindings::Failed(failure)), None);
        let input = nav.update(0).0;
        assert_eq!(
            nav.finish_scp(input.scp_attempt, ScpFindings::Unverified),
            None
        );
        let (input, command) = nav.update(0);
        assert_eq!(command, None);
        assert_eq!(input.page, CHARGER);
        assert_eq!(input.qc_request(), None);
    }
}

#[test]
fn back_cancels_the_local_probe_on_the_charger_page() {
    let mut nav = local_scan();
    for _ in 0..DEFAULT_THRESHOLD {
        let (_, command) = nav.update(BACK);
        assert_eq!(command, None);
    }
    let (input, command) = nav.update(0);
    assert_eq!(command, None);
    assert_eq!(input.page, CHARGER);
    assert_eq!(input.qc_request(), None);
}

#[test]
fn pd_full_scan_still_restarts_into_its_pd_stage() {
    let mut nav = Navigation::new(CHARGER, Settings::default());
    nav.set_selectable(true);
    let (input, command) = hold_ok(&mut nav);
    assert_eq!(command, Some(PdCommand::Restart));
    assert_eq!(
        input.settings.next_boot,
        NextBoot::ScanQuickCharge {
            then_cable_test: false
        }
    );
    let mut resumed = Navigation::new(Page::resume(Screen::Qc2, true), input.settings);
    resumed.set_silent(true, false);
    resumed.update(0);
    resumed.finish_probe(REPORT);
    assert_eq!(resumed.finish_scan(QcFindings::Detected(REPORT)), None);
    let input = resumed.update(0).0;
    assert_eq!(input.page, Page::Open(Open::Scp));
    assert_eq!(
        resumed.finish_scp(input.scp_attempt, ScpFindings::Confirmed),
        Some(PdCommand::Restart)
    );
    let (input, _) = resumed.update(0);
    assert_eq!(input.settings.next_boot, NextBoot::ScanPd);
    assert_eq!(input.settings.resume, Some(Screen::Charger));
    assert_eq!(
        input.settings.qc_findings,
        Some(QcFindings::Detected(REPORT))
    );
}

#[test]
fn a_manual_qc_probe_still_opens_its_trigger_controls() {
    let mut nav = Navigation::new(Page::Open(Open::Probe(QcPage::Qc2)), Settings::default());
    nav.finish_probe(REPORT);
    let (input, _) = nav.update(0);
    assert!(matches!(input.page, Page::Open(Open::Qc2(_))));
}
