//! Protect the ripple controls from changing tabs or issuing power requests
//! while changing scale, and keep hold to the Ripple tab.
use firmware_core::buttons::{
    BACK, DEFAULT_THRESHOLD, Input, LEFT, Navigation, OK, Open, POLL_MS, Page, RIGHT, Tab,
};
use firmware_core::ripple::Scale;
use firmware_core::settings::Settings;

fn sample(nav: &mut Navigation, pressed: u8, count: u32) -> Input {
    let mut input = Input::default();
    for _ in 0..count {
        let (update, command) = nav.update(pressed);
        assert_eq!(command, None);
        input = update;
    }
    input
}

fn tap(nav: &mut Navigation, pressed: u8) -> Input {
    sample(nav, pressed, u32::from(DEFAULT_THRESHOLD));
    sample(nav, 0, u32::from(DEFAULT_THRESHOLD))
}

#[test]
fn passthrough_can_navigate_to_ripple_and_hold_without_requesting_power() {
    // Prevent the passive monitor from disappearing behind the sink-only
    // tab filter; exercise the real navigation path rather than the allowlist.
    let mut nav = Navigation::new(Page::Tabs(Tab::Graph), Settings::default());
    nav.set_passthrough(true);
    assert_eq!(tap(&mut nav, RIGHT).page, Page::Tabs(Tab::Ripple));
    let focused = tap(&mut nav, OK);
    assert_eq!(focused.page, Page::Open(Open::Ripple));
    assert!(!focused.ripple_hold);
    assert!(tap(&mut nav, OK).ripple_hold);
    assert_eq!(tap(&mut nav, BACK).page, Page::Tabs(Tab::Ripple));
    assert_eq!(tap(&mut nav, RIGHT).page, Page::Tabs(Tab::Overview));
    // Returning must not show a stale held capture from an earlier visit.
    let returned = tap(&mut nav, LEFT);
    assert_eq!(returned.page, Page::Tabs(Tab::Ripple));
    assert!(!returned.ripple_hold);
}

#[test]
fn focused_arrows_change_scale_without_leaving_the_tab() {
    let mut nav = Navigation::new(Page::Tabs(Tab::Ripple), Settings::default());
    let focused = tap(&mut nav, OK);
    assert_eq!(focused.page, Page::Open(Open::Ripple));
    assert_eq!(focused.ripple_scale, Scale::Auto);
    assert!(tap(&mut nav, OK).ripple_hold);
    assert_eq!(tap(&mut nav, RIGHT).ripple_scale, Scale::Mv2);
    assert_eq!(tap(&mut nav, RIGHT).ripple_scale, Scale::Mv5);
    assert_eq!(tap(&mut nav, LEFT).ripple_scale, Scale::Mv2);
    assert_eq!(tap(&mut nav, LEFT).ripple_scale, Scale::Auto);
    let wrapped = tap(&mut nav, LEFT);
    assert_eq!(wrapped.ripple_scale, Scale::Mv1000);
    assert_eq!(wrapped.page, Page::Open(Open::Ripple));
    // Changing scale must not resume a held capture.
    assert!(wrapped.ripple_hold);
    assert_eq!(tap(&mut nav, BACK).page, Page::Tabs(Tab::Ripple));
    assert_eq!(tap(&mut nav, RIGHT).page, Page::Tabs(Tab::Overview));
}

#[test]
fn long_entry_press_still_just_focuses() {
    let mut nav = Navigation::new(Page::Tabs(Tab::Ripple), Settings::default());
    sample(&mut nav, OK, 600 / POLL_MS);
    let entered = sample(&mut nav, 0, u32::from(DEFAULT_THRESHOLD));
    assert_eq!(entered.page, Page::Open(Open::Ripple));
    assert!(!entered.ripple_hold);
}
