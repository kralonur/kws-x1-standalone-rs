//! Protect the ripple controls from changing tabs or issuing power requests
//! while adjusting scale, and distinguish a hold from a short freeze tap.
use firmware_core::buttons::{
    BACK, DEFAULT_THRESHOLD, Input, LEFT, Level, Navigation, OK, Open, POLL_MS, Page, RIGHT,
    RippleFocus, Tab,
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
    assert_eq!(focused.page, Page::Open(Open::Ripple(RippleFocus::Capture)));
    assert!(!focused.ripple_hold);
    assert!(tap(&mut nav, OK).ripple_hold);
    assert_eq!(tap(&mut nav, BACK).page, Page::Tabs(Tab::Ripple));
    assert_eq!(tap(&mut nav, RIGHT).page, Page::Tabs(Tab::Overview));
    assert_eq!(tap(&mut nav, LEFT).page, Page::Tabs(Tab::Ripple));
}

#[test]
fn entering_ripple_focus_does_not_freeze_the_capture() {
    let mut nav = Navigation::new(Page::Groups(Tab::Ripple), Settings::default());
    let tab = tap(&mut nav, OK);
    assert_eq!(tab.page, Page::Tabs(Tab::Ripple));
    assert!(!tab.ripple_hold);
    let entered = tap(&mut nav, OK);
    assert!(!entered.ripple_hold);
    assert_eq!(entered.page.level(), Level::Page);
    assert!(!entered.page.editing());
    assert_eq!(tap(&mut nav, LEFT).page, entered.page);
    assert_eq!(tap(&mut nav, RIGHT).page, entered.page);
    assert!(tap(&mut nav, OK).ripple_hold);
    let left = tap(&mut nav, BACK);
    assert_eq!(left.page, Page::Tabs(Tab::Ripple));
    assert!(left.ripple_hold);
    let reentered = tap(&mut nav, OK);
    assert_eq!(reentered.page.level(), Level::Page);
    assert!(reentered.ripple_hold);
    assert!(!tap(&mut nav, OK).ripple_hold);
}

#[test]
fn long_entry_press_cannot_skip_focus_into_scale_adjustment() {
    let mut nav = Navigation::new(Page::Tabs(Tab::Ripple), Settings::default());
    sample(&mut nav, OK, 600 / POLL_MS);
    let entered = sample(&mut nav, 0, u32::from(DEFAULT_THRESHOLD));
    assert_eq!(entered.page, Page::Open(Open::Ripple(RippleFocus::Capture)));
    assert!(!entered.ripple_hold);
    assert_eq!(entered.ripple_scale, Scale::Auto);
}

#[test]
fn hold_and_scale_edit_preserve_tab_navigation_and_never_request_power() {
    let mut nav = Navigation::new(Page::Tabs(Tab::Ripple), Settings::default());
    assert!(!tap(&mut nav, OK).ripple_hold);
    let held = tap(&mut nav, OK);
    assert!(held.ripple_hold);
    assert_eq!(held.page, Page::Open(Open::Ripple(RippleFocus::Capture)));
    // Chosen 600 ms press exceeds the documented 500 ms long-OK threshold,
    // including debounce. Its release must not also toggle freeze.
    sample(&mut nav, OK, 600 / POLL_MS);
    let editing = sample(&mut nav, 0, u32::from(DEFAULT_THRESHOLD));
    assert_eq!(editing.page, Page::Open(Open::Ripple(RippleFocus::Scale)));
    assert!(editing.page.editing());
    assert!(editing.ripple_hold);
    assert_eq!(tap(&mut nav, RIGHT).ripple_scale, Scale::Mv2);
    assert_eq!(tap(&mut nav, RIGHT).ripple_scale, Scale::Mv5);
    assert_eq!(tap(&mut nav, LEFT).ripple_scale, Scale::Mv2);
    assert_eq!(tap(&mut nav, LEFT).ripple_scale, Scale::Auto);
    assert_eq!(tap(&mut nav, LEFT).ripple_scale, Scale::Mv1000);
    let closed = tap(&mut nav, BACK);
    assert_eq!(closed.page, Page::Open(Open::Ripple(RippleFocus::Capture)));
    assert!(closed.ripple_hold);
    assert_eq!(closed.ripple_scale, Scale::Mv1000);
    assert!(!tap(&mut nav, OK).ripple_hold);
    assert_eq!(tap(&mut nav, BACK).page, Page::Tabs(Tab::Ripple));
    assert_eq!(tap(&mut nav, RIGHT).page, Page::Tabs(Tab::Overview));
}

#[test]
fn scale_edit_while_running_closes_with_ok_without_freezing() {
    let mut nav = Navigation::new(Page::Tabs(Tab::Ripple), Settings::default());
    assert!(!tap(&mut nav, OK).ripple_hold);
    sample(&mut nav, OK, 600 / POLL_MS);
    sample(&mut nav, 0, u32::from(DEFAULT_THRESHOLD));
    assert_eq!(
        tap(&mut nav, RIGHT).page,
        Page::Open(Open::Ripple(RippleFocus::Scale))
    );
    let closed = tap(&mut nav, OK);
    assert_eq!(closed.page, Page::Open(Open::Ripple(RippleFocus::Capture)));
    assert!(!closed.ripple_hold);
    assert_eq!(closed.ripple_scale, Scale::Mv2);
}
