//! Protect the ripple controls from changing tabs or issuing power requests
//! while changing settings, keep hold to the Ripple tab and let a long OK
//! restore the defaults.
use firmware_core::buttons::{
    BACK, DEFAULT_THRESHOLD, Input, LEFT, Navigation, OK, Open, POLL_MS, Page, RIGHT, RippleFocus,
    Tab,
};
use firmware_core::ripple::{Scale, Timebase};
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

const TIME: Page = Page::Open(Open::Ripple(RippleFocus::Time));
const SCALE: Page = Page::Open(Open::Ripple(RippleFocus::Scale));
const CAPTURE: Page = Page::Open(Open::Ripple(RippleFocus::Capture));

#[test]
fn passthrough_can_navigate_to_ripple_and_hold_without_requesting_power() {
    // Prevent the passive monitor from disappearing behind the sink-only
    // tab filter; exercise the real navigation path rather than the allowlist.
    let mut nav = Navigation::new(Page::Tabs(Tab::Graph), Settings::default());
    nav.set_passthrough(true);
    assert_eq!(tap(&mut nav, RIGHT).page, Page::Tabs(Tab::Ripple));
    let focused = tap(&mut nav, OK);
    assert_eq!(focused.page, TIME);
    assert!(!focused.ripple_hold);
    tap(&mut nav, OK);
    assert_eq!(tap(&mut nav, OK).page, CAPTURE);
    assert!(tap(&mut nav, RIGHT).ripple_hold);
    assert_eq!(tap(&mut nav, BACK).page, Page::Tabs(Tab::Ripple));
    assert_eq!(tap(&mut nav, RIGHT).page, Page::Tabs(Tab::Overview));
    // Returning must not show a stale held capture from an earlier visit.
    let returned = tap(&mut nav, LEFT);
    assert_eq!(returned.page, Page::Tabs(Tab::Ripple));
    assert!(!returned.ripple_hold);
}

#[test]
fn ok_moves_focus_and_arrows_change_only_the_focused_setting() {
    let mut nav = Navigation::new(Page::Tabs(Tab::Ripple), Settings::default());
    let focused = tap(&mut nav, OK);
    assert_eq!(focused.page, TIME);
    assert_eq!(focused.ripple_timebase, Timebase::Us50);
    assert_eq!(tap(&mut nav, RIGHT).ripple_timebase, Timebase::Us100);
    assert_eq!(tap(&mut nav, LEFT).ripple_timebase, Timebase::Us50);
    assert_eq!(tap(&mut nav, LEFT).ripple_timebase, Timebase::Us20);
    let wrapped = tap(&mut nav, LEFT);
    assert_eq!(wrapped.ripple_timebase, Timebase::Us500);
    assert_eq!(wrapped.ripple_scale, Scale::Auto);
    assert!(!wrapped.ripple_hold);

    assert_eq!(tap(&mut nav, OK).page, SCALE);
    let locked = tap(&mut nav, RIGHT);
    assert_eq!(locked.ripple_scale, Scale::Mv2);
    assert_eq!(locked.ripple_timebase, Timebase::Us500);
    assert_eq!(tap(&mut nav, LEFT).ripple_scale, Scale::Auto);
    assert_eq!(tap(&mut nav, LEFT).ripple_scale, Scale::Mv1000);

    assert_eq!(tap(&mut nav, OK).page, CAPTURE);
    let held = tap(&mut nav, LEFT);
    assert!(held.ripple_hold);
    assert_eq!(held.ripple_scale, Scale::Mv1000);
    assert!(!tap(&mut nav, RIGHT).ripple_hold);
    assert!(tap(&mut nav, RIGHT).ripple_hold);
    assert_eq!(tap(&mut nav, OK).page, TIME);
    assert_eq!(tap(&mut nav, BACK).page, Page::Tabs(Tab::Ripple));
    assert_eq!(tap(&mut nav, RIGHT).page, Page::Tabs(Tab::Overview));
}

#[test]
fn long_ok_restores_defaults_and_keeps_focus() {
    let mut nav = Navigation::new(Page::Tabs(Tab::Ripple), Settings::default());
    tap(&mut nav, OK);
    tap(&mut nav, RIGHT);
    tap(&mut nav, OK);
    tap(&mut nav, RIGHT);
    tap(&mut nav, OK);
    let changed = tap(&mut nav, RIGHT);
    assert_eq!(
        (
            changed.ripple_timebase,
            changed.ripple_scale,
            changed.ripple_hold
        ),
        (Timebase::Us100, Scale::Mv2, true)
    );
    sample(&mut nav, OK, 600 / POLL_MS);
    // Releasing a long press must not also move the focus.
    let reset = sample(&mut nav, 0, u32::from(DEFAULT_THRESHOLD));
    assert_eq!(reset.page, CAPTURE);
    assert_eq!(
        (reset.ripple_timebase, reset.ripple_scale, reset.ripple_hold),
        (Timebase::Us50, Scale::Auto, false)
    );
}

#[test]
fn long_entry_press_still_just_focuses() {
    let mut nav = Navigation::new(Page::Tabs(Tab::Ripple), Settings::default());
    sample(&mut nav, OK, 600 / POLL_MS);
    let entered = sample(&mut nav, 0, u32::from(DEFAULT_THRESHOLD));
    assert_eq!(entered.page, TIME);
    assert!(!entered.ripple_hold);
}
