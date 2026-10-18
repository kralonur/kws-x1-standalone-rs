//! The port scan gesture must never become a charger restart or passive-mode TX.
use firmware_core::buttons::{
    DEFAULT_THRESHOLD, Detail, Navigation, OK, POLL_MS, Page, PdCommand, Tab,
};
use firmware_core::settings::{NextBoot, Settings};

#[test]
fn only_an_active_session_can_request_a_port_scan_without_restarting() {
    for (selectable, passthrough) in [(true, false), (false, false), (true, true)] {
        let mut nav = Navigation::new(Page::Tabs(Tab::detail(Detail::Pd)), Settings::default());
        nav.set_selectable(selectable);
        nav.set_passthrough(passthrough);
        let mut commands = Vec::new();
        // 600 ms exceeds the public 500 ms hold plus debounce; then release.
        for (pressed, samples) in [(OK, 600 / POLL_MS), (0, u32::from(DEFAULT_THRESHOLD))] {
            for _ in 0..samples {
                let (input, command) = nav.update(pressed);
                assert_eq!(input.settings.next_boot, NextBoot::Normal);
                if let Some(command) = command {
                    commands.push(command);
                }
            }
        }
        assert_eq!(
            commands,
            if selectable && !passthrough {
                vec![PdCommand::QueryPort]
            } else {
                vec![]
            }
        );
    }
}
