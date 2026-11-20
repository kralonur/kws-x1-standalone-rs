#![cfg(feature = "logger")]

use super::*;

pub fn draw_logger_status<D: DrawTarget<Color = Rgb565>>(target: &mut D, message: &str) {
    // Bottom 16 pixels, matching the logger's status strip.
    const HEIGHT: u32 = 16;
    const TOP: i32 = firmware_core::panel::HEIGHT as i32 - HEIGHT as i32;
    Rectangle::new(Point::new(0, TOP), Size::new(WIDTH, HEIGHT))
        .into_styled(PrimitiveStyle::with_fill(BLACK))
        .draw(target)
        .ok();
    text(
        target,
        &PLAIN,
        message,
        Point::new(LEFT, TOP + 13),
        HorizontalAlignment::Left,
        CYAN,
    );
}

pub fn draw_logger_export<D: DrawTarget<Color = Rgb565>>(target: &mut D, ready: bool) {
    if ready {
        notice(
            target,
            "USB LOG EXPORT",
            "Read-only - copy logs to PC",
            CYAN,
        );
    } else {
        notice(target, "USB LOG EXPORT", "Use OFF/PASS at PC 5 V", CYAN);
    }
}
