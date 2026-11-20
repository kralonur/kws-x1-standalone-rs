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

pub fn draw_boot_notice<D: DrawTarget<Color = Rgb565>>(target: &mut D, title: &str, detail: &str) {
    notice(target, title, detail, CYAN);
}
