//! Panel geometry and initialization sequence (pure logic).
//!
//! The command sequence, the 240x135 geometry, the window offsets, and the memory
//! access control value are transcribed from the original firmware: its window
//! setter adds the X and Y offsets from its startup data, whose defaults are
//! X offset 40, Y offset 53, and MADCTL `0x60`.
//!
//! `ST` is the ST7789V controller data sheet; the README defines the cited revision.

pub const WIDTH: u16 = 240;
pub const HEIGHT: u16 = 135;
/// Pixels in one full frame.
pub const PIXELS: usize = WIDTH as usize * HEIGHT as usize;
/// Column offset applied by the window setter (original default).
pub const X_OFFSET: u16 = 40;
/// Row offset applied by the window setter (original default).
pub const Y_OFFSET: u16 = 53;
/// Memory access control value the original firmware sends with command `0x36`: MX | MV,
/// normal landscape (ST 9.1.28).
pub const MADCTL: u8 = 0x60;
/// MY | MV: the 180-degree inverted landscape (ST 9.1.28).
pub const MADCTL_INVERTED: u8 = 0xa0;
/// Controller rows addressed in landscape: 240 RGB source outputs (ST 2).
pub const CONTROLLER_ROWS: u16 = 240;
/// Row offset when inverted: MY mirrors the window inside the controller rows.
pub const Y_OFFSET_INVERTED: u16 = CONTROLLER_ROWS - HEIGHT - Y_OFFSET;

/// Original panel initialization: command byte, then its data bytes.
pub const INIT: &[(u8, &[u8])] = &[
    (CMD_SLEEP_OUT, &[]),
    (CMD_PIXEL_FORMAT, &[0x05]),
    (0xc5, &[0x1a]),
    (0xb2, &[0x05, 0x05, 0x00, 0x33, 0x33]),
    (0xb7, &[0x05]),
    (0xbb, &[0x3f]),
    (0xc0, &[0x2c]),
    (0xc2, &[0x01]),
    (0xc3, &[0x0f]),
    (0xc4, &[0x20]),
    (0xc6, &[0x01]),
    (0xd0, &[0xa4, 0xa1]),
    (0xe8, &[0x03]),
    (0xe9, &[0x09, 0x09, 0x08]),
    (
        0xe0,
        &[
            0xd0, 0x05, 0x09, 0x09, 0x08, 0x14, 0x28, 0x33, 0x3f, 0x07, 0x13, 0x14, 0x28, 0x30,
        ],
    ),
    (
        0xe1,
        &[
            0xd0, 0x05, 0x09, 0x09, 0x08, 0x03, 0x24, 0x32, 0x32, 0x3b, 0x14, 0x13, 0x28, 0x2f,
        ],
    ),
    (CMD_INVERSION_ON, &[]),
];

/// Command codes (ST 9.1).
pub const CMD_SLEEP_OUT: u8 = 0x11;
pub const CMD_INVERSION_ON: u8 = 0x21;
pub const CMD_DISPLAY_ON: u8 = 0x29;
pub const CMD_COLUMN_ADDRESS: u8 = 0x2a;
pub const CMD_ROW_ADDRESS: u8 = 0x2b;
pub const CMD_MEMORY_WRITE: u8 = 0x2c;
pub const CMD_MEMORY_ACCESS_CONTROL: u8 = 0x36;
pub const CMD_PIXEL_FORMAT: u8 = 0x3a;
/// Wait after Sleep Out, in milliseconds: commands need 5 ms and Sleep In needs
/// 120 ms (ST 9.1.12); the longer wait covers both.
pub const SLEEP_OUT_MS: u32 = 120;
