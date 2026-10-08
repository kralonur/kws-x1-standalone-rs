//! Panel geometry, initialization sequence, and pixel framing (pure logic).
//!
//! The command sequence, the 240x135 geometry, the window offsets, and the memory
//! access control value are transcribed from the original firmware: its window
//! setter adds the X and Y offsets from its startup data, whose defaults are
//! X offset 40, Y offset 53, and MADCTL `0x60`.
//!
//! `ST` is the ST7789V controller data sheet; the README defines the cited revision.
#![allow(dead_code)]

pub const WIDTH: u16 = 240;
pub const HEIGHT: u16 = 135;
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PanelError {
    /// The requested rectangle does not fit inside the panel.
    OutOfBounds,
    /// The output buffer cannot hold the requested pixel run.
    ShortBuffer,
}

/// Column and row range in panel coordinates, offsets already applied.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Window {
    pub x0: u16,
    pub x1: u16,
    pub y0: u16,
    pub y1: u16,
}

impl Window {
    pub fn width(&self) -> u16 {
        self.x1 - self.x0 + 1
    }

    pub fn height(&self) -> u16 {
        self.y1 - self.y0 + 1
    }

    /// Bytes needed to fill this window with RGB565 pixels.
    pub fn pixel_bytes(&self) -> usize {
        self.width() as usize * self.height() as usize * 2
    }
}

/// Bounds-check a rectangle and apply the panel offsets.
pub fn window(x: u16, y: u16, width: u16, height: u16) -> Result<Window, PanelError> {
    if width == 0 || height == 0 {
        return Err(PanelError::OutOfBounds);
    }
    let x1 = x as u32 + width as u32 - 1;
    let y1 = y as u32 + height as u32 - 1;
    if x1 >= WIDTH as u32 || y1 >= HEIGHT as u32 {
        return Err(PanelError::OutOfBounds);
    }
    Ok(Window {
        x0: X_OFFSET + x,
        x1: X_OFFSET + x1 as u16,
        y0: Y_OFFSET + y,
        y1: Y_OFFSET + y1 as u16,
    })
}

/// Pack 8-bit channels into RGB565.
pub fn rgb565(red: u8, green: u8, blue: u8) -> u16 {
    ((red as u16 & 0xf8) << 8) | ((green as u16 & 0xfc) << 3) | (blue as u16 >> 3)
}

/// Write `count` copies of a colour as big-endian RGB565 pairs.
///
/// The panel is configured for 8-bit transfers, most significant bit first, so each
/// pixel is two bytes with the high byte first.
pub fn write_pixels(color: u16, count: usize, out: &mut [u8]) -> Result<(), PanelError> {
    if out.len() < count * 2 {
        return Err(PanelError::ShortBuffer);
    }
    let bytes = color.to_be_bytes();
    for index in 0..count {
        out[index * 2] = bytes[0];
        out[index * 2 + 1] = bytes[1];
    }
    Ok(())
}
