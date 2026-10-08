//! FUSB302 register addresses, from the register map in the `fusb302b`
//! driver's `device.ddsl` (FUSB302B datasheet register definitions).

pub const DEVICE_ID: u8 = 0x01;
pub const SWITCHES0: u8 = 0x02;
pub const SWITCHES1: u8 = 0x03;
pub const MEASURE: u8 = 0x04;
pub const SLICE: u8 = 0x05;
pub const CONTROL0: u8 = 0x06;
pub const CONTROL1: u8 = 0x07;
pub const CONTROL2: u8 = 0x08;
pub const CONTROL3: u8 = 0x09;
pub const MASK: u8 = 0x0a;
pub const POWER: u8 = 0x0b;
pub const RESET: u8 = 0x0c;
pub const OCPREG: u8 = 0x0d;
pub const MASKA: u8 = 0x0e;
pub const MASKB: u8 = 0x0f;
pub const CONTROL4: u8 = 0x10;
pub const STATUS0A: u8 = 0x3c;
pub const STATUS1A: u8 = 0x3d;
pub const INTERRUPTA: u8 = 0x3e;
pub const INTERRUPTB: u8 = 0x3f;
pub const STATUS0: u8 = 0x40;
pub const STATUS1: u8 = 0x41;
pub const INTERRUPT: u8 = 0x42;
pub const FIFOS: u8 = 0x43;

/// The read-clear interrupt registers, in the order they are acknowledged.
pub const INTERRUPTS: [u8; 3] = [INTERRUPTA, INTERRUPTB, INTERRUPT];
