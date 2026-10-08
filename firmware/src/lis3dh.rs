//! LIS3DH accelerometer used as the tilt sensor: registers and configuration.
//!
//! `LIS` is the LIS3DH data sheet; the README defines the cited revision.

/// Target address with SA0 = 0 (LIS Table 16).
pub const ADDRESS: u8 = 0x18;

/// Register addresses (LIS Table 21).
pub const CTRL_REG1: u8 = 0x20;
pub const CTRL_REG3: u8 = 0x22;
pub const CTRL_REG4: u8 = 0x23;
pub const INT1_CFG: u8 = 0x30;
pub const INT1_THS: u8 = 0x32;

/// CTRL_REG1 ODR = 0101: 100 Hz, HR/normal mode (LIS Tables 29 to 31).
const ODR_100_HZ: u8 = 0b0101 << 4;
/// CTRL_REG1 Yen: only the Y axis is enabled (LIS Table 30).
const YEN: u8 = 1 << 1;
/// CTRL_REG3 I1_IA1: interrupt activity 1 on INT1 (LIS Tables 35 and 36).
const I1_IA1: u8 = 1 << 6;
/// CTRL_REG4 HR: high resolution, FS = 00 (+/-2 g) (LIS Tables 37 and 38).
const HR: u8 = 1 << 3;
/// INT1_THS: 40 x 16 mg = 640 mg at +/-2 g (LIS Tables 58 and 59).
const THRESHOLD: u8 = 40;

/// INT1_CFG AOI = 6D = 1: 6-direction position recognition (LIS Table 55).
const POSITION: u8 = 1 << 7 | 1 << 6;
/// INT1_CFG XLIE and XHIE: X low and X high events (LIS Tables 53 and 54).
const XLIE: u8 = 1 << 0;
const XHIE: u8 = 1 << 1;
/// INT1 on an X low position: the threshold set while upright.
pub const POSITION_X_LOW: u8 = POSITION | XLIE;
/// INT1 on an X high position: the threshold set while flipped.
pub const POSITION_X_HIGH: u8 = POSITION | XHIE;

/// Start-up writes, in order; INT1_CFG is written last, once the rest is set.
pub const INIT: [(u8, u8); 5] = [
    (CTRL_REG1, ODR_100_HZ | YEN),
    (CTRL_REG4, HR),
    (CTRL_REG3, I1_IA1),
    (INT1_THS, THRESHOLD),
    (INT1_CFG, POSITION_X_LOW),
];
