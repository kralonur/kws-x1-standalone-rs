//! INA226 current and bus voltage monitor: registers, start-up configuration
//! and conversions.
//!
//! `INA` is the INA226 data sheet; the README defines the cited revision.

/// Target address with A1 = A0 = GND (INA Table 6-2).
pub const ADDRESS: u8 = 0x40;

/// Register pointers (INA Table 7-1).
pub const CONFIGURATION: u8 = 0x00;
pub const BUS_VOLTAGE: u8 = 0x02;
pub const CURRENT: u8 = 0x04;
pub const CALIBRATION: u8 = 0x05;
pub const MASK_ENABLE: u8 = 0x06;

/// Configuration bit 14: reserved, 1 at power-on; written unchanged (INA Table 7-2).
const CONFIG_RESERVED: u16 = 1 << 14;
/// AVG = 011: 64 averages (INA Table 7-3).
const AVG_64: u16 = 0b011 << 9;
/// VBUSCT = 100: 1.1 ms bus voltage conversion (INA Table 7-4).
const VBUSCT_1100_US: u16 = 0b100 << 6;
/// VSHCT = 100: 1.1 ms shunt voltage conversion (INA Table 7-5).
const VSHCT_1100_US: u16 = 0b100 << 3;
/// MODE = 111: shunt and bus, continuous (INA Table 7-6).
const MODE_CONTINUOUS: u16 = 0b111;
/// Mask/Enable CNVR, bit 10: Alert follows Conversion Ready (INA Table 7-12).
const CNVR: u16 = 1 << 10;
/// CAL = 0.00512 / (Current_LSB x R_shunt) (INA Equation 1): 2048 gives a
/// 0.5 mA Current_LSB with a 5 mOhm shunt.
const CALIBRATION_VALUE: u16 = 2048;

/// Start-up writes, in order: configuration, alert source, then calibration.
pub const INIT: [(u8, u16); 3] = [
    (
        CONFIGURATION,
        CONFIG_RESERVED | AVG_64 | VBUSCT_1100_US | VSHCT_1100_US | MODE_CONTINUOUS,
    ),
    (MASK_ENABLE, CNVR),
    (CALIBRATION, CALIBRATION_VALUE),
];

/// Current register counts per milliamp, from the 0.5 mA Current_LSB set by
/// [`CALIBRATION_VALUE`].
pub const COUNTS_PER_MA: u32 = 2;

/// Bus Voltage register to millivolts: fixed 1.25 mV LSB (INA Table 7-8).
pub const fn bus_millivolts(raw: u16) -> u32 {
    raw as u32 * 5 / 4
}
