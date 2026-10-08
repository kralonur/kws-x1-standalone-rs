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

// Register bit fields, from the same register map.

pub mod control0 {
    /// Start transmission; self-clearing.
    pub const TX_START: u8 = 0x01;
}

pub mod control1 {
    /// Enable SOP' packets.
    pub const ENSOP1: u8 = 0x01;
    /// Enable SOP'' packets.
    pub const ENSOP2: u8 = 0x02;
    /// Flush the receive FIFO; self-clearing.
    pub const RX_FLUSH: u8 = 0x04;
    /// BIST Mode 01s pattern.
    pub const BIST_MODE2: u8 = 0x10;
}

pub mod control3 {
    /// BIST test mode: clear the RX FIFO after GoodCRC.
    pub const BIST_TMODE: u8 = 0x20;
    /// Send a Hard Reset; self-clearing.
    pub const SEND_HARD_RESET: u8 = 0x40;
}

pub mod reset {
    /// Reset the PD logic only; self-clearing.
    pub const PD_RESET: u8 = 0x02;
}

pub mod status0 {
    /// BC_LVL[1:0]: measured CC level as host current.
    pub const BC_LVL: u8 = 0x03;
    /// COMP: CC above the MDAC level.
    pub const COMP: u8 = 0x20;
    /// ACTIVITY: BMC transitions; BC_LVL/COMP are not meaningful then.
    pub const ACTIVITY: u8 = 0x40;
}

pub mod status1 {
    /// Over-current on VCONN.
    pub const OCP: u8 = 0x01;
    /// Over-temperature.
    pub const OVRTEMP: u8 = 0x02;
    pub const TX_FULL: u8 = 0x04;
    pub const TX_EMPTY: u8 = 0x08;
    pub const RX_FULL: u8 = 0x10;
    pub const RX_EMPTY: u8 = 0x20;
    /// FIFO and fault bits, without the last-packet SOP'/SOP'' flags.
    pub const FIFOS_AND_FAULTS: u8 = RX_EMPTY | RX_FULL | TX_EMPTY | TX_FULL | OVRTEMP | OCP;
    /// Both FIFOs empty and no fault, under [`FIFOS_AND_FAULTS`].
    pub const IDLE: u8 = RX_EMPTY | TX_EMPTY;
}

pub mod interrupta {
    pub const I_HARDRST: u8 = 0x01;
    pub const I_SOFTRST: u8 = 0x02;
    pub const I_TXSENT: u8 = 0x04;
    pub const I_HARDSENT: u8 = 0x08;
    pub const I_RETRYFAIL: u8 = 0x10;
    pub const I_SOFTFAIL: u8 = 0x20;
    pub const I_TOGDONE: u8 = 0x40;
    pub const I_OCP_TEMP: u8 = 0x80;
    /// TXSENT, HARDSENT, RETRYFAIL, SOFTFAIL and TOGDONE: none can be set while
    /// the port only receives and never transmits or toggles.
    pub const TX_OR_TOGGLE: u8 = I_TXSENT | I_HARDSENT | I_RETRYFAIL | I_SOFTFAIL | I_TOGDONE;
}

pub mod interruptb {
    pub const I_GCRCSENT: u8 = 0x01;
}

pub mod interrupt {
    pub const I_COLLISION: u8 = 0x02;
    pub const I_CRC_CHK: u8 = 0x10;
}
