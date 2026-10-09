//! FUSB302B register map, bit fields and TX FIFO tokens.
//!
//! `DS` is the FUSB302B data sheet; the README defines the cited revision.
//! Addresses are from DS Table 16; each bit-field module cites its register's
//! table. Only fields this crate uses are named.

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
/// TX FIFO on write, RX FIFO on read; no address auto-increment (DS Table 40).
pub const FIFOS: u8 = 0x43;

/// The read-clear interrupt registers, in the order they are acknowledged.
pub const INTERRUPTS: [u8; 3] = [INTERRUPTA, INTERRUPTB, INTERRUPT];

/// DS Table 18.
pub mod switches0 {
    /// Device pull-down (Rd) on CC1.
    pub const PDWN1: u8 = 0x01;
    /// Device pull-down (Rd) on CC2.
    pub const PDWN2: u8 = 0x02;
    /// Measure block monitors CC1.
    pub const MEAS_CC1: u8 = 0x04;
    /// Measure block monitors CC2.
    pub const MEAS_CC2: u8 = 0x08;
    /// VCONN current on CC2.
    pub const VCONN_CC2: u8 = 0x20;
}

/// DS Table 19.
pub mod switches1 {
    /// BMC transmit driver on CC1.
    pub const TXCC1: u8 = 0x01;
    /// BMC transmit driver on CC2.
    pub const TXCC2: u8 = 0x02;
    /// Send GoodCRC automatically for a message with a good CRC.
    pub const AUTO_CRC: u8 = 0x04;
    /// SPECREV[1:0] = 01: GoodCRC carries Specification Revision 2.0.
    pub const SPECREV_2_0: u8 = 0x20;
}

/// DS Table 22.
pub mod control0 {
    /// Start the transmitter from the TX FIFO; self-clearing.
    pub const TX_START: u8 = 0x01;
}

/// DS Table 23.
pub mod control1 {
    /// Receive SOP' packets.
    pub const ENSOP1: u8 = 0x01;
    /// Receive SOP'' packets.
    pub const ENSOP2: u8 = 0x02;
    /// Flush the receive FIFO; self-clearing.
    pub const RX_FLUSH: u8 = 0x04;
    /// Send the BIST Mode 01s pattern.
    pub const BIST_MODE2: u8 = 0x10;
}

/// DS Table 25.
pub mod control3 {
    /// Retry automatically when no GoodCRC arrives.
    pub const AUTO_RETRY: u8 = 0x01;
    /// BIST mode: clear the RX FIFO right after sending GoodCRC.
    pub const BIST_TMODE: u8 = 0x20;
    /// Send a Hard Reset; self-clearing.
    pub const SEND_HARD_RESET: u8 = 0x40;

    /// N_RETRIES[1:0] (bits 2:1): retries after the first transmission, 0..=3.
    pub const fn n_retries(retries: u8) -> u8 {
        (retries & 0b11) << 1
    }
}

/// DS Table 27.
pub mod power {
    /// PWR[0]: bandgap and wake circuit.
    pub const BANDGAP_WAKE: u8 = 0x01;
    /// PWR[1]: receiver and the measure block's current references.
    pub const RECEIVER: u8 = 0x02;
    /// PWR[2]: measure block.
    pub const MEASURE: u8 = 0x04;
    /// PWR[3]: internal oscillator.
    pub const OSCILLATOR: u8 = 0x08;
}

/// DS Table 28.
pub mod reset {
    /// Reset only the PD transmitter and receiver logic; self-clearing.
    pub const PD_RESET: u8 = 0x02;
}

/// DS Table 37.
pub mod status0 {
    /// BC_LVL[1:0]: measured CC level as host current.
    pub const BC_LVL: u8 = 0x03;
    /// COMP: CC above the MDAC level.
    pub const COMP: u8 = 0x20;
    /// ACTIVITY: BMC transitions; BC_LVL/COMP are not meaningful then.
    pub const ACTIVITY: u8 = 0x40;
}

/// DS Table 38.
pub mod status1 {
    /// Over-current or short on the VCONN switch.
    pub const OCP: u8 = 0x01;
    /// Device temperature too high.
    pub const OVRTEMP: u8 = 0x02;
    pub const TX_FULL: u8 = 0x04;
    pub const TX_EMPTY: u8 = 0x08;
    pub const RX_FULL: u8 = 0x10;
    pub const RX_EMPTY: u8 = 0x20;
    /// FIFO and fault bits, without the last-packet SOP'/SOP'' flags.
    pub const FIFOS_AND_FAULTS: u8 = RX_EMPTY | RX_FULL | TX_EMPTY | TX_FULL | OVRTEMP | OCP;
    /// Both FIFOs empty and no fault, under [`FIFOS_AND_FAULTS`]; also the
    /// register's reset value.
    pub const IDLE: u8 = RX_EMPTY | TX_EMPTY;
}

/// DS Table 35.
pub mod interrupta {
    /// Hard Reset ordered set received.
    pub const I_HARDRST: u8 = 0x01;
    /// Soft_Reset packet received.
    pub const I_SOFTRST: u8 = 0x02;
    /// A sent packet was acknowledged with GoodCRC.
    pub const I_TXSENT: u8 = 0x04;
    /// Hard Reset ordered set sent.
    pub const I_HARDSENT: u8 = 0x08;
    /// Automatic packet retries failed.
    pub const I_RETRYFAIL: u8 = 0x10;
    /// Automatic Soft_Reset retries failed.
    pub const I_SOFTFAIL: u8 = 0x20;
    /// Toggle stopped because a device was detected.
    pub const I_TOGDONE: u8 = 0x40;
    /// VCONN over-current or over-temperature.
    pub const I_OCP_TEMP: u8 = 0x80;
    /// TXSENT, HARDSENT, RETRYFAIL, SOFTFAIL and TOGDONE: none can be set while
    /// the port only receives and never transmits or toggles.
    pub const TX_OR_TOGGLE: u8 = I_TXSENT | I_HARDSENT | I_RETRYFAIL | I_SOFTFAIL | I_TOGDONE;
}

/// DS Table 36.
pub mod interruptb {
    /// GoodCRC sent for an incoming packet with a correct CRC.
    pub const I_GCRCSENT: u8 = 0x01;
}

/// DS Table 39.
pub mod interrupt {
    /// Transmit attempted during CC activity; not sent, the packet is received.
    pub const I_COLLISION: u8 = 0x02;
    /// ACTIVITY changed: BMC transitions started or stopped on the measured CC.
    pub const I_ACTIVITY: u8 = 0x40;
    /// CRC_CHK newly valid: an incoming packet's CRC was checked.
    pub const I_CRC_CHK: u8 = 0x10;
}

/// TX FIFO tokens (DS Table 41). Token codes are not the USB PD K-codes.
pub mod tx_token {
    /// Sync-1 symbol.
    pub const SOP1: u8 = 0x12;
    /// Sync-2 symbol.
    pub const SOP2: u8 = 0x13;
    /// Sync-3 symbol.
    pub const SOP3: u8 = 0x1b;
    /// EOP symbol.
    pub const EOP: u8 = 0x14;
    /// Followed by N packed data bytes, N in the five low bits (2..=30).
    pub const PACKSYM: u8 = 0x80;
    /// Insert the hardware-calculated CRC.
    pub const JAM_CRC: u8 = 0xff;
    /// Turn off the transmit driver, typically right after EOP.
    pub const TXOFF: u8 = 0xfe;
    /// Start the transmitter, like TX_START, as the last byte of a FIFO write.
    pub const TXON: u8 = 0xa1;
}
