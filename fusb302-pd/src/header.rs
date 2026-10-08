//! USB PD Message Header field masks (USB PD §6.2.1, Table 6.2).

/// Message Type, bits 4:0.
pub const MESSAGE_TYPE: u16 = 0x001f;
/// Specification Revision, bits 7:6.
pub const SPEC_REVISION: u16 = 0x00c0;
/// Port Power Role (SOP) or Cable Plug (SOP'/SOP''), bit 8.
pub const POWER_ROLE: u16 = 0x0100;
/// MessageID, bits 11:9.
pub const MESSAGE_ID: u16 = 0x0e00;
/// Number of Data Objects, bits 14:12.
pub const OBJECT_COUNT: u16 = 0x7000;
/// Extended, bit 15.
pub const EXTENDED: u16 = 0x8000;

/// Message kind: Extended plus Message Type.
pub const KIND: u16 = EXTENDED | MESSAGE_TYPE;
/// Message kind and Number of Data Objects, ignoring roles, revision and ID.
pub const KIND_COUNT: u16 = EXTENDED | OBJECT_COUNT | MESSAGE_TYPE;
/// Number of Data Objects plus Message Type; Extended is not compared.
pub const COUNT_TYPE: u16 = OBJECT_COUNT | MESSAGE_TYPE;
/// Every field except MessageID.
pub const WITHOUT_ID: u16 = !MESSAGE_ID;
/// Every field except MessageID and Specification Revision.
pub const WITHOUT_ID_REVISION: u16 = !(MESSAGE_ID | SPEC_REVISION);

/// Specification Revision field values (Table 6.2): 01b is Revision 2.0,
/// 10b is Revision 3.x.
pub const REVISION_2: u8 = 0b01;
pub const REVISION_3: u8 = 0b10;

/// A Specification Revision field value in place.
pub const fn revision(revision: u8) -> u16 {
    (revision as u16) << 6
}

/// A MessageID in place.
pub const fn message_id(id: u16) -> u16 {
    id << 9
}

/// A Number of Data Objects in place.
pub const fn objects(count: u16) -> u16 {
    count << 12
}

/// Control Message types (USB PD Table 6.4).
pub mod control {
    pub const GOOD_CRC: u16 = 0x01;
    pub const ACCEPT: u16 = 0x03;
    pub const REJECT: u16 = 0x04;
    pub const SOFT_RESET: u16 = 0x0d;
    pub const NOT_SUPPORTED: u16 = 0x10;
    pub const GET_STATUS: u16 = 0x12;
    pub const GET_PPS_STATUS: u16 = 0x14;
}

/// Data Message types (USB PD Table 6.5).
pub mod data {
    pub const REQUEST: u16 = 0x02;
    pub const SINK_CAPABILITIES: u16 = 0x04;
    pub const ALERT: u16 = 0x06;
    pub const REVISION: u16 = 0x0c;
    pub const VENDOR_DEFINED: u16 = 0x0f;
}

/// Extended Message types (USB PD Table 6.47), used with [`EXTENDED`].
pub mod extended {
    pub const STATUS: u16 = 0x02;
    pub const GET_MANUFACTURER_INFO: u16 = 0x06;
    pub const PPS_STATUS: u16 = 0x0c;
    pub const SINK_CAPABILITIES_EXTENDED: u16 = 0x0f;

    /// Extended Message Header fields (USB PD Table 6.48).
    pub mod header {
        /// Chunked, bit 15.
        pub const CHUNKED: u16 = 0x8000;
        /// Reserved, bit 9: the receiver ignores it.
        pub const RESERVED: u16 = 0x0200;
    }
}
