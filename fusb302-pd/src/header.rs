//! USB PD Message Header field masks (USB PD R3.2 §6.2.1, Table 6.2).

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
