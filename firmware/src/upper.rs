//! The upper code region (`memory.x`): the PD stack runs from flash the
//! update path never writes, programmed by the release image's unpacker.
//! Startup checks it before anything can call into it.

use firmware_image::{
    UPPER_BASE, UPPER_END, UPPER_ID_MAGIC, UPPER_ID_UNSET, UpperRecord, region_hash,
};

/// Length and hash of this build's upper region. `firmware-pack` finds the
/// record by its magic and fills both fields from the linked region's bytes.
#[used]
static UPPER_ID: UpperRecord = UpperRecord {
    magic: UPPER_ID_MAGIC,
    len: UPPER_ID_UNSET,
    hash: UPPER_ID_UNSET,
};

unsafe extern "C" {
    static __supper: u8;
    static __eupper: u8;
}

/// The installed upper region holds exactly this build's code: its linked
/// length matches the packed record and the flash bytes hash to it.
pub fn installed() -> bool {
    // SAFETY: a read of an initialized static; volatile so the placeholder
    // values are never folded in place of what the packer wrote.
    let id = unsafe { core::ptr::read_volatile(&UPPER_ID) };
    let (start, end) = (&raw const __supper as usize, &raw const __eupper as usize);
    let len = end - start;
    if start != UPPER_BASE as usize
        || len > (UPPER_END - UPPER_BASE) as usize
        || id.len as usize != len
    {
        return false;
    }
    // SAFETY: `[UPPER_BASE, UPPER_BASE + len)` is on-chip flash inside the
    // upper region, always readable.
    let bytes = unsafe { core::slice::from_raw_parts(start as *const u8, len) };
    region_hash(bytes) == id.hash
}
