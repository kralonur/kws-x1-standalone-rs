//! FUSB302 RX FIFO framing: one SOP* token byte, then the wire bytes (header,
//! data, CRC). The token gives the address; everything after it is decoded by
//! the controller-neutral `pd_protocol::pd_rx`, re-exported here.
//! Extended reception stays opt-in exactly as in the library.

pub use pd_protocol::pd_rx::*;

/// SOP* address from an RX FIFO token byte. The low five token bits are
/// undefined (FUSB302B datasheet Table 42), not flags.
fn address(token: u8) -> Result<Sop, Error> {
    match token >> 5 {
        7 => Ok(Sop::Partner),
        6 => Ok(Sop::Cable),
        5 => Ok(Sop::CableDoublePrime),
        4 => Ok(Sop::Debug),
        3 => Ok(Sop::DebugDoublePrime),
        _ => Err(Error::InvalidToken),
    }
}

/// Parse one frame from a captured FIFO byte prefix; returns bytes consumed.
/// Caller must stop/resynchronize on errors, not scan arbitrary payload for SOP.
/// CRC-valid is not proof of source identity, conformance or electrical safety.
pub fn decode(bytes: &[u8]) -> Result<(Frame<'_>, usize), Error> {
    decode_with(bytes, false, false)
}

/// Opt-in single-packet extended SOP'/SOP'' messages; partner frames stay unchanged.
pub fn decode_cable_extended(bytes: &[u8]) -> Result<(Frame<'_>, usize), Error> {
    decode_with(bytes, true, false)
}

/// Explicitly selected extended reception, as [`decode_wire_with`].
pub fn decode_with(bytes: &[u8], cable: bool, partner: bool) -> Result<(Frame<'_>, usize), Error> {
    if bytes.len() < 3 {
        return Err(Error::Truncated);
    }
    let (frame, consumed) = decode_wire_with(address(bytes[0])?, &bytes[1..], cable, partner)?;
    Ok((frame, consumed + 1))
}

/// Validated token/header length only, NOT a CRC-checked or complete packet.
/// Allows the FIFO reader to reject unsupported headers before consuming a body.
pub fn fifo_frame_len(prefix: &[u8; 3]) -> Result<usize, Error> {
    prefix_len(prefix, false, false)
}

/// FIFO length for an extended prefix under the explicitly selected addresses.
pub fn fifo_extended_frame_len(
    prefix: &[u8; 5],
    cable: bool,
    partner: bool,
) -> Result<usize, Error> {
    prefix_len(prefix, cable, partner)
}

fn prefix_len(prefix: &[u8], cable: bool, partner: bool) -> Result<usize, Error> {
    let sop = address(prefix[0])?;
    wire_frame_len(sop, &prefix[1..], cable, partner).map(|length| length + 1)
}
