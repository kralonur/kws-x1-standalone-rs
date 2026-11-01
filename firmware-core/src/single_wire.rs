//! Shared application envelope and measured-voltage checks for the
//! single-wire output protocols: SCP, FCP and AFC.

/// Application power-path envelope: 4–30 V input; negotiated current is
/// limited to 5 A, as on the existing PD trigger. Adapter ratings can only
/// narrow this envelope, never expand it.
pub const MIN_MV: u16 = 4_000;
pub const MAX_MV: u16 = 30_000;
pub const MAX_MA: u16 = 5_000;

/// Application VBUS acceptance tolerance, ±5% of the requested voltage.
/// Unlike QC's halfway heuristic, an SCP, FCP or AFC output must reach its
/// setpoint.
const VBUS_TOLERANCE_PERCENT: u32 = 5;

pub fn followed(mv: u16, vbus_mv: u16) -> bool {
    u32::from(vbus_mv.abs_diff(mv)) * 100 <= u32::from(mv) * VBUS_TOLERANCE_PERCENT
        && vbus_mv <= MAX_MV
}
