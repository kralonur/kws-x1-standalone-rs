//! FUSB302 CC sampling for the passthrough sniffer, from STATUS0 reads the
//! caller made. The message log and its text are the library's, re-exported here.

pub use pd_protocol::pd_sniff::*;

/// FUSB302 STATUS0 (datasheet "Status0"): BC_LVL[1:0], COMP (CC above the MDAC
/// level), ACTIVITY (BMC transitions; BC_LVL/COMP are not meaningful then).
const BC_LVL: u8 = 0x03;
const COMP: u8 = 0x20;
const ACTIVITY: u8 = 0x40;

/// Rp level from one STATUS0 sample with the sniffing port's own Rd removed.
/// BC_LVL thresholds 0.2/0.66/1.23 V separate the Type-C default/1.5 A/3 A vRd
/// ranges. `None` while BMC activity or COMP (line above the MDAC level, e.g.
/// open Rp) makes the sample unusable, or when BC_LVL reads below 0.2 V.
pub fn rp(status0: u8) -> Option<Rp> {
    if status0 & (ACTIVITY | COMP) != 0 {
        return None;
    }
    match status0 & BC_LVL {
        1 => Some(Rp::Default),
        2 => Some(Rp::Current1A5),
        3 => Some(Rp::Current3A0),
        _ => None,
    }
}

/// No BMC activity during the sample: BC_LVL/COMP describe the line (even
/// when `rp` is `None`, e.g. an open Rp after the sink detached).
pub fn quiet(status0: u8) -> bool {
    status0 & ACTIVITY == 0
}

/// Active CC (1 or 2) from one quiet STATUS0 sample per pin, measured with the
/// MDAC at 2.1 V and no Rd of the sniffing port. Active: Rd-loaded Rp, 0.2 V < CC <= 2.1 V.
/// The other pin must be quiet and either open Rp (COMP) or below 0.2 V.
/// Anything else, including activity on either pin, is ambiguous.
pub fn orientation(status0: [u8; 2]) -> Option<u8> {
    let active = |s: u8| s & (ACTIVITY | COMP) == 0 && s & BC_LVL != 0;
    let idle = |s: u8| s & ACTIVITY == 0 && (s & COMP != 0 || s & BC_LVL == 0);
    match (active(status0[0]), active(status0[1])) {
        (true, false) if idle(status0[1]) => Some(1),
        (false, true) if idle(status0[0]) => Some(2),
        _ => None,
    }
}
