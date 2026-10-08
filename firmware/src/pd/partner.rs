//! Charger page: what the Source said about itself.

use super::{Observation, decimal3, decimal4, hex, hex8, paint_lines, put};

/// Charger page (page 5): what the Source said about itself in reply to the
/// information requests (`pd_partner`), on three sub-pages (PB2/PB10). These
/// are the Source's claims, not measurements.
pub fn partner_lines(observation: Observation, page: u32) -> [[u8; 24]; 8] {
    use firmware_core::pd_partner::Answer;
    let mut rows = [[b' '; 24]; 8];
    let sub = page % 3;
    let second = sub == 1;
    put(&mut rows[0], b"CHARGER INFO        -/3");
    rows[0][20] = b'1' + sub as u8;
    let Some(spr) = observation.spr.filter(|_| observation.sniff.is_none()) else {
        put(&mut rows[1], b"NO PD SESSION");
        return rows;
    };
    let partner = spr.partner;
    if sub == 2 {
        return status_rows(rows, partner);
    }
    let source = match partner.source_extended {
        Answer::Value(source) => Some(source),
        _ => None,
    };
    if !second {
        put(&mut rows[1], b"REV:");
        match partner.revision {
            Answer::Value(revision) => {
                put(&mut rows[1], b"REV:-.- V-.-");
                rows[1][4] = b'0' + revision.major.min(9);
                rows[1][6] = b'0' + revision.minor.min(9);
                rows[1][9] = b'0' + revision.version_major.min(9);
                rows[1][11] = b'0' + revision.version_minor.min(9);
            }
            other => {
                rows[1][4..4 + answer_caption(&other).len()].copy_from_slice(answer_caption(&other))
            }
        }
        put(&mut rows[2], b"EXT CAPS:");
        let status = answer_caption(&partner.source_extended);
        rows[2][9..9 + status.len()].copy_from_slice(status);
        let Some(source) = source else {
            return rows;
        };
        put(&mut rows[3], b"VID:---- PID:----");
        hex(&mut rows[3], 4, source.vid);
        hex(&mut rows[3], 13, source.pid);
        put(&mut rows[4], b"XID:-------- FW:-- HW:--");
        hex(&mut rows[4], 4, (source.xid >> 16) as u16);
        hex(&mut rows[4], 8, source.xid as u16);
        hex8(&mut rows[4], 16, source.fw_version);
        hex8(&mut rows[4], 22, source.hw_version);
        put(&mut rows[5], b"PDP SPR:---W EPR:---W");
        if let Some(w) = source.spr_pdp_w {
            decimal3(&mut rows[5], 8, w.into());
        }
        if let Some(w) = source.epr_pdp_w {
            decimal3(&mut rows[5], 17, w.into());
        }
        put(&mut rows[6], b"HOLD:---MS LOAD:---");
        decimal3(&mut rows[6], 5, source.holdup_ms.into());
        decimal3(&mut rows[6], 16, source.load_step_ma_per_us);
        put(&mut rows[7], b"COMPL:");
        let mut at = 6;
        for (set, name) in [
            (source.lps, &b"LPS"[..]),
            (source.ps1, b"PS1"),
            (source.ps2, b"PS2"),
        ] {
            if set {
                rows[7][at..at + 3].copy_from_slice(name);
                at += 4;
            }
        }
        if at == 6 {
            rows[7][6..10].copy_from_slice(b"NONE");
        }
        return rows;
    }
    if let Some(source) = source {
        // Peak Current n: overload %, period ms, duty cycle %; * = VBUS may droop 5 %.
        for (index, peak) in source.peak.iter().enumerate() {
            let row = &mut rows[1 + index];
            put(row, b"P- OVL:--- MS:---- D:--");
            row[1] = b'1' + index as u8;
            decimal3(row, 7, peak.overload_percent);
            decimal4(row, 14, peak.period_ms);
            row[21] = b'0' + (peak.duty_percent / 10).min(9);
            row[22] = b'0' + peak.duty_percent % 10;
            if peak.droop {
                row[23] = b'*';
            }
        }
        put(&mut rows[4], b"TOUCH:");
        let mut at = 6;
        for (set, name) in [
            (source.low_touch_current, &b"LOW"[..]),
            (source.ground_pin, b"GND"),
            (source.protective_earth, b"PE"),
        ] {
            if set {
                rows[4][at..at + name.len()].copy_from_slice(name);
                at += name.len() + 1;
            }
        }
        let temp: &[u8] = match source.touch_temp {
            1 => b"T:TS1",
            2 => b"T:TS2",
            _ => b"T:60950",
        };
        rows[4][24 - temp.len()..].copy_from_slice(temp);
        put(&mut rows[5], b"IN:");
        let mut at = 3;
        for (set, name) in [
            (source.external_supply, &b"EXT"[..]),
            (source.external_unconstrained, b"UNC"),
            (source.internal_battery, b"BAT"),
        ] {
            if set {
                rows[5][at..at + 3].copy_from_slice(name);
                at += 4;
            }
        }
        if at == 3 {
            rows[5][3..7].copy_from_slice(b"NONE");
        }
        // Fixed Batteries / Hot Swappable Battery Slots.
        rows[5][16..24].copy_from_slice(b"BATT:-/-");
        rows[5][21] = b'0' + source.fixed_batteries.min(9);
        rows[5][23] = b'0' + source.hot_swap_slots.min(9);
    } else {
        put(&mut rows[1], b"EXT CAPS:");
        let status = answer_caption(&partner.source_extended);
        rows[1][9..9 + status.len()].copy_from_slice(status);
    }
    put(&mut rows[6], b"MFR:");
    match partner.manufacturer {
        Answer::Value(info) => {
            put(&mut rows[6], b"MFR VID:---- PID:----");
            hex(&mut rows[6], 8, info.vid);
            hex(&mut rows[6], 17, info.pid);
            // The Manufacturer String as sent, upper-cased; bytes the panel
            // cannot show become '_'.
            for (cell, &byte) in rows[7].iter_mut().zip(info.name()) {
                *cell = match byte.to_ascii_uppercase() {
                    b @ 0x20..=0x7e => b,
                    _ => b'_',
                };
            }
        }
        other => {
            rows[6][4..4 + answer_caption(&other).len()].copy_from_slice(answer_caption(&other))
        }
    }
    rows
}

/// Charger page wording for the outcome of one information request.
fn answer_caption<T>(answer: &firmware_core::pd_partner::Answer<T>) -> &'static [u8] {
    use firmware_core::pd_partner::Answer;
    match answer {
        Answer::Value(_) => b"OK",
        Answer::Pending => b"WAITING",
        Answer::NotSupported => b"NOT SUPPORTED",
        Answer::Timeout => b"NO REPLY",
        Answer::Deferred => b"BLOCKED",
        Answer::Interrupted => b"RESET",
        Answer::NotSent => b"PD2 N/A",
    }
}

/// Third charger sub-page: Source_Info and the latest Status.
fn status_rows(
    mut rows: [[u8; 24]; 8],
    partner: firmware_core::pd_partner::PartnerInfo,
) -> [[u8; 24]; 8] {
    use firmware_core::pd_partner::Answer;
    put(&mut rows[1], b"SRC INFO:");
    let text = answer_caption(&partner.source_info);
    rows[1][9..9 + text.len()].copy_from_slice(text);
    if let Answer::Value(info) = partner.source_info {
        put(
            &mut rows[2],
            if info.guaranteed {
                b"GUARANTEED  MAX:---W"
            } else {
                b"MANAGED     MAX:---W"
            },
        );
        decimal3(&mut rows[2], 16, info.maximum_w.into());
        put(&mut rows[3], b"NOW:---W REPORTED:---W");
        decimal3(&mut rows[3], 4, info.present_w.into());
        decimal3(&mut rows[3], 18, info.reported_w.into());
        if let Some(second) = info.second {
            // 0.5 W units: whole watts shown.
            put(&mut rows[4], b"GUAR:---W MAX2:---W DPS-");
            decimal3(&mut rows[4], 5, second.guaranteed_half_w / 2);
            decimal3(&mut rows[4], 15, second.maximum_half_w / 2);
            rows[4][23] = if second.dps { b'Y' } else { b'N' };
        }
    }
    put(&mut rows[5], b"STATUS:");
    let text = answer_caption(&partner.status);
    rows[5][7..7 + text.len()].copy_from_slice(text);
    if let Answer::Value(status) = partner.status {
        // Internal temperature; N: Status answers so far (the first, then one per Alert).
        rows[5][10..12].copy_from_slice(b"T:");
        match status.internal_temp {
            0 => rows[5][12..15].copy_from_slice(b"N/A"),
            1 => rows[5][12..15].copy_from_slice(b"<2C"),
            t => {
                decimal3(&mut rows[5], 12, t.into());
                rows[5][15] = b'C';
            }
        }
        rows[5][17..19].copy_from_slice(b"N:");
        decimal3(&mut rows[5], 19, partner.status_reads.into());
        put(&mut rows[6], b"IN:");
        rows[6][3..6].copy_from_slice(match status.external_power {
            1 => b"DC ",
            3 => b"AC ",
            _ => b"INT",
        });
        let mut at = 7;
        for (set, name) in [
            (status.ocp, &b"OCP"[..]),
            (status.otp, b"OTP"),
            (status.ovp, b"OVP"),
            (status.current_limit, b"CL"),
        ] {
            if set {
                rows[6][at..at + name.len()].copy_from_slice(name);
                at += name.len() + 1;
            }
        }
        put(&mut rows[7], b"TEMP:");
        let temp: &[u8] = match status.temperature {
            1 => b"NORMAL",
            2 => b"WARNING",
            3 => b"OVERTEMP",
            _ => b"N/A",
        };
        rows[7][5..5 + temp.len()].copy_from_slice(temp);
        // Power limited by: Cable, other Ports, eXternal power, Events, Temperature.
        if let Some(limits) = status.power_limited {
            rows[7][14..18].copy_from_slice(b"LIM:");
            for (i, letter) in b"CPXET".iter().enumerate() {
                rows[7][18 + i] = if limits & (1 << i) != 0 {
                    *letter
                } else {
                    b'-'
                };
            }
        }
    }
    rows
}

/// Paint the charger page (`partner_lines`).
pub fn paint_partner(framebuffer: &mut crate::render::Frame, observation: Observation, page: u32) {
    paint_lines(framebuffer, partner_lines(observation, page));
}
