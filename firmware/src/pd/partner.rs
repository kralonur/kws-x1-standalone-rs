//! Charger page: what the Source said about itself.

use super::{Observation, decimal3, decimal4, hex, hex8, put};

/// Sections of the charger list: with a PD session the capabilities (two
/// sections, the second often empty), the three answer sections and, for a
/// Source that reports a Battery, a battery section; else one.
pub fn partner_sections(observation: Observation) -> u32 {
    match observation.spr.filter(|_| observation.sniff.is_none()) {
        Some(spr) => CAPABILITY_SECTIONS + 3 + u32::from(reports_battery(spr.partner)),
        None => 1,
    }
}

/// The Source's extended capabilities report a Battery, so the battery
/// requests are asked (`pd_partner::PartnerInfo::battery_query`).
fn reports_battery(partner: firmware_core::pd_partner::PartnerInfo) -> bool {
    matches!(partner.source_extended, firmware_core::pd_partner::Answer::Value(source)
        if source.first_battery().is_some())
}

/// Section `page` of [`partner_sections`]: what the Source offers, then what
/// it said about itself in reply to the information requests (`pd_partner`).
/// These are the Source's claims, not measurements.
pub fn partner_lines(observation: Observation, page: u32) -> [[u8; 24]; 8] {
    use firmware_core::pd_partner::Answer;
    if page < CAPABILITY_SECTIONS {
        let rows = capability_rows(observation);
        let mut section = [[b' '; 24]; 8];
        section.copy_from_slice(&rows[page as usize * 8..][..8]);
        return section;
    }
    let mut rows = [[b' '; 24]; 8];
    let sub = page - CAPABILITY_SECTIONS;
    let second = sub == 1;
    put(
        &mut rows[0],
        match sub {
            0 => b"IDENTITY",
            1 => b"PEAKS AND MAKER",
            2 => b"POWER STATUS",
            _ => b"BATTERY",
        },
    );
    let Some(spr) = observation.spr.filter(|_| observation.sniff.is_none()) else {
        put(&mut rows[1], b"NO PD SESSION");
        return rows;
    };
    let partner = spr.partner;
    match sub {
        2 => return status_rows(rows, partner),
        3 => return battery_rows(rows, partner),
        _ => {}
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
/// EPR Mode Capable, bit 23 of the first (vSafe5V Fixed Supply) PDO (Table 6.9).
const EPR_MODE_CAPABLE: u32 = 1 << 23;
/// Fixed Supply PDO type, bits 31..30 = 00b (Table 6.7).
const PDO_TYPE_SHIFT: u32 = 30;
/// EPR Capabilities: the 5V Fixed Supply PDO and up to 10 more (section 6.5.15).
const EPR_OBJECTS: usize = firmware_core::pd_spr::EPR_OBJECTS;

/// Sections the capability list spans: a heading, up to 11 objects (the
/// EPR answer, section 6.5.15) and two note rows.
pub const CAPABILITY_SECTIONS: u32 = 2;
/// Where the reset cause follows the 12-character `CAPABILITIES` heading.
const TEST_RESET_COLUMN: usize = 13;

/// Everything the charger offers: the EPR answer once received (its first
/// seven positions repeat the SPR offer, section 6.5.15.1), else the latest
/// Source_Capabilities. A `*` marks a fixed object the answer rates above the
/// live offer: the charger gives that current only with a 5 A cable.
fn capability_rows(observation: Observation) -> [[u8; 24]; 16] {
    use firmware_core::pd_sniff::describe_pdo;
    use firmware_core::pd_spr::EprOffer;
    let mut rows = [[b' '; 24]; 16];
    put(&mut rows[0], b"CAPABILITIES");
    // How the test boot started: POWER alone is the restart's power cycle.
    if observation.cable_test {
        crate::render::reset_names(&mut rows[0][TEST_RESET_COLUMN..], observation.reset_flags);
    }
    let Some(spr) = observation.spr else {
        put(&mut rows[1], b"NO PD SESSION");
        return rows;
    };
    let Some(caps) = observation.capabilities.filter(|caps| caps.count > 0) else {
        put(&mut rows[1], b"NO OFFER YET");
        return rows;
    };
    let live = &caps.objects[..usize::from(caps.count)];
    let objects = match spr.epr_offer {
        EprOffer::Received(answer) => answer,
        _ => {
            let mut objects = [0; EPR_OBJECTS];
            for (object, advertised) in objects.iter_mut().zip(live) {
                *object = advertised.raw;
            }
            objects
        }
    };
    let mut next = 1;
    let mut marked = false;
    for (index, &raw) in objects.iter().enumerate().filter(|(_, raw)| **raw != 0) {
        rows[next] = describe_pdo(index, raw);
        if live
            .get(index)
            .is_some_and(|advertised| above_live(raw, advertised.raw))
        {
            rows[next][23] = b'*';
            marked = true;
        }
        next += 1;
    }
    if marked {
        put(&mut rows[next], b"* ONLY WITH 5A CABLE");
        next += 1;
    }
    // A long OK runs the full scan (`NextBoot::ScanQuickCharge`): Quick
    // Charge, then this list again with the EPR offer and, with the meter's
    // plug in the charger, as the 5 A cable test. Its PD part lists it all.
    let plug_in_charger = observation.listen.is_some();
    let scanned = observation.cable_test || observation.scan_pd;
    let note: &[u8] = if plug_in_charger && observation.cable_test_skipped {
        b"5A TEST DID NOT START"
    } else if let Some(notice) = super::epr_notice(spr.epr_state) {
        notice.as_bytes()
    } else if !scanned {
        b"HOLD OK: FULL RESCAN"
    } else if !epr_capable(live[0].raw) || spr.peer_revision != Some(3) {
        b""
    } else {
        match spr.epr_offer {
            EprOffer::NotAsked => b"HOLD OK FOR ALL",
            EprOffer::Asked => b"ASKING CHARGER",
            EprOffer::NoAnswer => b"NO ANSWER, HOLD OK AGAIN",
            EprOffer::NotSent => b"LINE BUSY, HOLD OK AGAIN",
            EprOffer::Received(_) => b"",
        }
    };
    put(&mut rows[next], note);
    // The test locks this page whether or not the plug sits in the charger.
    let follow: &[u8] = match (plug_in_charger, observation.cable_test) {
        (_, true) => b"BACK ENDS 5A CABLE TEST",
        (true, false) if observation.cable_test_skipped => b"HOLD OK TO TRY AGAIN",
        _ if scanned => b"",
        (true, false) => b"QC, 5A TEST; RESTARTS",
        (false, false) => b"QC SCAN; RESTARTS",
    };
    let follow_row = next + usize::from(!note.is_empty());
    put(&mut rows[follow_row], follow);
    if spr.epr_state == firmware_core::pd_spr::EprState::Active {
        let last = rows.len() - 1;
        let row = &mut rows[last];
        put(row, b"KEEP:---- GAP:----MS");
        // Saturate counters to the four decimal columns reserved by the label.
        const DISPLAY_MAX: u16 = 9999;
        super::decimal4(row, 5, spr.epr_keep_alives.min(DISPLAY_MAX));
        super::decimal4(row, 13, spr.epr_max_silence_ms.min(DISPLAY_MAX));
    }
    rows
}

/// Both Fixed Supply objects at the same voltage, `answer` with the higher
/// Maximum Current (Table 6.9: bits 19..10 voltage, 9..0 current).
fn above_live(answer: u32, live: u32) -> bool {
    const VOLTAGE: u32 = 0x3ff << 10;
    const CURRENT: u32 = 0x3ff;
    answer >> PDO_TYPE_SHIFT == 0
        && live >> PDO_TYPE_SHIFT == 0
        && answer & VOLTAGE == live & VOLTAGE
        && answer & CURRENT > live & CURRENT
}

fn epr_capable(first: u32) -> bool {
    first >> PDO_TYPE_SHIFT == 0 && first & EPR_MODE_CAPABLE != 0
}

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

/// Third charger section: Source_Info and the latest Status.
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

/// Fourth charger section: the first Battery the Source reports, as its
/// Battery_Capabilities and Battery_Status describe it (capacities in 0.1 Wh).
fn battery_rows(
    mut rows: [[u8; 24]; 8],
    partner: firmware_core::pd_partner::PartnerInfo,
) -> [[u8; 24]; 8] {
    use firmware_core::pd_partner::{Answer, Capacity, Charging};
    // "----.-WH" from 0.1 Wh, or "UNKNOWN".
    let capacity = |row: &mut [u8; 24], at: usize, capacity: Option<Capacity>| match capacity {
        Some(Capacity::DeciWattHours(tenths)) => {
            row[at..at + 8].copy_from_slice(b"----.-WH");
            decimal4(row, at, tenths / 10);
            row[at + 5] = b'0' + (tenths % 10) as u8;
        }
        Some(Capacity::Unknown) => row[at..at + 7].copy_from_slice(b"UNKNOWN"),
        None => row[at..at + 4].copy_from_slice(b"NONE"),
    };
    put(&mut rows[1], b"CAPS:");
    let text = answer_caption(&partner.battery_capabilities);
    rows[1][5..5 + text.len()].copy_from_slice(text);
    if let Answer::Value(caps) = partner.battery_capabilities {
        if caps.invalid_reference {
            put(&mut rows[2], b"NO SUCH BATTERY");
        } else {
            put(&mut rows[2], b"VID:---- PID:----");
            hex(&mut rows[2], 4, caps.vid);
            hex(&mut rows[2], 13, caps.pid);
            put(&mut rows[3], b"DESIGN:");
            capacity(&mut rows[3], 9, caps.design);
            put(&mut rows[4], b"FULL:");
            capacity(&mut rows[4], 9, caps.last_full);
        }
    }
    put(&mut rows[5], b"STATUS:");
    let text = answer_caption(&partner.battery_status);
    rows[5][7..7 + text.len()].copy_from_slice(text);
    if let Answer::Value(status) = partner.battery_status {
        if status.invalid_reference {
            put(&mut rows[6], b"NO SUCH BATTERY");
        } else {
            put(&mut rows[6], b"CHARGE:");
            capacity(&mut rows[6], 9, Some(status.present));
            put(
                &mut rows[7],
                match status.battery {
                    None => b"NOT PRESENT",
                    Some(Some(Charging::Charging)) => b"CHARGING",
                    Some(Some(Charging::Discharging)) => b"DISCHARGING",
                    Some(Some(Charging::Idle)) => b"IDLE",
                    Some(None) => b"PRESENT",
                },
            );
        }
    }
    rows
}
