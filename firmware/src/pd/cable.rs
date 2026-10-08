//! Cable page: the cable identity, SVIDs and modes, and the SVID selection.

use super::{CableStatus, Observation, Phase, hex, paint_lines, put, sniff_caps_lines};

pub fn paint_cable(
    framebuffer: &mut crate::render::Frame,
    observation: Observation,
    page: u32,
    svid: Option<u16>,
) {
    paint_lines(framebuffer, cable_lines(observation, page, svid));
}

/// Selected SVID's ascending-set index; no bitmap is copied into observations.
pub fn control_cable_selection(
    observation: &mut fusb302_pd::observation::Observation,
    selection: &firmware_core::pd_spr::Selection,
    page: Option<u32>,
    store: Option<&dyn firmware_core::pd_svids::Store>,
) {
    let (Some(page), Some(store), Some(spr)) = (page, store, observation.spr) else {
        return;
    };
    if selection.cable_active()
        || selection.has_pending()
        || spr.cable_modes.status != firmware_core::pd_cable_modes::Status::Ack
    {
        return;
    }
    let Some(index) = cable_svid_index((*observation).into(), page) else {
        return;
    };
    if selection.mode_cursor() == Some(index) {
        return;
    }
    let Some(svid) = store.at(index) else {
        return;
    };
    if spr.cable_modes.mode.svid == svid {
        selection.set_mode_cursor(Some(index));
    } else if selection.request_cable_mode(svid) {
        selection.set_mode_cursor(Some(index));
        if let Some(spr) = observation.spr.as_mut() {
            spr.cable_modes.mode = firmware_core::pd_cable_modes::Modes {
                svid,
                ..Default::default()
            };
        }
    }
}

pub fn cable_svid_index(observation: Observation, page: u32) -> Option<u16> {
    let spr = observation.spr?;
    let report = spr.cable_report?;
    let fields = firmware_core::pd_cable_details::field_count(&report);
    if fields == 0 {
        return None;
    }
    let start = fields.div_ceil(6) + 2;
    let end = start + spr.cable_modes.count as usize * 2;
    let page = page as usize % (end + 3);
    (start..end)
        .contains(&page)
        .then(|| ((page - start) / 2) as u16)
}

/// Raw identity, latest SVID reply and selected mode, plus decoded identity fields.
/// Encoded values use hexadecimal; they are claims, not electrical measurements.
pub fn cable_lines(observation: Observation, page: u32, svid: Option<u16>) -> [[u8; 24]; 8] {
    use firmware_core::{pd_cable_details, pd_cable_modes::Status};
    if let Some(sniff) = observation.sniff {
        return sniff_caps_lines(sniff);
    }
    let mut rows = [[b' '; 24]; 8];
    put(&mut rows[0], b"E-MARKER DETAILS");
    put(&mut rows[7], b"PB2 NEXT PB10 PREV HEX");
    let Some(spr) = observation.spr else {
        put(&mut rows[1], b"NO DISCOVERY YET");
        return rows;
    };
    let Some(report) = spr.cable_report else {
        put(
            &mut rows[1],
            if observation.phase == Phase::Failed {
                b"PD HALTED SEE PD PAGE"
            } else if spr.cable_active {
                b"DISCOVERY IN PROGRESS"
            } else {
                b"HOLD PB10 ON PD 5V PAGE"
            },
        );
        return rows;
    };
    let fields = pd_cable_details::field_count(&report);
    if fields == 0 {
        if report.status == CableStatus::Identity {
            put(&mut rows[1], b"NO VALID IDENTITY");
        } else {
            let (reason, hint) = cable_failure(report.status, spr.cable_modes.sync);
            put(&mut rows[1], reason);
            put(&mut rows[2], hint);
        }
        return rows;
    }
    let identity_pages = fields.div_ceil(6);
    let discovery = spr.cable_modes;
    let svid_pages = 2; // latest raw reply, not a transcript of every list packet
    let extended_start = identity_pages + svid_pages + discovery.count as usize * 2;
    let page = page as usize % (extended_start + 3);
    hex(&mut rows[0], 16, ((page + 1) >> 16) as u16);
    hex(&mut rows[0], 20, (page + 1) as u16);
    if page < identity_pages {
        for row in 0..6 {
            if let Some(field) = pd_cable_details::field(&report, page * 6 + row) {
                let name = field.name.as_bytes();
                put(&mut rows[row + 1], &name[..name.len().min(15)]);
                hex(&mut rows[row + 1], 16, (field.value >> 16) as u16);
                hex(&mut rows[row + 1], 20, field.value as u16);
            }
        }
    } else if page >= extended_start {
        let manufacturer = page == extended_start;
        let second = page == extended_start + 2;
        let info = if manufacturer {
            discovery.manufacturer
        } else if second {
            discovery.second_status
        } else {
            discovery.cable_status
        };
        rows[0].fill(b' ');
        put(
            &mut rows[0],
            if manufacturer {
                b"MANUFACTURER INFO"
            } else if second {
                b"SOP2 CABLE STATUS"
            } else {
                b"ACTIVE CABLE STATUS"
            },
        );
        if second {
            rows[7].fill(b' ');
            put(&mut rows[7], b"SYNC:");
            let caption = status_caption(discovery.second_sync);
            rows[7][5..5 + caption.len()].copy_from_slice(caption);
            if info.status != Status::Ack {
                put(
                    &mut rows[2],
                    match firmware_core::pd_cable::second_controller(&report) {
                        Some(false) => b"NO SOP2 ADVERTISED",
                        None => b"LAYOUT/PRESENCE UNKNOWN",
                        Some(true) if report.revision != fusb302_pd::header::REVISION_3 => {
                            b"PD2 NO GET_STATUS"
                        }
                        Some(true) if observation.phase == Phase::Failed => {
                            b"PD HALTED SEE PD PAGE"
                        }
                        Some(true) => b"SECOND PLUG NO DATA",
                    },
                );
            }
        }
        put(&mut rows[1], status_caption(info.status));
        hex(&mut rows[1], 20, info.header);
        if info.status == Status::Ack {
            if manufacturer {
                put(&mut rows[2], b"VID:---- PID:----");
                hex(
                    &mut rows[2],
                    4,
                    u16::from_le_bytes([info.data[0], info.data[1]]),
                );
                hex(
                    &mut rows[2],
                    13,
                    u16::from_le_bytes([info.data[2], info.data[3]]),
                );
            } else {
                put(
                    &mut rows[2],
                    match info.data[0] {
                        0 => b"T:N/A SHUTDOWN:-",
                        1 => b"T:LOW SHUTDOWN:-",
                        _ => b"T:--C SHUTDOWN:-",
                    },
                );
                if info.data[0] >= 2 {
                    rows[2][2] = b"0123456789ABCDEF"[(info.data[0] >> 4) as usize];
                    rows[2][3] = b"0123456789ABCDEF"[(info.data[0] & 15) as usize];
                }
                rows[2][15] = b'0' + (info.data[1] & 1);
            }
            for (index, &byte) in info.data[..info.count as usize].iter().enumerate() {
                let row = 3 + index / 8;
                let column = 8 + index % 8 * 2;
                rows[row][column] = b"0123456789ABCDEF"[(byte >> 4) as usize];
                rows[row][column + 1] = b"0123456789ABCDEF"[(byte & 15) as usize];
            }
        }
    } else {
        let page = page - identity_pages;
        let (status, objects, count, offset) = if page < svid_pages {
            put(&mut rows[0], b"LAST SVID REPLY");
            hex(&mut rows[0], 20, discovery.count);
            (
                discovery.status,
                discovery.svid_objects.as_slice(),
                discovery.svid_count,
                page * 5,
            )
        } else {
            let mode = &discovery.mode;
            if let Some(svid) = svid {
                put(&mut rows[0], b"SVID:---- MODES SORTED");
                hex(&mut rows[0], 5, svid);
            } else {
                put(&mut rows[0], b"SVID SET REFRESHING");
            }
            let cached = svid.is_some_and(|svid| svid == mode.svid);
            let status = if cached {
                mode.status
            } else {
                Status::NotQueried
            };
            (
                status,
                mode.objects.as_slice(),
                if cached { mode.count } else { 0 },
                ((page - svid_pages) % 2) * 5,
            )
        };
        put(&mut rows[1], status_caption(status));
        if page >= svid_pages && status == Status::NotQueried {
            put(
                &mut rows[1],
                if discovery.status != Status::Ack {
                    b"LIST NOT COMPLETE"
                } else if observation.phase == Phase::Failed {
                    b"PD HALTED SEE PD PAGE"
                } else if spr.cable_active {
                    b"READ IN PROGRESS"
                } else {
                    b"READ NEEDS SAFE FIXED5V"
                },
            );
        }
        // Five words per page leave a status row; no returned word is omitted.
        for row in 0..5 {
            let index = offset + row;
            if index < count as usize && index < objects.len() {
                put(&mut rows[row + 2], b"VDO:--");
                hex(&mut rows[row + 2], 4, index as u16);
                hex(&mut rows[row + 2], 16, (objects[index] >> 16) as u16);
                hex(&mut rows[row + 2], 20, objects[index] as u16);
            }
        }
    }
    rows
}

/// Why an inspection returned no identity, and an optional hint row. The
/// SOP' Soft_Reset result separates a silent cable from an unanswered request.
pub(super) fn cable_failure(
    status: CableStatus,
    sync: firmware_core::pd_cable_modes::Sync,
) -> (&'static [u8], &'static [u8]) {
    use firmware_core::pd_cable_modes::Sync;
    match (status, sync) {
        (CableStatus::Nak, _) => (b"CABLE ID NAK", b""),
        (CableStatus::Busy, _) => (b"CABLE ID BUSY NO RETRY", b""),
        // Sections 7.2/7.3: SinkTxNG or charger traffic; nothing was asked.
        (CableStatus::Deferred, _) => (b"CABLE: CHARGER BUSY", b""),
        // VCONN is only routed to CC2, so nothing is sent with CC2 active.
        (_, Sync::NotSent) => (b"CABLE ON CC2: NOT ASKED", b"TURN CABLE OVER"),
        (_, Sync::NoGoodCrc) => (b"CABLE NO GOODCRC", b"NO E-MARKER OR NO VCONN"),
        (_, Sync::NoAccept) => (b"CABLE RESET NO ACCEPT", b""),
        (_, Sync::Accepted) => (b"CABLE NO ID RESPONSE", b""),
    }
}

fn status_caption(status: firmware_core::pd_cable_modes::Status) -> &'static [u8] {
    use firmware_core::pd_cable_modes::Status;
    match status {
        Status::NotQueried => b"NOT QUERIED",
        Status::Ack => b"ACK",
        Status::Nak => b"NAK",
        Status::Busy => b"BUSY NO RETRY",
        Status::Timeout => b"TIMEOUT NO RETRY",
        Status::Continuing => b"LIST INCOMPLETE",
        Status::NotSupported => b"NOT SUPPORTED",
    }
}
