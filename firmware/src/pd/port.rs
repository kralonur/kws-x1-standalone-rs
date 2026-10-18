//! Explicit SOP partner scan; separate from cable discovery and charger scans.
use super::{Observation, put};
use firmware_core::pd_partner::{Answer, PartnerInfo, PortInfo};
use firmware_core::pd_sniff::{Row, SINK_PDO_COLUMNS};
use usbpd::protocol_layer::message::data::{
    source_capabilities::FixedSupply,
    vendor_defined::{ProductVDO, VdmHeaderStructured, VdmIdentityHeader},
};

type Rows = [[u8; SINK_PDO_COLUMNS]; 8];

pub fn widen(section: [[u8; 24]; 8]) -> Rows {
    let mut rows = [[b' '; SINK_PDO_COLUMNS]; 8];
    for (dst, src) in rows.iter_mut().zip(section) {
        dst[..src.len()].copy_from_slice(&src);
    }
    rows
}

#[derive(Clone, Copy)]
enum Section {
    Summary,
    Sink,
    Flags,
    Identity,
    Ufp,
}

fn partner(observation: Observation) -> Option<PartnerInfo> {
    observation
        .spr
        .filter(|_| {
            observation.sniff.is_none()
                && observation.capabilities.is_some_and(|caps| caps.count > 0)
        })
        .map(|spr| spr.partner)
}

/// The partner's first Source PDO when it is the Fixed 5V PDO it must be
/// (Table 6.8).
fn source_fixed(observation: Observation) -> Option<FixedSupply> {
    let raw = observation.capabilities?.objects[0].raw;
    (raw >> PDO_TYPE_SHIFT == 0).then_some(FixedSupply(raw))
}

/// Fixed 5V Source PDO capability bits (Table 6.8).
const SOURCE_FLAGS: [(u32, &[u8]); 7] = [
    (29, b"DRP"),
    (28, b"USB SUSPEND"),
    (27, b"UNCONSTRAINED"),
    (26, b"USB DATA"),
    (25, b"DRD"),
    (24, b"UNCHUNKED"),
    (23, b"EPR"),
];
/// Fixed 5V Sink PDO capability bits (Table 6.9).
const SINK_FLAGS: [(u32, &[u8]); 5] = [
    (29, b"DRP"),
    (28, b"HIGHER CAP"),
    (27, b"UNCONSTRAINED"),
    (26, b"USB DATA"),
    (25, b"DRD"),
];
/// Fast Role Swap required USB Type-C current, Fixed 5V Sink PDO bits 24..23
/// (Table 6.9).
const FAST_ROLE_SWAP_SHIFT: u32 = 23;
/// UFP VDO Alternate Modes bits 5..3, as read by `alternate_modes` (Table
/// 6.40): TBT3, modes that reconfigure the connector's signals, modes that do
/// not.
const ALT_MODES: [(u32, &[u8]); 3] = [(0, b"TBT3"), (1, b"RECONFIG"), (2, b"NO RECONFIG")];

/// The summary and flags always; the rest once a scan was asked, the UFP VDO only when
/// a known layout was received.
fn sections(port: Option<PortInfo>) -> impl Iterator<Item = Section> {
    let ufp = port.is_some_and(|port| match port.identity {
        Answer::Value(identity) => identity.ufp_vdo().is_some(),
        _ => false,
    });
    [
        (Section::Summary, true),
        (Section::Sink, port.is_some()),
        (Section::Flags, true),
        (Section::Identity, port.is_some()),
        (Section::Ufp, ufp),
    ]
    .into_iter()
    .filter_map(|(section, shown)| shown.then_some(section))
}

pub fn port_sections(observation: Observation) -> u32 {
    partner(observation).map_or(0, |info| sections(info.port).count() as u32)
}

pub fn port_lines(observation: Observation, section: u32) -> Rows {
    let mut rows = [[b' '; SINK_PDO_COLUMNS]; 8];
    let Some(info) = partner(observation) else {
        return rows;
    };
    let Some(section) = sections(info.port).nth(section as usize) else {
        return rows;
    };
    let port = info.port;
    match section {
        Section::Summary => {
            put(&mut rows[0], b"PORT CHECK");
            put(
                &mut rows[1],
                if source_fixed(observation).is_some_and(|pdo| pdo.dual_role_power()) {
                    b"PARTNER:DRP"
                } else {
                    b"PARTNER:SOURCE ONLY"
                },
            );
            match port {
                Some(port) => {
                    status(&mut rows[2], b"SINK PDOS:", &port.sink);
                    status(&mut rows[3], b"IDENTITY:", &port.identity);
                }
                None => put(&mut rows[2], b"HOLD OK: SCAN AT 5V"),
            }
        }
        Section::Sink => {
            put(&mut rows[0], b"SINK PDOS (ACCEPTS)");
            match port.map(|port| port.sink) {
                Some(Answer::Value(objects)) => {
                    for (index, &raw) in objects.as_slice().iter().enumerate() {
                        rows[index + 1] = firmware_core::pd_sniff::describe_sink_pdo(index, raw);
                    }
                }
                Some(other) => status(&mut rows[1], b"SINK PDOS:", &other),
                None => {}
            }
        }
        Section::Flags => {
            put(&mut rows[0], b"PDO 1 FLAGS");
            let mut row = 1;
            if let Some(pdo) = source_fixed(observation) {
                row += flag_rows(&mut rows[row..], b"SOURCE:", pdo.0, &SOURCE_FLAGS);
            }
            // Only a Fixed first Sink PDO carries these bits (Table 6.9); checked
            // on the raw object: `Objects::sink_fixed` costs over 500 bytes here.
            if let Some(Answer::Value(objects)) = port.map(|port| port.sink)
                && let Some(&raw) = objects.as_slice().first()
                && raw >> PDO_TYPE_SHIFT == 0
            {
                row += flag_rows(&mut rows[row..], b"SINK:", raw, &SINK_FLAGS);
                put(
                    &mut rows[row],
                    match (raw >> FAST_ROLE_SWAP_SHIFT) & 3 {
                        0 => b"FRS:NONE",
                        1 => b"FRS:DEFAULT USB",
                        2 => b"FRS:1.5A",
                        _ => b"FRS:3.0A",
                    },
                );
            }
        }
        Section::Identity => {
            put(&mut rows[0], b"DISCOVER IDENTITY");
            match port.map(|port| port.identity) {
                Some(Answer::Value(identity)) => {
                    let header = VdmIdentityHeader(identity.id_header());
                    let product = ProductVDO(identity.product());
                    let mut row = Row::new();
                    row.text(b"VID:")
                        .hex(u32::from(header.vid()), 4)
                        .text(b" PID:")
                        .hex(u32::from(product.pid()), 4);
                    put(&mut rows[1], &row.bytes);
                    // Tables 6.38/6.39: Cert Stat is the XID, not a pass/fail certificate.
                    let mut row = Row::new();
                    row.text(b"BCD:")
                        .hex(u32::from(product.bcd_device()), 4)
                        .text(b" XID:")
                        .hex(identity.xid(), 8);
                    put(&mut rows[2], &row.bytes);
                    // SVDM 1.0 (major 00b, Table 6.33) has no DFP Product Type.
                    let legacy =
                        VdmHeaderStructured(identity.objects()[0]).vdm_version_major() == 0;
                    let raw = identity.id_header();
                    let mut row = Row::new();
                    row.text(b"UFP:").text(match (raw >> UFP_TYPE_SHIFT) & 7 {
                        0 => b"NONE",
                        1 => b"HUB",
                        2 => b"PERIPHERAL",
                        3 => b"PSD",
                        5 => b"AMA",
                        _ => b"UNKNOWN",
                    });
                    put(&mut rows[3], &row.bytes);
                    let mut row = Row::new();
                    row.text(b"DFP:").text(match (raw >> DFP_TYPE_SHIFT) & 7 {
                        _ if legacy => b"N/A (SVDM 1.0)",
                        0 => b"NONE",
                        1 => b"HUB",
                        2 => b"HOST",
                        3 => b"POWER BRICK",
                        4 => b"AMC",
                        _ => b"UNKNOWN",
                    });
                    put(&mut rows[4], &row.bytes);
                    let mut row = Row::new();
                    row.text(b"USB DEVICE:")
                        .text(yes_no(header.device_data()))
                        .text(b" HOST:")
                        .text(yes_no(header.host_data()));
                    put(&mut rows[5], &row.bytes);
                    let mut row = Row::new();
                    row.text(b"MODAL:")
                        .text(yes_no(header.modal_supported()))
                        .text(b" CONN:")
                        .text(match (raw >> CONNECTOR_TYPE_SHIFT) & 3 {
                            2 => b"RECEPTACLE",
                            3 => b"PLUG",
                            _ => b"UNKNOWN",
                        });
                    put(&mut rows[6], &row.bytes);
                }
                Some(other) => status(&mut rows[1], b"IDENTITY:", &other),
                None => {}
            }
        }
        Section::Ufp => {
            let Some(Answer::Value(identity)) = port.map(|port| port.identity) else {
                return rows;
            };
            let Some(ufp) = identity.ufp_vdo() else {
                return rows;
            };
            put(&mut rows[0], b"UFP VDO");
            put(
                &mut rows[1],
                match ufp.usb_highest_speed() {
                    0 => b"SPEED:USB2.0",
                    1 => b"SPEED:USB3.2 GEN1",
                    2 => b"SPEED:USB3.2/4 GEN2",
                    3 => b"SPEED:USB4 GEN3",
                    4 => b"SPEED:USB4 GEN4",
                    _ => b"SPEED:UNKNOWN",
                },
            );
            // Device Capability bits 27..24 as in R3.2 V1.1 Table 6.39 and Linux:
            // USB 2.0, Billboard only, USB 3.2, USB4. V1.2 Table 6.40 rewrites bits
            // 25..24 as a code that swaps the USB 2.0 and Billboard meanings despite
            // the revision's stated technical equivalence, so V1.1 is followed.
            let capability = ufp.device_capability();
            let mut row = Row::new();
            row.text(b"USB2:")
                .text(yes_no(capability & 1 != 0))
                .text(b" BB:")
                .text(yes_no(capability & 2 != 0))
                .text(b" 3.2:")
                .text(yes_no(capability & 4 != 0))
                .text(b" 4:")
                .text(yes_no(capability & 8 != 0));
            put(&mut rows[2], &row.bytes);
            let modes = ufp.alternate_modes();
            let row = 3 + flag_rows(&mut rows[3..], b"ALT MODES:", u32::from(modes), &ALT_MODES);
            // Table 6.40: requirements are reserved without alternate modes;
            // VBUS Required is inverted (zero means required).
            if modes != 0 {
                let mut text = Row::new();
                text.text(b"VBUS REQ:")
                    .text(yes_no(!ufp.vbus_required()))
                    .text(b" VCONN:")
                    .text(yes_no(ufp.vconn_required()));
                put(&mut rows[row], &text.bytes);
                if ufp.vconn_required() {
                    put(
                        &mut rows[row + 1],
                        match ufp.vconn_power() {
                            0 => b"VCONN POWER:1W",
                            1 => b"VCONN POWER:1.5W",
                            2 => b"VCONN POWER:2W",
                            3 => b"VCONN POWER:3W",
                            4 => b"VCONN POWER:4W",
                            5 => b"VCONN POWER:5W",
                            6 => b"VCONN POWER:6W",
                            _ => b"VCONN POWER:RESERVED",
                        },
                    );
                }
            }
        }
    }
    rows
}

/// ID Header fields decoded here rather than through usbpd's enum conversions,
/// which panic on the Invalid values a partner may send (Table 6.34): Product
/// Type (UFP) bits 29..27, Product Type (DFP) bits 25..23, Connector Type bits
/// 22..21.
const UFP_TYPE_SHIFT: u32 = 27;
const DFP_TYPE_SHIFT: u32 = 23;
const CONNECTOR_TYPE_SHIFT: u32 = 21;

/// PDO Type, bits 31..30: 00b is a Fixed PDO (Table 6.6).
const PDO_TYPE_SHIFT: u32 = 30;

/// Flag rows wrap one column short of the row: the last column touches the
/// scrollbar on the display (seen on the device).
const FLAG_COLUMNS: usize = SINK_PDO_COLUMNS - 1;

/// `label` and the names of the set `flags` (`NONE` without any), wrapped
/// at the row width; returns the rows used.
fn flag_rows(
    rows: &mut [[u8; SINK_PDO_COLUMNS]],
    label: &[u8],
    raw: u32,
    flags: &[(u32, &[u8])],
) -> usize {
    put(&mut rows[0], label);
    let (mut line, mut column) = (0, label.len());
    let mut set = flags
        .iter()
        .filter(|(bit, _)| raw & (1 << bit) != 0)
        .peekable();
    if set.peek().is_none() {
        rows[0][column..column + 4].copy_from_slice(b"NONE");
    }
    for (_, name) in set {
        if column + 1 + name.len() > FLAG_COLUMNS {
            (line, column) = (line + 1, 0);
        }
        rows[line][column + 1..column + 1 + name.len()].copy_from_slice(name);
        column += 1 + name.len();
    }
    line + 1
}

/// `label` and the outcome; an unsent sink query means the source is not DRP.
fn status<T>(row: &mut [u8; SINK_PDO_COLUMNS], label: &[u8], answer: &Answer<T>) {
    let caption: &[u8] = match answer {
        Answer::NotSent => b"NOT DRP",
        _ => super::partner::answer_caption(answer),
    };
    let mut text = Row::new();
    text.text(label).text(caption);
    put(row, &text.bytes);
}

fn yes_no(value: bool) -> &'static [u8] {
    if value { b"Y" } else { b"N" }
}
