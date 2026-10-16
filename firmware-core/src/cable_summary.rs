//! A cable's identity in words, one labelled item per fact: what the cable
//! claims in its Discover Identity reply, not measurements.
//! Authority: USB PD R3.2 V1.2, Tables 6.35 and 6.42–6.44; USB PD R2.0 for
//! the legacy layout.

use crate::pd_cable::{self, CableKind, CableReport, CableStatus, Layout, Response};

/// Characters per value, enough for the longest, `USB4 GEN3 / 40Gbps`,
/// and room for several alt modes.
pub const VALUE_WIDTH: usize = 20;
/// Most items a cable can give: six for every cable, four more for an
/// active one.
pub const MAX_ITEMS: usize = 10;

pub type Value = [u8; VALUE_WIDTH];

/// Alt mode SVIDs with a name: VESA's standard ID for DisplayPort Alt Mode,
/// and Intel's vendor ID, which Thunderbolt Alt Mode is identified by (it
/// does not tell Thunderbolt 3 from 4).
pub const DISPLAYPORT_SVID: u16 = 0xff01;
pub const THUNDERBOLT_SVID: u16 = 0x8087;

/// An alt mode's short name, for the SVIDs above.
pub fn mode_name(svid: u16) -> Option<&'static str> {
    match svid {
        DISPLAYPORT_SVID => Some("DP"),
        THUNDERBOLT_SVID => Some("TBT"),
        _ => None,
    }
}

/// Discover Identity reply words (Table 6.34 order).
const CERT_STAT: usize = 2;
const CABLE_VDO: usize = 4;
const ACTIVE_VDO2: usize = 5;
/// Cable VDO fields shared by every layout: plug at bits 19..18, where 3 is
/// a captive cable (one end fixed to a device), and latency at 16..13.
const PLUG_SHIFT: u32 = 18;
const CAPTIVE_PLUG: u32 = 3;
const LATENCY_SHIFT: u32 = 13;
/// Latency codes 1..=7 are 10 ns steps of about a metre each; 8 is over
/// 70 ns (over about 7 m).
const LONGEST_METRE_CODE: u32 = 7;
const OVER_SEVEN_METRES_CODE: u32 = 8;
/// Active cables only: 1000, 2000 and 3000 ns, about 100, 200 and 300 m.
const LONG_ACTIVE_CODES: core::ops::RangeInclusive<u32> = 9..=11;
/// Active Cable VDO 1 bits (Table 6.43).
const SBU_UNSUPPORTED: u32 = 1 << 8;
const SBU_ACTIVE: u32 = 1 << 7;
const VBUS_THROUGH: u32 = 1 << 4;
/// Active Cable VDO 2 fields (Table 6.44): maximum operating temperature in
/// °C at bits 31..24 (0 = not given), optical, retimer and two lanes.
const MAX_TEMPERATURE_SHIFT: u32 = 24;
const OPTICAL: u32 = 1 << 10;
const RETIMER: u32 = 1 << 9;
const TWO_LANES: u32 = 1 << 3;
/// USB4's rates over USB-C's two lanes, in Gbps: Gen3 is 20 and Gen4 40
/// per lane. Only these and USB 2.0's single speed follow from the speed
/// code alone; a Gen1 or Gen2 code does not say whether one lane or two
/// are used, so it gets no rate.
const USB4_GEN3_GBPS: u32 = 40;
const USB4_GEN4_GBPS: u32 = 80;
const MV_PER_V: u32 = 1000;
const MA_PER_A: u16 = 1000;

/// Labelled items, in display order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Summary {
    items: [(&'static str, Value); MAX_ITEMS],
    count: usize,
}

impl Summary {
    pub fn items(&self) -> &[(&'static str, Value)] {
        &self.items[..self.count]
    }

    /// A new item; [`MAX_ITEMS`] covers every item [`summary`] pushes.
    fn push(&mut self, label: &'static str) -> Row<'_> {
        let index = self.count.min(MAX_ITEMS - 1);
        self.count = index + 1;
        self.items[index] = (label, [b' '; VALUE_WIDTH]);
        Row(&mut self.items[index].1, 0)
    }
}

/// The items, or `None` without a valid cable identity. A layout that cannot
/// be decoded still gives its type, VID/PID and certification ID.
pub fn summary(report: &CableReport) -> Option<Summary> {
    if report.status != CableStatus::Identity {
        return None;
    }
    let objects = report.objects.get(..usize::from(report.count))?;
    let Response::Identity(identity) = pd_cable::decode(objects, report.revision).ok()? else {
        return None;
    };
    let mut summary = Summary {
        items: [("", [b' '; VALUE_WIDTH]); MAX_ITEMS],
        count: 0,
    };
    let active = identity.kind == CableKind::Active;
    let cable = objects[CABLE_VDO];
    let mut kind = summary.push("TYPE");
    kind.text(if active { "ACTIVE" } else { "PASSIVE" });
    if identity.layout != Layout::Unknown && (cable >> PLUG_SHIFT) & 3 == CAPTIVE_PLUG {
        kind.text(" CAPTIVE");
    }
    if identity.layout != Layout::Unknown {
        // The generation, then its rate where the code alone fixes it.
        let mut usb = summary.push("USB");
        usb.text(generation(identity.layout, identity.speed_code));
        let modern = matches!(identity.layout, Layout::PassiveV10 | Layout::ActiveV13);
        match (identity.speed_code, modern && two_lanes(&identity, objects)) {
            (0, _) => {
                usb.text(" / 480Mbps");
            }
            (3, true) => {
                usb.text(" / ").number(USB4_GEN3_GBPS).text("Gbps");
            }
            (4, true) => {
                usb.text(" / ").number(USB4_GEN4_GBPS).text("Gbps");
            }
            _ => {}
        }
        let mut rating = summary.push("RATING");
        if let Some(mv) = identity.maximum_voltage_mv {
            rating.number(mv / MV_PER_V).text("V ");
        }
        match identity.current_ma {
            Some(ma) => {
                rating.number(u32::from(ma / MA_PER_A)).text("A");
                if identity.current_assumed {
                    rating.text(" ASSUMED");
                }
            }
            None if active && cable & VBUS_THROUGH == 0 => {
                rating.text("NO VBUS");
            }
            // A reserved current code.
            None => {
                rating.text("A ?");
            }
        }
        // Its own bit, which a charger checks before EPR; never inferred
        // from the voltage.
        if identity.epr_capable == Some(true) {
            rating.text(" EPR");
        }
        let mut length = summary.push("LENGTH");
        match (cable >> LATENCY_SHIFT) & 0xf {
            code @ 1..=LONGEST_METRE_CODE => {
                length.number(code - 1).text("-").number(code).text("m");
            }
            OVER_SEVEN_METRES_CODE => {
                length.text(">7m");
            }
            code if active && LONG_ACTIVE_CODES.contains(&code) => {
                length
                    .text("~")
                    .number((code - OVER_SEVEN_METRES_CODE) * 100)
                    .text("m");
            }
            _ => {
                length.text("?");
            }
        }
    }
    summary
        .push("VID/PID")
        .hex(identity.vid)
        .text("/")
        .hex(identity.pid);
    let certification = objects[CERT_STAT];
    if certification != 0 {
        summary
            .push("CERT ID")
            .hex((certification >> 16) as u16)
            .hex(certification as u16);
    }
    if identity.layout == Layout::ActiveV13 {
        summary.push("SBU").text(if cable & SBU_UNSUPPORTED != 0 {
            "NO"
        } else if cable & SBU_ACTIVE != 0 {
            "YES, ACTIVE"
        } else {
            "YES, PASSIVE"
        });
        // The decoder accepts this layout only with its second VDO.
        let second = objects[ACTIVE_VDO2];
        summary
            .push("BUILD")
            .text(if second & OPTICAL != 0 {
                "OPTICAL "
            } else {
                "COPPER "
            })
            .text(if second & RETIMER != 0 {
                "RETIMER"
            } else {
                "REDRIVER"
            });
        summary
            .push("LANES")
            .text(if second & TWO_LANES != 0 { "2" } else { "1" });
        let temperature = second >> MAX_TEMPERATURE_SHIFT;
        if temperature != 0 {
            summary.push("MAX TEMP").number(temperature).text("C");
        }
    }
    Some(summary)
}

/// The highest USB speed generation: Table 6.42/6.43 codes for PD 3
/// layouts, the PD 2.0 cable VDO's for the legacy one.
fn generation(layout: Layout, code: u8) -> &'static str {
    match (layout, code) {
        (_, 0) => "USB 2.0",
        (_, 1) => "USB3 GEN1",
        (Layout::Pd2, 2) => "USB3 GEN2",
        (Layout::PassiveV10 | Layout::ActiveV13, 2) => "USB3/4 GEN2",
        (Layout::PassiveV10 | Layout::ActiveV13, 3) => "USB4 GEN3",
        (Layout::PassiveV10 | Layout::ActiveV13, 4) => "USB4 GEN4",
        _ => "?",
    }
}

/// Whether the cable carries both lanes: every passive cable with
/// SuperSpeed wires does; a modern active cable says so in its second VDO.
fn two_lanes(identity: &pd_cable::Identity<'_>, objects: &[u32]) -> bool {
    match (identity.kind, identity.layout) {
        (CableKind::Passive, _) => true,
        // The decoder accepts this layout only with its second VDO.
        (CableKind::Active, Layout::ActiveV13) => objects[ACTIVE_VDO2] & TWO_LANES != 0,
        (CableKind::Active, _) => false,
    }
}

/// A value being written left to right; text past the end is dropped.
struct Row<'a>(&'a mut Value, usize);

impl Row<'_> {
    fn byte(&mut self, byte: u8) {
        if let Some(cell) = self.0.get_mut(self.1) {
            *cell = byte;
            self.1 += 1;
        }
    }

    fn text(&mut self, text: &str) -> &mut Self {
        text.bytes().for_each(|byte| self.byte(byte));
        self
    }

    fn number(&mut self, value: u32) -> &mut Self {
        if value >= 10 {
            self.number(value / 10);
        }
        self.byte(b'0' + (value % 10) as u8);
        self
    }

    fn hex(&mut self, value: u16) -> &mut Self {
        for shift in [12, 8, 4, 0] {
            self.byte(b"0123456789ABCDEF"[usize::from((value >> shift) & 15)]);
        }
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn report(objects: &[u32]) -> CableReport {
        let mut words = [0; 7];
        words[..objects.len()].copy_from_slice(objects);
        CableReport {
            status: CableStatus::Identity,
            objects: words,
            count: objects.len() as u8,
            revision: 2,
            next_tx_id: 0,
            last_rx_id: None,
        }
    }

    fn assert_items(report: &CableReport, expected: &[(&str, &str)]) {
        let summary = summary(report).expect("a valid identity");
        let items = summary.items();
        assert!(items.len() >= expected.len(), "{items:?}");
        for ((label, value), (expected_label, expected_value)) in items.iter().zip(expected) {
            let value = core::str::from_utf8(value).unwrap_or("").trim_ascii_end();
            assert_eq!((*label, value), (*expected_label, *expected_value));
        }
    }

    #[test]
    fn real_cables_read_as_the_reference_firmware_shows_them() {
        // Two cables' Discover Identity replies, photographed beside the
        // reference firmware's summary of the same cables (passive, USB 2.0,
        // rating with EPR on the second, 0-1 m).
        assert_items(
            &report(&[0xFF00_A041, 0x1860_2717, 0, 0, 0x0008_2040]),
            &[
                ("TYPE", "PASSIVE"),
                ("USB", "USB 2.0 / 480Mbps"),
                ("RATING", "20V 5A"),
                ("LENGTH", "0-1m"),
                ("VID/PID", "2717/0000"),
            ],
        );
        assert_items(
            &report(&[0xFF00_A841, 0x1860_0000, 0, 0, 0x000A_2640]),
            &[
                ("TYPE", "PASSIVE"),
                ("USB", "USB 2.0 / 480Mbps"),
                ("RATING", "50V 5A EPR"),
                ("LENGTH", "0-1m"),
            ],
        );
    }

    #[test]
    fn usb4_gen3_cable_reads_as_its_two_lane_maximum() {
        // The Discover Identity reply in the README's Cable screenshot: a
        // passive USB4 Gen3 cable, 20 Gbps per lane on both lanes.
        assert_items(
            &report(&[
                0xFF00_A841,
                0x1C60_2E99,
                0x0000_1520,
                0x0001_0000,
                0x000A_4643,
            ]),
            &[
                ("TYPE", "PASSIVE"),
                ("USB", "USB4 GEN3 / 40Gbps"),
                ("RATING", "50V 5A EPR"),
                ("LENGTH", "1-2m"),
                ("VID/PID", "2E99/0001"),
                ("CERT ID", "00001520"),
            ],
        );
    }
}
