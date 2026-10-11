//! Board observation adapter and renderer; controller I/O/telemetry live in fusb302-pd.
mod cable;
mod partner;

pub use cable::{cable_lines, cable_sections, cable_svid_index, control_cable_selection};
pub use partner::{CAPABILITY_SECTIONS, partner_lines, partner_sections};

use firmware_core::pd_cable::CableStatus;
use firmware_core::pd_sniff::Supply;
use firmware_core::pd_spr::Advertised;

use crate::ui::{self, Protocol};
// Compatibility exports are used by different subsets of board host tests.
#[allow(unused_imports)]
pub use fusb302_pd::observation::{Phase, SprObservation};
#[allow(unused_imports)]
pub use fusb302_pd::snapshot::{ADDRESS, Failure, REGISTERS, Snapshot, snapshot};

#[derive(Clone, Copy)]
pub struct Observation {
    pub phy: fusb302_pd::observation::Observation,
    pub pc13_high: Option<bool>,
    pub sniff: Option<SniffObservation>,
    pub last_boot: Option<LastBoot>,
    /// Sink startup: what the receive-only listen heard when both CCs carried
    /// Rp (`fusb302_pd::startup::Listen`).
    pub listen: Option<fusb302_pd::startup::Listen>,
    /// This boot runs the 5 A cable test (`Settings::cable_test`): with the
    /// meter's plug in the charger, answer as a 5 A cable.
    pub cable_test: bool,
    /// The saved settings asked for the test, but this boot did not follow a
    /// power-up or clearing that request failed, so it runs normally (`main`).
    pub cable_test_skipped: bool,
    /// This boot keeps PD silent for Quick Charge (`NextBoot::silent`): Rd
    /// held, nothing acknowledged or sent.
    pub quick_charge: bool,
    /// A silent boot outlasted the source's PD attempts
    /// (`pd_task::SILENT_SETTLED`), so it no longer restarts the meter.
    pub silent_settled: bool,
    /// This boot is the full scan's PD part (`NextBoot::ScanPd`), which also
    /// asks for the EPR offer.
    pub scan_pd: bool,
    /// The boot's reset cause flags, shown during the test, whose page is
    /// the only one open.
    pub reset_flags: u16,
}
impl core::ops::Deref for Observation {
    type Target = fusb302_pd::observation::Observation;
    fn deref(&self) -> &Self::Target {
        &self.phy
    }
}
impl core::ops::DerefMut for Observation {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.phy
    }
}
impl Observation {
    pub const fn new() -> Self {
        Self {
            phy: fusb302_pd::observation::Observation::new(),
            pc13_high: None,
            cable_test: false,
            cable_test_skipped: false,
            quick_charge: false,
            silent_settled: false,
            scan_pd: false,
            reset_flags: 0,
            sniff: None,
            last_boot: None,
            listen: None,
        }
    }
    /// Replace transport telemetry without dropping slider/sniffer/boot history.
    pub fn with_phy(self, phy: fusb302_pd::observation::Observation) -> Self {
        Self { phy, ..self }
    }
}

// Construct a fresh rendering view; existing board state must use with_phy.
impl From<fusb302_pd::observation::Observation> for Observation {
    fn from(phy: fusb302_pd::observation::Observation) -> Self {
        Self { phy, ..Self::new() }
    }
}

/// What the previous boot recorded last before it ended (`docs/SAFETY.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BootRecord {
    pub step: u8,
    pub error: u8,
    pub packets: u8,
    pub header: u16,
    /// ms since that boot's start, at the entry.
    pub ms: u16,
    /// ms since boot when that boot first reached step 3 (GoodCRC on) and
    /// step 5 (Source_Capabilities decoded); 0 = not reached.
    pub step3_ms: u16,
    pub step5_ms: u16,
    /// Longest gap between receive polls up to the entry, ms.
    pub poll_gap_ms: u8,
    /// Longest I2C3 lock wait, I2C transfer and 1 ms delay overshoot, ms.
    pub lock_wait_ms: u8,
    pub transfer_ms: u8,
    /// Fatal record (error 250 panic, 251 HardFault): code addresses, for a
    /// HardFault PC and LR first, then return addresses innermost first.
    /// Zero otherwise.
    pub chain: [u32; 5],
    pub delay_over_ms: u8,
}

/// `record` is `None` before the first logged boot; `logging` is false when no
/// flash sector is available for the log.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LastBoot {
    pub record: Option<BootRecord>,
    pub logging: bool,
}

/// Why the sniffer stopped. It never restarts in the same boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SniffStop {
    Orientation,
    Guard { register: u8, value: u8 },
    Verify { register: u8, value: u8 },
    Drift { register: u8, value: u8 },
    Boundary(u8),
    Bus(u8),
    HardReset,
    Overflow,
    Frame,
    UnexpectedTx,
    PhyFault,
    Halted,
}

/// Passthrough log. Wire observations only: the tester sends nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SniffObservation {
    pub cc: Option<u8>,
    pub cc_status: Option<[u8; 2]>,
    pub rp: Option<firmware_core::pd_sniff::Rp>,
    pub log: firmware_core::pd_sniff::Log,
    pub caps: [u32; 7],
    pub caps_count: u8,
    pub flags: [u8; 3],
    pub stop: Option<SniffStop>,
    /// Header and decoded supply of the newest Request, kept beyond the
    /// rolling log: a PPS sink's other traffic soon scrolls it out.
    pub request: Option<(u16, Supply)>,
    /// Object position of the last Request seen without captured
    /// capabilities (`uncaptured_request`).
    uncaptured_position: Option<u32>,
}

impl SniffObservation {
    pub const fn new() -> Self {
        Self {
            cc: None,
            cc_status: None,
            rp: None,
            log: firmware_core::pd_sniff::Log::new(),
            caps: [0; 7],
            caps_count: 0,
            flags: [0; 3],
            stop: None,
            request: None,
            uncaptured_position: None,
        }
    }
    /// Log one CRC-checked SOP frame. Source_Capabilities replace the stored
    /// PDOs first, so a later Request decodes against the newest offer.
    pub fn record(&mut self, frame: &firmware_core::pd_rx::Frame<'_>) {
        if frame.header() & fusb302_pd::header::EXTENDED == 0
            && let Ok(objects) = frame.source_capabilities()
        {
            self.caps_count = objects.len().min(7) as u8;
            for (slot, raw) in self.caps.iter_mut().zip(objects) {
                *slot = raw;
            }
            // New capabilities start a new negotiation.
            self.request = None;
            self.uncaptured_position = None;
        }
        let caps = &self.caps[..self.caps_count as usize];
        let entry = firmware_core::pd_sniff::Entry::classify(frame.header(), frame.data(), caps);
        if let Some(supply) = entry.supply {
            let supply = match supply {
                Supply::Raw if self.caps_count == 0 => self.uncaptured_request(entry.word),
                supply => supply,
            };
            self.request = Some((entry.header, supply));
        }
        self.log.push(entry);
    }

    /// A Request seen before any Source_Capabilities: the meter booted from
    /// the same VBUS after the negotiation, so this is a re-request. A sink
    /// re-requests on its own only to keep a PPS contract alive (at least
    /// every 10 s, USB PD tPPSRequest); a fixed sink only answers new
    /// capabilities, which the sniffer would have captured. So a second such
    /// Request for the same object is PPS and decodes as a PPS RDO.
    fn uncaptured_request(&mut self, rdo: u32) -> Supply {
        let position = rdo >> RDO_POSITION_SHIFT;
        let repeated = self.uncaptured_position == Some(position);
        self.uncaptured_position = Some(position);
        if !repeated {
            return Supply::Raw;
        }
        Supply::Pps {
            mv: (rdo >> PPS_RDO_VOLTAGE_SHIFT & PPS_RDO_VOLTAGE_MASK) * PPS_RDO_VOLTAGE_UNIT_MV,
            ma: (rdo & PPS_RDO_CURRENT_MASK) * PPS_RDO_CURRENT_UNIT_MA,
        }
    }
}

/// Request Data Object fields (USB PD Tables 6.19 and 6.21): object position
/// in bits 31..28; a PPS RDO carries the output voltage in bits 20..9 in
/// 20 mV units and the operating current in bits 6..0 in 50 mA units.
const RDO_POSITION_SHIFT: u32 = 28;
const PPS_RDO_VOLTAGE_SHIFT: u32 = 9;
const PPS_RDO_VOLTAGE_MASK: u32 = 0xfff;
const PPS_RDO_VOLTAGE_UNIT_MV: u32 = 20;
const PPS_RDO_CURRENT_MASK: u32 = 0x7f;
const PPS_RDO_CURRENT_UNIT_MA: u32 = 50;

/// The charging protocol shown on the meter page. A Request the passthrough
/// sniffer decoded wins, then this meter's own contract, then a capabilities
/// offer, then the Type-C current advertisement on the attached CC.
pub fn protocol(observation: &Observation) -> Protocol {
    if let Some(sniff) = observation.sniff {
        if let Some((header, supply)) = sniff.request {
            return match supply {
                Supply::Pps { mv, ma } => Protocol::Pps {
                    mv: mv.min(u32::from(u16::MAX)) as u16,
                    ma: ma.min(u32::from(u16::MAX)) as u16,
                },
                Supply::Fixed { mv, ma } => Protocol::Pd {
                    revision: header_revision(header),
                    mv: mv.min(u32::from(u16::MAX)) as u16,
                    ma: ma.min(u32::from(u16::MAX)) as u16,
                },
                Supply::Raw => Protocol::PdOffer,
            };
        }
        if sniff.caps_count > 0 {
            return Protocol::PdOffer;
        }
        if let Some(rp) = sniff.rp {
            return type_c(rp);
        }
    }
    if let Some(spr) = observation.spr
        && (spr.psrdy_seen || spr.contract_seen)
    {
        return if spr.pps {
            Protocol::Pps {
                mv: spr.target_mv,
                ma: spr.target_ma,
            }
        } else {
            Protocol::Pd {
                revision: spr.peer_revision,
                mv: spr.target_mv,
                ma: offer(observation)
                    .fixed_current(spr.target_mv)
                    .unwrap_or(spr.target_ma),
            }
        };
    }
    if observation.capabilities.is_some() {
        return Protocol::PdOffer;
    }
    attached_cc(observation)
        .zip(observation.cc_status)
        .and_then(|(cc, status)| status.get(usize::from(cc).checked_sub(1)?).copied())
        .and_then(firmware_core::pd_sniff::rp)
        .map_or(Protocol::Unknown, type_c)
}

/// The charger's offer and this meter's preview and contract, for the
/// selector pages. The offer comes from this meter's own session, else from
/// the passthrough sniffer.
pub fn offer(observation: &Observation) -> ui::Offer {
    let mut offer = ui::Offer::default();
    let mut add = |raw: u32| match firmware_core::pd_spr::advertised(raw) {
        Advertised::Fixed { mv, ma } => offer.push_fixed(mv, ma),
        Advertised::Pps {
            min_mv,
            max_mv,
            max_ma,
        } => offer.push_pps(ui::PpsRange {
            min_mv,
            max_mv,
            max_ma,
        }),
        Advertised::Other => {}
    };
    if let Some(caps) = observation.capabilities {
        for object in caps.objects.iter().take(usize::from(caps.count)) {
            add(object.raw);
        }
    } else if let Some(sniff) = observation.sniff {
        for &raw in sniff.caps.iter().take(usize::from(sniff.caps_count)) {
            add(raw);
        }
    }
    if let Some(spr) = observation.spr {
        offer.session = true;
        offer.preview = spr
            .selected_mv
            .zip(spr.selected_ma)
            .map(|(mv, ma)| ui::Supply {
                mv,
                ma,
                pps: spr.selected_pps,
            });
        // A fixed Request asks for the profile's maximum current, not the
        // session's nominal target current, so show the advertised one.
        let ma = if spr.pps {
            spr.target_ma
        } else {
            offer.fixed_current(spr.target_mv).unwrap_or(spr.target_ma)
        };
        offer.contract = (spr.psrdy_seen || spr.contract_seen).then_some(ui::Supply {
            mv: spr.target_mv,
            ma,
            pps: spr.pps,
        });
    } else if let Some((_, supply)) = observation.sniff.and_then(|sniff| sniff.request) {
        // Passthrough: the device's own choice is both shown and in force.
        let to_u16 = |value: u32| value.min(u32::from(u16::MAX)) as u16;
        offer.contract = match supply {
            Supply::Fixed { mv, ma } => Some(ui::Supply {
                mv: to_u16(mv),
                ma: to_u16(ma),
                pps: false,
            }),
            Supply::Pps { mv, ma } => Some(ui::Supply {
                mv: to_u16(mv),
                ma: to_u16(ma),
                pps: true,
            }),
            Supply::Raw => None,
        };
        offer.preview = offer.contract;
    }
    offer
}

/// Attached CC pin (1 or 2): the passthrough sniffer's, else this meter's.
pub fn attached_cc(observation: &Observation) -> Option<u8> {
    observation
        .sniff
        .and_then(|sniff| sniff.cc)
        .or(observation.selected_cc)
}

/// Major PD revision from a message header's Specification Revision field
/// (USB PD Table 6.1): 01b is 2.0, 10b is 3.x.
fn header_revision(header: u16) -> Option<u8> {
    match ((header & fusb302_pd::header::SPEC_REVISION) >> 6) as u8 {
        fusb302_pd::header::REVISION_2 => Some(2),
        fusb302_pd::header::REVISION_3 => Some(3),
        _ => None,
    }
}

/// Type-C source current advertisement (USB Type-C default, 1.5 A, 3 A).
fn type_c(rp: firmware_core::pd_sniff::Rp) -> Protocol {
    use firmware_core::pd_sniff::Rp;
    Protocol::TypeC {
        ma: match rp {
            Rp::Default => 0,
            Rp::Current1A5 => 1500,
            Rp::Current3A0 => 3000,
        },
    }
}

fn phase_line(row: &mut [u8; 24], observation: Observation) {
    match observation.phase {
        Phase::Waiting => put(row, b"WAIT CC TEST"),
        Phase::Passive => put(row, b"PASSIVE TX OFF"),
        Phase::Hold if observation.quick_charge => put(row, b"QUICK CHARGE PD OFF"),
        Phase::Hold => put(row, b"HOLD RD ON TX OFF"),
        Phase::Skipped => put(row, b"SKIP BUS LATCHED"),
        Phase::Measuring => put(row, b"MEAS RX POWER"),
        Phase::Receiving => put(row, b"RX LISTEN TX OFF"),
        Phase::Captured => put(row, b"CAPS RESTORED TX OFF"),
        Phase::NoCapabilities => put(
            row,
            if observation.spr.is_some() {
                b"NO CAPS / HALTED"
            } else {
                b"NO CAPS RESTORED"
            },
        ),
        Phase::Negotiating => {
            if let Some(spr) = observation.spr.filter(|s| s.psrdy_seen) {
                put(row, b"--V CONTRACT LIVE");
                decimal_voltage(row, 0, spr.target_mv);
                if spr.pps {
                    row.fill(b' ');
                    put(row, b"PPS--.--V --.--A LIVE");
                    decimal2(row, 3, spr.target_mv);
                    decimal2(row, 10, spr.target_ma);
                }
            } else {
                put(
                    row,
                    if observation.spr.is_some_and(|spr| spr.pps) {
                        b"PPS NEGOTIATING"
                    } else {
                        b"SPR NEGOTIATING"
                    },
                );
            }
        }
        Phase::Contracted => {
            put(row, b"--V SEEN ROK TX OFF");
            if let Some(spr) = observation.spr {
                decimal_voltage(row, 0, spr.target_mv);
                if spr.pps {
                    row.fill(b' ');
                    put(row, b"PPS--.--V --.--A TX OFF");
                    decimal2(row, 3, spr.target_mv);
                    decimal2(row, 10, spr.target_ma);
                }
            }
        }
        Phase::Failed | Phase::Guarded | Phase::Verify => {
            put(
                row,
                match observation.phase {
                    Phase::Guarded => b"SKIP REG:----",
                    Phase::Verify => b"DIFF REG:----",
                    _ => b"FAIL REG:----",
                },
            );
            hex(row, 9, observation.failed_register as u16);
        }
    }
}

/// Fixed-width CC diagnostic lines. PRE and POST are distinct; missing POST
/// never implies restoration. No renderer ownership, MMIO or bus access.
pub fn lines(observation: Observation) -> [[u8; 24]; 8] {
    if let Some(sniff) = observation.sniff {
        return sniff_lines(sniff);
    }
    if observation.phase == Phase::Passive {
        return passive_lines(observation);
    }
    if let Some(spr) = observation.spr {
        return spr_lines(observation, spr);
    }
    if observation.selected_cc.is_some() || observation.rx_error != 0 {
        return rx_lines(observation);
    }
    let mut rows = [[b' '; 24]; 8];
    put(&mut rows[0], b"PD RX TEST");
    put(&mut rows[1], b"PRE S0:---- PW:----");
    put(&mut rows[2], b"S1:---- C0:---- ID:----");
    if let Some(snapshot) = observation.snapshot {
        hex(&mut rows[1], 7, snapshot.values[1] as u16);
        hex(&mut rows[1], 15, snapshot.values[7] as u16);
        hex(&mut rows[2], 3, snapshot.values[2] as u16);
        hex(&mut rows[2], 11, snapshot.values[3] as u16);
        hex(&mut rows[2], 19, snapshot.values[0] as u16);
    }
    put(&mut rows[3], b"CC1:---- BC:- ACT:-");
    put(&mut rows[4], b"CC2:---- BC:- ACT:-");
    if let Some(status) = observation.cc_status {
        for (i, value) in status.into_iter().enumerate() {
            hex(&mut rows[i + 3], 4, value as u16);
            rows[i + 3][12] = b'0' + (value & 3);
            rows[i + 3][18] = b'0' + ((value >> 6) & 1);
        }
    }
    put(&mut rows[5], b"POST S0:---- PW:----");
    if let Some(snapshot) = observation.post_snapshot {
        hex(&mut rows[5], 8, snapshot.values[1] as u16);
        hex(&mut rows[5], 16, snapshot.values[7] as u16);
    }
    if observation.restore_unverified {
        rows[5] = [b' '; 24];
        put(&mut rows[5], b"RESTORE NOT VERIFIED");
    }
    if let Some(value) = observation.failed_value {
        rows[5] = [b' '; 24];
        put(&mut rows[5], b"VALUE:----");
        hex(&mut rows[5], 6, value as u16);
    }
    // Same current-level/count presentation as the configuration-only page.
    put(&mut rows[6], b"IRQ:1 EDGE:000000");
    rows[6][4] = if observation.irq_high { b'1' } else { b'0' };
    let mut count = observation.irq_count % 1_000_000;
    for i in (11..17).rev() {
        rows[6][i] = b'0' + (count % 10) as u8;
        count /= 10;
    }
    phase_line(&mut rows[7], observation);
    rows
}

/// Passthrough page: no session this boot; the PHY only has Rd released.
fn passive_lines(observation: Observation) -> [[u8; 24]; 8] {
    let mut rows = [[b' '; 24]; 8];
    put(&mut rows[0], b"PD PASSIVE  TX OFF");
    put(&mut rows[1], b"S0:---- PW:---- S1:----");
    if let Some(snapshot) = observation.snapshot {
        hex(&mut rows[1], 3, snapshot.values[1] as u16);
        hex(&mut rows[1], 11, snapshot.values[7] as u16);
        hex(&mut rows[1], 19, snapshot.values[2] as u16);
    }
    put(&mut rows[2], b"CC PULL-DOWNS OFF");
    let slider: &[u8] = match observation.pc13_high {
        Some(true) => b"PC13:1 PASSTHROUGH",
        Some(false) => b"PC13:0 SINK MODE",
        None => b"PC13:-",
    };
    put(&mut rows[3], slider);
    put(&mut rows[5], b"NO TRIGGER THIS BOOT");
    put(&mut rows[6], b"SINK MODE + REPLUG");
    phase_line(&mut rows[7], observation);
    rows
}

/// Passthrough page: status row, then the latest messages, oldest first.
fn sniff_lines(sniff: SniffObservation) -> [[u8; 24]; 8] {
    use firmware_core::pd_sniff::{Row, Rp, describe};
    let mut rows = [[b' '; 24]; 8];
    let mut head = Row::new();
    head.text(b"SNIFF ");
    if let Some(stop) = sniff.stop {
        head.text(b"STOP ");
        match stop {
            SniffStop::Orientation => head.text(b"CC ?"),
            SniffStop::Guard { register, value } => head
                .text(b"GUARD ")
                .hex(register as u32, 2)
                .text(b"=")
                .hex(value as u32, 2),
            SniffStop::Verify { register, value } => head
                .text(b"VERIFY ")
                .hex(register as u32, 2)
                .text(b"=")
                .hex(value as u32, 2),
            SniffStop::Drift { register, value } => head
                .text(b"DRIFT ")
                .hex(register as u32, 2)
                .text(b"=")
                .hex(value as u32, 2),
            SniffStop::Boundary(status) => head.text(b"FLUSH ST ").hex(status as u32, 2),
            SniffStop::Bus(register) => head.text(b"BUS ").hex(register as u32, 2),
            SniffStop::HardReset => head.text(b"HARD RESET"),
            SniffStop::Overflow => head.text(b"RX OVERFLOW"),
            SniffStop::Frame => head.text(b"BAD FRAME"),
            SniffStop::UnexpectedTx => head.text(b"TX FLAG"),
            SniffStop::PhyFault => head.text(b"PHY FAULT"),
            SniffStop::Halted => head.text(b"HALTED"),
        };
    } else {
        match sniff.cc {
            Some(cc) => head.text(b"CC").decimal(cc as u32),
            None => head.text(b"CC-"),
        };
        head.text(match sniff.rp {
            Some(Rp::Default) => b" RPDEF ",
            Some(Rp::Current1A5) => b" RP1.5A",
            Some(Rp::Current3A0) => b" RP3.0A",
            None => b" RP--  ",
        });
        head.text(b" M:").decimal(sniff.log.messages);
    }
    rows[0] = head.bytes;
    if sniff.log.entries().len() == 0 {
        let mut cc = Row::new();
        cc.text(b"S0 CC1:");
        match sniff.cc_status {
            Some(status) => cc
                .hex(status[0] as u32, 2)
                .text(b" CC2:")
                .hex(status[1] as u32, 2),
            None => cc.text(b"-- CC2:--"),
        };
        rows[1] = cc.bytes;
        put(&mut rows[2], b"PASSTHROUGH RX ONLY");
        put(&mut rows[3], b"NO TX NO GOODCRC");
        if sniff.stop.is_none() {
            put(
                &mut rows[5],
                if sniff.cc.is_some() {
                    b"WAITING FOR PD TRAFFIC"
                } else {
                    b"STARTING"
                },
            );
        }
        return rows;
    }
    for (row, entry) in rows[1..].iter_mut().zip(sniff.log.entries()) {
        *row = describe(entry);
    }
    rows
}

/// Sink startup, after `lines`: both CC samples (STATUS0: BC_LVL in bits
/// 1:0, ACTIVITY 0x40) and, when both carried Rp, the line the listen chose
/// and the packets it heard for the partner (SOP) and for a cable (SOP'/'').
pub fn startup_lines(observation: Observation) -> Option<[[u8; 24]; 8]> {
    let cc = observation
        .cc_status
        .filter(|_| observation.sniff.is_none())?;
    let mut rows = [[b' '; 24]; 8];
    put(&mut rows[0], b"STARTUP");
    put(&mut rows[1], b"CC1:-- CC2:--");
    hex8(&mut rows[1], 4, cc[0]);
    hex8(&mut rows[1], 11, cc[1]);
    let Some(listen) = observation.listen else {
        put(&mut rows[2], b"ONE CC WITH RP");
        return Some(rows);
    };
    put(
        &mut rows[2],
        match (listen.vconn, listen.traffic) {
            (Some(1), _) => &b"BOTH RP: VCONN ON CC1"[..],
            (Some(_), _) => b"BOTH RP: VCONN ON CC2",
            (None, Some(1)) => b"BOTH RP: TRAFFIC ON CC1",
            (None, Some(_)) => b"BOTH RP: TRAFFIC ON CC2",
            (None, None) => b"BOTH RP: NO SINGLE LINE",
        },
    );
    if listen.vconn.is_some() {
        return Some(rows);
    }
    for (index, row) in rows[3..5].iter_mut().enumerate() {
        put(row, b"CC- SOP:--- CABLE:---");
        row[2] = b'1' + index as u8;
        decimal3(row, 8, listen.partner[index].into());
        decimal3(row, 18, listen.cable[index].into());
    }
    Some(rows)
}

/// Passthrough: the latest advertised Source_Capabilities, after `lines`,
/// once one was captured. The charger sends it when the phone attaches, and
/// the meter, powered from that same VBUS, is often still booting then.
pub fn caps_lines(observation: Observation) -> Option<[[u8; 24]; 8]> {
    observation
        .sniff
        .filter(|sniff| sniff.caps_count > 0)
        .map(sniff_caps_lines)
}

fn sniff_caps_lines(sniff: SniffObservation) -> [[u8; 24]; 8] {
    use firmware_core::pd_sniff::{Row, describe_pdo};
    let mut rows = [[b' '; 24]; 8];
    let mut head = Row::new();
    head.text(b"CHARGER OFFER  GCRC:")
        .decimal(sniff.log.goodcrc);
    rows[0] = head.bytes;
    for (i, &raw) in sniff.caps[..sniff.caps_count as usize].iter().enumerate() {
        rows[i + 1] = describe_pdo(i, raw);
    }
    rows
}

fn rx_lines(observation: Observation) -> [[u8; 24]; 8] {
    let mut rows = [[b' '; 24]; 8];
    if let (Phase::Captured, Some(capabilities), Some(cc)) = (
        observation.phase,
        observation.capabilities,
        observation.selected_cc,
    ) {
        put(&mut rows[0], b"PD CAPS C- N- ROK TX OFF");
        rows[0][9] = b'0' + cc;
        rows[0][12] = b'0' + capabilities.count;
        for (i, object) in capabilities.objects[..capabilities.count as usize]
            .iter()
            .enumerate()
        {
            rows[i + 1][0] = b'1' + i as u8;
            if let Some((mv, ma)) = object.fixed {
                rows[i + 1][1..].copy_from_slice(b" FIX 00.00V 00.00A     ");
                decimal2(&mut rows[i + 1], 6, mv);
                decimal2(&mut rows[i + 1], 13, ma);
            } else {
                put(&mut rows[i + 1], b"- RAW --------");
                rows[i + 1][0] = b'1' + i as u8;
                hex(&mut rows[i + 1], 6, (object.raw >> 16) as u16);
                hex(&mut rows[i + 1], 10, object.raw as u16);
            }
        }
        return rows;
    }
    put(&mut rows[0], b"PD RX TEST C- TX OFF");
    if let Some(cc) = observation.selected_cc {
        rows[0][12] = b'0' + cc;
    }
    put(&mut rows[1], b"PKTS:---- POLLS:----");
    hex(&mut rows[1], 5, observation.rx_packets);
    hex(&mut rows[1], 16, observation.rx_polls);
    put(&mut rows[2], b"LAST HEADER:----");
    if let Some(header) = observation.rx_last_header {
        hex(&mut rows[2], 12, header);
    }
    put(&mut rows[3], b"IA:---- IB:---- I:----");
    for (offset, value) in [3, 11, 18].into_iter().zip(observation.rx_flags) {
        hex(&mut rows[3], offset, value as u16);
    }
    put(&mut rows[4], b"ERROR:---- REG:----");
    hex(&mut rows[4], 6, observation.rx_error as u16);
    hex(&mut rows[4], 15, observation.failed_register as u16);
    put(&mut rows[5], b"POST S0:---- PW:----");
    if let Some(after) = observation.post_snapshot {
        hex(&mut rows[5], 8, after.values[1] as u16);
        hex(&mut rows[5], 16, after.values[7] as u16);
    }
    if observation.restore_unverified {
        rows[5] = [b' '; 24];
        put(
            &mut rows[5],
            if observation.phase == Phase::Receiving {
                b"RX POWERED TX OFF"
            } else {
                b"RESTORE NOT VERIFIED"
            },
        );
    }
    if let Some(value) = observation.failed_value {
        put(&mut rows[6], b"VALUE:----");
        hex(&mut rows[6], 6, value as u16);
    } else {
        put(&mut rows[6], b"IRQ:- EDGE:000000");
        rows[6][4] = b'0' + observation.irq_high as u8;
        let mut count = observation.irq_count % 1_000_000;
        for i in (11..17).rev() {
            rows[6][i] = b'0' + (count % 10) as u8;
            count /= 10;
        }
    }
    phase_line(&mut rows[7], observation);
    rows
}
fn spr_lines(observation: Observation, spr: SprObservation) -> [[u8; 24]; 8] {
    let mut rows = [[b' '; 24]; 8];
    put(
        &mut rows[0],
        if spr.tx_off_verified {
            b"PD SPR --V C- TX OFF"
        } else if observation.phase == Phase::Negotiating && spr.psrdy_seen {
            b"PD SPR --V C- TX ON"
        } else {
            b"PD SPR --V C- TX UNKNOWN"
        },
    );
    decimal_voltage(&mut rows[0], 7, spr.target_mv);
    if spr.pps {
        rows[0][3..6].copy_from_slice(b"PPS");
    }
    if let Some(cc) = observation.selected_cc {
        rows[0][12] = b'0' + cc;
    }
    put(&mut rows[1], b"REQ:--.--V -.-A N- R- I-");
    decimal2(&mut rows[1], 4, spr.target_mv);
    // The current in the last Request sent (fixed: the object's maximum up to the
    // board rating; PPS: the operating current setpoint).
    let ma = if spr.pps {
        spr.target_ma
    } else {
        spr.request_ma
    };
    if ma != 0 {
        rows[1][11] = b'0' + (ma / 1000).min(9) as u8;
        rows[1][13] = b'0' + ((ma / 100) % 10) as u8;
    }
    if let Some(caps) = observation.capabilities {
        rows[1][17] = b'0' + caps.count.min(9);
    }
    if let Some(rev) = spr.peer_revision {
        rows[1][20] = b'0' + rev.min(9);
    }
    if let Some(id) = spr.request_id {
        rows[1][23] = b'0' + (id & 7);
    }
    put(&mut rows[2], b"TRY:- ACK:- ACC:- RDY:-");
    for (offset, flag) in [
        (4, spr.tx_written),
        (10, spr.tx_goodcrc && spr.tx_sent_irq),
        (16, spr.accept_seen),
        (22, spr.psrdy_seen),
    ] {
        rows[2][offset] = if flag { b'Y' } else { b'-' };
    }
    if spr.tx_ack_goodcrc_only {
        rows[2][10] = b'G'; // GoodCRC frame without I_TXSENT (section 7.32.1)
    }
    put(&mut rows[3], b"MS: AK:--- AC:--- RD:---");
    if let Some(ms) = spr.elapsed_ack_ms {
        decimal3(&mut rows[3], 7, ms);
    }
    if let Some(ms) = spr.elapsed_accept_ms {
        decimal3(&mut rows[3], 14, ms);
    }
    if let Some(ms) = spr.elapsed_psrdy_ms {
        decimal3(&mut rows[3], 21, ms);
    }
    put(&mut rows[4], b"V:G--.-- RD--.-- RT--.--");
    if let Some(mv) = spr.vbus_pre_mv {
        decimal2(&mut rows[4], 3, mv);
    }
    if let Some(mv) = spr.vbus_rdy_mv {
        decimal2(&mut rows[4], 11, mv);
    }
    if let Some(mv) = spr.vbus_post_mv {
        decimal2(&mut rows[4], 19, mv);
    }
    if let Some(mv) = spr.vbus_live_mv {
        rows[4] = [b' '; 24];
        put(&mut rows[4], b"VBUS:--.--V MEASURED");
        decimal2(&mut rows[4], 5, mv);
    }
    put(&mut rows[5], b"ERR:---- A:---- R:----");
    hex(&mut rows[5], 4, observation.rx_error as u16);
    hex(&mut rows[5], 11, spr.failed_address as u16);
    if observation.failed_register != 0 {
        hex(&mut rows[5], 18, observation.failed_register as u16);
    }
    // Section 7.2: the last AMS start was refused because Rp never showed
    // Report SinkTxOK gating explicitly. Cleared by the
    // next gated send.
    if observation.rx_error == 0 && spr.sink_tx_cause == b'G' {
        rows[5] = [b' '; 24];
        put(&mut rows[5], b"CHARGER BLOCKS CHANGES");
    }
    // Section 7.3: a Source Message arrived first and ours was dropped unsent.
    if observation.rx_error == 0 && spr.sink_tx_cause == b'C' {
        rows[5] = [b' '; 24];
        put(&mut rows[5], b"TX LOST TO CHARGER MSG");
    }
    // Owner decision: stale SinkTxNG on a quiet line is overridden; say so.
    if observation.rx_error == 0 && spr.sink_tx_cause == b'Q' {
        rows[5] = [b' '; 24];
        put(&mut rows[5], b"CHARGER BUSY: OVERRIDDEN");
    }
    put(&mut rows[6], b"POST S1:---- C3:----    ");
    if observation.phase == Phase::Contracted
        && let Some(after) = observation.post_snapshot
    {
        hex(&mut rows[6], 8, after.values[2] as u16);
        hex(&mut rows[6], 16, after.values[6] as u16);
        if spr.packets_serviced > 0 {
            rows[6][21] = b'N';
            rows[6][22] = b'0' + ((spr.packets_serviced / 10) % 10) as u8;
            rows[6][23] = b'0' + (spr.packets_serviced % 10) as u8;
        }
    }
    if observation.restore_unverified {
        rows[6] = [b' '; 24];
        put(&mut rows[6], b"RESTORE NOT VERIFIED");
    }
    if observation.phase == Phase::Negotiating && spr.psrdy_seen {
        rows[6] = [b' '; 24];
        put(&mut rows[6], b"HOLD BACK TO STOP");
    }
    if observation.phase == Phase::Negotiating
        && let Some(mv) = spr.selected_mv
    {
        rows[6] = [b' '; 24];
        put(&mut rows[6], b"SEL:--V");
        decimal_voltage(&mut rows[6], 4, mv);
        if spr.selected_pps {
            rows[6].fill(b' ');
            if spr.adjust_current {
                put(&mut rows[6], b"SEL:P--.-- --.--A [I] OK");
            } else {
                put(&mut rows[6], b"SEL:P--.-- --.--A [V] OK");
            }
            decimal2(&mut rows[6], 5, mv);
            if let Some(ma) = spr.selected_ma {
                decimal2(&mut rows[6], 11, ma);
            }
        }
    }
    if observation.phase == Phase::Negotiating && !spr.selected_pps && spr.selected_mv == Some(5000)
    {
        if spr.cable_active {
            rows[5].fill(b' ');
            rows[6].fill(b' ');
            put(&mut rows[5], b"CABLE QUERY - 5V ONLY");
            put(&mut rows[6], b"WAIT - NO RETRY");
        } else if let Some(report) = spr.cable_report {
            rows[5].fill(b' ');
            rows[6].fill(b' ');
            match report.status {
                CableStatus::Identity => {
                    let decoded = (report.count <= 7).then(|| {
                        firmware_core::pd_cable::decode(
                            &report.objects[..report.count as usize],
                            report.revision,
                        )
                    });
                    if let Some(Ok(firmware_core::pd_cable::Response::Identity(identity))) = decoded
                    {
                        if identity.epr_capable == Some(true) {
                            put(&mut rows[5], b"CABLE:P --V -A EPR S:-");
                            if identity.kind == firmware_core::pd_cable::CableKind::Active {
                                rows[5][6] = b'A';
                            }
                            if let Some(mv) = identity.maximum_voltage_mv {
                                decimal_voltage(&mut rows[5], 8, mv as u16);
                            }
                            if let Some(ma) = identity.current_ma {
                                rows[5][12] = b'0' + (ma / 1000) as u8;
                            }
                            rows[5][21] = b'0' + identity.speed_code;
                        } else {
                            put(&mut rows[5], b"CABLE:P --V -A SPD:-");
                            if identity.kind == firmware_core::pd_cable::CableKind::Active {
                                rows[5][6] = b'A';
                            }
                            if let Some(mv) = identity.maximum_voltage_mv {
                                decimal_voltage(&mut rows[5], 8, mv as u16);
                            }
                            if let Some(ma) = identity.current_ma {
                                rows[5][12] = b'0' + (ma / 1000) as u8;
                            }
                            rows[5][19] = b'0' + identity.speed_code;
                        }
                        if identity.current_assumed {
                            rows[5][14] = b'*';
                        }
                        put(&mut rows[6], b"VID:---- PID:----");
                        hex(&mut rows[6], 4, identity.vid);
                        hex(&mut rows[6], 13, identity.pid);
                    } else {
                        put(&mut rows[5], b"CABLE ID MALFORMED");
                    }
                }
                _ => {
                    let (reason, hint) = cable::cable_failure(report.status, spr.cable_modes.sync);
                    put(&mut rows[5], reason);
                    if !hint.is_empty() {
                        rows[6].fill(b' ');
                        put(&mut rows[6], hint);
                    }
                }
            }
        }
    }
    if (spr.cable_active || spr.vconn_on) && observation.rx_error != 0 {
        rows[6].fill(b' ');
        put(&mut rows[6], b"VCONN STATE UNVERIFIED");
    }
    if let Some(value) = observation.failed_value {
        rows[6] = [b' '; 24];
        put(&mut rows[6], b"VALUE:----");
        hex(&mut rows[6], 6, value as u16);
    }
    phase_line(&mut rows[7], observation);
    // ERR 000D evidence (the register is uninformative there): STATUS1,
    // INTERRUPTA during the wait, partner Messages meanwhile and the first one's
    // header; whether a GoodCRC frame arrived, and whether the stale-SinkTxNG
    // override sent it.
    if let (0x0d, Some(timeout)) = (observation.rx_error, spr.tx_timeout) {
        rows[6] = [b' '; 24];
        put(&mut rows[6], b"TO S1:-- IA:-- N- H:----");
        hex8(&mut rows[6], 6, timeout.status1);
        hex8(&mut rows[6], 12, timeout.flags);
        rows[6][16] = b'0' + timeout.packets.min(9);
        hex(&mut rows[6], 20, timeout.header);
        rows[7] = [b' '; 24];
        put(&mut rows[7], b"TO GOODCRC:- OVERRIDE:-");
        rows[7][11] = if spr.tx_goodcrc { b'Y' } else { b'N' };
        rows[7][22] = if timeout.cause == b'Q' { b'Y' } else { b'N' };
    }
    if observation.boot_step != 0 {
        rows[7] = [b' '; 24];
        put(&mut rows[7], b"S:-- P:-- H:---- T:----");
        rows[7][2] = b'0' + (observation.boot_step / 10 % 10);
        rows[7][3] = b'0' + observation.boot_step % 10;
        rows[7][7] = b'0' + ((observation.rx_packets / 10) % 10) as u8;
        rows[7][8] = b'0' + (observation.rx_packets % 10) as u8;
        if let Some(header) = observation.rx_last_header {
            hex(&mut rows[7], 12, header);
        }
        decimal4(&mut rows[7], 19, observation.boot_step_ms);
    }
    if let Some(last) = observation.last_boot {
        rows[6] = [b' '; 24];
        if !last.logging && last.record.is_none() {
            put(&mut rows[6], b"LOG: NO FREE FLASH");
        } else {
            put(&mut rows[6], b"L-- E--- P-- H---- T----");
        }
        // Session timings belong to session records (steps 1-9) only.
        if let Some(record) = last.record.filter(|r| r.step < 10) {
            rows[4] = [b' '; 24];
            put(&mut rows[4], b"3:---- 5:---- G:---");
            if record.step3_ms != 0 {
                decimal4(&mut rows[4], 2, record.step3_ms);
            }
            if record.step5_ms != 0 {
                decimal4(&mut rows[4], 9, record.step5_ms);
            }
            decimal3(&mut rows[4], 16, record.poll_gap_ms as u16);
            rows[2] = [b' '; 24];
            put(&mut rows[2], b"L:--- X:--- D:---");
            decimal3(&mut rows[2], 2, record.lock_wait_ms as u16);
            decimal3(&mut rows[2], 8, record.transfer_ms as u16);
            decimal3(&mut rows[2], 14, record.delay_over_ms as u16);
        }
        if let Some(record) = last.record {
            rows[6][1] = b'0' + record.step / 10 % 10;
            rows[6][2] = b'0' + record.step % 10;
            decimal3(&mut rows[6], 5, record.error as u16);
            rows[6][10] = b'0' + (record.packets / 10) % 10;
            rows[6][11] = b'0' + record.packets % 10;
            hex(&mut rows[6], 14, record.header);
            decimal4(&mut rows[6], 20, record.ms);
            // Fatal records (flash_log PANIC 250 / FAULT 251): five code
            // addresses replace the timings, the packet and header fields.
            if record.error >= 250 {
                let put_address = |row: &mut [u8; 24], at: usize, label: u8, address: u32| {
                    row[at] = label;
                    row[at + 1] = b':';
                    hex(row, at + 2, (address >> 16) as u16);
                    hex(row, at + 6, address as u16);
                };
                rows[2] = [b' '; 24];
                rows[4] = [b' '; 24];
                put_address(&mut rows[2], 0, b'A', record.chain[0]);
                put_address(&mut rows[2], 11, b'B', record.chain[1]);
                put_address(&mut rows[4], 0, b'C', record.chain[2]);
                put_address(&mut rows[4], 11, b'D', record.chain[3]);
                rows[6][9..].fill(b' ');
                put_address(&mut rows[6], 9, b'E', record.chain[4]);
            }
        }
    }
    // A previous boot's panic/fault record keeps row 2 (the Rd probe runs
    // right after the contract and would otherwise hide it).
    let fatal = observation
        .last_boot
        .and_then(|last| last.record)
        .is_some_and(|r| r.error >= 250);
    if let Some([both, single]) = observation.rp_probe.filter(|_| !fatal) {
        // BC_LVL is STATUS0 bits 1:0 (2 = SinkTxNG band, 3 = SinkTxOK).
        rows[2] = [b' '; 24];
        put(&mut rows[2], b"RP 2RD:-- 1RD:--");
        hex8(&mut rows[2], 7, both);
        hex8(&mut rows[2], 14, single);
    }
    rows
}
fn decimal4(row: &mut [u8; 24], start: usize, val: u16) {
    let val = val.min(9999);
    for (i, div) in [1000, 100, 10, 1].into_iter().enumerate() {
        row[start + i] = b'0' + ((val / div) % 10) as u8;
    }
}
fn decimal_voltage(row: &mut [u8; 24], start: usize, mv: u16) {
    row[start] = b'0' + ((mv / 10_000) % 10) as u8;
    row[start + 1] = b'0' + ((mv / 1_000) % 10) as u8;
}
fn decimal3(row: &mut [u8; 24], start: usize, val: u16) {
    let val = val.min(999);
    row[start] = b'0' + (val / 100) as u8;
    row[start + 1] = b'0' + ((val / 10) % 10) as u8;
    row[start + 2] = b'0' + (val % 10) as u8;
}
fn decimal2(row: &mut [u8; 24], start: usize, milli: u16) {
    row[start] = b'0' + (milli / 10000) as u8;
    row[start + 1] = b'0' + ((milli / 1000) % 10) as u8;
    row[start + 3] = b'0' + ((milli / 100) % 10) as u8;
    row[start + 4] = b'0' + ((milli / 10) % 10) as u8;
}

fn put(row: &mut [u8; 24], text: &[u8]) {
    row[..text.len()].copy_from_slice(text);
}
fn hex8(row: &mut [u8; 24], offset: usize, value: u8) {
    row[offset] = b"0123456789ABCDEF"[(value >> 4) as usize];
    row[offset + 1] = b"0123456789ABCDEF"[(value & 15) as usize];
}
fn hex(row: &mut [u8; 24], offset: usize, value: u16) {
    for i in 0..4 {
        row[offset + i] = b"0123456789ABCDEF"[((value >> (12 - 4 * i)) & 15) as usize];
    }
}
