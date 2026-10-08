//! X1 observation adapter and renderer; controller I/O/telemetry live in fusb302-pd.
use kws_x1_core::pd_cable::CableStatus;
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
            sniff: None,
            last_boot: None,
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

/// What the previous boot recorded last before it ended (`docs/PD-BOOT-RESTART.md`).
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
    pub rp: Option<kws_x1_core::pd_sniff::Rp>,
    pub log: kws_x1_core::pd_sniff::Log,
    pub caps: [u32; 7],
    pub caps_count: u8,
    pub flags: [u8; 3],
    pub stop: Option<SniffStop>,
}

impl SniffObservation {
    pub const fn new() -> Self {
        Self {
            cc: None,
            cc_status: None,
            rp: None,
            log: kws_x1_core::pd_sniff::Log::new(),
            caps: [0; 7],
            caps_count: 0,
            flags: [0; 3],
            stop: None,
        }
    }
    /// Log one CRC-checked SOP frame. Source_Capabilities replace the stored
    /// PDOs first, so a later Request decodes against the newest offer.
    pub fn record(&mut self, frame: &kws_x1_core::pd_rx::Frame<'_>) {
        if frame.header() & 0x8000 == 0
            && let Ok(objects) = frame.source_capabilities()
        {
            self.caps_count = objects.len().min(7) as u8;
            for (slot, raw) in self.caps.iter_mut().zip(objects) {
                *slot = raw;
            }
        }
        let caps = &self.caps[..self.caps_count as usize];
        self.log.push(kws_x1_core::pd_sniff::Entry::classify(
            frame.header(),
            frame.data(),
            caps,
        ));
    }
}

fn phase_line(row: &mut [u8; 24], observation: Observation) {
    match observation.phase {
        Phase::Waiting => put(row, b"WAIT CC TEST"),
        Phase::Passive => put(row, b"PASSIVE TX OFF"),
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
    if observation.spr.is_some() {
        return spr_lines(observation);
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
    use kws_x1_core::pd_sniff::{Row, Rp, describe};
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
        put(&mut rows[7], b"NEXT PAGE: CAPS");
        return rows;
    }
    for (row, entry) in rows[1..].iter_mut().zip(sniff.log.entries()) {
        *row = describe(entry);
    }
    rows
}

/// Passthrough detail page: the latest advertised Source_Capabilities.
fn sniff_caps_lines(sniff: SniffObservation) -> [[u8; 24]; 8] {
    use kws_x1_core::pd_sniff::{Row, describe_pdo};
    let mut rows = [[b' '; 24]; 8];
    let mut head = Row::new();
    head.text(b"SNIFF CAPS N")
        .decimal(sniff.caps_count as u32)
        .text(b" GCRC:")
        .decimal(sniff.log.goodcrc);
    rows[0] = head.bytes;
    if sniff.caps_count == 0 {
        put(&mut rows[1], b"NO SOURCE CAPS SEEN");
        put(&mut rows[2], b"REPLUG SINK TO CAPTURE");
    }
    for (i, &raw) in sniff.caps[..sniff.caps_count as usize].iter().enumerate() {
        rows[i + 1] = describe_pdo(i, raw);
    }
    rows
}

fn rx_lines(observation: Observation) -> [[u8; 24]; 8] {
    let mut rows = [[b' '; 24]; 8];
    if observation.phase == Phase::Captured {
        let capabilities = observation.capabilities.expect("captured advertisements");
        put(&mut rows[0], b"PD CAPS C- N- ROK TX OFF");
        rows[0][9] = b'0' + observation.selected_cc.unwrap();
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
fn spr_lines(observation: Observation) -> [[u8; 24]; 8] {
    let spr = observation.spr.unwrap();
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
        rows[2][10] = b'G'; // GoodCRC frame without I_TXSENT (section 6.7.1)
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
    // SinkTxOK (owner decision 2026-10-06: stay strict, say so). Cleared by the
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
        put(&mut rows[6], b"HOLD PB2 PB10 TO STOP");
    }
    if observation.phase == Phase::Negotiating
        && let Some(mv) = spr.selected_mv
    {
        rows[6] = [b' '; 24];
        put(&mut rows[6], b"SEL:--V PB2 NEXT PB10 OK");
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
            put(&mut rows[6], b"PB2 NEXT PB10 OK");
            match report.status {
                CableStatus::Identity => {
                    let decoded = (report.count <= 7).then(|| {
                        kws_x1_core::pd_cable::decode(
                            &report.objects[..report.count as usize],
                            report.revision,
                        )
                    });
                    if let Some(Ok(kws_x1_core::pd_cable::Response::Identity(identity))) = decoded {
                        if identity.epr_capable == Some(true) {
                            put(&mut rows[5], b"CABLE:P --V -A EPR S:-");
                            if identity.kind == kws_x1_core::pd_cable::CableKind::Active {
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
                            if identity.kind == kws_x1_core::pd_cable::CableKind::Active {
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
                    let (reason, hint) = cable_failure(report.status, spr.cable_modes.sync);
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

/// Paint compact 2x glyphs into the caller-owned, already-cleared meter frame.
/// Eight rows fit the 135px panel; the original meter font remains unchanged.
pub fn paint(framebuffer: &mut [u16; 32400], observation: Observation, glyph: fn(u8) -> u16) {
    paint_lines(framebuffer, lines(observation), glyph);
}

pub fn paint_cable(
    framebuffer: &mut [u16; 32400],
    observation: Observation,
    page: u32,
    svid: Option<u16>,
    glyph: fn(u8) -> u16,
) {
    paint_lines(framebuffer, cable_lines(observation, page, svid), glyph);
}

/// Selected SVID's ascending-set index; no bitmap is copied into observations.
pub fn control_cable_selection(
    observation: &mut fusb302_pd::observation::Observation,
    selection: &kws_x1_core::pd_spr::Selection,
    page: Option<u32>,
    store: Option<&dyn kws_x1_core::pd_svids::Store>,
) {
    let (Some(page), Some(store), Some(spr)) = (page, store, observation.spr) else {
        return;
    };
    if selection.cable_active()
        || selection.has_pending()
        || spr.cable_modes.status != kws_x1_core::pd_cable_modes::Status::Ack
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
        observation.spr.as_mut().unwrap().cable_modes.mode = kws_x1_core::pd_cable_modes::Modes {
            svid,
            ..Default::default()
        };
    }
}

pub fn cable_svid_index(observation: Observation, page: u32) -> Option<u16> {
    let spr = observation.spr?;
    let report = spr.cable_report?;
    let fields = kws_x1_core::pd_cable_details::field_count(&report);
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
    use kws_x1_core::{pd_cable_details, pd_cable_modes::Status};
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
                    match kws_x1_core::pd_cable::second_controller(&report) {
                        Some(false) => b"NO SOP2 ADVERTISED",
                        None => b"LAYOUT/PRESENCE UNKNOWN",
                        Some(true) if report.revision != 2 => b"PD2 NO GET_STATUS",
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
fn cable_failure(
    status: CableStatus,
    sync: kws_x1_core::pd_cable_modes::Sync,
) -> (&'static [u8], &'static [u8]) {
    use kws_x1_core::pd_cable_modes::Sync;
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

fn status_caption(status: kws_x1_core::pd_cable_modes::Status) -> &'static [u8] {
    use kws_x1_core::pd_cable_modes::Status;
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

/// Charger page (page 5): what the Source said about itself in reply to the
/// information requests (`pd_partner`), on three sub-pages (PB2/PB10). These
/// are the Source's claims, not measurements.
pub fn partner_lines(observation: Observation, page: u32) -> [[u8; 24]; 8] {
    use kws_x1_core::pd_partner::Answer;
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
fn answer_caption<T>(answer: &kws_x1_core::pd_partner::Answer<T>) -> &'static [u8] {
    use kws_x1_core::pd_partner::Answer;
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
    partner: kws_x1_core::pd_partner::PartnerInfo,
) -> [[u8; 24]; 8] {
    use kws_x1_core::pd_partner::Answer;
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
pub fn paint_partner(
    framebuffer: &mut [u16; 32400],
    observation: Observation,
    page: u32,
    glyph: fn(u8) -> u16,
) {
    paint_lines(framebuffer, partner_lines(observation, page), glyph);
}

fn paint_lines(framebuffer: &mut [u16; 32400], rows: [[u8; 24]; 8], glyph: fn(u8) -> u16) {
    for (line_index, line) in rows.iter().enumerate() {
        for (char_index, &c) in line.iter().enumerate() {
            // Compact diagnostic captions need N; do not alter the meter font.
            let bitmap = if c == b'N' {
                0b101_101_111_111_101
            } else {
                glyph(c)
            };
            for row in 0..5 {
                for column in 0..3 {
                    if bitmap & (1 << (row * 3 + 2 - column)) != 0 {
                        for dy in 0..2 {
                            for dx in 0..2 {
                                let x = 12 + char_index * 8 + column * 2 + dx;
                                let y = 6 + line_index * 16 + row * 2 + dy;
                                framebuffer[y * 240 + x] = 0;
                            }
                        }
                    }
                }
            }
        }
    }
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
