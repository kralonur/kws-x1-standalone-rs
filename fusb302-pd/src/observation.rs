//! Controller/protocol telemetry; board UI and flash history stay with callers.
use crate::snapshot::Snapshot;
use pd_protocol::pd_cable::CableReport;
use pd_protocol::pd_spr::Supply;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    Waiting,
    /// Passive configuration: FUSB302 Rd released, no TX/GoodCRC/RX.
    Passive,
    Failed,
    Skipped,
    Measuring,
    Guarded,
    Verify,
    Receiving,
    Captured,
    NoCapabilities,
    Negotiating,
    Contracted,
    /// `pd-hold-diagnostic` build: Rd kept after the CC check, no session.
    Hold,
}

/// What the PHY showed when a transmission timed out (`pd_spr_phy`, ERR 000D).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxTimeout {
    /// STATUS1 at the last poll: bit 3 TX_EMPTY (clear = our bytes still queued),
    /// bit 5 RX_EMPTY.
    pub status1: u8,
    /// INTERRUPTA bits seen while ours waited: 0x04 TXSENT, 0x10 RETRYFAIL.
    pub flags: u8,
    /// Partner Messages received while ours waited, and the first one's header.
    pub packets: u8,
    pub header: u16,
    /// `sink_tx_cause` of this send: b'Q' sent through the stale-SinkTxNG override.
    pub cause: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SprObservation {
    pub vbus_pre_mv: Option<u16>,
    pub vbus_rdy_mv: Option<u16>,
    pub vbus_post_mv: Option<u16>,
    pub vbus_live_mv: Option<u16>,
    pub selected_mv: Option<u16>,
    pub selected_ma: Option<u16>,
    pub selected_supply: Supply,
    pub adjust_current: bool,
    pub cable_active: bool,
    pub vconn_on: bool,
    /// BIST Carrier Mode transmissions (section 5.4.1).
    pub bist_carriers: u8,
    pub cable_report: Option<CableReport>,
    pub cable_modes: pd_protocol::pd_cable_modes::Discovery,
    /// What the target asks for: fixed, PPS or SPR AVS.
    pub supply: Supply,
    pub tx_off_verified: bool,
    pub tx_written: bool,
    pub tx_sent_irq: bool,
    pub tx_goodcrc: bool,
    pub accept_seen: bool,
    pub psrdy_seen: bool,
    pub contract_seen: bool,
    pub failed_address: u8,
    pub peer_revision: Option<u8>,
    pub request_id: Option<u8>,
    pub elapsed_ack_ms: Option<u16>,
    pub elapsed_accept_ms: Option<u16>,
    pub elapsed_psrdy_ms: Option<u16>,
    pub packets_serviced: u16,
    pub target_mv: u16,
    pub target_ma: u16,
    /// Operating current of the last Request sent, in mA (RDO bits 19..10 for
    /// fixed, 6..0 for PPS and SPR AVS).
    pub request_ma: u16,
    /// Last deferral: b'G' SinkTxOK not seen in time, b'M' SOP Message first, b'C' a
    /// SOP Message arrived and ours was dropped unsent;
    /// or b'Q': the last AMS start was sent after a quiet stale SinkTxNG window.
    pub sink_tx_cause: u8,
    /// What the Source answered to the information requests (`pd_partner`).
    pub partner: pd_protocol::pd_partner::PartnerInfo,
    /// The Source's EPR offer, when asked (`pd_spr::Limits::EPR_OFFER`).
    pub epr_offer: pd_protocol::pd_spr::EprOffer,
    pub epr_state: pd_protocol::pd_spr::EprState,
    pub epr_keep_alives: u16,
    /// Largest observed SOP traffic gap during EPR, ms. Timestamped at the
    /// native receive/TX-ack boundary, not an electrical waveform measurement.
    pub epr_max_silence_ms: u16,
    /// Last transmission that saw neither GoodCRC nor I_RETRYFAIL (ERR 000D).
    pub tx_timeout: Option<TxTimeout>,
    /// The last transmission was acknowledged by its GoodCRC frame without the
    /// FUSB302's I_TXSENT (`ACK:G`).
    pub tx_ack_goodcrc_only: bool,
}
impl SprObservation {
    pub const fn pending() -> Self {
        Self {
            vbus_pre_mv: None,
            vbus_rdy_mv: None,
            vbus_post_mv: None,
            vbus_live_mv: None,
            selected_mv: None,
            selected_ma: None,
            selected_supply: Supply::Fixed,
            adjust_current: false,
            cable_active: false,
            vconn_on: false,
            bist_carriers: 0,
            cable_report: None,
            cable_modes: pd_protocol::pd_cable_modes::Discovery::EMPTY,
            supply: Supply::Fixed,
            tx_off_verified: false,
            tx_written: false,
            tx_sent_irq: false,
            tx_goodcrc: false,
            accept_seen: false,
            psrdy_seen: false,
            contract_seen: false,
            failed_address: 0x22,
            peer_revision: None,
            request_id: None,
            elapsed_ack_ms: None,
            elapsed_accept_ms: None,
            elapsed_psrdy_ms: None,
            packets_serviced: 0,
            target_mv: 0,
            target_ma: 0,
            request_ma: 0,
            sink_tx_cause: 0,
            partner: pd_protocol::pd_partner::PartnerInfo::PENDING,
            epr_offer: pd_protocol::pd_spr::EprOffer::NotAsked,
            epr_state: pd_protocol::pd_spr::EprState::Spr,
            epr_keep_alives: 0,
            epr_max_silence_ms: 0,
            tx_timeout: None,
            tx_ack_goodcrc_only: false,
        }
    }
}

#[derive(Clone, Copy)]
pub struct Observation {
    pub snapshot: Option<Snapshot>,
    pub phase: Phase,
    pub failed_register: u8,
    pub failed_value: Option<u8>,
    pub irq_high: bool,
    pub irq_count: u32,
    pub restore_unverified: bool,
    pub cc_status: Option<[u8; 2]>,
    pub post_snapshot: Option<Snapshot>,
    pub selected_cc: Option<u8>,
    pub capabilities: Option<pd_protocol::pd_rx::Advertisements>,
    pub rx_polls: u16,
    pub rx_packets: u16,
    pub rx_last_header: Option<u16>,
    pub rx_flags: [u8; 3],
    pub rx_error: u8,
    pub spr: Option<SprObservation>,
    /// `pd-boot-diagnostic` only (`docs/SAFETY.md`): furthest session
    /// step reached (0 = not traced) and ms since GoodCRC was enabled.
    pub boot_step: u8,
    pub boot_step_ms: u16,
    /// Furthest cable inspection breadcrumb (10-31, `pd_spr_phy::cable_step`),
    /// with `trace_steps` or `trace_cable`; 0 = none.
    pub cable_step: u8,
    /// Cable queries started in this session (diagnostic builds).
    pub cable_queries: u8,
    /// `pd-boot-diagnostic` only: longest gap between two receive polls since
    /// GoodCRC was enabled, ms (capped at 255).
    pub poll_gap_ms: u8,
    /// `pd-boot-diagnostic` only (`docs/SAFETY.md`): STATUS0 on the active CC
    /// once after the first contract, with Rd on both CCs, then with Rd on the
    /// active CC only; the original SWITCHES0 is restored.
    pub rp_probe: Option<[u8; 2]>,
}

impl Default for Observation {
    fn default() -> Self {
        Self::new()
    }
}
impl Observation {
    pub const fn new() -> Self {
        Self {
            snapshot: None,
            phase: Phase::Waiting,
            failed_register: 0,
            failed_value: None,
            irq_high: true,
            irq_count: 0,
            restore_unverified: false,
            cc_status: None,
            post_snapshot: None,
            selected_cc: None,
            capabilities: None,
            rx_polls: 0,
            rx_packets: 0,
            rx_last_header: None,
            rx_flags: [0; 3],
            rx_error: 0,
            spr: None,
            boot_step: 0,
            boot_step_ms: 0,
            cable_step: 0,
            cable_queries: 0,
            poll_gap_ms: 0,
            rp_probe: None,
        }
    }
}
