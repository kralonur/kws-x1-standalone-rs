//! X1 limits and button adapter for the reusable PD session engine.
use core::cell::Cell;
use pd_protocol::pd_spr as shared;
use usbpd::protocol_layer::message::data::{request, source_capabilities};

pub use shared::{
    Contract, Error, MaintainedTrace, Mode, PPS_MIN_MA, PpsStatus, ReviewedRdo, Timer,
};
// Preserve the existing X1 scheduling: delays are policy, not wire timers.
pub const PPS_STATUS_SETTLE_MS: u64 = 300;
pub const QUERY_SETTLE_MS: u64 = 500;
pub const STATUS_MIN_INTERVAL_MS: u64 = 200;
pub const SESSION_POLICY: shared::SessionPolicy = shared::SessionPolicy {
    pps_status_settle_ms: PPS_STATUS_SETTLE_MS,
    query_settle_ms: QUERY_SETTLE_MS,
    status_min_interval_ms: STATUS_MIN_INTERVAL_MS,
    retry_delay_ms: 100,
    pps_expiry_grace_ms: 5_000,
    pps_status_max_deferrals: 3,
    query_max_deferrals: 3,
};
pub const MAX_REQUEST_MV: u32 = 30_000;
pub const MAX_PPS_MV: u32 = 21_000;
pub const TARGET_MV: u32 = 9_000;
pub const REQUEST_MA: u32 = 100;
/// Board marking "4-30V 0-12A" (owner, 2026-10-07). Fixed Requests ask for the
/// object's Maximum Current; PD objects never exceed 5 A, so 12 A never binds.
pub const MAX_CURRENT_MA: u32 = 12_000;
pub const PPS_REFRESH_MS: u64 = 4_000;

pub struct X1Limits;
impl shared::Limits for X1Limits {
    // Preserve the no-VID/no-metadata convention (§6.1.5, §7.19.4.1).
    const SINK_IDENTITY: shared::SinkIdentity = shared::SinkIdentity::UNASSIGNED;
    const SESSION_POLICY: shared::SessionPolicy = SESSION_POLICY;
    const MAX_REQUEST_MV: u32 = MAX_REQUEST_MV;
    const MIN_PPS_MV: u32 = 5_000;
    const MAX_PPS_MV: u32 = MAX_PPS_MV;
    const TARGET_MV: u32 = TARGET_MV;
    const REQUEST_MA: u32 = REQUEST_MA;
    const MAX_CURRENT_MA: u32 = MAX_CURRENT_MA;
    // The charger-facing port has no USB data path to the charger.
    const USB_COMMUNICATIONS_CAPABLE: bool = false;
    const NO_USB_SUSPEND: bool = true;
    // Trigger mode needs 9-20 V for full function (Table 6.9).
    const HIGHER_CAPABILITY: bool = true;
    const UNCONSTRAINED_POWER: bool = false;
    // Powered from VBUS only: no battery, no own supply (Table 6.61 bit 1).
    const SINK_POWER_MODES: u8 = 0b0000_0010;
    const FIXED_TARGETS: &'static [u16] = &[5_000, 9_000, 12_000, 15_000, 20_000];
    const PPS_REFRESH_MS: u64 = PPS_REFRESH_MS;
    fn status() -> Option<[u8; 7]> {
        // Table 6.51: DC input (VBUS), no battery; temperature reporting and
        // power-state change reporting unsupported. No qualified fault events.
        Some([0, 2, 0, 0, 0, 0, 0])
    }
}
pub type Trial = shared::Trial<X1Limits>;

pub fn request_pps(
    caps: &source_capabilities::SourceCapabilities,
    object: u8,
    mv: u16,
    ma: u16,
) -> Result<request::PowerSource, Error> {
    shared::request_pps::<X1Limits>(caps, object, mv, ma)
}
pub fn request_target(
    caps: &source_capabilities::SourceCapabilities,
) -> Result<request::PowerSource, Error> {
    shared::request_target::<X1Limits>(caps)
}
pub fn request_fixed(
    caps: &source_capabilities::SourceCapabilities,
    mv: u32,
) -> Result<request::PowerSource, Error> {
    shared::request_fixed::<X1Limits>(caps, mv)
}

pub struct Selection {
    inner: shared::Selection<X1Limits>,
    held_since_ms: Cell<u64>,
    pb10_held_since_ms: Cell<u64>,
    pb10_held: Cell<bool>,
    repeat_ms: Cell<u64>,
    buttons: Cell<u8>,
    mode_cursor: Cell<Option<u16>>,
}
impl core::ops::Deref for Selection {
    type Target = shared::Selection<X1Limits>;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}
impl Selection {
    pub const fn new() -> Self {
        Self {
            inner: shared::Selection::new(),
            held_since_ms: Cell::new(0),
            pb10_held_since_ms: Cell::new(0),
            pb10_held: Cell::new(false),
            repeat_ms: Cell::new(0),
            buttons: Cell::new(0),
            mode_cursor: Cell::new(None),
        }
    }
    pub fn mode_cursor(&self) -> Option<u16> {
        self.mode_cursor.get()
    }
    pub fn set_mode_cursor(&self, index: Option<u16>) {
        self.mode_cursor.set(index);
    }
    /// PB2 previews, PB10 confirms/holds. Screen and hold timing stay X1-local.
    pub fn update_buttons(&self, snapshot: u8) {
        let buttons = snapshot & 15;
        let old = self.buttons.replace(buttons);
        let rising = buttons & !old;
        let falling = !buttons & old;
        let now = self.now_ms();
        if snapshot >> 4 != 3
            || buttons & 3 == 3
            || self.has_pending()
            || self.request_in_flight()
            || self.cable_active()
        {
            self.held_since_ms.set(now);
            self.repeat_ms.set(now);
            self.pb10_held_since_ms.set(now);
            // A PB10 press during a busy period (cable query, Request, other
            // page) is used up until released: holding it past the end must
            // not start another query or confirm 500 ms later.
            self.pb10_held.set(buttons & 2 != 0);
            return;
        }
        if self.option_count() == 0 {
            return;
        }
        let is_fixed = !self.candidate_is_pps();
        if rising & 2 != 0 {
            self.pb10_held_since_ms.set(now);
            self.pb10_held.set(false);
        }
        if buttons & 2 != 0
            && !self.pb10_held.get()
            && now.saturating_sub(self.pb10_held_since_ms.get()) >= 500
        {
            if is_fixed {
                self.request_cable();
            } else {
                self.toggle_adjustment();
            }
            self.pb10_held.set(true);
        }
        let should_confirm = (is_fixed && rising & 2 != 0)
            || (!is_fixed && falling & 2 != 0 && !self.pb10_held.get());
        if should_confirm {
            self.confirm();
        }
        if buttons & 2 == 0 {
            self.pb10_held.set(false);
        }
        if rising & 1 != 0 {
            self.held_since_ms.set(now);
            self.repeat_ms.set(now);
        }
        let repeat = !is_fixed
            && buttons & 1 != 0
            && now.saturating_sub(self.held_since_ms.get()) >= 500
            && now.saturating_sub(self.repeat_ms.get()) >= 100;
        if rising & 1 != 0 || repeat {
            self.repeat_ms.set(now);
            self.advance(if repeat { 100 } else { 20 }, if repeat { 250 } else { 50 });
        }
    }
}
impl Default for Selection {
    fn default() -> Self {
        Self::new()
    }
}
