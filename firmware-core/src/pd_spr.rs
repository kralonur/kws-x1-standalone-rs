//! Board limits and button adapter for the reusable PD session engine.
use core::cell::Cell;

use crate::buttons::PdCommand;
use pd_protocol::pd_spr as shared;
use usbpd::protocol_layer::message::data::{request, source_capabilities};

pub use shared::{
    Contract, EPR_OBJECTS, EprOffer, Error, MaintainedTrace, Mode, PPS_MIN_MA, PpsStatus,
    ReviewedRdo, Timer,
};
// Preserve the existing board scheduling: delays are policy, not wire timers.
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
/// Board marking "4-30V 0-12A". Fixed Requests ask for the
/// object's Maximum Current; PD objects never exceed 5 A, so 12 A never binds.
pub const MAX_CURRENT_MA: u32 = 12_000;
pub const PPS_REFRESH_MS: u64 = 4_000;
/// Fixed supply PDO units: 50 mV voltage, 10 mA maximum current (USB PD Table 6.9).
const FIXED_VOLTAGE_UNIT_MV: u16 = 50;
const FIXED_CURRENT_UNIT_MA: u16 = 10;
/// SPR PPS APDO voltage unit: 100 mV (USB PD Table 6.13).
const PPS_VOLTAGE_UNIT_MV: u16 = 100;
/// PPS current unit: 50 mA, in the APDO and the PPS Request (USB PD Tables
/// 6.13 and 6.21).
const PPS_CURRENT_UNIT_MA: u16 = 50;

/// What one advertised power data object offers, for display.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Advertised {
    Fixed {
        mv: u16,
        ma: u16,
    },
    Pps {
        min_mv: u16,
        max_mv: u16,
        max_ma: u16,
    },
    /// Battery, variable, EPR or unknown objects.
    Other,
}

/// Decode one raw Source_Capabilities object.
pub fn advertised(raw: u32) -> Advertised {
    use source_capabilities::{Augmented, PowerDataObject, parse_raw_pdo};
    match parse_raw_pdo(raw) {
        PowerDataObject::FixedSupply(pdo) => Advertised::Fixed {
            mv: pdo.raw_voltage() * FIXED_VOLTAGE_UNIT_MV,
            ma: pdo.raw_max_current() * FIXED_CURRENT_UNIT_MA,
        },
        PowerDataObject::Augmented(Augmented::Spr(pdo)) => Advertised::Pps {
            min_mv: pdo.raw_min_voltage() as u16 * PPS_VOLTAGE_UNIT_MV,
            max_mv: pdo.raw_max_voltage() as u16 * PPS_VOLTAGE_UNIT_MV,
            max_ma: pdo.raw_max_current() as u16 * PPS_CURRENT_UNIT_MA,
        },
        _ => Advertised::Other,
    }
}

pub struct BoardLimits;
impl shared::Limits for BoardLimits {
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
    // Long OK on the charger page asks an EPR capable charger for its EPR
    // offer, as the source firmware's capability test did.
    const EPR_OFFER: bool = true;
    fn status() -> Option<[u8; 7]> {
        // Table 6.51: DC input (VBUS), no battery; temperature reporting and
        // power-state change reporting unsupported. No qualified fault events.
        Some([0, 2, 0, 0, 0, 0, 0])
    }
}
pub type Trial = shared::Trial<BoardLimits>;

pub fn request_pps(
    caps: &source_capabilities::SourceCapabilities,
    object: u8,
    mv: u16,
    ma: u16,
) -> Result<request::PowerSource, Error> {
    shared::request_pps::<BoardLimits>(caps, object, mv, ma)
}
pub fn request_target(
    caps: &source_capabilities::SourceCapabilities,
) -> Result<request::PowerSource, Error> {
    shared::request_target::<BoardLimits>(caps)
}
pub fn request_fixed(
    caps: &source_capabilities::SourceCapabilities,
    mv: u32,
) -> Result<request::PowerSource, Error> {
    shared::request_fixed::<BoardLimits>(caps, mv)
}

pub struct Selection {
    inner: shared::Selection<BoardLimits>,
    mode_cursor: Cell<Option<u16>>,
}
impl core::ops::Deref for Selection {
    type Target = shared::Selection<BoardLimits>;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}
impl Selection {
    pub const fn new() -> Self {
        Self {
            inner: shared::Selection::new(),
            mode_cursor: Cell::new(None),
        }
    }
    pub fn mode_cursor(&self) -> Option<u16> {
        self.mode_cursor.get()
    }
    pub fn set_mode_cursor(&self, index: Option<u16>) {
        self.mode_cursor.set(index);
    }
    /// Apply one request from the input task. Previews go through the shared
    /// selection's own checks; only `Confirm` schedules a Request.
    pub fn apply(&self, command: PdCommand) {
        match command {
            PdCommand::StepFixed(step) => self.step_fixed(step),
            PdCommand::PpsVoltage(delta) => self.adjust_pps(delta, 0),
            PdCommand::PpsCurrent(delta) => self.adjust_pps(0, delta),
            PdCommand::Confirm => {
                self.confirm();
            }
            PdCommand::QueryCable => {
                self.request_cable();
            }
            PdCommand::QueryEpr => {
                self.request_epr_offer();
            }
            // The board's PD task restarts the meter; nothing to request.
            PdCommand::Restart => {}
        }
    }

    fn options(&self) -> impl Iterator<Item = shared::Target> + '_ {
        (0..self.option_count()).filter_map(|index| self.option(index))
    }

    /// Preview the fixed profile `step` places from the previewed one (the
    /// first one when a PPS range is previewed).
    fn step_fixed(&self, step: i8) {
        let mut fixed = [None; 7];
        let mut count = 0;
        for (slot, option) in fixed
            .iter_mut()
            .zip(self.options().filter(|option| option.pps_object == 0))
        {
            *slot = Some(option);
            count += 1;
        }
        if count == 0 {
            return;
        }
        let current = fixed[..count].iter().position(|option| {
            !self.candidate_is_pps() && option.is_some_and(|o| o.mv == self.candidate_mv())
        });
        let index = current.map_or(0, |index| {
            (index as isize + isize::from(step)).rem_euclid(count as isize) as usize
        });
        if let Some(option) = fixed[index] {
            self.preview(option);
        }
    }

    /// Preview a PPS voltage/current change, in any advertised range that
    /// accepts it. A voltage rise lowers the current when the new voltage
    /// allows less; a change no range accepts leaves the preview unchanged.
    fn adjust_pps(&self, delta_mv: i16, delta_ma: i16) {
        if !self.candidate_is_pps() {
            if let Some(first) = self.options().find(|option| option.pps_object != 0) {
                self.preview(first);
            }
            return;
        }
        let mv = (i32::from(self.candidate_mv()) + i32::from(delta_mv)).clamp(0, 0xffff) as u16;
        let mut ma = (i32::from(self.candidate_ma()) + i32::from(delta_ma))
            .clamp(i32::from(PPS_MIN_MA), 0xffff) as u16;
        loop {
            for option in self.options().filter(|option| option.pps_object != 0) {
                let target = shared::Target {
                    mv,
                    ma,
                    pps_object: option.pps_object,
                };
                if self.preview(target) {
                    return;
                }
            }
            if delta_ma != 0 || ma <= PPS_MIN_MA {
                return;
            }
            ma = ma.saturating_sub(PPS_CURRENT_UNIT_MA).max(PPS_MIN_MA);
        }
    }
}
impl Default for Selection {
    fn default() -> Self {
        Self::new()
    }
}
