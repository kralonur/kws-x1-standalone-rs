//! Dedicated charging ports that tell their current by fixed D+/D− voltages
//! instead of the BC1.2 short. Levels and adapter ratings come from the TI
//! TPS2513/TPS2514 datasheet (SLVSBY8D, June 2020): section 8.3.3 "Divider
//! DCP", section 8.3.4 and the Electrical Characteristics (section 7.5) at
//! VIN = 5 V.

use qc_protocol::{
    detect::{Follow, Outcome},
    lines::{self, Request},
    qc2::Voltage,
};

/// A port's D+/D− signature.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dcp {
    /// D+ 2 V, D− 2.7 V: a 5 W adapter.
    Divider1,
    /// D+ 2.7 V, D− 2 V: a 10 W adapter.
    Divider2,
    /// D+ 2.7 V, D− 2.7 V: a 12 W adapter.
    Divider3,
    /// 1.2 V on both lines, which "some tablet USB chargers" need; the
    /// datasheet gives no rating.
    Bias1V2,
}

/// The output the ratings and levels hold at, in mV (VIN = 5 V).
const OUTPUT_MV: u32 = 5_000;

impl Dcp {
    /// The adapter's rated current in mA at [`OUTPUT_MV`], from its rated
    /// power; `None` for the 1.2 V mode.
    pub const fn milliamps(self) -> Option<u32> {
        let milliwatts = match self {
            Self::Divider1 => 5_000,
            Self::Divider2 => 10_000,
            Self::Divider3 => 12_000,
            Self::Bias1V2 => return None,
        };
        Some(milliwatts * 1_000 / OUTPUT_MV)
    }
}

/// A line level's datasheet range in mV (min, max).
type Range = (u16, u16);
/// VDP_2.7V / VDM_2.7V.
const V2_7: Range = (2_570, 2_840);
/// VDM_2V.
const V2_0: Range = (1_900, 2_100);
/// VDP_1.2V / VDM_1.2V.
const V1_2: Range = (1_120, 1_280);
/// How far a line reading may be off on another unit, in percent: the ADC's
/// internal reference has no published tolerance, and the one measured unit
/// suggests a few percent.
const READING_ERROR_PERCENT: u32 = 3;

/// `mv` is within `range`, widened by [`READING_ERROR_PERCENT`].
fn within(mv: u16, (min, max): Range) -> bool {
    let low = u32::from(min) * (100 - READING_ERROR_PERCENT) / 100;
    let high = u32::from(max) * (100 + READING_ERROR_PERCENT) / 100;
    (low..=high).contains(&u32::from(mv))
}

/// The signature D+ at `dp_mv` and D− at `dm_mv` show, if any. Read with
/// nothing driving the lines but the port.
pub fn read(dp_mv: u16, dm_mv: u16) -> Option<Dcp> {
    [
        (Dcp::Divider1, V2_0, V2_7),
        (Dcp::Divider2, V2_7, V2_0),
        (Dcp::Divider3, V2_7, V2_7),
        (Dcp::Bias1V2, V1_2, V1_2),
    ]
    .into_iter()
    .find(|(_, dp, dm)| within(dp_mv, *dp) && within(dm_mv, *dm))
    .map(|(dcp, ..)| dcp)
}

/// CHY100 `VDAT(REF)` bounds in mV: D+ above `DP_ACTIVE_MIN_MV` keeps an
/// entered handshake open; D− below `DM_LOW_MAX_MV` is pulled low by the
/// charger's `RDM_DOWN`, and above `DM_PULSE_MIN_MV` is driven above low.
const DP_ACTIVE_MIN_MV: u16 = 400;
const DM_LOW_MAX_MV: u16 = 250;
const DM_PULSE_MIN_MV: u16 = 400;
/// Hold time in ms before accepting Quick Charge levels or clearing pulse
/// evidence for a stable divider signature. Application debounce choice:
/// four nominal 100 ms sampling intervals, not a protocol timing limit.
const QC_SETTLE_MS: u64 = 400;
/// Maximum gap in ms from an observed D− low to high, or high to low.
/// Application heuristic: four nominal 100 ms sampling intervals. This
/// bounds stale evidence; it does not validate the 160 µs FCP/SCP symbols.
const SINGLE_WIRE_WINDOW_MS: u64 = 400;

/// Protocol another device drives on D+/D− during passthrough.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Driven {
    Qc2 {
        mv: u16,
    },
    Qc3,
    /// Suspected FCP or SCP single-wire traffic from sampled D− transitions.
    /// Slow voltage sampling neither validates frames nor distinguishes them.
    SingleWire,
}

/// Passive D+/D− heuristic separating steady Quick Charge levels from
/// possible single-wire bursts. Slow sampling can miss bursts entirely.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Passive {
    dm_low_since_ms: Option<u64>,
    dm_pulse_since_ms: Option<u64>,
    dcp_candidate: Option<(Dcp, u64)>,
    single_wire_latched: bool,
    qc_candidate: Option<(Driven, u64)>,
}

impl Passive {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(
        &mut self,
        now_ms: u64,
        dp_mv: u16,
        dm_mv: u16,
        vbus_mv: Option<u32>,
    ) -> Option<Driven> {
        if dp_mv <= DP_ACTIVE_MIN_MV {
            self.reset();
            return None;
        }
        let divider = read(dp_mv, dm_mv);
        match (self.dcp_candidate, divider) {
            (Some((previous, since)), Some(current)) if previous == current => {
                if now_ms.saturating_sub(since) >= QC_SETTLE_MS {
                    self.reset();
                    return None;
                }
            }
            (_, Some(current)) => self.dcp_candidate = Some((current, now_ms)),
            (_, None) => self.dcp_candidate = None,
        }
        for since in [&mut self.dm_low_since_ms, &mut self.dm_pulse_since_ms] {
            if since.is_some_and(|since| now_ms.saturating_sub(since) >= SINGLE_WIRE_WINDOW_MS) {
                *since = None;
            }
        }
        if dm_mv < DM_LOW_MAX_MV {
            if self.dm_pulse_since_ms.take().is_some() {
                self.single_wire_latched = true;
            }
            self.dm_low_since_ms = Some(now_ms);
            self.qc_candidate = None;
        } else if dm_mv > DM_PULSE_MIN_MV && self.dm_low_since_ms.take().is_some() {
            self.dm_pulse_since_ms = Some(now_ms);
        }
        // A divider-like sample may be a pulse, but is not QC evidence.
        let instant = if divider.is_none() {
            instant_quick_charge(dp_mv, dm_mv, vbus_mv)
        } else {
            None
        };
        match (self.qc_candidate, instant) {
            (Some((prev, since)), Some(curr)) if prev == curr => {
                if now_ms.saturating_sub(since) >= QC_SETTLE_MS {
                    self.single_wire_latched = false;
                    return Some(curr);
                }
            }
            (Some((Driven::Qc3, since)), None)
                if divider.is_none()
                    && dm_mv > DM_PULSE_MIN_MV
                    && now_ms.saturating_sub(since) >= QC_SETTLE_MS =>
            {
                return Some(Driven::Qc3);
            }
            (_, Some(curr)) => self.qc_candidate = Some((curr, now_ms)),
            (_, None) => self.qc_candidate = None,
        }
        self.single_wire_latched.then_some(Driven::SingleWire)
    }
}

fn instant_quick_charge(dp_mv: u16, dm_mv: u16, vbus_mv: Option<u32>) -> Option<Driven> {
    match lines::request(dp_mv, dm_mv)? {
        Request::Continuous => Some(Driven::Qc3),
        Request::Fixed(Voltage::V5) => None,
        Request::Fixed(voltage) => {
            let vbus_mv = u16::try_from(vbus_mv?).unwrap_or(u16::MAX);
            let from_5v = Follow::start(0, Voltage::V5.millivolts(), voltage.millivolts());
            (from_5v.poll(0, vbus_mv) == Outcome::Followed).then_some(Driven::Qc2 {
                mv: voltage.millivolts(),
            })
        }
    }
}
