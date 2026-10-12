//! Dedicated charging ports that tell their current by fixed D+/D− voltages
//! instead of the BC1.2 short. Levels and adapter ratings come from the TI
//! TPS2513/TPS2514 datasheet (SLVSBY8D, June 2020): section 8.3.3 "Divider
//! DCP", section 8.3.4 and the Electrical Characteristics (section 7.5) at
//! VIN = 5 V.

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
