//! Display settings changed on the settings page, the mode the next boot
//! starts in, and their flash record encoding. The firmware stores the
//! record; this module only packs it.

use qc_protocol::detect::Report;
use qc_protocol::qc2::Failure;

/// Backlight duty levels, percent of the PWM period. The timer needs a
/// compare inside the period, so the top level stays below 100%. 60% is the
/// retained, hardware-tested level.
pub const BRIGHTNESS_PERCENT: [u8; 5] = [20, 40, 60, 80, 95];
/// Index of the 60% level in [`BRIGHTNESS_PERCENT`].
const DEFAULT_BRIGHTNESS: u8 = 2;

/// Screen orientation: follow the tilt sensor, or stay fixed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Rotation {
    #[default]
    Auto,
    Normal,
    Flipped,
}

impl Rotation {
    const ALL: [Self; 3] = [Self::Auto, Self::Normal, Self::Flipped];
}

/// What the next boot starts as. Set just before the meter restarts the
/// charger (and with it itself) into that mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum NextBoot {
    #[default]
    Normal,
    /// The 5 A cable test; that boot clears it.
    CableTest,
    /// PD silent: the sink holds Rd but never acknowledges the charger, so a
    /// USB-C charger falls back to its non-PD mode, where it takes Quick
    /// Charge requests. Kept until OK on a PD tab leaves it; `steps`
    /// reopens the Quick Charge 3.0 tab rather than 2.0.
    QuickCharge { steps: bool },
    /// The full charger scan's first part: PD silent, the Quick Charge probe
    /// runs. Kept until it ends (a charger may restart the meter meanwhile),
    /// then `then_cable_test` picks the PD part.
    ScanQuickCharge { then_cable_test: bool },
    /// The full scan's PD part: a normal session that also asks for the EPR
    /// offer, opening on the charger page; that boot clears it.
    ScanPd,
}

impl NextBoot {
    /// Record values, by position; new values go last, so older records
    /// still decode.
    const ALL: [Self; 7] = [
        Self::Normal,
        Self::CableTest,
        Self::QuickCharge { steps: false },
        Self::ScanQuickCharge {
            then_cable_test: false,
        },
        Self::ScanQuickCharge {
            then_cable_test: true,
        },
        Self::ScanPd,
        Self::QuickCharge { steps: true },
    ];

    /// PD stays silent in this boot's sink session.
    pub fn silent(self) -> bool {
        matches!(
            self,
            Self::QuickCharge { .. } | Self::ScanQuickCharge { .. }
        )
    }

    /// The mode lasts one boot, which clears it before any task starts.
    pub fn one_boot(self) -> bool {
        matches!(self, Self::CableTest | Self::ScanPd)
    }
}

/// What a Quick Charge run found, carried from the scan's Quick Charge part
/// to its PD part.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QcFindings {
    Failed(Failure),
    Detected(Report),
}

impl QcFindings {
    /// Five bits: 0 none, 1 not a charging port, 2 no Quick Charge, or bit 4
    /// set with the report's 9 V, 12 V, 20 V and steps in bits 0-3.
    fn encode(findings: Option<Self>) -> u32 {
        match findings {
            None => 0,
            Some(Self::Failed(Failure::NotDcp)) => 1,
            Some(Self::Failed(Failure::NoEntry)) => 2,
            Some(Self::Detected(report)) => {
                FINDINGS_DETECTED
                    | u32::from(report.v9)
                    | u32::from(report.v12) << 1
                    | u32::from(report.v20) << 2
                    | u32::from(report.steps) << 3
            }
        }
    }

    /// Unpack [`Self::encode`]: the findings, `Some(None)` for none, or
    /// `None` for an unused value.
    fn decode(bits: u32) -> Option<Option<Self>> {
        Some(match bits {
            0 => None,
            1 => Some(Self::Failed(Failure::NotDcp)),
            2 => Some(Self::Failed(Failure::NoEntry)),
            bits if bits & FINDINGS_DETECTED != 0 && bits <= FINDINGS_MASK => {
                Some(Self::Detected(Report {
                    v9: bits & 1 != 0,
                    v12: bits & 1 << 1 != 0,
                    v20: bits & 1 << 2 != 0,
                    steps: bits & 1 << 3 != 0,
                }))
            }
            _ => return None,
        })
    }
}

/// Findings value bit marking a probe report.
const FINDINGS_DETECTED: u32 = 1 << 4;
/// Record fields beside the tag: the next boot's position in
/// [`NextBoot::ALL`] (three bits) and the findings (five bits).
const NEXT_BOOT_SHIFT: u32 = 16;
const NEXT_BOOT_MASK: u32 = 0b111;
const FINDINGS_SHIFT: u32 = 19;
const FINDINGS_MASK: u32 = 0b1_1111;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Settings {
    /// Index into [`BRIGHTNESS_PERCENT`].
    pub brightness: u8,
    pub rotation: Rotation,
    pub next_boot: NextBoot,
    /// The full scan's Quick Charge result, for its PD part to show; that
    /// boot clears it.
    pub qc_findings: Option<QcFindings>,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            brightness: DEFAULT_BRIGHTNESS,
            rotation: Rotation::Auto,
            next_boot: NextBoot::Normal,
            qc_findings: None,
        }
    }
}

/// Record tag in the top byte of an encoded settings word, so a blank or
/// foreign word never decodes.
const RECORD_TAG: u32 = 0x5e;

impl Settings {
    /// One level brighter (`brighter`) or dimmer, stopping at either end.
    pub fn step_brightness(&mut self, brighter: bool) {
        let top = BRIGHTNESS_PERCENT.len() as u8 - 1;
        self.brightness = if brighter {
            (self.brightness + 1).min(top)
        } else {
            self.brightness.saturating_sub(1)
        };
    }

    /// The next (`forward`) or previous rotation choice, wrapping.
    pub fn step_rotation(&mut self, forward: bool) {
        let all = Rotation::ALL;
        let index = all.iter().position(|r| *r == self.rotation).unwrap_or(0);
        let next = if forward {
            index + 1
        } else {
            index + all.len() - 1
        };
        self.rotation = all[next % all.len()];
    }

    pub fn brightness_percent(&self) -> u8 {
        BRIGHTNESS_PERCENT[usize::from(self.brightness) % BRIGHTNESS_PERCENT.len()]
    }

    /// Pack into one word: tag in bits 31..24, Quick Charge findings in
    /// 23..19, the next boot in 18..16, rotation in 15..8, brightness in 7..0.
    /// Earlier records kept the cable test in bit 16 and Quick Charge mode in
    /// 17, which read as the same next boot here.
    pub fn encode(&self) -> u32 {
        let rotation = Rotation::ALL
            .iter()
            .position(|r| *r == self.rotation)
            .unwrap_or(0) as u32;
        let next_boot = NextBoot::ALL
            .iter()
            .position(|boot| *boot == self.next_boot)
            .unwrap_or(0) as u32;
        RECORD_TAG << 24
            | QcFindings::encode(self.qc_findings) << FINDINGS_SHIFT
            | next_boot << NEXT_BOOT_SHIFT
            | rotation << 8
            | u32::from(self.brightness)
    }

    /// Unpack a word from [`Self::encode`]; anything else is `None`.
    pub fn decode(word: u32) -> Option<Self> {
        if word >> 24 != RECORD_TAG {
            return None;
        }
        let brightness = word as u8;
        let rotation = *Rotation::ALL.get(usize::from((word >> 8) as u8))?;
        let next_boot = *NextBoot::ALL.get((word >> NEXT_BOOT_SHIFT & NEXT_BOOT_MASK) as usize)?;
        let qc_findings = QcFindings::decode(word >> FINDINGS_SHIFT & FINDINGS_MASK)?;
        (usize::from(brightness) < BRIGHTNESS_PERCENT.len()).then_some(Self {
            brightness,
            rotation,
            next_boot,
            qc_findings,
        })
    }
}

/// Settings page rows, in cursor order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Item {
    #[default]
    Brightness,
    Rotation,
    ResetPeaks,
}

impl Item {
    pub const ALL: [Self; 3] = [Self::Brightness, Self::Rotation, Self::ResetPeaks];
}
