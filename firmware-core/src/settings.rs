//! Display settings changed on the settings page, the mode the next boot
//! starts in, and their flash record encoding. The firmware stores the
//! record; this module only packs it.

use crate::calibration::Calibration;
use qc_protocol::detect::Report;
use qc_protocol::qc2::Failure;

/// Backlight duty levels, percent of the PWM period. The timer needs a
/// compare inside the period, so the top level stays below 100%. 60% is the
/// retained, hardware-tested level.
pub const BRIGHTNESS_PERCENT: [u8; 5] = [20, 40, 60, 80, 95];
/// Index of the 60% level in [`BRIGHTNESS_PERCENT`].
const DEFAULT_BRIGHTNESS: u8 = 2;

/// Energy checkpoint interval. Off never writes totals and starts each boot
/// from zero; the other choices trade flash endurance for less power-loss lag.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EnergyMemory {
    #[default]
    Off,
    Seconds5,
    Seconds10,
    Seconds30,
    Seconds60,
}

impl EnergyMemory {
    pub const ALL: [Self; 5] = [
        Self::Off,
        Self::Seconds5,
        Self::Seconds10,
        Self::Seconds30,
        Self::Seconds60,
    ];

    pub fn seconds(self) -> Option<u64> {
        match self {
            Self::Off => None,
            Self::Seconds5 => Some(5),
            Self::Seconds10 => Some(10),
            Self::Seconds30 => Some(30),
            Self::Seconds60 => Some(60),
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Off => "OFF",
            Self::Seconds5 => "5s",
            Self::Seconds10 => "10s",
            Self::Seconds30 => "30s",
            Self::Seconds60 => "60s",
        }
    }
}

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

/// What session the next boot runs. Set, with the screen it opens on
/// ([`Settings::set_next_boot`]), just before the meter restarts the
/// charger (and with it itself) into that mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum NextBoot {
    #[default]
    Normal,
    /// The 5 A cable test; that boot clears it.
    CableTest,
    /// PD silent: the sink holds Rd but never acknowledges the charger, so a
    /// USB-C charger falls back to its non-PD mode, where it takes Quick
    /// Charge requests. Kept until OK on a PD tab leaves it.
    QuickCharge,
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
    /// still decode. The last held Quick Charge with its 3.0 tab, which
    /// [`Screen`] now carries; it still reads as Quick Charge.
    const ALL: [Self; 7] = [
        Self::Normal,
        Self::CableTest,
        Self::QuickCharge,
        Self::ScanQuickCharge {
            then_cable_test: false,
        },
        Self::ScanQuickCharge {
            then_cable_test: true,
        },
        Self::ScanPd,
        Self::QuickCharge,
    ];

    /// PD stays silent in this boot's sink session.
    pub fn silent(self) -> bool {
        matches!(self, Self::QuickCharge | Self::ScanQuickCharge { .. })
    }

    /// The mode lasts one boot, which clears it before any task starts.
    fn one_boot(self) -> bool {
        matches!(self, Self::CableTest | Self::ScanPd)
    }
}

/// A screen the next boot opens on, as the settings keep it: each of the
/// navigation's tabs, the detail lists at their top.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Screen {
    Overview,
    Lines,
    Fixed,
    Pps,
    Qc2,
    Qc3,
    PdDetails,
    Charger,
    Cable,
    System,
    Avs,
    Energy,
    Ripple,
    Scp,
    Fcp,
}

impl Screen {
    /// Record values, by position; new values go last, so older records
    /// still decode.
    const ALL: [Self; 15] = [
        Self::Overview,
        Self::Lines,
        Self::Fixed,
        Self::Pps,
        Self::Qc2,
        Self::Qc3,
        Self::PdDetails,
        Self::Charger,
        Self::Cable,
        Self::System,
        Self::Avs,
        Self::Energy,
        Self::Ripple,
        Self::Scp,
        Self::Fcp,
    ];

    /// 0 for none, else the position in [`Self::ALL`] plus one.
    fn encode(screen: Option<Self>) -> u32 {
        screen.map_or(0, |screen| {
            Self::ALL
                .iter()
                .position(|known| *known == screen)
                .map_or(0, |index| index as u32 + 1)
        })
    }

    /// Unpack [`Self::encode`]: the screen, `Some(None)` for none, or `None`
    /// for an unused value.
    fn decode(bits: u32) -> Option<Option<Self>> {
        match bits {
            0 => Some(None),
            bits => Self::ALL.get(bits as usize - 1).copied().map(Some),
        }
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
/// Rotation's position in [`Rotation::ALL`] (two bits), and above it the
/// resume screen (six bits). Earlier records kept rotation, at most 2, in
/// the whole byte, so their resume bits read as none.
const ROTATION_SHIFT: u32 = 8;
const ROTATION_MASK: u32 = 0b11;
const RESUME_SHIFT: u32 = 10;
const RESUME_MASK: u32 = 0b11_1111;
const _: () = assert!(Screen::ALL.len() <= RESUME_MASK as usize);

/// Brightness uses only the low three bits; energy memory occupies formerly
/// zero bits 5..3, so old records retain Off. SCP findings use bits 7..6;
/// old records retain an untested result.
const BRIGHTNESS_MASK: u32 = 0b111;
const ENERGY_MEMORY_SHIFT: u32 = 3;
const ENERGY_MEMORY_MASK: u32 = 0b111;
/// Previously reserved settings bits: compact SCP evidence across scan boots.
const SCP_SHIFT: u32 = 6;
const SCP_MASK: u32 = 0b11;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScpFindings {
    Unverified,
    Confirmed,
    /// The adapter showed it has no SCP.
    Absent,
}

impl ScpFindings {
    fn encode(value: Option<Self>) -> u32 {
        match value {
            None => 0,
            Some(Self::Unverified) => 1,
            Some(Self::Confirmed) => 2,
            Some(Self::Absent) => 3,
        }
    }

    fn decode(bits: u32) -> Option<Option<Self>> {
        match bits {
            0 => Some(None),
            1 => Some(Some(Self::Unverified)),
            2 => Some(Some(Self::Confirmed)),
            3 => Some(Some(Self::Absent)),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Settings {
    /// Index into [`BRIGHTNESS_PERCENT`].
    pub brightness: u8,
    pub rotation: Rotation,
    pub energy_memory: EnergyMemory,
    pub next_boot: NextBoot,
    /// The full scan's Quick Charge result, for its PD part to show; that
    /// boot clears it.
    pub qc_findings: Option<QcFindings>,
    /// Read-only SCP adapter identification, carried into the PD/cable boot.
    pub scp_findings: Option<ScpFindings>,
    /// The screen the next boot opens on; it lasts as long as `next_boot`
    /// (see [`Self::finish_boot`]).
    pub resume: Option<Screen>,
    /// Kept in records of its own beside the settings word
    /// ([`Calibration::records`]); [`Self::encode`] leaves it out.
    pub calibration: Calibration,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            brightness: DEFAULT_BRIGHTNESS,
            rotation: Rotation::Auto,
            energy_memory: EnergyMemory::Off,
            next_boot: NextBoot::Normal,
            qc_findings: None,
            scp_findings: None,
            resume: None,
            calibration: Calibration::default(),
        }
    }
}

/// Record tag in the top byte of an encoded settings word, so a blank or
/// foreign word never decodes.
const RECORD_TAG: u32 = 0x5f;
/// The QC-only layout used tag 0x5e with bits 7..6 reserved and screen IDs
/// up to 13. Keep reading those records, but do not reinterpret another
/// experimental layout's evidence or screen ID as an SCP result.
const LEGACY_RECORD_TAG: u32 = 0x5e;
const LEGACY_SCREEN_MAX: u32 = 13;

impl Settings {
    /// Restart into `boot`, opening on `resume`.
    pub fn set_next_boot(&mut self, boot: NextBoot, resume: Option<Screen>) {
        self.next_boot = boot;
        self.resume = resume;
    }

    /// Clear what only this boot needed: a one-boot mode, and the resume
    /// screen unless the mode outlasts the charger's restarts.
    pub fn finish_boot(&mut self) {
        if self.next_boot.one_boot() {
            self.next_boot = NextBoot::Normal;
        }
        if self.next_boot == NextBoot::Normal {
            self.resume = None;
        }
    }

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

    pub fn step_energy_memory(&mut self, forward: bool) {
        let all = EnergyMemory::ALL;
        let index = all
            .iter()
            .position(|value| *value == self.energy_memory)
            .unwrap_or(0);
        let next = if forward {
            index + 1
        } else {
            index + all.len() - 1
        };
        self.energy_memory = all[next % all.len()];
    }

    pub fn brightness_percent(&self) -> u8 {
        BRIGHTNESS_PERCENT[usize::from(self.brightness) % BRIGHTNESS_PERCENT.len()]
    }

    /// Pack into one word: tag in bits 31..24, Quick Charge findings in
    /// 23..19, the next boot in 18..16, the resume screen in 15..10, rotation
    /// in 9..8, SCP findings in 7..6, energy memory in 5..3, brightness in 2..0.
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
            | Screen::encode(self.resume) << RESUME_SHIFT
            | rotation << ROTATION_SHIFT
            | ScpFindings::encode(self.scp_findings) << SCP_SHIFT
            | (self.energy_memory as u32) << ENERGY_MEMORY_SHIFT
            | u32::from(self.brightness)
    }

    /// Unpack a word from [`Self::encode`], with the default calibration;
    /// anything else is `None`.
    pub fn decode(word: u32) -> Option<Self> {
        match word >> 24 {
            RECORD_TAG => {}
            LEGACY_RECORD_TAG
                if word >> SCP_SHIFT & SCP_MASK == 0
                    && word >> RESUME_SHIFT & RESUME_MASK <= LEGACY_SCREEN_MAX => {}
            _ => return None,
        }
        let brightness = (word & BRIGHTNESS_MASK) as u8;
        let energy_memory =
            *EnergyMemory::ALL.get((word >> ENERGY_MEMORY_SHIFT & ENERGY_MEMORY_MASK) as usize)?;
        let rotation = *Rotation::ALL.get((word >> ROTATION_SHIFT & ROTATION_MASK) as usize)?;
        let next_boot = *NextBoot::ALL.get((word >> NEXT_BOOT_SHIFT & NEXT_BOOT_MASK) as usize)?;
        let qc_findings = QcFindings::decode(word >> FINDINGS_SHIFT & FINDINGS_MASK)?;
        let scp_findings = ScpFindings::decode(word >> SCP_SHIFT & SCP_MASK)?;
        let resume = Screen::decode(word >> RESUME_SHIFT & RESUME_MASK)?;
        (usize::from(brightness) < BRIGHTNESS_PERCENT.len()).then_some(Self {
            brightness,
            rotation,
            energy_memory,
            next_boot,
            qc_findings,
            scp_findings,
            resume,
            calibration: Calibration::default(),
        })
    }
}

/// Settings page rows, in cursor order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Item {
    #[default]
    Brightness,
    Rotation,
    EnergyMemory,
    Calibration,
}

impl Item {
    pub const ALL: [Self; 4] = [
        Self::Brightness,
        Self::Rotation,
        Self::EnergyMemory,
        Self::Calibration,
    ];
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn energy_memory_preserves_legacy_records_and_other_fields() {
        assert_eq!(Settings::decode(0x5e00_0002), Some(Settings::default()));
        for memory in EnergyMemory::ALL {
            for brightness in 0..BRIGHTNESS_PERCENT.len() as u8 {
                let settings = Settings {
                    brightness,
                    rotation: Rotation::Flipped,
                    energy_memory: memory,
                    next_boot: NextBoot::ScanQuickCharge {
                        then_cable_test: true,
                    },
                    resume: Some(Screen::Energy),
                    qc_findings: Some(QcFindings::Failed(Failure::NoEntry)),
                    scp_findings: None,
                    calibration: Calibration::default(),
                };
                assert_eq!(Settings::decode(settings.encode()), Some(settings));
            }
        }
        assert!(Settings::decode(0x5e00_003a).is_none());
        assert!(Settings::decode(0x5e00_0042).is_none());
        assert!(Settings::decode(0x5e00_00c2).is_none());
        assert_eq!(
            Settings::decode(0x5f00_0042).map(|s| s.scp_findings),
            Some(Some(ScpFindings::Unverified))
        );
    }

    #[test]
    fn interval_choices_cycle_in_both_directions() {
        let mut settings = Settings::default();
        for seconds in [Some(5), Some(10), Some(30), Some(60), None] {
            settings.step_energy_memory(true);
            assert_eq!(settings.energy_memory.seconds(), seconds);
        }
        for seconds in [Some(60), Some(30), Some(10), Some(5), None] {
            settings.step_energy_memory(false);
            assert_eq!(settings.energy_memory.seconds(), seconds);
        }
    }
}
