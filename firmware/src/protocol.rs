//! The charging protocol the overview names, from every source the meter
//! has. The first that applies wins: this meter's own Quick Charge, SCP or
//! FCP output (a PD contract would have stopped it), PD, Quick Charge or
//! FCP/SCP data another device drives, register-verified SCP at the default
//! output, a Type-C current above default, a D+/D− signature, the BC1.2
//! short the Quick Charge test found, then the Type-C default.

use crate::line_adc::LineVoltages;
use crate::pd::{self, Observation};
use crate::qc_drive::Status;
use crate::scp_drive::DEFAULT_VBUS_MV;
use crate::ui::Protocol;
use firmware_core::dcp::{self, Driven};
use firmware_core::settings::{QcFindings, ScpFindings};
use qc_protocol::detect::Outcome;
use qc_protocol::qc2::Failure;

/// What the meter knows about the port.
pub struct Sources<'a> {
    pub observation: &'a Observation,
    /// The line voltages, while they are sampled.
    pub lines: Option<LineVoltages>,
    /// Debounced protocol another device drives on D+/D−.
    pub driven: Option<Driven>,
    /// This meter's Quick Charge trigger.
    pub qc: Status,
    /// The Quick Charge test's result.
    pub findings: Option<QcFindings>,
    /// Verified read-only SCP evidence, not a negotiated output contract.
    pub scp: Option<ScpFindings>,
    /// The bus voltage in mV.
    pub vbus_mv: Option<u32>,
}

/// The protocol the overview names, in the order the module docs give.
pub fn identify(sources: &Sources<'_>) -> Protocol {
    let observation = sources.observation;
    let type_c = pd::type_c_current(observation, sources.lines);
    // With this meter's trigger off, the lines show the port and any device.
    let lines = sources.lines.filter(|_| sources.qc == Status::Off);
    own_quick_charge(sources.qc)
        .or_else(|| pd::protocol(observation))
        .or_else(|| {
            sources
                .driven
                .filter(|_| sources.qc == Status::Off)
                .map(|driven| match driven {
                    Driven::Qc2 { mv } => Protocol::Qc2 { mv },
                    Driven::Qc3 => Protocol::Qc3 { mv: None },
                    Driven::SingleWire => Protocol::SingleWire,
                    Driven::Fcp { mv } => Protocol::Fcp { mv },
                    Driven::Scp { mv } => Protocol::DrivenScp { mv },
                })
        })
        .or_else(|| {
            // Never label a raised, unnegotiated output as an SCP contract.
            let at_default = sources
                .vbus_mv
                .and_then(|mv| u16::try_from(mv).ok())
                .is_some_and(|mv| DEFAULT_VBUS_MV.contains(&mv));
            (sources.scp == Some(ScpFindings::Confirmed) && at_default)
                .then_some(Protocol::ScpDetected)
        })
        .or(type_c.filter(|protocol| *protocol != Protocol::TypeC { ma: 0 }))
        .or_else(|| {
            lines
                .and_then(|lines| dcp::read(lines.dp_mv, lines.dm_mv))
                .map(Protocol::Dcp)
        })
        .or_else(|| {
            matches!(
                sources.findings,
                Some(QcFindings::Detected(_) | QcFindings::Failed(Failure::NoEntry))
            )
            .then_some(Protocol::Bc12)
        })
        .or(type_c)
        .unwrap_or_default()
}

/// The request this meter's trigger has in force; one the charger ignored
/// is not.
fn own_quick_charge(qc: Status) -> Option<Protocol> {
    match qc {
        Status::Scp(
            _,
            crate::scp_drive::Status::Output {
                output,
                applied: true,
            },
        ) => Some(Protocol::Scp {
            mv: output.millivolts(),
            ma: output.milliamps(),
        }),
        Status::Fcp(
            _,
            crate::scp_drive::FcpStatus::Output {
                output,
                applied: true,
            },
        ) => Some(Protocol::Fcp {
            mv: Some(output.millivolts()),
        }),
        Status::Active(_, Some(Outcome::Ignored)) => None,
        Status::Active(voltage, _) => Some(Protocol::Qc2 {
            mv: voltage.millivolts(),
        }),
        Status::Stepping(mv) => Some(Protocol::Qc3 { mv: Some(mv) }),
        _ => None,
    }
}
