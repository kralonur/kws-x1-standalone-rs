//! The charging protocol the overview names, from every source the meter
//! has. The first that applies wins: this meter's own Quick Charge (a PD
//! contract would have stopped it), PD, Quick Charge another device drives,
//! a Type-C current above default, a D+/D− signature, the BC1.2 short the
//! Quick Charge test found, then the Type-C default.

use crate::line_adc::LineVoltages;
use crate::pd::{self, Observation};
use crate::qc_drive::Status;
use crate::ui::Protocol;
use firmware_core::dcp;
use firmware_core::settings::QcFindings;
use qc_protocol::detect::{Follow, Outcome};
use qc_protocol::lines::{self, Request};
use qc_protocol::qc2::{Failure, Voltage};

/// What the meter knows about the port.
pub struct Sources<'a> {
    pub observation: &'a Observation,
    /// The line voltages, while they are sampled.
    pub lines: Option<LineVoltages>,
    /// This meter's Quick Charge trigger.
    pub qc: Status,
    /// The Quick Charge test's result.
    pub findings: Option<QcFindings>,
    /// The bus voltage in mV.
    pub vbus_mv: Option<u32>,
}

/// The protocol the overview names, in the order the module docs give.
pub fn identify(sources: &Sources<'_>) -> Protocol {
    let observation = sources.observation;
    let type_c = pd::type_c_current(observation);
    // With this meter's trigger off, the lines show the port and any device.
    let lines = sources.lines.filter(|_| sources.qc == Status::Off);
    own_quick_charge(sources.qc)
        .or_else(|| pd::protocol(observation))
        .or_else(|| lines.and_then(|lines| driven_quick_charge(lines, sources.vbus_mv)))
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
        Status::Active(_, Some(Outcome::Ignored)) => None,
        Status::Active(voltage, _) => Some(Protocol::Qc2 {
            mv: voltage.millivolts(),
        }),
        Status::Stepping(mv) => Some(Protocol::Qc3 { mv: Some(mv) }),
        _ => None,
    }
}

/// Quick Charge another device drives on the lines. A raised request
/// counts once the output moved past halfway to it, the same check this
/// meter's trigger makes; 5 V looks like a device's BC1.2 detection, so it
/// never counts.
fn driven_quick_charge(lines: LineVoltages, vbus_mv: Option<u32>) -> Option<Protocol> {
    match lines::request(lines.dp_mv, lines.dm_mv)? {
        Request::Continuous => Some(Protocol::Qc3 { mv: None }),
        Request::Fixed(Voltage::V5) => None,
        Request::Fixed(voltage) => {
            let vbus_mv = u16::try_from(vbus_mv?).unwrap_or(u16::MAX);
            let from_5v = Follow::start(0, Voltage::V5.millivolts(), voltage.millivolts());
            (from_5v.poll(0, vbus_mv) == Outcome::Followed).then_some(Protocol::Qc2 {
                mv: voltage.millivolts(),
            })
        }
    }
}
