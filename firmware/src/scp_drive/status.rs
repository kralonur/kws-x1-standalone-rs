//! SCP detection and output status, also used by the host renderer.

use core::ops::RangeInclusive;

/// VBUS window in mV that counts as the default 5 V output: vSafe5V's
/// 5.5 V maximum (USB PD), with the minimum widened to 4.5 V for cable
/// drop. Application policy: read-only detection must never raise VBUS, and
/// SCP evidence is only shown at this output.
pub const DEFAULT_VBUS_MV: RangeInclusive<u16> = 4_500..=5_500;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Failure {
    Entry,
    DriveLevel,
    Vbus,
    Clock,
    Wire(scp_protocol::Error<core::convert::Infallible>),
    Control(scp_protocol::control::ControlError<core::convert::Infallible>),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Entering,
    Reading,
    Detected(scp_protocol::Adapter),
    Output {
        output: scp_protocol::control::Output,
        applied: bool,
    },
    Stopping,
    Stopped,
    Failed(Failure),
}
