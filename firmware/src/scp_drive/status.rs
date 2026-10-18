//! SCP and FCP detection and output status, also used by the host renderer.

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
    /// No answer to the first read, after its retries.
    Silent,
    Wire(scp_protocol::Error<core::convert::Infallible>),
    Control(scp_protocol::control::ControlError<core::convert::Infallible>),
}

impl Failure {
    /// The adapter showed it lacks the protocol: no BC1.2 short or no
    /// reaction to the entry handshake, no answer at all, a type that is not
    /// programmable or no FCP voltages. Other failures leave support
    /// undetermined.
    pub fn absent(self) -> bool {
        matches!(
            self,
            Self::Entry
                | Self::Silent
                | Self::Wire(scp_protocol::Error::NotScp | scp_protocol::Error::NotFcp)
        )
    }
}

/// A trigger's progress, by default SCP's: the adapter `A` detection found
/// and the output `O` requested.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status<A = scp_protocol::Adapter, O = scp_protocol::control::Output> {
    Entering,
    Reading,
    Detected(A),
    Output { output: O, applied: bool },
    Stopping,
    Stopped,
    Failed(Failure),
}
