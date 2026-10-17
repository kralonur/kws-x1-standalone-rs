//! Read-only SCP probe status, also used by the host renderer.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Failure {
    Entry,
    DriveLevel,
    Vbus,
    Clock,
    Wire(scp_protocol::Error<core::convert::Infallible>),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Entering,
    Reading,
    Detected(scp_protocol::Adapter),
    Failed(Failure),
}
