//! Board policy and pure input, display and image logic for the firmware.
//! Hardware ownership stays in the application; protocol implementations are
//! supplied by the pinned dependencies and the local controller adapter.
#![no_std]

pub mod buttons;
pub mod clock;
pub mod image;
pub mod latch;
pub mod panel;
pub mod stack;
// Compatibility paths for the X1 firmware; implementations live in the shared
// crate, plus the FUSB302 FIFO and STATUS0 adapters for `pd_rx`/`pd_sniff`.
pub use fusb302_pd::{pd_rx, pd_sniff};
pub use pd_protocol::{pd_cable, pd_cable_details, pd_cable_modes, pd_partner, pd_svids};
pub mod pd_spr;
