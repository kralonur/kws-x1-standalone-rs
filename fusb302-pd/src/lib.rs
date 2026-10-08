//! FUSB302 transport, checked startup and receive-only adapters around the
//! controller-neutral `pd-protocol` library. Bus/time ownership, physical
//! observations, limits and board policy are supplied by callers; no MCU or
//! Embassy runtime, UI, sensor model or bootloader configuration is assumed.
#![no_std]

pub mod config;
pub mod observation;
pub mod pd_rx;
pub mod pd_sniff;
pub mod phy;
pub mod register;
pub mod service;
pub mod snapshot;
pub mod sniffer;
pub mod startup;
