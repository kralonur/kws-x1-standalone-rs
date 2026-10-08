//! One-shot failure latch for a shared bus.
//!
//! A wedged I²C transfer must not be retried forever: the first failure latches
//! the bus off, and every later user sees the latched state and leaves the
//! peripheral alone. The screen, the timer and the buttons keep running, so a
//! bus fault costs measurements but never the UI.

use core::sync::atomic::{AtomicBool, Ordering};

/// Latches on the first failure and stays latched.
pub struct Latch {
    failed: AtomicBool,
}

impl Latch {
    pub const fn new() -> Self {
        Self {
            failed: AtomicBool::new(false),
        }
    }

    /// Record a failure. Returns `true` only for the failure that latched it.
    pub fn fail(&self) -> bool {
        !self.failed.swap(true, Ordering::SeqCst)
    }

    pub fn is_failed(&self) -> bool {
        self.failed.load(Ordering::SeqCst)
    }
}

impl Default for Latch {
    fn default() -> Self {
        Self::new()
    }
}
