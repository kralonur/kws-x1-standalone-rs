//! Passive FCP/SCP decoding: D−'s external interrupt timestamps every edge
//! and feeds the protocol decoder; complete transactions go to `meter_task`.
//! [`LineAdc`](crate::line_adc::LineAdc) owns the pin and decides when its
//! edges are watched.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::interrupt::InterruptNumber;
use cortex_m::peripheral::{DWT, NVIC};
use embassy_hc32::gpio::Port;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_sync::channel::Channel;
use single_wire_protocols::sniff::{Decoder, Transaction};

/// D− is PB0, and pin n of every port is external-interrupt channel n
/// (RM 10.5.5). Errata 2.4.1: no other port's pin 0 may enable it too.
pub const EIRQ: u8 = 0;
/// Decoded transactions waiting for `meter_task`, which drains them every
/// 10 ms tick; one transaction takes about 30 ms, so a few cover a stall.
const QUEUE: usize = 4;
/// Inside one transaction no level lasts longer than a ping (Table B.4
/// `Tp-MR`, at most 2816 µs) or the 5-UI reply wait (880 µs at the longest
/// UI); D− quiet this long in µs is between transactions.
const QUIET_US: u32 = 4_000;
/// Valid traffic changes level about a dozen times per ms at most (a 144 µs
/// UI with ¼-UI syncs); more edges than this in one window are noise, such
/// as a line held near the pin's switching threshold.
const STORM_EDGES: u32 = 64;
const STORM_WINDOW_US: u32 = 1_000;

/// Set when the interrupt masked itself for a storm; see [`take_storm`].
static STORM: AtomicBool = AtomicBool::new(false);

pub static TRANSACTIONS: Channel<CriticalSectionRawMutex, Transaction, QUEUE> = Channel::new();

/// The decoder and its µs clock, `None` while edges are not watched.
static SNIFFER: Mutex<CriticalSectionRawMutex, RefCell<Option<Sniffer>>> =
    Mutex::new(RefCell::new(None));

struct Sniffer {
    decoder: Decoder,
    /// Cycle count the µs clock has reached.
    cycles: u32,
    /// The decoder's µs clock, which wraps at 2³² µs rather than with the
    /// cycle counter.
    us: u32,
    cycles_per_us: u32,
    /// Cycle count of the latest edge.
    last_edge: u32,
    /// Start of the current storm window, in cycles, and its edges so far.
    window: u32,
    window_edges: u32,
}

impl Sniffer {
    /// Count an edge; `true` once a window holds more than [`STORM_EDGES`].
    fn storm(&mut self, cycles: u32) -> bool {
        if cycles.wrapping_sub(self.window) >= STORM_WINDOW_US * self.cycles_per_us {
            self.window = cycles;
            self.window_edges = 0;
        }
        self.window_edges += 1;
        self.window_edges > STORM_EDGES
    }

    fn edge(&mut self, cycles: u32, high: bool) -> Option<Transaction> {
        self.last_edge = cycles;
        let elapsed = cycles.wrapping_sub(self.cycles) / self.cycles_per_us;
        // Keep the remainder, so rounding never accumulates.
        self.cycles = self
            .cycles
            .wrapping_add(elapsed.wrapping_mul(self.cycles_per_us));
        self.us = self.us.wrapping_add(elapsed);
        self.decoder.push(self.us, high)
    }
}

/// Decode from scratch: edges went unwatched, so any partial packet is
/// lost. `false` without a µs clock: nothing can be decoded, nor an edge
/// storm caught, so the caller must not watch.
pub fn start() -> bool {
    let sniffer = crate::scp_drive::cycle_clock().map(|cycles_per_us| {
        // Only now is the cycle counter known to run.
        let now = DWT::cycle_count();
        Sniffer {
            decoder: Decoder::default(),
            cycles: now,
            us: 0,
            cycles_per_us,
            last_edge: now,
            window: now,
            window_edges: 0,
        }
    });
    let started = sniffer.is_some();
    SNIFFER.lock(|cell| cell.replace(sniffer));
    started
}

/// No transaction is in flight, so pausing now loses none. A cycle count
/// that wrapped since the last edge only delays a pause by [`QUIET_US`].
pub fn quiet() -> bool {
    SNIFFER.lock(|cell| {
        cell.borrow().as_ref().is_none_or(|sniffer| {
            DWT::cycle_count().wrapping_sub(sniffer.last_edge)
                >= QUIET_US.saturating_mul(sniffer.cycles_per_us)
        })
    })
}

pub fn stop() {
    SNIFFER.lock(|cell| cell.replace(None));
}

/// Whether the interrupt masked itself for an edge storm since the last
/// call. Its owner must then stop watching before watching again, since
/// that re-enables the line.
pub fn take_storm() -> bool {
    STORM.swap(false, Ordering::Relaxed)
}

/// The D− edge interrupt on NVIC `line`: timestamp the edge, then decode
/// it. An edge storm masks `line`, so noise cannot starve other work.
pub fn on_interrupt(line: impl InterruptNumber) {
    let cycles = DWT::cycle_count();
    if !embassy_hc32::intc::eirq_pending(EIRQ) {
        return;
    }
    embassy_hc32::intc::eirq_clear(EIRQ);
    // Only reads the input level of PB0 (bit 0), which the line ADC owns.
    let high = embassy_hc32::gpio::read_port(Port::B) & 1 != 0;
    let (storm, transaction) = SNIFFER.lock(|cell| {
        let mut cell = cell.borrow_mut();
        let Some(sniffer) = cell.as_mut() else {
            return (false, None);
        };
        (sniffer.storm(cycles), sniffer.edge(cycles, high))
    });
    if storm {
        NVIC::mask(line);
        STORM.store(true, Ordering::Relaxed);
    }
    if let Some(transaction) = transaction {
        // A full queue drops this one: `meter_task` drains it far faster
        // than transactions arrive, so only a long stall loses any.
        let _ = TRANSACTIONS.try_send(transaction);
    }
}
