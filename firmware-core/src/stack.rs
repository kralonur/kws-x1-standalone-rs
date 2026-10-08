//! Peak stack use from a painted stack.
//!
//! `cortex-m-rt`'s `paint-stack` feature fills `[_stack_end, _stack_start)` with
//! [`PAINT`] in the reset handler, before `main`. The stack grows down from
//! `_stack_start`, so the lowest word that no longer holds the paint marks the
//! deepest point any code (tasks and interrupt handlers share the one MSP
//! stack) has reached since boot. A word written with exactly the paint value
//! would be missed; that is the usual, accepted limit of this method.

/// `cortex_m_rt::STACK_PAINT_VALUE`.
pub const PAINT: u32 = 0xcccc_cccc;

/// Peak stack use in bytes for a stack of `words` words, scanned from its low
/// end (`_stack_end`) upward; `read(i)` returns word `i` counted from there.
/// Stops at the first word that is not [`PAINT`].
pub fn peak_bytes(words: usize, mut read: impl FnMut(usize) -> u32) -> usize {
    let untouched = (0..words).take_while(|&i| read(i) == PAINT).count();
    (words - untouched) * 4
}
