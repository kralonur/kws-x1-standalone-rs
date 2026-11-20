//! Bounded selective logging policy, independent of flash and the PD driver.

// Minimum change from the last committed voltage, in millivolts.
const VOLTAGE_MV: u32 = 100;
// Minimum change from the last committed current, in milliamps.
const CURRENT_MA: u32 = 100;
// Keep a quiet observation after one minute since the last commit.
const HEARTBEAT_MS: u32 = 60_000;

/// One nonempty second of calibrated meter reads. The peak retains its sign
/// so a load spike in either plug orientation survives selection and export.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Summary {
    pub at: u32, // session-relative time of the last valid read in this window
    pub avg_mv: u16,
    pub avg_ma: i16,
    pub min_mv: u16,
    pub peak_ma: i16,
}

impl Summary {
    /// Compare like fields to the last *committed* window, so a dip is kept
    /// even when its contribution to the average is below the threshold.
    pub fn keep(self, last: Option<Self>) -> bool {
        last.is_none_or(|last| {
            u32::from(self.avg_mv.abs_diff(last.avg_mv)) >= VOLTAGE_MV
                || u32::from(self.avg_ma.abs_diff(last.avg_ma)) >= CURRENT_MA
                || u32::from(self.min_mv.abs_diff(last.min_mv)) >= VOLTAGE_MV
                || u32::from(self.peak_ma.abs_diff(last.peak_ma)) >= CURRENT_MA
                || self.at.saturating_sub(last.at) >= HEARTBEAT_MS
        })
    }
}

/// One-second buckets anchored to session start. Invalid reads advance time
/// but are never averaged as zero; missed seconds produce no synthetic rows.
#[derive(Default)]
pub struct Window {
    at: u32,
    count: u16,
    sum_mv: u32,
    sum_ma: i32,
    min_mv: u16,
    peak_ma: i16,
}

impl Window {
    pub fn observe(&mut self, at: u32, reading: Option<(u16, i16)>) -> Option<Summary> {
        // Width of a VI bucket, in milliseconds.
        const SECOND_MS: u32 = 1000;
        let finished = if self.at / SECOND_MS != at / SECOND_MS {
            self.finish()
        } else {
            None
        };
        if let Some((mv, ma)) = reading {
            if self.count == 0 || mv < self.min_mv {
                self.min_mv = mv;
            }
            if self.count == 0 || ma.unsigned_abs() > self.peak_ma.unsigned_abs() {
                self.peak_ma = ma;
            }
            // Meter reads are at least 100 ms apart within a 1 s bucket.
            self.count += 1;
            self.sum_mv += u32::from(mv);
            self.sum_ma += i32::from(ma);
            self.at = at;
        }
        finished
    }

    pub fn finish(&mut self) -> Option<Summary> {
        if self.count == 0 {
            return None;
        }
        let window = core::mem::take(self);
        Some(Summary {
            at: window.at,
            avg_mv: (window.sum_mv / u32::from(window.count)) as u16,
            avg_ma: (window.sum_ma / i32::from(window.count)) as i16,
            min_mv: window.min_mv,
            peak_ma: window.peak_ma,
        })
    }
}

/// Words copied from a COMPLETE, CRC-checked transport packet: word 0 is raw
/// header | (SOP code << 16), followed by every data object. Do not call for
/// incomplete input/diagnostics. Retain unknown SOP and inconsistent lengths.
pub fn routine(words: &[u32]) -> bool {
    let Some(&first) = words.first() else {
        return false;
    };
    // The receive token's high three bits encode SOP as 0b111.
    const SOP: u32 = 0b111;
    if first >> 16 != SOP {
        return false;
    }
    let header = first as u16;
    let count = usize::from((header >> 12) & 7);
    if words.len() != 1 + count {
        return false;
    }
    let extended = header & 0x8000 != 0;
    let kind = header & 31;
    // USB PD message types for control GoodCRC and extended EPR KeepAlive.
    const GOOD_CRC: u16 = 1;
    const EXTENDED_CONTROL: u16 = 16;
    // The exact unchunked KeepAlive/Ack object patterns from the tested policy.
    const KEEP_ALIVE: u32 = 0x0003_8002;
    const KEEP_ALIVE_ACK: u32 = 0x0004_8002;
    (!extended && kind == GOOD_CRC && count == 0)
        || (extended
            && kind == EXTENDED_CONTROL
            && count == 1
            && matches!(words[1], KEEP_ALIVE | KEEP_ALIVE_ACK))
}
