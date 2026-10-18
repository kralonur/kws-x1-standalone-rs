//! Standalone SCP output selection and measured-voltage checks.

use scp_protocol::{Adapter, control::Output};

/// Application power-path envelope: 4–30 V input; negotiated current is
/// limited to 5 A, as on the existing PD trigger. Adapter ratings can only
/// narrow this envelope, never expand it.
pub const MIN_MV: u16 = 4_000;
pub const MAX_MV: u16 = 30_000;
pub const MAX_MA: u16 = 5_000;
/// Initial current-limit preview, in mA. This is a default, not a ceiling.
const INITIAL_MA: u16 = 1_000;
/// Application VBUS acceptance tolerance, ±5% of the requested voltage.
/// Unlike QC's halfway heuristic, an SCP output must reach its setpoint.
const VBUS_TOLERANCE_PERCENT: u32 = 5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Selection {
    adapter: Adapter,
    output: Output,
}

impl Selection {
    pub fn new(adapter: Adapter) -> Option<Self> {
        let cap = adapter.capabilities?;
        let step = cap.vstep_mv.filter(|step| *step > 0)?;
        let minimum = cap.min_mv.max(MIN_MV);
        let start = minimum.checked_add((step - (minimum - cap.min_mv) % step) % step)?;
        for mv in (start..=cap.max_mv.min(MAX_MV)).step_by(usize::from(step)) {
            if let Some(output) = Self::at(adapter, mv, INITIAL_MA) {
                return Some(Self { adapter, output });
            }
        }
        None
    }

    pub const fn output(self) -> Output {
        self.output
    }
    pub const fn adapter(self) -> Adapter {
        self.adapter
    }

    /// Return the preview to `mv`/`mA`, the output in force after a cancel.
    pub fn reset_to(&mut self, mv: u16, ma: u16) {
        if let Ok(output) = Output::new(self.adapter, mv, ma) {
            self.output = output;
        }
    }

    /// Move the voltage (mV) or current limit (mA) by at least `amount`,
    /// in whole advertised steps and never less than one.
    pub fn step(&mut self, voltage: bool, forward: bool, amount: u16) {
        let Some(cap) = self.adapter.capabilities else {
            return;
        };
        let Some(step) = (if voltage { cap.vstep_mv } else { cap.istep_ma }).filter(|s| *s > 0)
        else {
            return;
        };
        let times = amount.div_ceil(step).max(1);
        let current = if voltage {
            self.output.millivolts()
        } else {
            self.output.milliamps()
        };
        let (minimum, maximum) = if voltage {
            (cap.min_mv.max(MIN_MV), cap.max_mv.min(MAX_MV))
        } else {
            (cap.min_ma.max(1), cap.max_ma.min(MAX_MA))
        };
        let mut next = current;
        for _ in 0..times {
            next = if forward {
                next.saturating_add(step).min(maximum)
            } else {
                next.saturating_sub(step).max(minimum)
            };
        }
        // Some legacy encodings have a coarser quantum than the advertised
        // step. Skip unrepresentable previews rather than rounding a write.
        while next != current && (minimum..=maximum).contains(&next) {
            let output = if voltage {
                Self::at(self.adapter, next, self.output.milliamps())
            } else {
                Output::new(self.adapter, self.output.millivolts(), next).ok()
            };
            if let Some(output) = output {
                self.output = output;
                return;
            }
            let candidate = if forward {
                next.saturating_add(step)
            } else {
                next.saturating_sub(step)
            };
            if candidate == next {
                break;
            }
            next = candidate;
        }
    }

    fn at(adapter: Adapter, mv: u16, preferred_ma: u16) -> Option<Output> {
        let cap = adapter.capabilities?;
        let step = cap.istep_ma.filter(|step| *step > 0)?;
        let power_ma = u32::from(cap.max_power_dw?) * 100_000 / u32::from(mv);
        let maximum = cap
            .max_ma
            .min(MAX_MA)
            .min(u16::try_from(power_ma).unwrap_or(u16::MAX));
        let preferred = preferred_ma.min(maximum);
        if preferred < cap.min_ma {
            return None;
        }
        let mut ma = preferred - (preferred - cap.min_ma) % step;
        while ma > 0 && ma >= cap.min_ma {
            if let Ok(output) = Output::new(adapter, mv, ma) {
                return Some(output);
            }
            ma = ma.checked_sub(step)?;
        }
        None
    }
}

pub fn request(
    adapter: Adapter,
    mv: u16,
    ma: u16,
) -> Result<Output, scp_protocol::control::RequestError> {
    if !(MIN_MV..=MAX_MV).contains(&mv) || ma > MAX_MA {
        return Err(scp_protocol::control::RequestError::Range);
    }
    Output::new(adapter, mv, ma)
}

pub fn followed(mv: u16, vbus_mv: u16) -> bool {
    u32::from(vbus_mv.abs_diff(mv)) * 100 <= u32::from(mv) * VBUS_TOLERANCE_PERCENT
        && vbus_mv <= MAX_MV
}
