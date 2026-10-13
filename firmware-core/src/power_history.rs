//! Recent readings for the overview's power graph: one averaged sample per
//! second, kept in RAM only, so it adds no flash wear and a restart clears it.

/// Seconds of history: the longest graph window at one sample per second.
pub const SECONDS: usize = GraphWindow::Min10.seconds();
/// Milliseconds per history sample.
const SAMPLE_MS: u64 = 1000;

/// How far back the graph reaches.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GraphWindow {
    Min1,
    #[default]
    Min2,
    Min5,
    Min10,
}

impl GraphWindow {
    /// The windows, in Left/Right order.
    pub const ALL: [Self; 4] = [Self::Min1, Self::Min2, Self::Min5, Self::Min10];

    pub const fn seconds(self) -> usize {
        match self {
            Self::Min1 => 60,
            Self::Min2 => 120,
            Self::Min5 => 300,
            Self::Min10 => 600,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::Min1 => "1 MIN",
            Self::Min2 => "2 MIN",
            Self::Min5 => "5 MIN",
            Self::Min10 => "10 MIN",
        }
    }
}

/// What the graph plots.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GraphTrace {
    #[default]
    Power,
    Voltage,
    Current,
}

impl GraphTrace {
    /// The traces, in Left/Right order.
    pub const ALL: [Self; 3] = [Self::Power, Self::Voltage, Self::Current];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Power => "POWER",
            Self::Voltage => "VOLTAGE",
            Self::Current => "CURRENT",
        }
    }

    pub const fn unit(self) -> &'static str {
        match self {
            Self::Power => "W",
            Self::Voltage => "V",
            Self::Current => "A",
        }
    }

    /// This trace's value in `sample`, in mW, mV or mA.
    pub fn value(self, sample: Sample) -> u32 {
        match self {
            Self::Power => sample.mw,
            Self::Voltage => sample.mv.into(),
            Self::Current => sample.ma.into(),
        }
    }
}

/// Voltage, current magnitude and power, in mV, mA and mW. 16 bits hold any
/// USB voltage (48 V at most) and current, keeping the history small.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Sample {
    pub mv: u16,
    pub ma: u16,
    pub mw: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct History {
    /// Average of each second, a ring whose oldest entry is at `head`;
    /// `None` for a second without a reading.
    samples: [Option<Sample>; SECONDS],
    head: usize,
    /// The second being averaged, and its readings' sums and count.
    second: Option<u64>,
    sums: [u64; 3],
    count: u32,
}

impl Default for History {
    fn default() -> Self {
        Self {
            samples: [None; SECONDS],
            head: 0,
            second: None,
            sums: [0; 3],
            count: 0,
        }
    }
}

impl History {
    /// Add a reading taken at `now_ms` (`None` when the meter had none). A
    /// finished second is stored as its average; whole seconds without any
    /// call (a stalled loop) are stored as gaps.
    pub fn sample(&mut self, now_ms: u64, reading: Option<Sample>) {
        let second = now_ms / SAMPLE_MS;
        if let Some(current) = self.second
            && second > current
        {
            let count = u64::from(self.count);
            let [mv, ma, mw] = self.sums.map(|sum| sum.checked_div(count).unwrap_or(0));
            // Averages of u16 and u32 readings fit their own types.
            self.push((count > 0).then_some(Sample {
                mv: mv as u16,
                ma: ma as u16,
                mw: mw as u32,
            }));
            for _ in 0..(second - current - 1).min(SECONDS as u64) {
                self.push(None);
            }
        }
        if self.second != Some(second) {
            self.second = Some(second);
            self.sums = [0; 3];
            self.count = 0;
        }
        if let Some(Sample { mv, ma, mw }) = reading {
            let values = [u64::from(mv), u64::from(ma), u64::from(mw)];
            for (sum, value) in self.sums.iter_mut().zip(values) {
                *sum += value;
            }
            self.count += 1;
        }
    }

    fn push(&mut self, sample: Option<Sample>) {
        self.samples[self.head] = sample;
        self.head = (self.head + 1) % SECONDS;
    }

    /// The seconds within `window`, oldest first.
    pub fn recent(&self, window: GraphWindow) -> impl Iterator<Item = Option<Sample>> + '_ {
        let (newer, older) = self.samples.split_at(self.head);
        older
            .iter()
            .chain(newer)
            .copied()
            .skip(SECONDS - window.seconds())
    }

    /// The newest stored second.
    pub fn latest(&self) -> Option<Sample> {
        self.samples[(self.head + SECONDS - 1) % SECONDS]
    }

    /// The highest voltage, current and power within `window`, each on its
    /// own.
    pub fn max(&self, window: GraphWindow) -> Option<Sample> {
        self.recent(window).flatten().reduce(|max, sample| Sample {
            mv: max.mv.max(sample.mv),
            ma: max.ma.max(sample.ma),
            mw: max.mw.max(sample.mw),
        })
    }
}
