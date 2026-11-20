//! The meter task owns the journal; the passive sniffer only queues receive events.
use crate::{I2C3, Orientation, flash, framebuffer, ina226, panel, read_meter, render, ui};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use embassy_hc32::gpio::InputPin;
use embassy_time::{Duration, Instant, Timer};
use firmware_core::selective_log;
use firmware_core::{buttons, charging_log as format};
use render::{Canvas, Frame};
// Six reserved erase sectors starting at the logger build's upper end.
const START: usize = firmware_image::LOGGER_UPPER_END as usize;
const END: usize = START + format::SECTORS * flash::SECTOR;
const _: () = assert!(flash::SECTOR == format::SECTOR);
/// Physical L+R; a third button disqualifies the chord.
pub const CHORD: u8 = buttons::LEFT | buttons::RIGHT;
/// Boot-only clear request; it still requires explicit on-screen confirmation.
pub const CLEAR_CHORD: u8 = CHORD | buttons::BACK;
/// The reference hold time, in input polls, before toggling once per press.
pub const HOLD_POLLS: u8 = (500 / buttons::POLL_MS) as u8;
// Queue capacity and per-meter-pass drain bound for asynchronous PD RX.
const QUEUE_CAPACITY: usize = 32;
const DRAIN_LIMIT: usize = 4;
// XSP1 uses the top 24 bits of the tag for the packet ID.
const MAX_EVENT_ID: u32 = 0x00ff_ffff;
// XSP1 reserves tag 8 for a loss count and fragment kind 9 for RX stop.
const DROP_TAG: u32 = 8;
const STOP_KIND: u32 = 9;
const FULL: &str = "LOGGER FULL - EXPORT";
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
static RECORDING: AtomicBool = AtomicBool::new(false);
static DROPPED: AtomicU32 = AtomicU32::new(0);
struct Event {
    at: u64,
    words: [u32; format::PACKET_WORDS],
    count: usize,
    kind: u32,
}
static EVENTS: Channel<CriticalSectionRawMutex, Event, QUEUE_CAPACITY> = Channel::new();
/// Called only by the passive sniffer: copy checked RX bytes, never wait or write flash.
#[inline(never)]
pub fn pd_packet(bytes: &[u8]) {
    if !RECORDING.load(Ordering::Relaxed) {
        return;
    }
    let Some((&token, rest)) = bytes.split_first() else {
        return;
    };
    let Some((header, objects)) = rest.split_first_chunk::<2>() else {
        return;
    };
    let header = u16::from_le_bytes(*header);
    let count = usize::from((header >> 12) & 7);
    let Some(objects) = objects.get(..count * size_of::<u32>()) else {
        return;
    };
    let mut words = [0; format::PACKET_WORDS];
    words[0] = u32::from(header) | (u32::from(token >> 5) << 16);
    for (word, object) in words[1..].iter_mut().zip(objects.as_chunks::<4>().0) {
        *word = u32::from_le_bytes(*object);
    }
    enqueue(Event {
        at: embassy_time::Instant::now().as_millis(),
        words,
        count: count + 1,
        kind: 0,
    });
}
#[inline(never)]
pub fn pd_stopped() {
    if RECORDING.load(Ordering::Relaxed) {
        enqueue(Event {
            at: embassy_time::Instant::now().as_millis(),
            words: [1; format::PACKET_WORDS],
            count: 1,
            kind: STOP_KIND,
        });
    }
}
fn enqueue(event: Event) {
    if EVENTS.try_send(event).is_err() {
        DROPPED.fetch_add(1, Ordering::Relaxed);
    }
}
static TOGGLE: AtomicBool = AtomicBool::new(false);

pub fn toggle() {
    TOGGLE.store(true, Ordering::Relaxed);
}

pub struct Writer {
    window: selective_log::Window,
    last_summary: Option<selective_log::Summary>,
    active: bool,
    stopping: bool,
    pd_base: usize,
    pd_index: usize,
    event_id: u32,
    vi_pending: bool,
    pd_pending: bool,
    start: u64,
    session: u32,
    segment: u32,
    base: usize,
    index: usize,
    message: &'static str,
}
impl Writer {
    pub fn new() -> Self {
        Self {
            window: Default::default(),
            last_summary: None,
            active: false,
            stopping: false,
            pd_base: 0,
            pd_index: 0,
            event_id: 0,
            vi_pending: false,
            pd_pending: false,
            start: 0,
            session: 0,
            segment: 0,
            base: 0,
            index: 0,
            message: ready_message(),
        }
    }
    pub fn message(&self) -> &'static str {
        self.message
    }
    pub fn handle_request(&mut self, now: u64) {
        if !TOGGLE.swap(false, Ordering::Relaxed) {
            return;
        }
        if self.active {
            RECORDING.store(false, Ordering::Relaxed);
            self.stopping = true;
            self.message = "LOGGER STOPPING";
            return;
        }
        let vi = tail(false);
        let pd = tail(true);
        let needed = usize::from(vi.is_none()) + usize::from(pd.is_none());
        let free = (START..END)
            .step_by(flash::SECTOR)
            .filter(|&b| available(b))
            .count();
        if free < needed {
            self.message = FULL;
            return;
        }
        let latest = (START..END)
            .step_by(flash::SECTOR)
            .map(|b| format::latest_session(bytes(b)))
            .max()
            .unwrap_or(0);
        if latest >= format::MAX_SESSIONS {
            self.message = "LOGGER SESSION LIMIT";
            return;
        }
        let session = latest + 1;
        while EVENTS.try_receive().is_ok() {}
        DROPPED.store(0, Ordering::Relaxed);
        self.stopping = false;
        (self.pd_base, self.pd_index) = pd.unwrap_or((0, 0));
        self.pd_pending = pd.is_some();
        self.event_id = 0;
        self.session = session;
        self.segment = 0;
        (self.base, self.index) = vi.unwrap_or((0, 0));
        self.vi_pending = vi.is_some();
        self.start = now;
        {
            self.window = Default::default();
            self.last_summary = None;
        }
        self.active = true;
        self.message = "MIN/PEAK 1s - L+R STOP";
        RECORDING.store(true, Ordering::Relaxed);
    }
    pub async fn sample(&mut self, now: u64, reading: Option<(u32, i32)>) {
        if !self.active {
            return;
        }
        // Bound each drain so the meter and PD tasks continue making progress.
        for _ in 0..DRAIN_LIMIT {
            let Ok(event) = EVENTS.try_receive() else {
                break;
            };
            let Ok(elapsed) = u32::try_from(event.at.saturating_sub(self.start)) else {
                self.fail("LOGGER TIME LIMIT");
                return;
            };
            if elapsed == u32::MAX {
                self.fail("LOGGER TIME LIMIT");
                return;
            }
            if self.event_id == MAX_EVENT_ID {
                self.fail("LOGGER EVENT LIMIT");
                return;
            }
            self.event_id += 1;
            // Assign IDs before suppression to preserve the journal numbering.
            // Only complete checked PD packets use kind 0.
            if event.kind == 0 && selective_log::routine(&event.words[..event.count]) {
                continue;
            }
            let bus = I2C3.lock().await;
            // Reserve the whole packet before committing its first fragment.
            let remaining = if self.pd_base == 0 {
                0
            } else {
                format::SAMPLES - self.pd_index - usize::from(self.pd_pending)
            };
            if !format::packet_fits(
                remaining,
                event.count,
                remaining < event.count && (START..END).step_by(flash::SECTOR).any(available),
            ) {
                self.fail(FULL);
                return;
            }
            for i in 0..event.count {
                let tag = (self.event_id << 8) | (event.kind + i as u32);
                if !self.append(true, format::record(elapsed, tag, event.words[i] as i32)) {
                    return;
                }
            }
            drop(bus);
            embassy_time::Timer::after_millis(1).await;
        }
        let _bus = I2C3.lock().await;
        let dropped = DROPPED.swap(0, Ordering::Relaxed);
        let Ok(elapsed) = u32::try_from(now - self.start) else {
            self.fail("LOGGER TIME LIMIT");
            return;
        };
        if elapsed == u32::MAX {
            self.fail("LOGGER TIME LIMIT");
            return;
        }
        if dropped != 0 && !self.append(true, format::record(elapsed, DROP_TAG, dropped as i32)) {
            return;
        }
        if self.stopping {
            if !EVENTS.is_empty() {
                return;
            }
            if let Some(summary) = self.window.finish()
                && !self.save_summary(summary)
            {
                return;
            }
            self.active = false;
            self.stopping = false;
            self.message = "LOGGER STOPPED - L+R START";
            return;
        }
        {
            let reading = match reading {
                Some((mv, ma)) => {
                    let (Ok(mv), Ok(ma)) = (u16::try_from(mv), i16::try_from(ma)) else {
                        self.fail("LOGGER READING RANGE");
                        return;
                    };
                    Some((mv, ma))
                }
                None => None,
            };
            if let Some(summary) = self.window.observe(elapsed, reading) {
                self.save_summary(summary);
            }
        }
    }
    fn save_summary(&mut self, s: selective_log::Summary) -> bool {
        if s.keep(self.last_summary) {
            if !self.append(
                false,
                format::extrema_record(s.at, s.avg_mv, s.avg_ma, s.min_mv, s.peak_ma),
            ) {
                return false;
            }
            self.last_summary = Some(s);
        }
        true
    }
    fn append(&mut self, event: bool, words: [u32; 4]) -> bool {
        let (mut base, mut index) = if event {
            (self.pd_base, self.pd_index)
        } else {
            (self.base, self.index)
        };
        let pending = if event {
            self.pd_pending
        } else {
            self.vi_pending
        };
        if pending {
            if !commit(
                base,
                base + format::HEADER + index * format::RECORD,
                format::session_marker(self.session),
            ) {
                self.fail("LOGGER WRITE FAILED");
                return false;
            }
            index += 1;
            if event {
                self.pd_pending = false;
            } else {
                self.vi_pending = false;
            }
        }
        if base == 0 || index == format::SAMPLES {
            let Some(free) = (START..END).step_by(flash::SECTOR).find(|&b| available(b)) else {
                self.fail(FULL);
                return false;
            };
            base = free;
            let h = format::header(self.session, self.segment, event);
            if !commit(base, base, h) {
                self.fail("LOGGER WRITE FAILED");
                return false;
            }
            index = 0;
            self.segment += 1;
        }
        if !commit(base, base + format::HEADER + index * format::RECORD, words) {
            self.fail("LOGGER WRITE FAILED");
            return false;
        }
        if event {
            self.pd_base = base;
            self.pd_index = index + 1;
        } else {
            self.base = base;
            self.index = index + 1;
        }
        true
    }
    fn fail(&mut self, message: &'static str) {
        RECORDING.store(false, Ordering::Relaxed);
        self.active = false;
        self.message = message;
    }
}
fn available(base: usize) -> bool {
    format::sector_free(sectors(), (base - START) / flash::SECTOR)
}
/// The latest sector of each stream is the only append candidate. A torn tail
/// seals that sector; a later header starts a fresh segment without overwriting it.
fn tail(event: bool) -> Option<(usize, usize)> {
    let base = (START..END)
        .step_by(flash::SECTOR)
        .filter(|&b| {
            format::valid_header(bytes(b)).is_some() && format::is_event(bytes(b)) == event
        })
        .max_by_key(|&b| {
            (
                format::latest_session(bytes(b)),
                flash::read(b + 4),
                flash::read(b + 8),
                b,
            )
        })?;
    format::reusable_tail(bytes(base), event).map(|index| (base, index))
}
fn ready_message() -> &'static str {
    let needed = usize::from(tail(false).is_none()) + usize::from(tail(true).is_none());
    let free = (START..END)
        .step_by(flash::SECTOR)
        .filter(|&b| available(b))
        .count();
    if (START..END)
        .step_by(flash::SECTOR)
        .map(|b| format::latest_session(bytes(b)))
        .max()
        .unwrap_or(0)
        >= format::MAX_SESSIONS
    {
        "LOGGER SESSION LIMIT"
    } else if free >= needed {
        "READY - HOLD L+R"
    } else {
        FULL
    }
}
fn commit(base: usize, at: usize, words: [u32; 4]) -> bool {
    // No erase during logging. Commit/checksum is a separate final operation.
    (START..END).contains(&base)
        && flash::program_blank(base, at, &words[..3])
        && flash::program_blank(base, at + format::RECORD - size_of::<u32>(), &words[3..])
}
fn bytes(base: usize) -> &'static [u8] {
    // SAFETY: only called for the six fixed sectors. Callers finish
    // reading before a write; export boot starts no writer tasks.
    unsafe { core::slice::from_raw_parts(base as *const u8, flash::SECTOR) }
}
fn sectors() -> [&'static [u8]; format::SECTORS] {
    core::array::from_fn(|i| bytes(START + i * flash::SECTOR))
}

/// The export boot never spawns a Writer, so this snapshot stays immutable.
pub fn export_disk() -> format::Disk<'static> {
    format::Disk::new(sectors())
}

pub fn button_mask(inputs: &[InputPin; 4]) -> u8 {
    inputs
        .iter()
        .enumerate()
        .fold(0, |mask, (i, pin)| mask | (u8::from(pin.is_low()) << i))
}

pub async fn at_pc_voltage(slider: &InputPin) -> bool {
    // Plausible PC VBUS window in millivolts, matching the export guard.
    const PC_VBUS_MV: core::ops::RangeInclusive<u32> = 4400..=5500;
    read_meter(ina226::BUS_VOLTAGE)
        .await
        .map(ina226::bus_millivolts)
        .is_some_and(|mv| PC_VBUS_MV.contains(&mv))
        && !slider.is_low()
}

/// Only called during clear boot, with exclusive use of the display and flash.
pub async fn confirm_clear(inputs: &[InputPin; 4], slider: &InputPin, allowed: bool) {
    if !allowed {
        boot_notice("CLEAR BLOCKED", "Use OFF/PASS at PC 5 V").await;
        return;
    }
    if !boot_notice("ERASE ALL CHARGING LOGS?", "Hold OK 2s; BACK cancels").await {
        return;
    }
    let title = if !clear_confirmed(inputs).await {
        "CLEAR CANCELLED"
    } else if !at_pc_voltage(slider).await {
        "CLEAR BLOCKED"
    } else {
        match erase_owned() {
            Some(true) => "CLEARED - UNKNOWN KEPT",
            Some(false) => "LOGS CLEARED",
            None => "CLEAR FAILED",
        }
    };
    boot_notice(title, "Unplug to restart").await;
}

async fn clear_confirmed(inputs: &[InputPin; 4]) -> bool {
    // The reference requires a fresh, continuous two-second OK hold.
    const HOLD: Duration = Duration::from_secs(2);
    let poll = Duration::from_millis(u64::from(buttons::POLL_MS));
    while button_mask(inputs) != 0 {
        Timer::after(poll).await;
    }
    let mut held = None;
    loop {
        match button_mask(inputs) {
            0 => held = None,
            buttons::OK => {
                if held.get_or_insert_with(Instant::now).elapsed() >= HOLD {
                    return true;
                }
            }
            _ => return false,
        }
        Timer::after(poll).await;
    }
}

pub async fn boot_notice(title: &str, detail: &str) -> bool {
    let fb = framebuffer();
    // SAFETY: no meter task is running and the prior flush has completed.
    unsafe {
        let frame = &mut *fb.cast::<Frame>();
        render::clear(frame);
        ui::logger::draw_boot_notice(&mut Canvas(frame), title, detail);
        panel::flush(fb, Orientation::Normal).await
    }
}

/// Returns whether any foreign sectors were kept, or None on erase failure.
#[inline(never)]
fn erase_owned() -> Option<bool> {
    let snapshot = sectors();
    let mask = format::clear_mask(snapshot);
    let kept = mask
        .iter()
        .enumerate()
        .any(|(i, &owned)| !owned && !format::sector_free(snapshot, i));
    let erase = core::hint::black_box(flash::erase_sector as unsafe extern "C" fn(u32) -> u32);
    for (i, owned) in mask.into_iter().enumerate() {
        if !owned {
            continue;
        }
        let base = START + i * flash::SECTOR;
        // SAFETY: the flash snapshot is no longer borrowed. Only the confirmed
        // clear boot calls this, with all other flash users idle and erases
        // limited to the six positively identified sectors. Code runs from RAM.
        let errors = cortex_m::interrupt::free(|_| unsafe { erase(base as u32) });
        let blank = flash::blank(base);
        if errors != 0 || !blank {
            return None;
        }
    }
    Some(kept)
}
