//! Async meter firmware with passive boot, receive-only passthrough and
//! explicitly selected fixed/PPS sink sessions.
//!
//! The application owns clock startup, pins, shared I2C3, display and tasks.
//! Portable PD policy and the controller PHY independently authorize traffic.
//! The vector table starts at the bootloader's application entry; SysTick
//! drives Embassy time. Bus faults latch closed rather than retrying.

#![no_std]
#![no_main]

mod bus;
mod clock_setup;
mod ina226;
mod lis3dh;
#[path = "panel.rs"]
mod panel;
mod pd;
mod pd_cc;
mod pd_spr_phy;
mod pd_task;
mod pd_trial;

mod pd_service;
mod pd_sniff;
mod render;

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_hc32::gpio::{self, AlternateMode, AlternatePin, InputPin, Port};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
// The `#[cortex_m_rt::interrupt]` expansion checks `interrupt::INT0xx`, and the
// PAC exposes the generated `Interrupt` enum through that path.
use bus::{
    ASYNC_READS, BUS_LATCH, I2C_ERRORS, I2C_FAULT_STEP, I2C_IRQ_COUNT, I2C_LAST_CR1, I2C_LAST_CR4,
    I2C_LAST_SR, I2C3, bus_failed, read_meter, write_meter, write_sensor,
};
use embassy_time::{Instant, Timer};
use firmware_core::buttons as button_logic;
use hc32f460_pac::interrupt;
use panel::Orientation;
#[cfg(not(any(feature = "pd-boot-diagnostic", feature = "cable-log")))]
use panic_halt as _;
use render::{Counters, Frame, clear_screen, glyph, render_irq_test, render_measure, render_ports};

/// Return addresses the walk accepts: the application image after its initial
/// stack pointer word, up to the end of the flash tool's application erase
/// envelope.
#[cfg(any(feature = "pd-boot-diagnostic", feature = "cable-log"))]
const RETURN_RANGE: core::ops::Range<u32> = firmware_core::image::APP_BASE + 4..0x44000;
/// BL encoding T1: first halfword 11110xxx_xxxxxxxx, second 11x1xxxx_xxxxxxxx
/// (Armv7-M ARM A7.7.18).
#[cfg(any(feature = "pd-boot-diagnostic", feature = "cable-log"))]
const BL_FIRST: (u16, u16) = (0xf800, 0xf000);
#[cfg(any(feature = "pd-boot-diagnostic", feature = "cable-log"))]
const BL_SECOND: (u16, u16) = (0xd000, 0xd000);
/// BLX (register) encoding T1: 01000111_1xxxx000 (Armv7-M ARM A7.7.19).
#[cfg(any(feature = "pd-boot-diagnostic", feature = "cable-log"))]
const BLX: (u16, u16) = (0xff87, 0x4780);

/// `pd-boot-diagnostic`: return addresses on the stack above `sp`, innermost
/// first: odd words inside the application image whose preceding instruction
/// is a BL or BLX. They map to functions with the image's ELF. The panic
/// message and location are not used, so their formatting code is not linked.
#[cfg(any(feature = "pd-boot-diagnostic", feature = "cable-log"))]
fn return_addresses(mut sp: usize, out: &mut [u32]) {
    unsafe extern "C" {
        static _stack_start: u32;
    }
    let top = core::ptr::addr_of!(_stack_start) as usize;
    let half = |address: u32| unsafe { (address as *const u16).read_volatile() };
    let is = |address: u32, (mask, value): (u16, u16)| half(address) & mask == value;
    let mut n = 0;
    while sp + 4 <= top && n < out.len() {
        let value = unsafe { (sp as *const u32).read_volatile() };
        let ret = value & !1;
        if value & 1 == 1 && RETURN_RANGE.contains(&ret) {
            let bl = is(ret - 4, BL_FIRST) && is(ret - 2, BL_SECOND);
            let blx = is(ret - 2, BLX);
            if bl || blx {
                out[n] = ret;
                n += 1;
            }
        }
        sp += 4;
    }
}

/// `pd-boot-diagnostic`: record the call chain of a panic in the flash log,
/// then halt (as `panic-halt` does in normal builds).
#[cfg(any(feature = "pd-boot-diagnostic", feature = "cable-log"))]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    let mut chain = [0; 5];
    return_addresses(cortex_m::register::msp::read() as usize, &mut chain);
    flash_log::fatal(flash_log::PANIC, chain);
    loop {
        cortex_m::asm::nop();
    }
}

/// Exception basic frame: eight words (Armv7-M ARM B1.5.6). An extended frame
/// with FP state is larger; the walk then also scans that state, where a word
/// is only reported if it passes the range and BL/BLX checks.
#[cfg(any(feature = "pd-boot-diagnostic", feature = "cable-log"))]
const BASIC_FRAME_BYTES: usize = 8 * 4;

/// `pd-boot-diagnostic`: record the faulting PC, LR and the return addresses
/// above the exception frame, then halt.
#[cfg(any(feature = "pd-boot-diagnostic", feature = "cable-log"))]
#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    cortex_m::interrupt::disable();
    let mut chain = [frame.pc(), frame.lr(), 0, 0, 0];
    return_addresses(
        core::ptr::from_ref(frame) as usize + BASIC_FRAME_BYTES,
        &mut chain[2..],
    );
    flash_log::fatal(flash_log::FAULT, chain);
    loop {
        cortex_m::asm::nop();
    }
}

/// Heartbeat square colours, alternating with each pulse.
const ALIVE_A: u16 = render::WHITE;
const ALIVE_B: u16 = render::BLUE;
/// Heartbeat colour while the I2C3 latch is tripped.
const FAULT: u16 = render::RED;

/// 240 x 135 RGB565 frame, memory byte order expected by `panel::flush`.
/// It lives alone in SRAM2 (`memory.x`): in SRAMH+SRAM1 it left the stack only
/// ~7.9 KiB, and the deepest PD paths overflowed into the timer queue just
/// below it (HardFault at PC 0 in a timer wake, `docs/SAFETY.md`).
/// Not zeroed by the runtime: `main` zeroes it before any task reads it.
// SAFETY: memory.x reserves SRAM2 for this frame; startup initializes it before use.
#[unsafe(link_section = ".sram2.framebuffer")]
static mut FRAMEBUFFER: core::mem::MaybeUninit<Frame> = core::mem::MaybeUninit::uninit();
/// Incremented by `pulse_task` every [`PULSE_MS`].
static PULSES: AtomicU32 = AtomicU32::new(0);
/// Coherent input snapshot: buttons 0..3, main page 4..6, cable detail 8..31.
/// `button_task` sends it when it changes; the renderer and PD trial read it.
static BUTTON_STATE: Watch<CriticalSectionRawMutex, u32, 1> = Watch::new();
// One bitmap, outside both observations and async futures.
static CABLE_SVIDS: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    core::cell::RefCell<firmware_core::pd_svids::Svids>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(
    firmware_core::pd_svids::Svids::EMPTY,
));
struct CableSvidStore;
static CABLE_SVID_STORE: CableSvidStore = CableSvidStore;
impl firmware_core::pd_svids::Store for CableSvidStore {
    fn clear(&self) {
        CABLE_SVIDS.lock(|cell| cell.borrow_mut().clear());
    }
    fn response(&self, objects: &[u32]) -> Option<firmware_core::pd_svids::Progress> {
        CABLE_SVIDS.lock(|cell| cell.borrow_mut().response(objects))
    }
    fn at(&self, index: u16) -> Option<u16> {
        CABLE_SVIDS.lock(|cell| cell.borrow().at(index))
    }
    fn contains(&self, svid: u16) -> bool {
        CABLE_SVIDS.lock(|cell| cell.borrow().contains(svid))
    }
}
/// Nominal timer milliseconds, not calibrated wall-clock measurements.
static FRAME_MS: AtomicU32 = AtomicU32::new(0);
static BUTTON_GAP_MAX_MS: AtomicU32 = AtomicU32::new(0);
static PD_IRQ_COUNT: AtomicU32 = AtomicU32::new(0);
static PD_OBSERVATION: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    core::cell::RefCell<pd::Observation>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(pd::Observation::new()));

/// Store the PD observation for the renderer. `pd-boot-diagnostic` also keeps
/// it in the boot record, so the next boot can show how far this one got.
fn store_observation(observation: pd::Observation) {
    #[cfg(feature = "pd-boot-diagnostic")]
    flash_log::save(&observation);
    // `cable-log`: nothing is written before a cable query starts, so the
    // boot and the contract negotiation are untouched.
    #[cfg(feature = "cable-log")]
    if observation.cable_step != 0 {
        flash_log::save(&observation);
    }
    PD_OBSERVATION.lock(|cell| *cell.borrow_mut() = observation);
}

#[cfg(any(feature = "pd-boot-diagnostic", feature = "cable-log"))]
mod flash_log;

fn framebuffer() -> *mut u16 {
    addr_of_mut!(FRAMEBUFFER) as *mut u16
}

#[cortex_m_rt::exception]
fn SysTick() {
    embassy_hc32::time_driver::systick_tick();
    #[cfg(feature = "cable-log")]
    stall::check();
}

/// `cable-log` stall detector, armed by the display task's first loop (a
/// boot-time hang was seen too). That task bumps `BEAT` every loop (about
/// 10 ms) and never pauses for seconds. If `BEAT` stays unchanged for 3 s,
/// this 1 ms SysTick records the return addresses above the interrupted code
/// once (flash log error 252): a busy loop shows its callers, all tasks
/// waiting shows the executor's idle loop. Silent if interrupts are off.
#[cfg(feature = "cable-log")]
mod stall {
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
    pub static ARMED: AtomicBool = AtomicBool::new(false);
    pub static BEAT: AtomicU32 = AtomicU32::new(0);
    static SEEN: AtomicU32 = AtomicU32::new(0);
    static SINCE_MS: AtomicU32 = AtomicU32::new(0);
    static DONE: AtomicBool = AtomicBool::new(false);
    /// Unchanged `BEAT` this long, in milliseconds, is recorded as a hang.
    const HANG_MS: u32 = 3000;

    pub fn check() {
        if !ARMED.load(Ordering::Relaxed) || DONE.load(Ordering::Relaxed) {
            return;
        }
        let now = embassy_time::Instant::now().as_millis() as u32;
        let beat = BEAT.load(Ordering::Relaxed);
        if beat != SEEN.load(Ordering::Relaxed) || SINCE_MS.load(Ordering::Relaxed) == 0 {
            SEEN.store(beat, Ordering::Relaxed);
            SINCE_MS.store(now.max(1), Ordering::Relaxed);
            return;
        }
        if now.wrapping_sub(SINCE_MS.load(Ordering::Relaxed)) >= HANG_MS {
            DONE.store(true, Ordering::Relaxed);
            let mut chain = [0; 5];
            crate::return_addresses(cortex_m::register::msp::read() as usize, &mut chain);
            crate::flash_log::fatal(crate::flash_log::HANG, chain);
        }
    }
}

/// INTC validation counters, shown on the "IRQ TEST" page.
static EIRQ_COUNT: AtomicU32 = AtomicU32::new(0);
static SWI_COUNT: AtomicU32 = AtomicU32::new(0);
/// Screen orientation: `tilt_task` sends it, `meter_task` applies it.
static ORIENTATION: Watch<CriticalSectionRawMutex, Orientation, 1> = Watch::new();
/// Completed 180-degree flips, shown next to the raw edge count.
static TILT_FLIPS: AtomicU32 = AtomicU32::new(0);

/// Failed `intc::register` calls. The INTC is not reset by a jump into this
/// image, so a `SEL` entry left mapped by the bootloader makes routing fail;
/// this must stay 0 now that startup clears the table first.
static IRQ_REG_FAILURES: AtomicU32 = AtomicU32::new(0);

/// External interrupt channels: PB12 carries the PD controller's alert, PB13
/// the tilt sensor's INT1. Channel n is also interrupt event n (RM Table 10-2).
const PD_EIRQ: u8 = 12;
const TILT_EIRQ: u8 = 13;
/// Software interrupt channel (and event) used for the dispatch self-test.
const SWI_CHANNEL: u8 = 8;
/// NVIC lines; each must match its `INTnnn` handler below.
const PD_LINE: embassy_hc32::intc::Line = line(4);
const TILT_LINE: embassy_hc32::intc::Line = line(13);
const SWI_LINE: embassy_hc32::intc::Line = line(8);
/// NVIC priorities (lower is more urgent). Both stay below SysTick (0), so
/// timing never starves; bus and tilt wakes rank ahead of the PD alert and self-test.
const BUS_PRIORITY: u8 = 8;
const PD_PRIORITY: u8 = 9;

/// PB12 / EIRQ12: acknowledge/latch/wake only. No I2C or PHY service in ISR.
#[cortex_m_rt::interrupt]
fn INT004() {
    if embassy_hc32::exti::on_interrupt(PD_EIRQ) {
        PD_IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

/// PB13 external interrupt: the async driver clears the flag and wakes
/// `tilt_task`; the counter is the same evidence shown on the INTC page.
#[cortex_m_rt::interrupt]
fn INT013() {
    if embassy_hc32::exti::on_interrupt(TILT_EIRQ) {
        EIRQ_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

/// Software interrupt channel 8 (self-test that `SEL`/`IER`/NVIC dispatch work).
#[cortex_m_rt::interrupt]
fn INT008() {
    embassy_hc32::intc::software_interrupt_clear(SWI_CHANNEL);
    SWI_COUNT.fetch_add(1, Ordering::Relaxed);
}

/// I2C3 events (RXI 428, TXI 429, TEI 430, EEI 431) on lines 110..=113: the
/// async driver only needs a wakeup, then it re-reads `SR` itself.
#[cortex_m_rt::interrupt]
fn INT110() {
    I2C_IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
    embassy_hc32::i2c::on_interrupt();
}

#[cortex_m_rt::interrupt]
fn INT111() {
    I2C_IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
    embassy_hc32::i2c::on_interrupt();
}

#[cortex_m_rt::interrupt]
fn INT112() {
    I2C_IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
    embassy_hc32::i2c::on_interrupt();
}

#[cortex_m_rt::interrupt]
fn INT113() {
    I2C_IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
    embassy_hc32::i2c::on_interrupt();
}

/// SPI3 TX-empty, physical idle and error sources (group 9, lines 86..88).
#[cortex_m_rt::interrupt]
fn INT086() {
    embassy_hc32::spi::on_interrupt();
}
#[cortex_m_rt::interrupt]
fn INT087() {
    embassy_hc32::spi::on_interrupt();
}
#[cortex_m_rt::interrupt]
fn INT088() {
    embassy_hc32::spi::on_interrupt();
}

/// Peak stack use since boot and the stack size, in bytes. The reset handler
/// paints `[_stack_end, _stack_start)` (`cortex-m-rt` `paint-stack`); the scan
/// stops at the first scrubbed word, so it reads at most the unused part.
fn stack_use() -> (u32, u32) {
    unsafe extern "C" {
        static _stack_end: u32;
        static _stack_start: u32;
    }
    let low = core::ptr::addr_of!(_stack_end) as usize;
    let words = (core::ptr::addr_of!(_stack_start) as usize - low) / 4;
    let peak = firmware_core::stack::peak_bytes(words, |i| unsafe {
        (low as *const u32).add(i).read_volatile()
    });
    (peak as u32, (words * 4) as u32)
}

/// PH2 Func4 is TIMA_4_PWM7 (HC32 Datasheet Table 2-1): TimerA4 channel 7.
const BACKLIGHT_FUNCTION: u8 = 4;
const BACKLIGHT_CHANNEL: u8 = 7;
/// TimerA4 counts PCLK1 / 256 (RM 21.5.4, set up by the HAL), so 1875 counts
/// give 125 Hz at a 60 MHz PCLK1, 156 Hz at 75 MHz. Retained, hardware-tested.
const BACKLIGHT_PERIOD: u16 = 1875;
/// Compare at 60% of [`BACKLIGHT_PERIOD`]: the retained backlight level.
const BACKLIGHT_COMPARE: u16 = 1125;

unsafe fn init_backlight(
    compare: u16,
) -> Result<(embassy_hc32::pwm::TimerA4Pwm, AlternatePin), embassy_hc32::pwm::Error> {
    unsafe {
        let mut pwm = embassy_hc32::pwm::TimerA4Pwm::new(
            hc32f460_pac::Tmra4::steal(),
            BACKLIGHT_CHANNEL,
            BACKLIGHT_PERIOD,
        )?;

        let pin = AlternatePin::new(Port::H, 2, BACKLIGHT_FUNCTION, AlternateMode::DigitalOutput);
        pwm.start(compare)?;
        Ok((pwm, pin))
    }
}

/// An interrupt line checked at compile time: call it inside `const { }` so a
/// number past the last NVIC line fails the build instead of panicking.
const fn line(n: u8) -> embassy_hc32::intc::Line {
    match embassy_hc32::intc::Line::new(n) {
        Some(line) => line,
        None => panic!("interrupt line out of range"),
    }
}

/// Heartbeat period in milliseconds; the meter page shows the pulse count.
const PULSE_MS: u64 = 500;
/// Button sampling period in milliseconds; debouncing counts these samples.
const BUTTON_POLL_MS: u64 = 10;
/// Meter task tick in milliseconds: inputs and the screen are checked this often.
const METER_TICK_MS: u64 = 10;
/// INA226 read pace; a deadline, so a slow frame never queues extra reads.
const METER_READ_PERIOD: embassy_time::Duration = embassy_time::Duration::from_millis(100);
/// I2C3 clock in Hz: Fast-mode 400 kHz, the LIS3DH maximum (LIS Table 8);
/// the FUSB302B (DS Table 14) and INA226 accept faster clocks.
const I2C3_HZ: u32 = 400_000;

#[embassy_executor::task]
async fn pulse_task() {
    loop {
        Timer::after_millis(PULSE_MS).await;
        PULSES.fetch_add(1, Ordering::Relaxed);
    }
}

/// Async tilt handling: wait on the PB13 external interrupt, then hand the
/// orientation change to `meter_task` through `ORIENTATION`.
#[embassy_executor::task]
async fn tilt_task(input: InputPin) {
    let mut tilt = unsafe {
        embassy_hc32::exti::ExtiInput::from_input(input, embassy_hc32::intc::EirqTrigger::Rising)
    };
    // Errata 2.4.2: the peripheral side is already enabled by
    // `ExtiInput::new`, so claiming the line here clears pending before NVIC.
    if unsafe { embassy_hc32::intc::register(u16::from(TILT_EIRQ), TILT_LINE, BUS_PRIORITY) }
        .is_err()
    {
        IRQ_REG_FAILURES.fetch_add(1, Ordering::Relaxed);
    }
    // One physical tilt makes the LIS3DH emit a burst of short pulses, and the
    // gaps inside that burst can exceed any short pre-flip window (a longer one
    // produced three flips per tilt). So the flip happens *immediately* on the
    // first edge — the sensor threshold is then reconfigured, which is what
    // actually ends the burst — and everything that follows is suppressed by a
    // refractory lockout. Edges latched during the lockout are drained so the
    // next tilt starts clean.
    //
    // INT1 is armed for the position opposite the displayed one, and in 6D
    // position mode it stays high while the device is in that position (LIS
    // Table 55). So a high level means "flip" even without a new edge: a turn
    // back during the lockout, or a device started upside down, otherwise
    // leaves the screen wrong until the next full flip.
    const REFRACTORY_MS: u64 = 1500;
    let orientation_sender = ORIENTATION.sender();
    let mut orientation = Orientation::Normal;
    loop {
        if !tilt.is_high() {
            tilt.wait_for_edge().await;
        }

        // Re-arm the sensor before showing the flip: if the write fails, the
        // sensor still watches the old position, the screen stays consistent
        // with it, and the level check retries after the lockout.
        let next = orientation.flipped();
        if write_sensor(lis3dh::INT1_CFG, armed_position(next)).await {
            orientation = next;
            orientation_sender.send(orientation);
            TILT_FLIPS.fetch_add(1, Ordering::Relaxed);
        }

        // Suppress the rest of the burst, then discard anything it latched.
        Timer::after_millis(REFRACTORY_MS).await;
        while embassy_hc32::exti::take_edge(TILT_EIRQ) {}
    }
}

/// INT1_CFG for an orientation: raise INT1 in the position opposite it.
fn armed_position(orientation: Orientation) -> u8 {
    match orientation {
        Orientation::Normal => lis3dh::POSITION_X_LOW,
        Orientation::Inverted => lis3dh::POSITION_X_HIGH,
    }
}

/// This task never awaits display/I2C work. Navigation is retained even when a
/// press and release both finish during a frame; only the renderer owns pixels.
#[embassy_executor::task]
async fn button_task(inputs: [InputPin; 4]) {
    // `pd-boot-diagnostic`: open on the PD page (3) so the state before a
    // source Hard Reset is on screen (docs/SAFETY.md).
    // `cable-log`: a panic/fault/hang record from the previous boot opens the
    // PD page, which shows it, so no navigation is needed after a freeze.
    #[cfg(feature = "cable-log")]
    let fatal = flash_log::LAST
        .lock(|cell| cell.get())
        .and_then(|last| last.record)
        .is_some_and(|record| record.error >= flash_log::PANIC);
    #[cfg(not(feature = "cable-log"))]
    let fatal = false;
    let mut navigation = if cfg!(feature = "pd-boot-diagnostic") || fatal {
        button_logic::Navigation::at_page(3)
    } else {
        button_logic::Navigation::default()
    };
    let input_sender = BUTTON_STATE.sender();
    let mut previous = Instant::now();
    loop {
        let now = Instant::now();
        BUTTON_GAP_MAX_MS.fetch_max(
            now.duration_since(previous).as_millis() as u32,
            Ordering::Relaxed,
        );
        previous = now;
        let raw = inputs.iter().enumerate().fold(0u8, |mask, (index, pin)| {
            mask | (u8::from(pin.is_low()) << index)
        });
        let snapshot = u32::from(navigation.update(raw)) | (navigation.detail() << 8);
        input_sender.send_if_modified(|current| {
            let changed = *current != Some(snapshot);
            *current = Some(snapshot);
            changed
        });
        Timer::after_millis(BUTTON_POLL_MS).await;
    }
}
#[inline(never)]
unsafe fn render_pd(framebuffer: *mut u16, raw_vbus: u16, cable_detail: Option<u32>) {
    let mut observation = PD_OBSERVATION.lock(|cell| *cell.borrow());
    #[cfg(feature = "pd-boot-diagnostic")]
    {
        observation.last_boot = flash_log::LAST.lock(|cell| cell.get());
    }
    // `cable-log`: the previous logged boot's last record, until this boot
    // starts its own cable query.
    #[cfg(feature = "cable-log")]
    if observation
        .spr
        .is_some_and(|spr| !spr.cable_active && spr.cable_report.is_none())
    {
        observation.last_boot = flash_log::LAST
            .lock(|cell| cell.get())
            .filter(|last| last.record.is_some());
    }
    if let Some(spr) = observation.spr.as_mut() {
        spr.vbus_live_mv = if raw_vbus != u16::MAX && !bus_failed() {
            Some(ina226::bus_millivolts(raw_vbus).min(u16::MAX as u32) as u16)
        } else {
            None
        };
    }
    unsafe {
        clear_screen(framebuffer);
        // This task is the sole framebuffer owner; other tasks publish values.
        if let Some(page) = cable_detail {
            let svid = pd::cable_svid_index(observation, page)
                .and_then(|index| firmware_core::pd_svids::Store::at(&CABLE_SVID_STORE, index));
            pd::paint_cable(
                &mut *(framebuffer as *mut Frame),
                observation,
                page,
                svid,
                glyph,
            );
        } else {
            pd::paint(&mut *(framebuffer as *mut Frame), observation, glyph);
        }
    }
}

/// Charger info page (`pd::partner_lines`); `page` selects the sub-page.
unsafe fn render_partner(framebuffer: *mut u16, page: u32) {
    let observation = PD_OBSERVATION.lock(|cell| *cell.borrow());
    unsafe {
        clear_screen(framebuffer);
        pd::paint_partner(&mut *(framebuffer as *mut Frame), observation, page, glyph);
    }
}

#[embassy_executor::task]
async fn meter_task() {
    let fb = framebuffer();
    let mut input_updates = BUTTON_STATE.anon_receiver();
    let mut buttons = 0;
    let mut dirty = true;
    // Last raw readings, `u16::MAX` until the first read; a changed reading
    // redraws the meter page.
    let mut last_raw_v: u16 = u16::MAX;
    let mut last_raw_i: u16 = u16::MAX;
    // Reads are paced by an absolute deadline, never by subtracting from an
    // `Instant` (embassy-time panics on underflow, e.g. `Instant::now() - 200ms`
    // right after boot). The first read is due one period late on purpose: the
    // first frame must be drawn *before* any bus traffic, so a bus problem can
    // never leave the display uninitialized.
    let mut next_read = Instant::now() + METER_READ_PERIOD;
    let mut orientation_updates = ORIENTATION.anon_receiver();
    let mut current_rotation = Orientation::Normal;
    let mut last_secs = u32::MAX;
    let mut last_pulse: u16 = u16::MAX;
    let mut ports = [0u16; 6];

    loop {
        let now = Instant::now();
        #[cfg(feature = "cable-log")]
        {
            stall::BEAT.fetch_add(1, Ordering::Relaxed);
            stall::ARMED.store(true, Ordering::Relaxed);
        }

        // The input task owns debounce/navigation, independently of this frame.
        if let Some(update) = input_updates.try_changed() {
            buttons = update;
            dirty = true;
        }
        let page = ((buttons >> 4) & 7) as u8;

        // `tilt_task` owns the orientation (it flipped and reconfigured the
        // sensor); this task only follows it for flushing and redraws.
        if let Some(rotation_now) = orientation_updates.try_changed()
            && rotation_now != current_rotation
        {
            // Only the display task changes MADCTL, before its matching RASET.
            unsafe {
                panel::set_orientation(rotation_now).await;
            }
            current_rotation = rotation_now;
            dirty = true;
        }

        let mut changed = false;
        if (page == 0 || page == 3) && now >= next_read {
            next_read = now + METER_READ_PERIOD;
            if let Some(v) = read_meter(ina226::BUS_VOLTAGE).await {
                if v != last_raw_v {
                    last_raw_v = v;
                    changed = true;
                }
                if page == 3 {
                    changed = true;
                }
            }
            if page == 0
                && let Some(i) = read_meter(ina226::CURRENT).await
                && i != last_raw_i
            {
                last_raw_i = i;
                changed = true;
            }
        }

        let secs = now.as_secs() as u32;
        let pulses = PULSES.load(Ordering::Relaxed) & 1 != 0;
        // A latched bus turns the heartbeat square red on the meter page.
        let pulse_now = if bus_failed() {
            FAULT
        } else if pulses {
            ALIVE_A
        } else {
            ALIVE_B
        };
        if secs != last_secs {
            last_secs = secs;
            dirty = true;
        }
        if pulse_now != last_pulse {
            last_pulse = pulse_now;
            dirty = true;
        }

        if dirty || changed {
            dirty = false;
            if page == 0 {
                unsafe {
                    render_measure(fb, last_raw_v, last_raw_i, secs, pulse_now);
                }
            } else if page == 2 {
                let (stack_peak, stack_size) = stack_use();
                unsafe {
                    render_irq_test(
                        fb,
                        Counters {
                            step: I2C_FAULT_STEP.load(Ordering::Relaxed),
                            sr: I2C_LAST_SR.load(Ordering::Relaxed),
                            cr1: I2C_LAST_CR1.load(Ordering::Relaxed),
                            cr4: I2C_LAST_CR4.load(Ordering::Relaxed),
                            stack_peak,
                            stack_size,
                            reg_failures: IRQ_REG_FAILURES.load(Ordering::Relaxed),
                            errors: I2C_ERRORS.load(Ordering::Relaxed),
                            reads: ASYNC_READS.load(Ordering::Relaxed),
                        },
                        bus_failed(),
                    );
                }
            } else if page == 3 || page == 4 {
                let detail = (page == 4).then_some(buttons >> 8);
                unsafe {
                    render_pd(fb, last_raw_v, detail);
                }
            } else if page == 5 {
                unsafe {
                    render_partner(fb, buttons >> 8);
                }
            } else {
                for (i, p) in ports.iter_mut().enumerate() {
                    let port = [Port::A, Port::B, Port::C, Port::D, Port::E, Port::H];
                    *p = gpio::read_port(port[i]);
                }
                unsafe {
                    render_ports(fb, &ports);
                }
            }
            let frame_start = Instant::now();
            unsafe {
                panel::flush(fb, || {}, current_rotation).await;
            }
            FRAME_MS.store(frame_start.elapsed().as_millis() as u32, Ordering::Relaxed);
        }

        Timer::after_millis(METER_TICK_MS).await;
    }
}

/// PB14 and PB15 pin functions: Func48 I2C3_SDA and Func49 I2C3_SCL in
/// Func_Grp2 (HC32 Datasheet Tables 2-1 and 2-2).
const I2C3_SDA: u8 = 48;
const I2C3_SCL: u8 = 49;

/// SRAM wait control, wait protection and check protection registers (RM 8.2).
const SRAM_WTCR: *mut u16 = 0x4005_0800 as *mut u16;
const SRAM_WTPR: *mut u16 = 0x4005_0804 as *mut u16;
const SRAM_CKPR: *mut u16 = 0x4005_080c as *mut u16;
/// WTPR/CKPR write key 0x3b in bits 7:1 (RM 8.2.2, 8.2.4).
const SRAM_KEY: u16 = 0x3b << 1;
/// Key plus WTPRC/CKPRC (bit 0) set: WTCR/CKCR writes enabled.
const SRAM_UNLOCK: u16 = SRAM_KEY | 1;
/// Key with WTPRC/CKPRC clear: WTCR/CKCR writes disabled.
const SRAM_LOCK: u16 = SRAM_KEY;
/// WTCR SRAM3_RWT and SRAM3_WWT = 001b: one wait (2-cycle) read and write
/// (RM 8.2.1); SRAM1/2/3 need a wait above 100 MHz (RM Table 8-1).
const SRAM3_ONE_WAIT: u16 = 1 << 12 | 1 << 8;
/// CPACR CP10 and CP11 = 11b: full FPU access.
const FPU_FULL_ACCESS: u32 = 0xf << 20;
/// NVIC ICER masks covering the 144 external IRQs of the PAC vector table:
/// four full banks, then the low 16 bits of the fifth.
const IRQ_CLEAR_MASKS: [u32; 5] = [!0, !0, !0, !0, 0xffff];

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    unsafe {
        // SRAM wait states for high frequency: unlock WTCR and CKCR, set the
        // SRAM3 waits, then lock both again.
        SRAM_WTPR.write_volatile(SRAM_UNLOCK);
        SRAM_CKPR.write_volatile(SRAM_UNLOCK);
        SRAM_WTCR.write_volatile(SRAM3_ONE_WAIT);
        SRAM_WTPR.write_volatile(SRAM_LOCK);
        SRAM_CKPR.write_volatile(SRAM_LOCK);

        // Enable CP10/CP11 for the FPU (matches SystemInit).
        let scb = &*cortex_m::peripheral::SCB::PTR;
        scb.cpacr.modify(|cpacr| cpacr | FPU_FULL_ACCESS);

        // Disable every external IRQ; this image uses none of them.
        let nvic = &*cortex_m::peripheral::NVIC::PTR;
        for (icer, &mask) in nvic.icer.iter().zip(&IRQ_CLEAR_MASKS) {
            icer.write(mask);
        }
        scb.vtor.write(firmware_core::image::APP_BASE);
        core::arch::asm!("dsb", "isb", options(nostack, preserves_flags));
    }

    // SRAM2 holds only the frame buffer; write all of it before any read
    // (SRAM parity, RM section 8: uninitialized words must not be read).
    unsafe {
        core::ptr::write_bytes(framebuffer(), 0, firmware_core::panel::PIXELS);
    }
    #[cfg(any(feature = "pd-boot-diagnostic", feature = "cable-log"))]
    flash_log::start();
    // A clock error leaves startup fail-stopped; dependent drivers stay inactive.
    // SAFETY: SRAM timing is configured; startup owns clocks with IRQs/drivers inactive.
    let Ok(clocks) = (unsafe { clock_setup::init() }) else {
        loop {
            cortex_m::asm::wfi();
        }
    };
    unsafe {
        embassy_hc32::init(clocks);
    }
    unsafe {
        cortex_m::interrupt::enable();
    }

    let button_inputs;
    let tilt_input;
    let pd_input;
    let pc13_input;
    unsafe {
        // Release JTAG only; retain SWD. Move each input owner into its task.
        gpio::release_jtag();
        button_inputs = [
            InputPin::new_pull_up(Port::B, 2),
            InputPin::new_pull_up(Port::B, 10),
            InputPin::new_pull_up(Port::H, 0),
            InputPin::new_pull_up(Port::C, 14),
        ];
        tilt_input = InputPin::new_pull_up(Port::B, 13);
        // PC13 input + pull-up, no INTE, as the original firmware configures it.
        // Read-only slider sense (the original shows "Switch To Sink Mode" while
        // it reads high; observed low with the slider ON).
        pc13_input = InputPin::new_pull_up(Port::C, 13);

        // The INTC is not reset by a jump into this image, so a bootloader or an
        // earlier image can leave `SEL` entries mapped; `register` then refuses
        // those lines. Start from a clean slate: this application owns the INTC
        // and installs every handler it uses.
        embassy_hc32::intc::clear_routes();
        // PB12, not nearby PC13, supplies EIRQ12 (original firmware configuration).
        pd_input = embassy_hc32::exti::ExtiInput::from_input(
            InputPin::new_pull_up(Port::B, 12),
            embassy_hc32::intc::EirqTrigger::Falling,
        );
        if embassy_hc32::intc::register(u16::from(PD_EIRQ), PD_LINE, PD_PRIORITY).is_err() {
            IRQ_REG_FAILURES.fetch_add(1, Ordering::Relaxed);
        }

        // I2C3 first (PB14/PB15, FCG1), as in the original startup order. The
        // async master publishes itself; every later use goes through that lock.
        let pins = [
            AlternatePin::new(Port::B, 15, I2C3_SCL, AlternateMode::FunctionOnly),
            AlternatePin::new(Port::B, 14, I2C3_SDA, AlternateMode::FunctionOnly),
        ];
        if let Ok(bus) = embassy_hc32::i2c::I2c3::new(clocks.pclk[3], I2C3_HZ, pins) {
            *I2C3.try_lock().expect("startup owns I2C3") = Some(bus);
        } else {
            // Keep running: the meter screen stays alive without measurements,
            // but the latch makes the missing bus visible (red heartbeat square).
            BUS_LATCH.fail();
        }

        // INTC validation (vendor-documented sequence, RM 10.4.2 + Errata
        // 2.4.1/2.4.2):
        //   1. Errata 2.4.1: a pin position may only have one port's INTE set,
        //      and the lowest port number wins, so clear channel 13 elsewhere.
        //   2. Enable the peripheral side (PCR.INTE + EIRQCR) first.
        //   3. Then claim the line, clear pending and enable NVIC (register()).
        // The timer arguments are constants, so this only fails on an edit
        // that breaks them; stop like a failed clock or panel start.
        let Ok(backlight) = init_backlight(BACKLIGHT_COMPARE) else {
            loop {
                core::hint::spin_loop();
            }
        };

        // I2C3 wake sources: group 13's window is lines 110..=115 (RM Table 10-2),
        // so each event gets its own line and handler. They must be live before
        // the first awaited transfer below.
        const I2C3_EVENTS: [(u16, embassy_hc32::intc::Line); 4] = [
            (428, line(110)),
            (429, line(111)),
            (430, line(112)),
            (431, line(113)),
        ];
        for (event, line) in I2C3_EVENTS {
            if embassy_hc32::intc::register(event, line, BUS_PRIORITY).is_err() {
                // Never ignore this: an unrouted line means a wake that never
                // arrives, which is a hang, not an error code.
                IRQ_REG_FAILURES.fetch_add(1, Ordering::Relaxed);
            }
        }

        // Wake-path self-test, now that the I2C3 lines are routed: arming the
        // idle "transmit buffer empty" source must make the handler run. `I2C IRQ`
        // reading at least 1 on the test page proves the SEL/NVIC routing works,
        // independently of whether any transaction succeeds.
        {
            let mut guard = I2C3.lock().await;
            if let Some(bus) = guard.as_mut() {
                bus.wake_self_test();
            }
        }

        // Route a software interrupt to INT008.
        if embassy_hc32::intc::register(u16::from(SWI_CHANNEL), SWI_LINE, PD_PRIORITY).is_err() {
            IRQ_REG_FAILURES.fetch_add(1, Ordering::Relaxed);
        }
        // One software interrupt at startup: if SWI ch8 reads 1 on the test
        // page, `SEL`/`IER`/NVIC dispatch and the vector table all work.
        embassy_hc32::intc::software_interrupt_trigger(SWI_CHANNEL);

        // Configure the INA226 and the LIS3DH over the async master.
        for (reg, value) in ina226::INIT {
            if !write_meter(reg, value).await {
                break;
            }
        }
        for (reg, value) in lis3dh::INIT {
            if !write_sensor(reg, value).await {
                break;
            }
        }

        if !panel::init(backlight.1).await {
            // Backlight is already on, so a lit blank screen means the panel
            // sequence failed; a dark screen means we failed before this point.
            loop {
                core::hint::spin_loop();
            }
        }
    }

    // Each task is spawned once, here, into its default one-slot pool, so a
    // token is always free (`SpawnError::Busy` needs a second instance).
    const ONE_SPAWN: &str = "each task is spawned once into its one-slot pool";
    spawner.spawn(pulse_task().expect(ONE_SPAWN));
    spawner.spawn(tilt_task(tilt_input).expect(ONE_SPAWN));
    spawner.spawn(button_task(button_inputs).expect(ONE_SPAWN));
    spawner.spawn(meter_task().expect(ONE_SPAWN));
    spawner.spawn(pd_task::run(pd_input, pc13_input).expect(ONE_SPAWN));
}
