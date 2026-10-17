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
mod energy_store;
mod flash;
#[cfg(feature = "flash-probe")]
mod flash_probe;
mod ina226;
mod line_adc;
mod lis3dh;
#[path = "panel.rs"]
mod panel;
mod pd;
mod pd_cc;
mod pd_spr_phy;
mod pd_task;
mod pd_trial;
mod protocol;
mod qc_drive;
mod scp_drive;

mod pd_service;
mod pd_sniff;
mod render;
mod ripple;
mod settings_store;
mod storage_diag;
mod ui;
mod upper;

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_hc32::gpio::{self, AlternateMode, AlternatePin, InputPin, Port};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
// The `#[cortex_m_rt::interrupt]` expansion checks `interrupt::INT0xx`, and the
// PAC exposes the generated `Interrupt` enum through that path.
use bus::{
    ASYNC_READS, BUS_LATCH, I2C_ERRORS, I2C_FAULT_STEP, I2C_LAST_CR1, I2C_LAST_CR4, I2C_LAST_SR,
    I2C3, bus_failed, read_meter, write_meter, write_sensor,
};
use button_logic::{Detail, MAX_TABS, Page, QcTarget, Tab, View};
use embassy_time::{Instant, Timer};
use embedded_graphics::pixelcolor::{Rgb565, RgbColor};
use firmware_core::buttons as button_logic;
use firmware_core::calibration::{Average, Calibration, Mean};
use firmware_core::energy::{Totals, Tracker};
use firmware_core::power_history::{self, History};
use firmware_core::settings::{NextBoot, QcFindings, Rotation, ScpFindings, Screen};
use hc32f460_pac::interrupt;
use panel::Orientation;
#[cfg(not(any(feature = "pd-boot-diagnostic", feature = "cable-log")))]
use panic_halt as _;
use render::{Canvas, Counters, Frame};

/// Return addresses the walk accepts: the application image after its initial
/// stack pointer word, up to the end of the flash tool's application erase
/// envelope.
#[cfg(any(feature = "pd-boot-diagnostic", feature = "cable-log"))]
const RETURN_RANGE: core::ops::Range<u32> = firmware_core::APP_BASE + 4..0x44000;
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

/// Heartbeat dot colours, alternating with each pulse.
const ALIVE_A: Rgb565 = Rgb565::WHITE;
const ALIVE_B: Rgb565 = Rgb565::new(8, 16, 8);
/// Heartbeat colour while the I2C3 latch is tripped.
const FAULT: Rgb565 = Rgb565::RED;

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
/// `button_task` sends the input state when it changes; the renderer and PD
/// trial read it.
static BUTTON_STATE: Watch<CriticalSectionRawMutex, button_logic::Input, 1> = Watch::new();
/// Last settings-save result, published by the input task for its status line.
static SETTINGS_SAVE: Watch<CriticalSectionRawMutex, settings_store::Status, 1> = Watch::new();
/// PD requests from `button_task` (profile previews, confirmations, the cable
/// query), applied by the PD session in order.
static PD_COMMANDS: Channel<CriticalSectionRawMutex, button_logic::PdCommand, 8> = Channel::new();
/// OK was held on the Cable page since boot, so a refused query can say why.
static CABLE_ASKED: AtomicBool = AtomicBool::new(false);
/// How the renderer laid out the shown detail list: `button_task` limits
/// scrolling to it and the PD trial reads the cable section in view.
static DETAIL_VIEW: Watch<CriticalSectionRawMutex, (Detail, ui::ListView), 1> = Watch::new();
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
/// The longest panel refresh in the last full [`PEAK_WINDOW`], in nominal
/// timer milliseconds, not calibrated wall-clock measurements.
static FRAME_MS: AtomicU32 = AtomicU32::new(0);
/// The longest gap between two button polls in the last full [`PEAK_WINDOW`].
static BUTTON_GAP_MS: AtomicU32 = AtomicU32::new(0);
/// How long the System page's FRAME and BUTTON GAP peaks cover. A since-boot
/// peak kept a startup or settings-save stall on screen for the whole run,
/// and the last value alone hid a full redraw behind the next small one; two
/// seconds holds a short peak long enough to read. Chosen, not specified.
const PEAK_WINDOW: embassy_time::Duration = embassy_time::Duration::from_secs(2);

/// The largest sample of each [`PEAK_WINDOW`], published once it ends.
struct WindowPeak {
    published: &'static AtomicU32,
    end: Instant,
    peak: u32,
}

impl WindowPeak {
    fn new(published: &'static AtomicU32) -> Self {
        Self {
            published,
            end: Instant::now() + PEAK_WINDOW,
            peak: 0,
        }
    }

    /// Count `sample`, taken at `now`; a sample past the window's end closes it.
    fn add(&mut self, now: Instant, sample: u32) {
        self.peak = self.peak.max(sample);
        if now >= self.end {
            self.published.store(self.peak, Ordering::Relaxed);
            self.peak = 0;
            self.end = now + PEAK_WINDOW;
        }
    }
}
static PD_IRQ_COUNT: AtomicU32 = AtomicU32::new(0);
/// Latest PD observation: the PD task sends it, the renderer reads it.
static PD_OBSERVATION: Watch<CriticalSectionRawMutex, pd::Observation, 1> =
    Watch::new_with(pd::Observation::new());

/// Send the PD observation to the renderer without a flash log entry.
fn publish_observation(observation: pd::Observation) {
    #[cfg(any(feature = "pd-boot-diagnostic", feature = "cable-log"))]
    flash_log::note_progress(&observation);
    PD_OBSERVATION.sender().send(observation);
}

/// The last published PD observation.
fn latest_observation() -> pd::Observation {
    PD_OBSERVATION
        .try_get()
        .unwrap_or_else(pd::Observation::new)
}

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
    publish_observation(observation);
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

/// Screen orientation: `tilt_task` sends it, `meter_task` applies it.
static ORIENTATION: Watch<CriticalSectionRawMutex, Orientation, 1> = Watch::new();
/// Line voltages: `line_task` sends each scan (`None` when one fails) while
/// the lines page is shown; `meter_task` draws them.
static LINE_VOLTAGES: Watch<CriticalSectionRawMutex, Option<line_adc::LineVoltages>, 1> =
    Watch::new();
/// ADC2 reference count; cleared when scans are idle or fail, never replaced
/// with a nominal supply voltage for the ripple estimate.
static ADC_REFERENCE: Watch<CriticalSectionRawMutex, Option<u16>, 1> = Watch::new();
/// Quick Charge trigger state: `line_task` sends it, `meter_task` draws it.
static QC_STATUS: Watch<CriticalSectionRawMutex, qc_drive::Status, 1> = Watch::new();
// Terminal SCP evidence must survive a fast transition back to an idle driver.
static SCP_RESULT: Watch<CriticalSectionRawMutex, (u8, scp_drive::Status), 1> = Watch::new();
/// The meter's averaged raw reading: `meter_task` sends it, the button task
/// hands it to the calibration steps.
static METER_MEAN: Watch<CriticalSectionRawMutex, Option<Mean>, 1> = Watch::new();
/// Ripple captures: acquisition task publishes, display task consumes.
static RIPPLE: Watch<CriticalSectionRawMutex, Option<firmware_core::ripple::Reading>, 1> =
    Watch::new();

/// Failed `intc::register` calls. The INTC is not reset by a jump into this
/// image, so a `SEL` entry left mapped by the bootloader makes routing fail;
/// this must stay 0 now that startup clears the table first.
static IRQ_REG_FAILURES: AtomicU32 = AtomicU32::new(0);

/// External interrupt channels: PB12 carries the PD controller's alert, PB13
/// the tilt sensor's INT1. Channel n is also interrupt event n (RM Table 10-2).
const PD_EIRQ: u8 = 12;
const TILT_EIRQ: u8 = 13;
/// NVIC lines; each must match its `INTnnn` handler below.
const PD_LINE: embassy_hc32::intc::Line = line(4);
const TILT_LINE: embassy_hc32::intc::Line = line(13);
/// ADC2 end of scan (event 452): group 14's window is lines 116..=121
/// (RM Table 10-2).
const ADC2_EOCA: u16 = 452;
const ADC_LINE: embassy_hc32::intc::Line = line(116);
/// The panel's DMA completion and error events (DMA2 TC0 = 36 and
/// DMA2_ERR = 49, group 1, whose window is lines 38..=43; RM Table 10-2).
const PANEL_DMA_EVENTS: [(u16, embassy_hc32::intc::Line); 2] = [
    (
        panel::DMA_UNIT.complete_event(panel::DMA_CHANNEL).0,
        line(38),
    ),
    (panel::DMA_UNIT.error_event().0, line(39)),
];
/// DMA1 TC0 = 32 and ERR = 48 share group 1's lines 38..=43 with the
/// panel; use separate lines so each handler touches only its own unit.
const RIPPLE_DMA_EVENTS: [(u16, embassy_hc32::intc::Line); 2] = [
    (
        ripple::DMA_UNIT.complete_event(ripple::DMA_CHANNEL).0,
        line(40),
    ),
    (ripple::DMA_UNIT.error_event().0, line(41)),
];
/// NVIC priorities (lower is more urgent). Both stay below SysTick (0), so
/// timing never starves; bus and tilt wakes rank ahead of the PD alert.
const BUS_PRIORITY: u8 = 8;
const PD_PRIORITY: u8 = 9;
/// The PD session's executor: above thread mode, so a frame draw (measured
/// at up to 42 ms, uninterrupted) cannot hold back a reply past the source's
/// SenderResponseTimer (at least 27 ms, USB PD Table 7.9), which a source
/// answers with a Hard Reset; below the bus and PD alert wakes it waits on.
const PD_EXECUTOR_PRIORITY: u8 = 10;
/// NVIC line pended only by `PD_EXECUTOR`: startup clears every `SEL`, and
/// nothing routes an event to it.
const PD_EXECUTOR_LINE: interrupt = interrupt::INT031;
static PD_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

/// PB12 / EIRQ12: acknowledge/latch/wake only. No I2C or PHY service in ISR.
#[cortex_m_rt::interrupt]
fn INT004() {
    if embassy_hc32::exti::on_interrupt(PD_EIRQ) {
        PD_IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

/// PB13 external interrupt: the async driver clears the flag and wakes
/// `tilt_task`.
#[cortex_m_rt::interrupt]
fn INT013() {
    embassy_hc32::exti::on_interrupt(TILT_EIRQ);
}

/// Runs the PD session's executor; see [`PD_EXECUTOR_PRIORITY`].
#[cortex_m_rt::interrupt]
fn INT031() {
    // SAFETY: the handler of `PD_EXECUTOR_LINE`, unmasked only by `start`.
    unsafe { PD_EXECUTOR.on_interrupt() }
}

/// I2C3 events (RXI 428, TXI 429, TEI 430, EEI 431) on lines 110..=113: the
/// async driver only needs a wakeup, then it re-reads `SR` itself.
#[cortex_m_rt::interrupt]
fn INT110() {
    embassy_hc32::i2c::on_interrupt();
}

#[cortex_m_rt::interrupt]
fn INT111() {
    embassy_hc32::i2c::on_interrupt();
}

#[cortex_m_rt::interrupt]
fn INT112() {
    embassy_hc32::i2c::on_interrupt();
}

#[cortex_m_rt::interrupt]
fn INT113() {
    embassy_hc32::i2c::on_interrupt();
}

/// ADC2 end of scan, on `ADC_LINE`.
#[cortex_m_rt::interrupt]
fn INT116() {
    embassy_hc32::adc::on_interrupt();
}

/// The panel DMA's completion and error events, on `PANEL_DMA_EVENTS`' lines.
#[cortex_m_rt::interrupt]
fn INT038() {
    embassy_hc32::dma::on_interrupt(panel::DMA_UNIT);
}
#[cortex_m_rt::interrupt]
fn INT039() {
    embassy_hc32::dma::on_interrupt(panel::DMA_UNIT);
}

/// Ripple DMA completion and error events.
#[cortex_m_rt::interrupt]
fn INT040() {
    embassy_hc32::dma::on_interrupt(ripple::DMA_UNIT);
}
#[cortex_m_rt::interrupt]
fn INT041() {
    embassy_hc32::dma::on_interrupt(ripple::DMA_UNIT);
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

/// Reset cause flags other than power-on, the reset pin and "multiple"
/// (bits 2 to 13, `render::RESET_FLAGS`). Nothing clears the flags, so any
/// of these means the MCU reset without losing power since it powered up.
const RESET_WITHOUT_POWER_LOSS: u16 = 0x3ffc;

/// The reset cause flags, read only: nothing here clears them.
fn reset_flags() -> u16 {
    // SAFETY: a read of a status register that no other code accesses.
    unsafe { hc32f460_pac::Rmu::steal() }.rstf0().read().bits()
}

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
    //
    // The sensor raises INT1 on the first sample past the threshold, and a
    // button press can jolt the device that far for a moment. So a flip also
    // needs INT1 to stay high for `SETTLE_MS`: a turn holds the new position,
    // a jolt does not.
    const REFRACTORY_MS: u64 = 1500;
    let orientation_sender = ORIENTATION.sender();
    let mut orientation = Orientation::Normal;
    loop {
        if !tilt.is_high() {
            tilt.wait_for_edge().await;
        }
        if !held_high(&tilt).await {
            continue;
        }

        // Re-arm the sensor before showing the flip: if the write fails, the
        // sensor still watches the old position, the screen stays consistent
        // with it, and the level check retries after the lockout.
        let next = orientation.flipped();
        if write_sensor(lis3dh::INT1_CFG, armed_position(next)).await {
            orientation = next;
            orientation_sender.send(orientation);
        }

        // Suppress the rest of the burst, then discard anything it latched.
        Timer::after_millis(REFRACTORY_MS).await;
        while embassy_hc32::exti::take_edge(TILT_EIRQ) {}
    }
}

/// How long INT1 must stay high before the screen flips, in ms. Chosen, not
/// measured: well past a button press's jolt, short enough that a turn
/// still feels immediate.
const SETTLE_MS: u64 = 300;
/// Pace of the INT1 level checks while it settles, in ms; chosen.
const SETTLE_CHECK_MS: u64 = 20;

/// Whether INT1 stays high for [`SETTLE_MS`], checked every
/// [`SETTLE_CHECK_MS`].
async fn held_high(tilt: &embassy_hc32::exti::ExtiInput) -> bool {
    let settled = Instant::now() + embassy_time::Duration::from_millis(SETTLE_MS);
    while tilt.is_high() {
        if Instant::now() >= settled {
            return true;
        }
        Timer::after_millis(SETTLE_CHECK_MS).await;
    }
    false
}

/// Scan the line voltages and reference at the meter's read pace while the
/// overview, lines, ripple or a Quick Charge tab is shown or the trigger runs, ADC2 idle
/// otherwise, and step the Quick Charge trigger toward the request in force,
/// whichever tab is shown. Without one it releases D+/D−.
#[embassy_executor::task]
async fn line_task(mut adc: line_adc::LineAdc, mut trigger: qc_drive::Trigger) {
    let lines_sender = LINE_VOLTAGES.sender();
    let reference_sender = ADC_REFERENCE.sender();
    let status_sender = QC_STATUS.sender();
    loop {
        let input = BUTTON_STATE.try_get();
        let request = input.and_then(button_logic::Input::qc_request);
        // The trigger watches D− during its handshake. Ripple also needs a
        // current ADC2 reference, but it never takes ownership of that ADC.
        let scan = if request.is_some()
            || input.is_some_and(|input| {
                let view = input.page.view();
                view.shows_lines() || view == View::Ripple
            }) {
            adc.read().await
        } else {
            None
        };
        let lines = scan.map(|(lines, _)| lines);
        lines_sender.send(lines);
        reference_sender.send(scan.map(|(_, reference)| reference));
        // The trigger judges requests by the output, read here rather than
        // at the meter page's frame pace.
        let vbus_mv = match request {
            Some(_) => read_meter(ina226::BUS_VOLTAGE)
                .await
                .filter(|_| !bus_failed())
                .map(|raw| u16::try_from(ina226::bus_millivolts(raw)).unwrap_or(u16::MAX)),
            None => None,
        };
        let now_ms = Instant::now().as_millis();
        let status = trigger.update(now_ms, request, lines, vbus_mv, &mut adc);
        if let qc_drive::Status::Scp(
            attempt,
            terminal @ (scp_drive::Status::Detected(_) | scp_drive::Status::Failed(_)),
        ) = status
        {
            SCP_RESULT.sender().send((attempt, terminal));
        }
        status_sender.send(status);
        Timer::after(METER_READ_PERIOD).await;
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
async fn button_task(
    inputs: [InputPin; 4],
    mut store: settings_store::Store,
    saved: firmware_core::settings::Settings,
    // What the previous boot asked this one to be, and the screen it opens on.
    requested: NextBoot,
    resume: Option<Screen>,
) {
    // `pd-boot-diagnostic`: open on the PD page so the state before a
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
        button_logic::Navigation::new(Page::Groups(Tab::detail(Detail::Pd)), saved)
    } else if let Some(screen) = resume {
        button_logic::Navigation::new(Page::resume(screen, requested.silent()), saved)
    } else {
        button_logic::Navigation::new(Page::default(), saved)
    };
    let input_sender = BUTTON_STATE.sender();
    let mut qc_updates = QC_STATUS.anon_receiver();
    let mut orientation_updates = ORIENTATION.anon_receiver();
    let mut sensed_rotation = Orientation::Normal;
    let mut rotation = saved.rotation;
    let mut previous = Instant::now();
    let mut gap = WindowPeak::new(&BUTTON_GAP_MS);
    loop {
        let now = Instant::now();
        gap.add(now, now.duration_since(previous).as_millis() as u32);
        previous = now;
        if let Some(tilted) = orientation_updates.try_changed() {
            sensed_rotation = tilted;
        }
        let mut raw = inputs.iter().enumerate().fold(0u8, |mask, (index, pin)| {
            mask | (u8::from(pin.is_low()) << index)
        });
        // The arrows follow the screen: turned over, the viewer's left button
        // is the Right one.
        if screen_orientation(rotation, sensed_rotation) == Orientation::Inverted {
            raw = button_logic::swap_arrows(raw);
        }
        // Selecting needs a sink session that has received the charger's offer.
        let observation = latest_observation();
        navigation.set_selectable(observation.spr.is_some() && observation.capabilities.is_some());
        navigation.set_passthrough(observation.pc13_high == Some(true));
        navigation.set_cable_test(observation.listen.is_some(), observation.cable_test);
        navigation.set_silent(observation.quick_charge, observation.silent_settled);
        if let Some((_, view)) = DETAIL_VIEW.try_get() {
            navigation.set_list_end(view.end);
        }
        navigation.set_mean(METER_MEAN.try_get().flatten());
        // The scan's Quick Charge part ends with the probe's result.
        let scan_end = match qc_updates.try_get() {
            Some(qc_drive::Status::Detected(report)) => {
                navigation.finish_probe(report);
                navigation.finish_scan(QcFindings::Detected(report))
            }
            Some(qc_drive::Status::Failed(failure)) => {
                navigation.fail_probe();
                navigation.finish_scan(QcFindings::Failed(failure))
            }
            Some(qc_drive::Status::Scp(attempt, scp_drive::Status::Detected(_))) => {
                navigation.finish_scp(attempt, true)
            }
            Some(qc_drive::Status::Scp(attempt, scp_drive::Status::Failed(_))) => {
                navigation.finish_scp(attempt, false)
            }
            _ => None,
        };
        let (input, command) = navigation.update(raw);
        let command = scan_end.or(command);
        rotation = input.settings.rotation;
        // Save once a value is no longer being changed, and before a restart
        // (a scrolling list counts as editing) so the next boot sees it.
        if !input.page.editing() || command == Some(firmware_core::buttons::PdCommand::Restart) {
            store.save(input.settings);
            SETTINGS_SAVE.sender().send_if_modified(|current| {
                let status = store.status();
                let changed = *current != Some(status);
                *current = Some(status);
                changed
            });
        }
        // The PD session drains these between transactions; a full queue
        // drops the press, as a missed tap would.
        if let Some(command) = command {
            if command == firmware_core::buttons::PdCommand::QueryCable {
                CABLE_ASKED.store(true, Ordering::Relaxed);
            }
            PD_COMMANDS.try_send(command).ok();
        }
        input_sender.send_if_modified(|current| {
            let changed = *current != Some(input);
            *current = Some(input);
            changed
        });
        Timer::after_millis(u64::from(button_logic::POLL_MS)).await;
    }
}

/// The screen orientation the `rotation` setting picks, given the sensed one.
fn screen_orientation(rotation: Rotation, sensed: Orientation) -> Orientation {
    match rotation {
        Rotation::Auto => sensed,
        Rotation::Normal => Orientation::Normal,
        Rotation::Flipped => Orientation::Inverted,
    }
}

/// The header for `page`: the shown group's tabs, written into `tabs`, and
/// where Left/Right move.
fn header(page: Page, passthrough: bool, tabs: &mut [Tab; MAX_TABS]) -> ui::Header<'_> {
    let Some(shown) = page.tab() else {
        return ui::Header::Settings;
    };
    let group = shown.group();
    let mut count = 0;
    let mut current = 0;
    for (slot, tab) in tabs.iter_mut().zip(group.available(passthrough)) {
        *slot = tab;
        if tab.same(shown) {
            current = count;
        }
        count += 1;
    }
    ui::Header::Tabs {
        tabs: &tabs[..count],
        current,
        group: group.position(passthrough),
        level: page.level(),
        passthrough,
    }
}

/// The latest observation as the detail pages show it, with the live bus
/// voltage and, in the logging builds, the previous boot's record.
fn detail_observation(raw_vbus: Option<u16>, calibration: Calibration) -> pd::Observation {
    let mut observation = latest_observation();
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
        spr.vbus_live_mv = raw_vbus
            .filter(|_| !bus_failed())
            .map(|raw| calibration.millivolts(raw).min(u16::MAX as u32) as u16);
    }
    observation
}

/// Meter page values from the latest raw INA226 registers, calibrated;
/// `None` until both have been read and while the bus is latched.
fn reading(
    raw_v: Option<u16>,
    raw_i: Option<u16>,
    calibration: Calibration,
) -> Option<ui::Reading> {
    let (raw_v, raw_i) = (raw_v?, raw_i?);
    if bus_failed() {
        return None;
    }
    Some(ui::Reading {
        mv: calibration.millivolts(raw_v),
        ma: calibration.milliamps(raw_i),
    })
}

/// How the PD side of the port stands for Quick Charge.
fn qc_mode(observation: pd::Observation) -> ui::QcMode {
    if observation.quick_charge {
        ui::QcMode::Silent
    } else if observation.spr.is_some() && observation.capabilities.is_some() {
        ui::QcMode::Pd
    } else {
        ui::QcMode::Plain
    }
}

/// Backlight compare for a settings level, inside `1..BACKLIGHT_PERIOD`.
fn backlight_compare(settings: firmware_core::settings::Settings) -> u16 {
    (u32::from(BACKLIGHT_PERIOD) * u32::from(settings.brightness_percent()) / 100) as u16
}

/// How long the missing-upper notice sleeps between wakes; it never changes.
const UPPER_MISSING_IDLE_S: u64 = 3600;

#[embassy_executor::task]
async fn meter_task(
    mut backlight: embassy_hc32::pwm::TimerA4Pwm,
    // Legacy results carried into the full scan's PD/cable part.
    findings: (Option<QcFindings>, Option<ScpFindings>),
    upper_installed: bool,
    saved: firmware_core::settings::Settings,
    mut energy_store: Option<energy_store::Store>,
    mut energy_save_status: energy_store::Status,
    // ADC1's clock in Hz for the nominal ripple timebase; `None` without a capture.
    ripple_clock_hz: Option<u32>,
) {
    let (mut qc_findings, carried_scp) = findings;
    let fb = framebuffer();
    let mut input_updates = BUTTON_STATE.anon_receiver();
    let mut input = button_logic::Input {
        settings: saved,
        ..Default::default()
    };
    let mut energy_memory = saved.energy_memory;
    let totals = if energy_memory.seconds().is_some() {
        energy_store
            .as_ref()
            .map_or_else(Totals::default, energy_store::Store::totals)
    } else {
        Totals::default()
    };
    let mut energy = Tracker::new(totals);
    let mut energy_reset = input.energy_reset;
    let mut next_save = energy_memory
        .seconds()
        .map(|seconds| Instant::now() + embassy_time::Duration::from_secs(seconds));
    let mut dirty = true;
    // Last raw readings, `None` until the first read; a changed reading
    // redraws the meter page.
    let mut last_raw_v: Option<u16> = None;
    let mut last_raw_i: Option<u16> = None;
    let mut line_updates = LINE_VOLTAGES.anon_receiver();
    let mut lines = None;
    let mut passive_lines = firmware_core::dcp::Passive::default();
    let mut driven = None;
    let mut ripple_updates = RIPPLE.anon_receiver();
    let mut ripple_reading: Option<firmware_core::ripple::Reading> = None;
    let mut reference_updates = ADC_REFERENCE.anon_receiver();
    let mut ripple_reference = None;
    let mut qc_updates = QC_STATUS.anon_receiver();
    let mut qc_status = qc_drive::Status::Off;
    let mut scp_updates = SCP_RESULT.anon_receiver();
    let mut scp_status = None;
    // Peaks follow the meter page's reads; a reset hold on the overview
    // clears them.
    let mut peaks = ui::Peaks::default();
    let mut peaks_reset = input.peaks_reset;
    // The overview's power graph samples every read, whatever the page; a
    // reset hold on the graph clears it.
    let mut power_history = History::default();
    let mut graph_reset = input.graph_reset;
    let mut average = Average::default();
    let mut brightness = input.settings.brightness;
    // Reads are paced by an absolute deadline, never by subtracting from an
    // `Instant` (embassy-time panics on underflow, e.g. `Instant::now() - 200ms`
    // right after boot). The first read is due one period late on purpose: the
    // first frame must be drawn *before* any bus traffic, so a bus problem can
    // never leave the display uninitialized.
    let mut next_read = Instant::now() + METER_READ_PERIOD;
    let mut orientation_updates = ORIENTATION.anon_receiver();
    let mut current_rotation = Orientation::Normal;
    let mut sensed_rotation = Orientation::Normal;
    let mut last_secs = u32::MAX;
    let mut last_pulse = None;
    let detail_sender = DETAIL_VIEW.sender();
    let mut frame_time = WindowPeak::new(&FRAME_MS);

    // Without this build's upper region the pages would call the PD stack's
    // missing code: show only the notice.
    if !upper_installed {
        let frame = unsafe { &mut *(fb as *mut Frame) };
        render::clear(frame);
        ui::draw_upper_missing(&mut Canvas(frame));
        unsafe {
            panel::flush(fb, current_rotation).await;
        }
        loop {
            Timer::after_secs(UPPER_MISSING_IDLE_S).await;
        }
    }

    loop {
        let now = Instant::now();
        #[cfg(feature = "cable-log")]
        {
            stall::BEAT.fetch_add(1, Ordering::Relaxed);
            stall::ARMED.store(true, Ordering::Relaxed);
        }

        // The input task owns debounce/navigation, independently of this frame.
        if let Some(update) = input_updates.try_changed() {
            input = update;
            dirty = true;
        }
        let page = input.page;
        let view = page.view();
        let mut save_energy = false;
        // Previewing intervals must not write checkpoints on every arrow tap.
        if !matches!(
            view,
            View::Settings(button_logic::SettingsFocus::EnergyMemory)
        ) && input.settings.energy_memory != energy_memory
        {
            energy_memory = input.settings.energy_memory;
            next_save = energy_memory
                .seconds()
                .map(|seconds| now + embassy_time::Duration::from_secs(seconds));
            save_energy = energy_memory.seconds().is_some();
        }
        if input.energy_reset != energy_reset {
            energy_reset = input.energy_reset;
            energy.reset();
            save_energy = energy_memory.seconds().is_some();
        }
        if input.peaks_reset != peaks_reset {
            peaks_reset = input.peaks_reset;
            peaks = ui::Peaks::default();
        }
        if input.graph_reset != graph_reset {
            graph_reset = input.graph_reset;
            power_history = History::default();
        }
        if input.settings.brightness != brightness {
            brightness = input.settings.brightness;
            // The compare is inside the period by construction; on an error
            // the previous level simply stays.
            backlight
                .set_compare(backlight_compare(input.settings))
                .ok();
        }

        // `tilt_task` owns the sensed orientation (it flipped and reconfigured
        // the sensor); this task follows it unless the settings lock one.
        if let Some(tilted) = orientation_updates.try_changed() {
            sensed_rotation = tilted;
        }
        let rotation_now = screen_orientation(input.settings.rotation, sensed_rotation);
        if rotation_now != current_rotation {
            // Only the display task changes MADCTL, before its matching RASET.
            unsafe {
                panel::set_orientation(rotation_now).await;
            }
            current_rotation = rotation_now;
            dirty = true;
        }

        let mut changed = false;
        // Energy sampling is independent of the page. Keep the existing peak
        // updates and redraw decisions limited to their live-value pages.
        let reads_current = matches!(
            view,
            View::Overview | View::Fixed { .. } | View::Pps(_) | View::Avs { .. }
        ) || view.quick_charge();
        let pd_details = matches!(
            view,
            View::Detail {
                page: Detail::Pd,
                ..
            }
        );
        let calibrating = matches!(
            view,
            View::Settings(button_logic::SettingsFocus::Calibrate(_))
        );
        if now >= next_read {
            next_read = now + METER_READ_PERIOD;
            let raw_v = read_meter(ina226::BUS_VOLTAGE).await;
            let raw_i = read_meter(ina226::CURRENT).await;
            changed |= (reads_current || pd_details || calibrating || view == View::Ripple)
                && (raw_v != last_raw_v || raw_i != last_raw_i);
            changed |= pd_details;
            last_raw_v = raw_v;
            last_raw_i = raw_i;
            let sample = reading(raw_v, raw_i, input.settings.calibration);
            // The calibration steps take the raw readings' average.
            match (raw_v, raw_i) {
                (Some(raw_v), Some(raw_i)) if !bus_failed() => average.add(raw_v, raw_i),
                _ => average.clear(),
            }
            METER_MEAN.sender().send(average.mean());
            let read_ms = Instant::now().as_millis();
            energy.sample(read_ms, sample.map(|reading| (reading.mv, reading.ma)));
            power_history.sample(
                read_ms,
                sample.map(|reading| power_history::Sample {
                    mv: u16::try_from(reading.mv).unwrap_or(u16::MAX),
                    ma: u16::try_from(reading.ma.unsigned_abs()).unwrap_or(u16::MAX),
                    mw: reading.mw(),
                }),
            );
            // The graph's readings count toward the overview's peaks.
            if (reads_current || matches!(view, View::Graph(_)))
                && let Some(reading) = sample
            {
                peaks.update(reading);
            }
        }

        if energy_memory.seconds().is_some()
            && (save_energy || next_save.is_some_and(|deadline| Instant::now() >= deadline))
        {
            // No I2C transaction may be suspended by the flash erase. Saving
            // happens before starting the next panel DMA, whose last flush
            // completed in the preceding loop.
            let _bus = I2C3.lock().await;
            energy_save_status = match energy_store.as_mut() {
                None => energy_save_status,
                Some(store) => {
                    if store.save(energy.totals) {
                        energy_store::Status::Ready
                    } else {
                        energy_store::Status::SaveFailed
                    }
                }
            };
            next_save = energy_memory
                .seconds()
                .map(|seconds| Instant::now() + embassy_time::Duration::from_secs(seconds));
            dirty |= view == View::Energy;
        }

        let mut line_scanned = false;
        if let Some(update) = line_updates.try_changed() {
            lines = update;
            line_scanned = true;
            changed |= view.shows_lines();
        }
        // Freeze the reference together with the capture, so supply updates
        // cannot change held statistics. Drain in-flight updates without using
        // them; bus V/A above remain live, even while the trace is held.
        if let Some(update) = reference_updates.try_changed()
            && !input.ripple_hold
        {
            ripple_reference = update;
            changed |= view == View::Ripple;
        }
        if let Some(update) = ripple_updates.try_changed()
            && !input.ripple_hold
        {
            ripple_reading = update;
            changed |= view == View::Ripple;
        }
        if let Some(update) = scp_updates.try_changed() {
            scp_status = Some(update);
            dirty = true;
        }
        if let Some(update) = qc_updates.try_changed() {
            qc_status = update;
            qc_findings = match update {
                qc_drive::Status::Detected(report) => Some(QcFindings::Detected(report)),
                qc_drive::Status::Failed(failure) => Some(QcFindings::Failed(failure)),
                _ => qc_findings,
            };
            changed |= view.quick_charge();
        }
        if qc_status != qc_drive::Status::Off {
            passive_lines.reset();
            if driven.take().is_some() {
                dirty |= view == View::Overview;
            }
        } else if line_scanned {
            let vbus_mv = reading(last_raw_v, last_raw_i, input.settings.calibration).map(|r| r.mv);
            let next = match lines {
                Some(lines) => {
                    passive_lines.update(now.as_millis(), lines.dp_mv, lines.dm_mv, vbus_mv)
                }
                None => {
                    passive_lines.reset();
                    None
                }
            };
            if next != driven {
                driven = next;
                dirty |= view == View::Overview;
            }
        }

        let secs = now.as_secs() as u32;
        let pulses = PULSES.load(Ordering::Relaxed);
        // A latched bus turns the heartbeat dot red on the meter page.
        let pulse_now = if bus_failed() {
            FAULT
        } else if pulses & 1 != 0 {
            ALIVE_A
        } else {
            ALIVE_B
        };
        // Once a second, redraw what changes without a signal: the PD
        // observation, the System screen's counters and timings.
        if secs != last_secs {
            last_secs = secs;
            dirty = true;
        }
        if Some(pulses) != last_pulse {
            last_pulse = Some(pulses);
            // Only the meter page draws the heartbeat dot and the flow
            // shimmer; opening it is an input change, which redraws them.
            dirty |= view == View::Overview;
        }

        if dirty || changed {
            dirty = false;
            // SAFETY: this task is the sole framebuffer owner.
            let frame = unsafe { &mut *(fb as *mut Frame) };
            let mut tabs = [Tab::default(); MAX_TABS];
            let header = header(page, input.passthrough, &mut tabs);
            render::clear(frame);
            match view {
                View::Overview => {
                    let observation = latest_observation();
                    let reading = reading(last_raw_v, last_raw_i, input.settings.calibration);
                    ui::draw_meter(
                        &mut Canvas(frame),
                        &header,
                        &ui::Meter {
                            reading,
                            peaks,
                            protocol: protocol::identify(&protocol::Sources {
                                observation: &observation,
                                lines,
                                driven,
                                qc: qc_status,
                                findings: qc_findings,
                                scp: input.settings.scp_findings.or(carried_scp),
                                vbus_mv: reading.map(|reading| reading.mv),
                            }),
                            cc: pd::attached_cc(&observation),
                            flipped: current_rotation == Orientation::Inverted,
                            alive: pulse_now,
                            pulses,
                            reset_hold: input.reset_hold,
                        },
                    );
                }
                View::Graph(focus) => {
                    ui::draw_graph(
                        &mut Canvas(frame),
                        &header,
                        &power_history,
                        input.graph_trace,
                        input.graph_window,
                        focus,
                        input.reset_hold,
                    );
                }
                View::Energy => {
                    ui::draw_energy(
                        &mut Canvas(frame),
                        &header,
                        energy.totals,
                        energy_memory,
                        energy_save_status,
                        input.reset_hold,
                    );
                }
                View::Lines => {
                    ui::draw_lines(&mut Canvas(frame), &header, lines);
                }
                View::Ripple => {
                    ui::draw_ripple(
                        &mut Canvas(frame),
                        &header,
                        &ui::Ripple {
                            reading: ripple_reading.as_ref(),
                            reference: ripple_reference,
                            bus: reading(last_raw_v, last_raw_i, input.settings.calibration),
                            hold: input.ripple_hold,
                            timebase: input.ripple_timebase,
                            clock_hz: ripple_clock_hz,
                            scale: input.ripple_scale,
                            focus: match page {
                                Page::Open(button_logic::Open::Ripple(focus)) => Some(focus),
                                _ => None,
                            },
                        },
                    );
                }
                View::Scp { probing } => {
                    let result = scp_status
                        .filter(|(attempt, _)| *attempt == input.scp_attempt)
                        .map(|(_, status)| status);
                    ui::draw_scp(
                        &mut Canvas(frame),
                        &header,
                        probing,
                        qc_mode(latest_observation()),
                        input.settings.scp_findings.or(carried_scp),
                        result,
                    );
                }
                View::Qc2(preview) => {
                    let mode = qc_mode(latest_observation());
                    ui::draw_qc2(
                        &mut Canvas(frame),
                        &header,
                        &ui::Qc2 {
                            preview,
                            request: input.trigger.and_then(QcTarget::volts),
                            mode,
                            status: qc_status,
                            qc: input.qc,
                        },
                    );
                }
                View::Probe => {
                    ui::draw_probe(&mut Canvas(frame), &header, qc_status);
                }
                View::Qc3 { adjusting } => {
                    let mode = qc_mode(latest_observation());
                    ui::draw_qc3(
                        &mut Canvas(frame),
                        &header,
                        &ui::Qc3 {
                            target_mv: input.trigger.and_then(QcTarget::steps),
                            adjusting,
                            mode,
                            status: qc_status,
                            qc: input.qc,
                            lines,
                            reading: reading(last_raw_v, last_raw_i, input.settings.calibration),
                        },
                    );
                }
                View::Fixed { choosing } => {
                    ui::draw_fixed(
                        &mut Canvas(frame),
                        &header,
                        &pd::offer(&latest_observation()),
                        choosing,
                    );
                }
                View::Pps(focus) => {
                    ui::draw_pps(
                        &mut Canvas(frame),
                        &header,
                        &pd::offer(&latest_observation()),
                        focus,
                        reading(last_raw_v, last_raw_i, input.settings.calibration),
                    );
                }
                View::Avs { adjusting } => {
                    ui::draw_avs(
                        &mut Canvas(frame),
                        &header,
                        &pd::offer(&latest_observation()),
                        adjusting,
                        reading(last_raw_v, last_raw_i, input.settings.calibration),
                    );
                }
                View::Settings(button_logic::SettingsFocus::Calibrate(wizard)) => {
                    ui::draw_calibration(
                        &mut Canvas(frame),
                        &header,
                        &wizard,
                        reading(last_raw_v, last_raw_i, wizard.pending),
                    );
                }
                View::Settings(focus) => {
                    ui::draw_settings(
                        &mut Canvas(frame),
                        &header,
                        input.settings,
                        focus,
                        SETTINGS_SAVE.try_get().unwrap_or_default(),
                        input.reset_hold,
                    );
                }
                View::Detail {
                    page: detail,
                    top,
                    scrolling,
                } => {
                    let observation = detail_observation(last_raw_v, input.settings.calibration);
                    let canvas = &mut Canvas(frame);
                    let view = match detail {
                        Detail::Pd => ui::draw_detail(
                            canvas,
                            &header,
                            detail,
                            core::iter::once(pd::lines(observation))
                                .chain(pd::startup_lines(observation))
                                .chain(pd::caps_lines(observation)),
                            top,
                            scrolling,
                        ),
                        Detail::Charger => {
                            // The PD offer, Quick Charge, then the rest of
                            // what the charger said.
                            let partner = pd::partner_sections(observation);
                            let offer = partner.min(pd::CAPABILITY_SECTIONS);
                            let mode = qc_mode(observation);
                            let scp = if input.scp_scanning() {
                                input.settings.scp_findings
                            } else {
                                input.settings.scp_findings.or(carried_scp)
                            };
                            let qc = input.settings.qc_findings.or(qc_findings);
                            let qc_sections = ui::qc_charger_sections(qc);
                            let scp_section = offer + qc_sections;
                            ui::draw_detail(
                                canvas,
                                &header,
                                detail,
                                (0..partner + qc_sections + 1).map(|section| {
                                    if section < offer {
                                        pd::partner_lines(observation, section)
                                    } else if section < scp_section {
                                        ui::qc_charger_lines(qc, mode, section - offer)
                                    } else if section == scp_section {
                                        ui::scp_charger_lines(
                                            scp,
                                            scp_status
                                                .filter(|(attempt, _)| {
                                                    *attempt == input.scp_attempt
                                                })
                                                .map(|(_, status)| status),
                                        )
                                    } else {
                                        pd::partner_lines(observation, section - qc_sections - 1)
                                    }
                                }),
                                top,
                                scrolling,
                            )
                        }
                        Detail::Cable => ui::draw_detail(
                            canvas,
                            &header,
                            detail,
                            (0..pd::cable_sections(observation)).map(|section| {
                                let svid =
                                    pd::cable_svid_index(observation, section).and_then(|index| {
                                        firmware_core::pd_svids::Store::at(&CABLE_SVID_STORE, index)
                                    });
                                pd::cable_lines(
                                    observation,
                                    section,
                                    svid,
                                    &CABLE_SVID_STORE,
                                    CABLE_ASKED.load(Ordering::Relaxed),
                                )
                            }),
                            top,
                            scrolling,
                        ),
                        Detail::System => {
                            let (stack_peak, stack_size) = stack_use();
                            let counters = Counters {
                                step: I2C_FAULT_STEP.load(Ordering::Relaxed),
                                sr: I2C_LAST_SR.load(Ordering::Relaxed),
                                cr1: I2C_LAST_CR1.load(Ordering::Relaxed),
                                cr4: I2C_LAST_CR4.load(Ordering::Relaxed),
                                stack_peak,
                                stack_size,
                                reg_failures: IRQ_REG_FAILURES.load(Ordering::Relaxed),
                                errors: I2C_ERRORS.load(Ordering::Relaxed),
                                reads: ASYNC_READS.load(Ordering::Relaxed),
                                reset_flags: reset_flags(),
                            };
                            let sections = render::system_lines(counters, bus_failed())
                                .into_iter()
                                .chain([storage_diag::lines()]);
                            #[cfg(feature = "flash-probe")]
                            let sections = sections.chain([flash_probe::lines()]);
                            ui::draw_detail(canvas, &header, detail, sections, top, scrolling)
                        }
                    };
                    detail_sender.send_if_modified(|current| {
                        let changed = *current != Some((detail, view));
                        *current = Some((detail, view));
                        changed
                    });
                }
            }
            let frame_start = Instant::now();
            unsafe {
                panel::flush(fb, current_rotation).await;
            }
            frame_time.add(Instant::now(), frame_start.elapsed().as_millis() as u32);
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
        scb.vtor.write(firmware_core::APP_BASE);
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
    // Checked before any task: the PD stack's code lives in the upper region.
    let upper_installed = upper::installed();
    // Migrate only legacy logs in the energy pair before any bus, display DMA
    // or protocol task starts. No blanket cleanup of other occupied sectors.
    let (energy_store, energy_save_status) = if upper_installed {
        energy_store::open()
    } else {
        (None, energy_store::Status::Occupied)
    };

    // Each task is spawned once, here, into its default one-slot pool, so a
    // token is always free (`SpawnError::Busy` needs a second instance).
    const ONE_SPAWN: &str = "each task is spawned once into its one-slot pool";

    // A restart (`PdCommand::Restart`) left the mode of this boot and the
    // screen it opens on in the saved settings. What only this boot needed
    // and the carried Quick Charge result are cleared here, before any task:
    // the flash write masks interrupts, which must not happen in the PD
    // session's start window, when the charger asks the cable question.
    let (mut store, saved) = settings_store::Store::open();
    let mut settings = saved.unwrap_or_default();
    let requested = settings.next_boot;
    let resume = settings.resume;
    let qc_findings = settings.qc_findings;
    // A normal new attachment must not inherit another charger's SCP proof.
    // Carry evidence only through the explicitly requested scan continuation.
    let scp_findings = settings.scp_findings.filter(|_| {
        matches!(
            requested,
            NextBoot::ScanQuickCharge { .. } | NextBoot::ScanPd | NextBoot::CableTest
        )
    });
    // Retain QC across source restarts during the SCP stage. The final
    // PD/cable boot consumes the results and displays them from RAM.
    if !matches!(requested, NextBoot::ScanQuickCharge { .. }) {
        settings.qc_findings = None;
        settings.scp_findings = None;
    }
    settings.finish_boot();
    let cleared = store.save(settings);
    SETTINGS_SAVE.sender().send(store.status());
    // The cable test runs only on a boot after a power-up (the restart's power
    // cycle), with its request verifiably cleared first. A reset during that
    // clear, or any other reset without power loss, cannot repeat it: that
    // boot runs normally and the button task clears the request once up.
    // The silent modes stay until left: a charger may restart the meter
    // before it gives up on PD.
    let powered_up = reset_flags() & RESET_WITHOUT_POWER_LOSS == 0;
    let boot = match requested {
        NextBoot::CableTest if !(powered_up && cleared) => NextBoot::Normal,
        NextBoot::ScanPd if !cleared => NextBoot::Normal,
        requested => requested,
    };
    let cable_test_skipped = requested == NextBoot::CableTest && boot != NextBoot::CableTest;

    let backlight_pwm;
    let line_adc;
    let mut ripple_adc;
    let qc_trigger;
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
        // PA6, PA7, PB0 and PB1 become analog inputs owned by ADC2.
        let Ok(adc) = line_adc::LineAdc::new(hc32f460_pac::Adc2::steal()).await else {
            // Do not start protocol tasks without a valid line-voltage reader.
            loop {
                core::hint::spin_loop();
            }
        };
        line_adc = adc;
        // Independent PA2 capture. Failure leaves only ripple unavailable.
        ripple_adc = ripple::new().await.ok();
        // PB3, PB4, PB8 and PB9 (free after the JTAG release) drive D+/D−
        // only for the Quick Charge page.
        qc_trigger = qc_drive::Trigger::new();

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
            // but the latch makes the missing bus visible (red heartbeat dot).
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
        let Ok(backlight) = init_backlight(backlight_compare(settings)) else {
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

        // The line ADC waits for its scans on this wake.
        if embassy_hc32::intc::register(ADC2_EOCA, ADC_LINE, BUS_PRIORITY).is_err() {
            IRQ_REG_FAILURES.fetch_add(1, Ordering::Relaxed);
        }

        // The panel waits on these for its pixel transfers.
        for (event, line) in PANEL_DMA_EVENTS {
            if embassy_hc32::intc::register(event, line, BUS_PRIORITY).is_err() {
                IRQ_REG_FAILURES.fetch_add(1, Ordering::Relaxed);
            }
        }

        for (event, line) in RIPPLE_DMA_EVENTS {
            if embassy_hc32::intc::register(event, line, BUS_PRIORITY).is_err() {
                IRQ_REG_FAILURES.fetch_add(1, Ordering::Relaxed);
                // Never start DMA when its wake sources are not both routed.
                ripple_adc = None;
            }
        }

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

        // The PD task starts before the panel's sleep-out wait: in passthrough
        // the charger sends its offer once, at most tFirstSourceCap (250 ms in
        // the USB PD timing table) after VBUS, and the meter boots from that
        // same VBUS. It runs on `PD_EXECUTOR`, whose priority is set before
        // `start` unmasks its line, as `InterruptExecutor::start` requires.
        // SAFETY: only this line's priority byte is written, while it is masked.
        cortex_m::Peripherals::steal().NVIC.set_priority(
            PD_EXECUTOR_LINE,
            PD_EXECUTOR_PRIORITY << (u8::BITS as u8 - hc32f460_pac::NVIC_PRIO_BITS),
        );
        if upper_installed {
            PD_EXECUTOR.start(PD_EXECUTOR_LINE).spawn(
                pd_task::run(pd_input, pc13_input, boot, cable_test_skipped).expect(ONE_SPAWN),
            );
        }
        backlight_pwm = backlight.0;
        if !panel::init(backlight.1).await {
            // Backlight is already on, so a lit blank screen means the panel
            // sequence failed; a dark screen means we failed before this point.
            loop {
                core::hint::spin_loop();
            }
        }
    }

    spawner.spawn(
        meter_task(
            backlight_pwm,
            (qc_findings, scp_findings),
            upper_installed,
            settings,
            energy_store,
            energy_save_status,
            ripple_adc.as_ref().map(|_| clocks.pclk[2]),
        )
        .expect(ONE_SPAWN),
    );
    // The other tasks reach PD state and pages; none runs without the upper region.
    if !upper_installed {
        return;
    }
    spawner.spawn(pulse_task().expect(ONE_SPAWN));
    spawner.spawn(tilt_task(tilt_input).expect(ONE_SPAWN));
    spawner.spawn(line_task(line_adc, qc_trigger).expect(ONE_SPAWN));
    spawner.spawn(ripple::run(ripple_adc, clocks.pclk[2]).expect(ONE_SPAWN));
    spawner.spawn(button_task(button_inputs, store, settings, requested, resume).expect(ONE_SPAWN));
}
