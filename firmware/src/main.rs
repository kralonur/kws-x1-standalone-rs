//! Async meter firmware with passive boot, receive-only passthrough and
//! explicitly selected fixed/PPS sink sessions.
//!
//! The application owns clock startup, pins, shared I2C3, display and tasks.
//! Portable PD policy and the controller PHY independently authorize traffic.
//! The vector table starts at the bootloader's application entry; SysTick
//! drives Embassy time. Bus faults latch closed rather than retrying.

#![no_std]
#![no_main]

mod clock_setup;
#[path = "panel.rs"]
mod panel;
mod pd;
mod pd_cc;
mod pd_spr_phy;
mod pd_trial;

// Board-owned shared bus: platform drivers never depend on X1 task ownership.
static I2C3: embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    Option<embassy_hc32::i2c::I2c3>,
> = embassy_sync::mutex::Mutex::new(None);
mod pd_service;
mod pd_sniff;

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_hc32::gpio::{self, AlternateMode, AlternatePin, InputPin, Port};
// The `#[cortex_m_rt::interrupt]` expansion checks `interrupt::INT0xx`, and the
// PAC exposes the generated `Interrupt` enum through that path.
use embassy_time::{Instant, Timer};
use hc32f460_pac::interrupt;
use kws_x1_core::buttons as button_logic;
use kws_x1_core::latch::Latch;
#[cfg(not(any(feature = "pd-boot-diagnostic", feature = "cable-log")))]
use panic_halt as _;

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
    let mut n = 0;
    while sp + 4 <= top && n < out.len() {
        let value = unsafe { (sp as *const u32).read_volatile() };
        let ret = value & !1;
        if value & 1 == 1 && (0x10004..0x44000).contains(&ret) {
            let bl = half(ret - 4) & 0xf800 == 0xf000 && half(ret - 2) & 0xd000 == 0xd000;
            let blx = half(ret - 2) & 0xff87 == 0x4780;
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

/// `pd-boot-diagnostic`: record the faulting PC, LR and the return addresses
/// above the exception frame, then halt.
#[cfg(any(feature = "pd-boot-diagnostic", feature = "cable-log"))]
#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    cortex_m::interrupt::disable();
    let mut chain = [frame.pc(), frame.lr(), 0, 0, 0];
    return_addresses(core::ptr::from_ref(frame) as usize + 32, &mut chain[2..]);
    flash_log::fatal(flash_log::FAULT, chain);
    loop {
        cortex_m::asm::nop();
    }
}

const CYAN: u16 = 0xff07;
const ALIVE_A: u16 = 0xffff;
const ALIVE_B: u16 = 0x1f00;
/// Heartbeat colour while the I2C3 latch is tripped.
const FAULT: u16 = 0xf800;

/// 240 x 135 RGB565 frame, memory byte order expected by `panel::flush`.
/// It lives alone in SRAM2 (`memory.x`): in SRAMH+SRAM1 it left the stack only
/// ~7.9 KiB, and the deepest PD paths overflowed into the timer queue just
/// below it (HardFault at PC 0 in a timer wake, `docs/SAFETY.md`).
/// Not zeroed by the runtime: `main` zeroes it before any task reads it.
// SAFETY: memory.x reserves SRAM2 for this frame; startup initializes it before use.
#[unsafe(link_section = ".sram2.framebuffer")]
static mut FRAMEBUFFER: core::mem::MaybeUninit<[u16; 32400]> = core::mem::MaybeUninit::uninit();
/// Incremented by `pulse_task` every 500 ms.
static PULSES: AtomicU32 = AtomicU32::new(0);
/// Coherent input snapshot: buttons 0..3, main page 4..6, cable detail 8..31.
static BUTTON_STATE: AtomicU32 = AtomicU32::new(0);
// One bitmap, outside both observations and async futures.
static CABLE_SVIDS: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    core::cell::RefCell<kws_x1_core::pd_svids::Svids>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(
    kws_x1_core::pd_svids::Svids::EMPTY,
));
struct CableSvidStore;
static CABLE_SVID_STORE: CableSvidStore = CableSvidStore;
impl kws_x1_core::pd_svids::Store for CableSvidStore {
    fn clear(&self) {
        CABLE_SVIDS.lock(|cell| cell.borrow_mut().clear());
    }
    fn response(&self, objects: &[u32]) -> Option<kws_x1_core::pd_svids::Progress> {
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
        if now.wrapping_sub(SINCE_MS.load(Ordering::Relaxed)) >= 3000 {
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
/// Screen orientation owned by `tilt_task` (0 = MADCTL 0x60, 1 = 0xa0).
static ROTATION: AtomicU32 = AtomicU32::new(0);
/// Completed 180-degree flips, shown next to the raw edge count.
static TILT_FLIPS: AtomicU32 = AtomicU32::new(0);

/// How many times the four I2C3 interrupt lines fired. Zero means the wake path
/// (routing or `CR2` arming) never worked; a climbing count means the waits are
/// woken and any failure is in the transaction logic instead.
static I2C_IRQ_COUNT: AtomicU32 = AtomicU32::new(0);

/// The two chips on I2C3: the INA226 meter and the LIS3DH tilt sensor.
const INA226_ADDR: u8 = 0x40;
const LIS3DH_ADDR: u8 = 0x18;

/// Completed async I2C3 reads and failures, shown on the test page. Timeouts
/// (no wake at all) and bus errors (NACK/arbitration/timeout flag) are counted
/// separately: they point at completely different faults.
static ASYNC_READS: AtomicU32 = AtomicU32::new(0);
static I2C_ERRORS: AtomicU32 = AtomicU32::new(0);
static I2C_TIMEOUTS: AtomicU32 = AtomicU32::new(0);
/// `SR` and `CR1` as they stood when the bus last failed, for diagnosis.
static I2C_LAST_SR: AtomicU32 = AtomicU32::new(0);
static I2C_LAST_CR1: AtomicU32 = AtomicU32::new(0);
/// Bring-up step the driver was in when it timed out (`0` = no timeout).
static I2C_FAULT_STEP: AtomicU32 = AtomicU32::new(0);
/// `CR4` as it stood when the bus last failed (`BUSWAIT` is bit 10).
static I2C_LAST_CR4: AtomicU32 = AtomicU32::new(0);
/// Failed `intc::register` calls. The INTC is not reset by a jump into this
/// image, so a `SEL` entry left mapped by the bootloader makes routing fail;
/// this must stay 0 now that startup clears the table first.
static IRQ_REG_FAILURES: AtomicU32 = AtomicU32::new(0);
/// Latches the bus off after the first failure (`kws_x1_core::latch`).
static BUS_LATCH: Latch = Latch::new();

/// PB12 / EIRQ12: acknowledge/latch/wake only. No I2C or PHY service in ISR.
#[cortex_m_rt::interrupt]
fn INT004() {
    if embassy_hc32::exti::on_interrupt(12) {
        PD_IRQ_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

/// PB13 external interrupt: the async driver clears the flag and wakes
/// `tilt_task`; the counter is the same evidence shown on the INTC page.
#[cortex_m_rt::interrupt]
fn INT013() {
    if embassy_hc32::exti::on_interrupt(13) {
        EIRQ_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

/// Software interrupt channel 8 (self-test that `SEL`/`IER`/NVIC dispatch work).
#[cortex_m_rt::interrupt]
fn INT008() {
    embassy_hc32::intc::software_interrupt_clear(8);
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

// 3 columns x 5 rows bitmap font (same glyphs as the hybrid application).
fn glyph(c: u8) -> u16 {
    match c {
        b'0' => 0b111_101_101_101_111,
        b'1' => 0b111_010_010_110_010,
        b'2' => 0b111_100_111_001_111,
        b'3' => 0b111_001_111_001_111,
        b'4' => 0b001_001_111_101_101,
        b'5' => 0b111_001_111_100_111,
        b'6' => 0b111_101_111_100_111,
        b'7' => 0b010_010_010_001_111,
        b'8' => 0b111_101_111_101_111,
        b'9' => 0b111_001_111_101_111,
        b'A' => 0b101_101_111_101_010,
        b'B' => 0b110_101_110_101_110,
        b'C' => 0b111_100_100_100_111,
        b'D' => 0b110_101_101_101_110,
        b'E' => 0b111_100_110_100_111,
        b'F' => 0b100_100_110_100_111,
        b'G' => 0b111_101_101_100_111,
        b'H' => 0b101_101_111_101_101,
        b'I' => 0b111_010_010_010_111,
        b'K' => 0b101_101_110_101_101,
        b'L' => 0b111_100_100_100_100,
        b'M' => 0b101_101_101_111_101,
        b'O' => 0b111_101_101_101_111,
        b'P' => 0b100_100_110_101_110,
        b'Q' => 0b001_111_101_101_111,
        b'R' => 0b101_101_110_101_110,
        b'S' => 0b111_001_111_100_111,
        b'T' => 0b010_010_010_010_111,
        b'U' => 0b111_101_101_101_101,
        b'V' => 0b010_101_101_101_101,
        b'W' => 0b101_111_101_101_101,
        b'X' => 0b101_101_010_101_101,
        b'Y' => 0b010_010_010_101_101,
        b'/' => 0b001_001_010_100_100,
        b'_' => 0b000_000_000_000_111,
        b'.' => 0b010_000_000_000_000,
        b'-' => 0b000_000_111_000_000,
        b':' => 0b000_010_000_010_000,
        b'*' => 0b101_010_111_010_101,
        b'[' => 0b111_100_100_100_111,
        b']' => 0b111_001_001_001_111,
        _ => 0,
    }
}

#[inline(never)]
unsafe fn clear_screen(framebuffer: *mut u16) {
    for offset in 0..32400 {
        unsafe {
            framebuffer.add(offset).write_volatile(CYAN);
        }
    }
}

#[inline(never)]
unsafe fn text(framebuffer: *mut u16, mut x: usize, y: usize, bytes: &[u8]) {
    for &c in bytes {
        let bitmap = glyph(c);
        for row in 0..5 {
            for column in 0..3 {
                if bitmap & (1 << (row * 3 + 2 - column)) != 0 {
                    for dy in 0..3 {
                        for dx in 0..3 {
                            let px = x + column * 3 + dx;
                            let py = y + row * 3 + dy;
                            if px < 240 && py < 135 {
                                unsafe {
                                    framebuffer.add(py * 240 + px).write_volatile(0);
                                }
                            }
                        }
                    }
                }
            }
        }
        x += 12;
    }
}

#[inline(always)]
fn copy_4(dest: &mut [u8], offset: usize, src: &[u8; 4]) {
    dest[offset] = src[0];
    dest[offset + 1] = src[1];
    dest[offset + 2] = src[2];
    dest[offset + 3] = src[3];
}

#[inline(always)]
fn copy_6(dest: &mut [u8], offset: usize, src: &[u8; 6]) {
    dest[offset] = src[0];
    dest[offset + 1] = src[1];
    dest[offset + 2] = src[2];
    dest[offset + 3] = src[3];
    dest[offset + 4] = src[4];
    dest[offset + 5] = src[5];
}

// Format fixed point decimal: value is integer, divisor is 1000 for 3 decimals or 100 for 2
#[inline(never)]
fn fmt_fixed_3(mut val: u32, buf: &mut [u8; 6]) {
    for i in (3..6).rev() {
        buf[i] = b'0' + (val % 10) as u8;
        val /= 10;
    }
    buf[2] = b'.';
    buf[1] = b'0' + (val % 10) as u8;
    val /= 10;
    buf[0] = b'0' + (val % 10) as u8;
}

fn fmt_time(total_secs: u32, line: &mut [u8; 14]) {
    let hrs = (total_secs / 3600) % 100;
    let mins = (total_secs / 60) % 60;
    let secs = total_secs % 60;
    line[6] = b'0' + (hrs / 10) as u8;
    line[7] = b'0' + (hrs % 10) as u8;
    line[9] = b'0' + (mins / 10) as u8;
    line[10] = b'0' + (mins % 10) as u8;
    line[12] = b'0' + (secs / 10) as u8;
    line[13] = b'0' + (secs % 10) as u8;
}

#[inline(never)]
fn hex_digits(value: u16) -> [u8; 4] {
    let mut result = [b'0'; 4];
    for (i, digit) in result.iter_mut().enumerate() {
        let nibble = ((value >> ((3 - i) * 4)) & 0xf) as u8;
        *digit = if nibble < 10 {
            b'0' + nibble
        } else {
            b'A' + (nibble - 10)
        };
    }
    result
}

unsafe fn render_measure(
    framebuffer: *mut u16,
    raw_v: u16,
    raw_i: u16,
    total_secs: u32,
    alive: u16,
) {
    unsafe {
        clear_screen(framebuffer);
        text(framebuffer, 12, 6, b"KOWSI X1 EMBASSY");

        let v_mv = (raw_v as u32 * 5) / 4;
        let mut line_v = *b"VBUS: 00.000 V";
        let mut v_buf = [0u8; 6];
        fmt_fixed_3(v_mv, &mut v_buf);
        copy_6(&mut line_v, 6, &v_buf);
        text(framebuffer, 12, 30, &line_v);

        let i_raw = raw_i as i16;
        // Preserve the existing sensor conversion, including its signed range.
        #[allow(clippy::cast_abs_to_unsigned)]
        let i_ma = (i_raw.abs() as u32) / 2;
        let mut line_i = *b"CURR:  00.000 A";
        if i_raw < 0 {
            line_i[6] = b'-';
        }
        let mut i_buf = [0u8; 6];
        fmt_fixed_3(i_ma, &mut i_buf);
        copy_6(&mut line_i, 7, &i_buf);
        text(framebuffer, 12, 54, &line_i);

        let mut line_p = *b"POWR: 00.000 W";
        let mut p_buf = [0u8; 6];
        fmt_fixed_3((v_mv * i_ma) / 1000, &mut p_buf);
        copy_6(&mut line_p, 6, &p_buf);
        text(framebuffer, 12, 78, &line_p);

        let mut line_t = *b"TIME: 00:00:00";
        fmt_time(total_secs, &mut line_t);
        text(framebuffer, 12, 106, &line_t);

        // 500 ms heartbeat square: proves meter_task and pulse_task both run.
        for y in 6..18 {
            for x in 222..234 {
                framebuffer.add(y * 240 + x).write_volatile(alive);
            }
        }
    }
}

#[inline(never)]
unsafe fn render_ports(framebuffer: *mut u16, ports: &[u16; 6]) {
    unsafe {
        clear_screen(framebuffer);
        let mut timing = *b"F:000000 G:000000";
        copy_6(&mut timing, 2, &decimal6(FRAME_MS.load(Ordering::Relaxed)));
        copy_6(
            &mut timing,
            11,
            &decimal6(BUTTON_GAP_MAX_MS.load(Ordering::Relaxed)),
        );
        text(framebuffer, 12, 6, &timing);
        const NAMES: [u8; 6] = *b"ABCDEH";
        for (i, &name) in NAMES.iter().enumerate() {
            let y = 28 + i * 17;
            let mut line = [name, b':', b' ', 0, 0, 0, 0];
            let h = hex_digits(ports[i]);
            copy_4(&mut line, 3, &h);
            text(framebuffer, 12, y, &line);
        }
    }
}

/// Six decimal digits (same rolling format as the fault counters).
fn decimal6(value: u32) -> [u8; 6] {
    let mut digits = [b'0'; 6];
    for (i, d) in digits.iter_mut().enumerate() {
        *d = b'0' + ((value / 10u32.pow(5 - i as u32)) % 10) as u8;
    }
    digits
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
    let peak = kws_x1_core::stack::peak_bytes(words, |i| unsafe {
        (low as *const u32).add(i).read_volatile()
    });
    (peak as u32, (words * 4) as u32)
}

/// Counters shown on the "I2C FAULT" validation page.
struct Counters {
    step: u32,
    sr: u32,
    cr1: u32,
    cr4: u32,
    /// Peak stack use and stack size in bytes (`stack_use`).
    stack_peak: u32,
    stack_size: u32,
    reg_failures: u32,
    errors: u32,
    reads: u32,
}

/// Validation page: one value per row (CR1/CR4 share one), because this is
/// read off a 135 px panel by eye. `STEP` is the bring-up step whose wait hung (`0` = no timeout), which
/// names the failure without needing a debugger:
/// 1 bus free, 2 start accepted, 3 tx buffer empty, 4 byte transferred,
/// 5 stop completed, 6 repeated start, 7 buffer before the read address,
/// 8 receive mode entered, 9 first byte received, 10 second byte received.
#[inline(never)]
unsafe fn render_irq_test(framebuffer: *mut u16, counters: Counters, latched: bool) {
    /// Five hex digits (covers `SR`'s and `CR1`'s low 20 bits).
    fn hex5(value: u32) -> [u8; 5] {
        let mut digits = [b'0'; 5];
        for (i, d) in digits.iter_mut().enumerate() {
            let nibble = ((value >> ((4 - i) * 4)) & 0xf) as u8;
            *d = if nibble < 10 {
                b'0' + nibble
            } else {
                b'A' + (nibble - 10)
            };
        }
        digits
    }

    unsafe {
        clear_screen(framebuffer);
        // Title doubles as the latch indicator, so the eight rows below stay one
        // value each: STEP names the wait that hung (see the legend above).
        text(
            framebuffer,
            6,
            4,
            if latched {
                b"I2C FAULT  BUS!"
            } else {
                b"I2C FAULT  OK"
            },
        );

        let mut line = *b"STEP  : 000000";
        copy_6(&mut line, 8, &decimal6(counters.step));
        text(framebuffer, 6, 20, &line);

        let mut line = *b"SR    : 00000";
        line[8..13].copy_from_slice(&hex5(counters.sr));
        text(framebuffer, 6, 36, &line);

        let mut line = *b"CR1 00000 CR4 00000";
        line[4..9].copy_from_slice(&hex5(counters.cr1));
        line[14..19].copy_from_slice(&hex5(counters.cr4));
        text(framebuffer, 6, 52, &line);

        // Peak stack bytes since boot / stack size: the margin left before an
        // overflow into `.bss` (see docs/SAFETY.md).
        let mut line = *b"STK 000000/000000";
        copy_6(&mut line, 4, &decimal6(counters.stack_peak));
        copy_6(&mut line, 11, &decimal6(counters.stack_size));
        text(framebuffer, 6, 68, &line);

        let mut line = *b"REGF  : 000000";
        copy_6(&mut line, 8, &decimal6(counters.reg_failures));
        text(framebuffer, 6, 84, &line);

        let mut line = *b"ERR   : 000000";
        copy_6(&mut line, 8, &decimal6(counters.errors));
        text(framebuffer, 6, 100, &line);

        let mut line = *b"RD    : 000000";
        copy_6(&mut line, 8, &decimal6(counters.reads));
        text(framebuffer, 6, 116, &line);
    }
}

unsafe fn init_backlight(compare: u16) -> (embassy_hc32::pwm::TimerA4Pwm, AlternatePin) {
    unsafe {
        let mut pwm =
            embassy_hc32::pwm::TimerA4Pwm::new(hc32f460_pac::Tmra4::steal(), 7, 1875).unwrap();

        let pin = AlternatePin::new(Port::H, 2, 4, AlternateMode::DigitalOutput);
        pwm.start(compare).unwrap();
        (pwm, pin)
    }
}

const INA_CONFIG: [(u8, u16); 3] = [(0, 0x4727), (6, 0x0400), (5, 0x0800)];
const SENSOR_CONFIG: [(u8, u8); 5] = [
    (0x20, 0x52),
    (0x23, 0x08),
    (0x22, 0x40),
    (0x32, 0x28),
    (0x30, 0xc1),
];

/// Whether the shared bus is latched off after a failure.
fn bus_failed() -> bool {
    BUS_LATCH.is_failed()
}

/// Record a bus failure: count its kind, keep `SR` for the test page, latch off.
fn note_bus_failure(bus: &mut embassy_hc32::i2c::I2c3, error: embassy_hc32::i2c::Error) {
    match error {
        embassy_hc32::i2c::Error::Timeout(step) => {
            I2C_TIMEOUTS.fetch_add(1, Ordering::Relaxed);
            I2C_FAULT_STEP.store(step, Ordering::Relaxed);
        }
        _ => {
            I2C_ERRORS.fetch_add(1, Ordering::Relaxed);
        }
    }
    I2C_LAST_SR.store(bus.status(), Ordering::Relaxed);
    I2C_LAST_CR1.store(bus.control(), Ordering::Relaxed);
    I2C_LAST_CR4.store(bus.control4(), Ordering::Relaxed);
    BUS_LATCH.fail();
}

/// Read one big-endian INA226 register through the async I2C3 master.
///
/// `tilt_task` shares the bus, so the lock is what keeps a single owner at a
/// time. The first failure latches the bus off: a wedged transfer must not be
/// retried for ever, and the screen stays alive either way.
async fn read_meter(reg: u8) -> Option<u16> {
    if bus_failed() {
        return None;
    }
    let mut guard = I2C3.lock().await;
    if bus_failed() {
        return None;
    }
    let bus = guard.as_mut()?;
    match bus.read_register(INA226_ADDR, reg).await {
        Ok(value) => {
            ASYNC_READS.fetch_add(1, Ordering::Relaxed);
            Some(value)
        }
        Err(error) => {
            note_bus_failure(bus, error);
            None
        }
    }
}

/// Write one big-endian INA226 register (startup configuration).
async fn write_meter(reg: u8, value: u16) -> bool {
    if bus_failed() {
        return false;
    }
    let mut guard = I2C3.lock().await;
    if bus_failed() {
        return false;
    }
    let Some(bus) = guard.as_mut() else {
        return false;
    };
    match bus.write_register(INA226_ADDR, reg, value).await {
        Ok(()) => true,
        Err(error) => {
            note_bus_failure(bus, error);
            false
        }
    }
}

/// Write one LIS3DH register; the tilt sensor sits on the same bus.
async fn write_sensor(reg: u8, value: u8) -> bool {
    if bus_failed() {
        return false;
    }
    let mut guard = I2C3.lock().await;
    if bus_failed() {
        return false;
    }
    let Some(bus) = guard.as_mut() else {
        return false;
    };
    match bus.write_register8(LIS3DH_ADDR, reg, value).await {
        Ok(()) => true,
        Err(error) => {
            note_bus_failure(bus, error);
            false
        }
    }
}

#[embassy_executor::task]
async fn pulse_task() {
    loop {
        Timer::after_millis(500).await;
        PULSES.fetch_add(1, Ordering::Relaxed);
    }
}

/// Async tilt handling: wait on the PB13 external interrupt, then hand the
/// orientation change to `meter_task` through `TILT_REQUESTS`.
#[embassy_executor::task]
async fn tilt_task(input: InputPin) {
    let mut tilt = unsafe {
        embassy_hc32::exti::ExtiInput::from_input(input, embassy_hc32::intc::EirqTrigger::Rising)
    };
    // Errata Rev 1.41 §2.4.2: the peripheral side is already enabled by
    // `ExtiInput::new`, so claiming the line here clears pending before NVIC.
    if unsafe { embassy_hc32::intc::register(13, embassy_hc32::intc::Line::new(13).unwrap(), 8) }
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
    const REFRACTORY_MS: u64 = 1500;
    loop {
        tilt.wait_for_edge().await;

        let rotation = ROTATION.load(Ordering::Relaxed) ^ 1;
        ROTATION.store(rotation, Ordering::Relaxed);
        let cfg = if rotation == 1 { 0xc2 } else { 0xc1 };
        let _ = write_sensor(0x30, cfg).await;
        TILT_FLIPS.fetch_add(1, Ordering::Relaxed);

        // Suppress the rest of the burst, then discard anything it latched.
        Timer::after_millis(REFRACTORY_MS).await;
        while embassy_hc32::exti::take_edge(13) {}
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
        BUTTON_STATE.store(snapshot, Ordering::Relaxed);
        Timer::after_millis(10).await;
    }
}

/// Slider-selected boot: one persistent fixed-target SPR session, or passive.
///
/// PC13 (stock slider sense, latched once at boot like stock `0x15ee2`) is read
/// before any PHY access. Low (sink mode): the session starts at boot, inside
/// the source's Source_Capabilities window (nCapsCount, PD R3.2 §7.32.6); a
/// later start must not use the optional SinkWaitCapTimer Hard Reset (§7.31.3.2).
/// High (passthrough): read-only snapshot plus one verified SWITCHES0 = 0x00
/// write (FUSB302 Rd off, as stock), then the receive-only sniffer
/// (`docs/SAFETY.md`): never GoodCRC, TX or reset signaling.
/// PHY interrupts remain globally masked. Errors halt without reset, retry or
/// cleanup.
#[embassy_executor::task]
async fn pd_task(mut input: embassy_hc32::exti::ExtiInput, pc13: InputPin) {
    let mut observation = pd::Observation::new();
    // Three equal samples or passthrough: an unsettled level never starts a sink.
    let mut sink_mode = true;
    for _ in 0..3 {
        sink_mode &= pc13.is_low();
        Timer::after_millis(1).await;
    }
    observation.pc13_high = Some(!sink_mode);
    observation.irq_high = input.is_high();
    observation.irq_count = PD_IRQ_COUNT.load(Ordering::Relaxed);
    if sink_mode {
        let mut spr = pd::SprObservation::pending();
        spr.target_mv = 5_000;
        observation.spr = Some(spr);
    }
    store_observation(observation);
    if sink_mode {
        // `pd-boot-diagnostic`: step 1 = boot, before the CC check.
        let steps = cfg!(feature = "pd-boot-diagnostic");
        if steps {
            observation.boot_step = 1;
            store_observation(observation);
        }
        if let Some(report) = sink_session(&mut observation).await {
            if steps {
                observation.boot_step = 2;
                store_observation(observation);
            }
            if cfg!(feature = "pd-hold-diagnostic") {
                // CC check restored the power-on baseline: Rd on, AUTO_CRC off.
                observation.spr = None;
                observation.phase = pd::Phase::Hold;
            } else {
                pd_trial::run(&mut observation, report).await;
            }
        }
    } else if let Some(released) = passthrough(&mut observation).await {
        sniff(&mut observation, released).await;
    }
    loop {
        observation.irq_high = input.is_high();
        observation.irq_count = PD_IRQ_COUNT.load(Ordering::Relaxed);
        store_observation(observation);
        // Already-low input is still published without depending on an edge.
        // Timer updates level even if the PHY holds INT_N low indefinitely.
        let _ = embassy_time::with_timeout(
            embassy_time::Duration::from_millis(1000),
            input.wait_for_edge(),
        )
        .await;
    }
}

/// Passthrough: release the FUSB302 Rd once; no GoodCRC or TX. Returns the
/// verified released snapshot, the only state the sniffer may start from.
async fn passthrough(observation: &mut pd::Observation) -> Option<pd::Snapshot> {
    let mut guard = I2C3.lock().await;
    if bus_failed() {
        observation.phase = pd::Phase::Skipped;
        return None;
    }
    let Some(bus) = guard.as_mut() else {
        observation.phase = pd::Phase::Skipped;
        return None;
    };
    let mut released = None;
    match pd::snapshot(bus).await {
        Ok(snapshot) => {
            observation.snapshot = Some(snapshot);
            match pd_cc::passive().release(bus, snapshot).await {
                Ok(after) => {
                    observation.snapshot = Some(after);
                    observation.phase = pd::Phase::Passive;
                    released = Some(after);
                }
                Err(failure) => {
                    observation.restore_unverified |= failure.unverified;
                    cc_failure(observation, bus, failure.error);
                }
            }
        }
        Err(failure) => {
            observation.failed_register = failure.register;
            observation.phase = pd::Phase::Failed;
            note_bus_failure(bus, failure.source);
        }
    }
    released
}

/// Receive-only passthrough sniffer: verified start (orientation, flush,
/// listen), then one poll per I2C3 lock every nominal 1 ms and a read-only
/// configuration check every second. Any error stops it for good, without
/// cleanup writes; bus errors and configuration mismatches also latch the bus.
async fn sniff(observation: &mut pd::Observation, released: pd::Snapshot) {
    let mut view = pd::SniffObservation::new();
    observation.sniff = Some(view);
    PD_OBSERVATION.lock(|cell| *cell.borrow_mut() = *observation);
    let started = {
        let mut guard = I2C3.lock().await;
        if bus_failed() {
            view.stop = Some(pd::SniffStop::Halted);
            None
        } else if let Some(bus) = guard.as_mut() {
            let mut delay = embassy_time::Delay;
            // Sole bus ownership through both nominal 10 ms CC settles.
            match pd_sniff::once().start(bus, &mut delay, released).await {
                Ok(mut sniffer) => {
                    view.cc_status = Some(sniffer.cc_status());
                    view.cc = Some(sniffer.selected());
                    view.flags = sniffer.flags();
                    Some(sniffer)
                }
                Err(failure) => {
                    view.cc_status = failure.cc_status;
                    view.flags = failure.flags;
                    observation.restore_unverified |= failure.unverified;
                    sniff_stop(&mut view, bus, failure.error);
                    None
                }
            }
        } else {
            view.stop = Some(pd::SniffStop::Halted);
            None
        }
    };
    observation.sniff = Some(view);
    PD_OBSERVATION.lock(|cell| *cell.borrow_mut() = *observation);
    let Some(mut sniffer) = started else {
        return;
    };
    let mut next_check = Instant::now() + embassy_time::Duration::from_secs(1);
    while view.stop.is_none() {
        Timer::after_millis(1).await;
        let mut changed = false;
        {
            let mut guard = I2C3.lock().await;
            let Some(bus) = guard.as_mut().filter(|_| !bus_failed()) else {
                view.stop = Some(pd::SniffStop::Halted);
                break;
            };
            let result = if Instant::now() >= next_check {
                next_check += embassy_time::Duration::from_secs(1);
                sniffer.check(bus).await.map(|()| None)
            } else {
                sniffer.poll(bus).await.map(Some)
            };
            view.flags = sniffer.flags();
            match result {
                Ok(Some((outcome, status0))) => {
                    // BC_LVL is only meaningful between BMC transmissions.
                    let rp = kws_x1_core::pd_sniff::rp(status0);
                    if kws_x1_core::pd_sniff::quiet(status0) && rp != view.rp {
                        view.rp = rp;
                        changed = true;
                    }
                    if let pd_service::Outcome::Packet(packet) = outcome {
                        view.record(&packet.frame());
                        changed = true;
                    }
                }
                Ok(None) => {}
                Err(error) => {
                    sniff_stop(&mut view, bus, error);
                    changed = true;
                }
            }
        }
        if changed {
            observation.sniff = Some(view);
            PD_OBSERVATION.lock(|cell| *cell.borrow_mut() = *observation);
        }
    }
    observation.sniff = Some(view);
    PD_OBSERVATION.lock(|cell| *cell.borrow_mut() = *observation);
}

/// Record why the sniffer stopped. Bus errors and configuration mismatches latch
/// the bus; wire events (Hard Reset, overflow, bad frame) stop only the sniffer.
fn sniff_stop(
    view: &mut pd::SniffObservation,
    bus: &mut embassy_hc32::i2c::I2c3,
    error: pd_sniff::Error<embassy_hc32::i2c::Error>,
) {
    use pd::SniffStop as Stop;
    view.stop = Some(match error {
        pd_sniff::Error::Stopped => Stop::Halted,
        pd_sniff::Error::Orientation(_) => Stop::Orientation,
        pd_sniff::Error::Boundary { status } => Stop::Boundary(status),
        pd_sniff::Error::Config(pd_cc::Error::Baseline { register, value }) => {
            Stop::Guard { register, value }
        }
        pd_sniff::Error::Config(pd_cc::Error::Verify { register, value }) => {
            BUS_LATCH.fail();
            Stop::Verify { register, value }
        }
        pd_sniff::Error::Drift { register, value } => {
            BUS_LATCH.fail();
            Stop::Drift { register, value }
        }
        pd_sniff::Error::Config(pd_cc::Error::Bus { register, source })
        | pd_sniff::Error::Rx(pd_service::Error::Bus { register, source }) => {
            note_bus_failure(bus, source);
            Stop::Bus(register)
        }
        pd_sniff::Error::Rx(pd_service::Error::HardReset) => Stop::HardReset,
        pd_sniff::Error::Rx(pd_service::Error::RxFull) => Stop::Overflow,
        pd_sniff::Error::Rx(pd_service::Error::Frame(_)) => Stop::Frame,
        pd_sniff::Error::Rx(
            pd_service::Error::UnexpectedTransmission | pd_service::Error::TransmissionFault,
        ) => Stop::UnexpectedTx,
        pd_sniff::Error::Rx(pd_service::Error::PhyFault) => Stop::PhyFault,
        pd_sniff::Error::Rx(pd_service::Error::Halted) => Stop::Halted,
    });
}

/// Record a configuration failure; only a bus error latches the bus here.
fn cc_failure(
    observation: &mut pd::Observation,
    bus: &mut embassy_hc32::i2c::I2c3,
    error: pd_cc::Error<embassy_hc32::i2c::Error>,
) {
    match error {
        pd_cc::Error::Baseline { register, value } => {
            observation.failed_register = register;
            observation.failed_value = Some(value);
            observation.phase = pd::Phase::Guarded;
        }
        pd_cc::Error::Bus { register, source } => {
            observation.failed_register = register;
            observation.phase = pd::Phase::Failed;
            note_bus_failure(bus, source);
        }
        pd_cc::Error::Verify { register, value } => {
            observation.failed_register = register;
            observation.failed_value = Some(value);
            observation.phase = pd::Phase::Verify;
            BUS_LATCH.fail();
        }
    }
}

/// Sink mode: the one-shot CC diagnostic on the power-on configuration.
async fn sink_session(observation: &mut pd::Observation) -> Option<pd_cc::Report> {
    let mut guard = I2C3.lock().await;
    if bus_failed() {
        observation.phase = pd::Phase::Skipped;
        return None;
    }
    let Some(bus) = guard.as_mut() else {
        observation.phase = pd::Phase::Skipped;
        return None;
    };
    let snapshot = match pd::snapshot(bus).await {
        Ok(snapshot) => snapshot,
        Err(failure) => {
            observation.failed_register = failure.register;
            observation.phase = pd::Phase::Failed;
            note_bus_failure(bus, failure.source);
            return None;
        }
    };
    observation.snapshot = Some(snapshot);
    observation.phase = pd::Phase::Measuring;
    store_observation(*observation);
    let mut delay = embassy_time::Delay;
    // Deliberately retain sole bus ownership through the two nominal 10 ms
    // settles: one coherent, non-repeating session.
    // A successful report requires full restored baseline verification; no
    // failed/cancelled session claims restore.
    match pd_cc::diagnostic().run(bus, &mut delay, snapshot).await {
        Ok(report) => {
            observation.cc_status = Some(report.status);
            observation.post_snapshot = Some(report.after);
            Some(report)
        }
        Err(failure) => {
            observation.restore_unverified |= failure.unverified;
            cc_failure(observation, bus, failure.error);
            None
        }
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
            Some((raw_vbus as u32 * 5 / 4).min(u16::MAX as u32) as u16)
        } else {
            None
        };
    }
    unsafe {
        clear_screen(framebuffer);
        // This task is the sole framebuffer owner; other tasks publish values.
        if let Some(page) = cable_detail {
            let svid = pd::cable_svid_index(observation, page)
                .and_then(|index| kws_x1_core::pd_svids::Store::at(&CABLE_SVID_STORE, index));
            pd::paint_cable(
                &mut *(framebuffer as *mut [u16; 32400]),
                observation,
                page,
                svid,
                glyph,
            );
        } else {
            pd::paint(&mut *(framebuffer as *mut [u16; 32400]), observation, glyph);
        }
    }
}

/// Charger info page (`pd::partner_lines`); `page` selects the sub-page.
unsafe fn render_partner(framebuffer: *mut u16, page: u32) {
    let observation = PD_OBSERVATION.lock(|cell| *cell.borrow());
    unsafe {
        clear_screen(framebuffer);
        pd::paint_partner(
            &mut *(framebuffer as *mut [u16; 32400]),
            observation,
            page,
            glyph,
        );
    }
}

#[embassy_executor::task]
async fn meter_task() {
    let fb = framebuffer();
    let mut last_buttons = u32::MAX;
    let mut dirty = true;
    let mut last_raw_v: u16 = 0xffff;
    let mut last_raw_i: u16 = 0xffff;
    // Reads are paced by an absolute deadline, never by subtracting from an
    // `Instant` (embassy-time panics on underflow, e.g. `Instant::now() - 200ms`
    // right after boot). The first read is due one period late on purpose: the
    // first frame must be drawn *before* any bus traffic, so a bus problem can
    // never leave the display uninitialized.
    let mut next_read = Instant::now() + embassy_time::Duration::from_millis(100);
    let mut current_rotation: u8 = 0;
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
        let buttons = BUTTON_STATE.load(Ordering::Relaxed);
        let page = ((buttons >> 4) & 7) as u8;
        if buttons != last_buttons {
            last_buttons = buttons;
            dirty = true;
        }

        // `tilt_task` owns the orientation (it flipped and reconfigured the
        // sensor); this task only follows it for flushing and redraws.
        let rotation_now = ROTATION.load(Ordering::Relaxed) as u8;
        if rotation_now != current_rotation {
            // Only the display task changes MADCTL, before its matching RASET.
            unsafe {
                panel::set_orientation(rotation_now).await;
            }
            current_rotation = rotation_now;
            dirty = true;
        }

        let mut changed = false;
        if (page == 0 || page == 3) && now >= next_read {
            next_read = now + embassy_time::Duration::from_millis(100);
            if let Some(v) = read_meter(2).await {
                if v != last_raw_v {
                    last_raw_v = v;
                    changed = true;
                }
                if page == 3 {
                    changed = true;
                }
            }
            if page == 0
                && let Some(i) = read_meter(4).await
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

        Timer::after_millis(10).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    unsafe {
        // SRAM wait states for high frequency (matches stock 0x10298).
        (0x40050804 as *mut u16).write_volatile(0x77);
        (0x4005080c as *mut u16).write_volatile(0x77);
        (0x40050800 as *mut u16).write_volatile(0x1100);
        (0x40050804 as *mut u16).write_volatile(0x76);
        (0x4005080c as *mut u16).write_volatile(0x76);

        // Enable CP10/CP11 for the FPU (matches SystemInit).
        let cpacr = 0xe000ed88 as *mut u32;
        cpacr.write_volatile(cpacr.read_volatile() | (0xf << 20));

        // Disable every external IRQ; this image uses none of them.
        for (bank, &mask) in [!0u32, !0, !0, !0, 0xffff].iter().enumerate() {
            ((0xe000e180usize + bank * 4) as *mut u32).write_volatile(mask);
        }
        (0xe000ed08 as *mut u32).write_volatile(0x10000);
        core::arch::asm!("dsb", "isb", options(nostack, preserves_flags));
    }

    // SRAM2 holds only the frame buffer; write all of it before any read
    // (SRAM parity, RM section 8: uninitialized words must not be read).
    unsafe {
        core::ptr::write_bytes(framebuffer(), 0, 32400);
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
        // Stock 0x15f8c: PC13 input + pull-up, no INTE. Read-only slider sense
        // (stock 0x206e0 shows "Switch To Sink Mode" while it reads high;
        // observed low with the slider ON).
        pc13_input = InputPin::new_pull_up(Port::C, 13);

        // The INTC is not reset by a jump into this image, so a bootloader or an
        // earlier image can leave `SEL` entries mapped; `register` then refuses
        // those lines. Start from a clean slate: this application owns the INTC
        // and installs every handler it uses.
        embassy_hc32::intc::clear_routes();
        // Stock config-field audit: PB12, not nearby PC13, supplies EIRQ12.
        pd_input = embassy_hc32::exti::ExtiInput::from_input(
            InputPin::new_pull_up(Port::B, 12),
            embassy_hc32::intc::EirqTrigger::Falling,
        );
        if embassy_hc32::intc::register(12, embassy_hc32::intc::Line::new(4).unwrap(), 9).is_err() {
            IRQ_REG_FAILURES.fetch_add(1, Ordering::Relaxed);
        }

        // I2C3 first (PB14/PB15, FCG1), as in the hybrid startup order. The
        // async master publishes itself; every later use goes through that lock.
        let pins = [
            AlternatePin::new(Port::B, 15, 0x31, AlternateMode::FunctionOnly),
            AlternatePin::new(Port::B, 14, 0x30, AlternateMode::FunctionOnly),
        ];
        if let Ok(bus) = embassy_hc32::i2c::I2c3::new(clocks.pclk[3], 400_000, pins) {
            *I2C3.try_lock().expect("startup owns I2C3") = Some(bus);
        } else {
            // Keep running: the meter screen stays alive without measurements,
            // but the latch makes the missing bus visible (red heartbeat square).
            BUS_LATCH.fail();
        }

        // INTC validation (vendor-documented sequence, RM 10.4.2 + Errata
        // Rev 1.41 2.4.1/2.4.2):
        //   1. Err消费 2.4.1: a pin position may only have one port's INTE set,
        //      and the lowest port number wins, so clear channel 13 elsewhere.
        //   2. Enable the peripheral side (PCR.INTE + EIRQCR) first.
        //   3. Then claim the line, clear pending and enable NVIC (register()).
        let backlight = init_backlight(1125);

        // I2C3 wake sources: group 13's window is lines 110..=115 (RM Table 10-2),
        // so each event gets its own line and handler. They must be live before
        // the first awaited transfer below.
        for (event, line) in [(428u16, 110u8), (429, 111), (430, 112), (431, 113)] {
            if embassy_hc32::intc::register(event, embassy_hc32::intc::Line::new(line).unwrap(), 8)
                .is_err()
            {
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
        // Priorities 8/9 keep them below SysTick (0), so timing never starves.
        if embassy_hc32::intc::register(8, embassy_hc32::intc::Line::new(8).unwrap(), 9).is_err() {
            IRQ_REG_FAILURES.fetch_add(1, Ordering::Relaxed);
        }
        // One software interrupt at startup: if SWI ch8 reads 1 on the test
        // page, `SEL`/`IER`/NVIC dispatch and the vector table all work.
        embassy_hc32::intc::software_interrupt_trigger(8);

        // Configure the INA226 and the LIS3DH over the async master.
        for (reg, value) in INA_CONFIG {
            if !write_meter(reg, value).await {
                break;
            }
        }
        for (reg, value) in SENSOR_CONFIG {
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

    spawner.spawn(pulse_task().unwrap());
    spawner.spawn(tilt_task(tilt_input).unwrap());
    spawner.spawn(button_task(button_inputs).unwrap());
    spawner.spawn(meter_task().unwrap());
    spawner.spawn(pd_task(pd_input, pc13_input).unwrap());
}
