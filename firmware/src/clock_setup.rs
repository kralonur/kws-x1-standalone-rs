//! Clock startup with checked HRC/MPLL transitions and inherited bus gates.
//! Normalize onto HRC before changing PLL factors, then verify the final clock
//! state before publishing frequencies to the HAL.

pub use embassy_hc32::clocks::Clocks;

const CMU: usize = 0x40054000;
const PWC: usize = 0x40048000;
const EFM: usize = 0x40010400;
const POLLS: usize = 100_000;
const DIVIDERS: u32 = 0x00112110;
const PLL_MASK: u32 = (0xf << 28) | (0xf << 24) | (0xf << 20) | (0x1ff << 8) | 0x1f | 0x80;
const PLL_FIELDS: u32 = (1 << 28) | (1 << 24) | (1 << 20) | (29 << 8) | 1 | 0x80;

trait Registers {
    fn read8(&mut self, a: usize) -> u8;
    fn read16(&mut self, a: usize) -> u16;
    fn read32(&mut self, a: usize) -> u32;
    fn write8(&mut self, a: usize, v: u8);
    fn write16(&mut self, a: usize, v: u16);
    fn write32(&mut self, a: usize, v: u32);
    fn settle(&mut self);
}

fn unlock(io: &mut impl Registers) {
    let v = io.read16(CMU + 0x3fe);
    io.write16(CMU + 0x3fe, v | 0xa501);
}
fn lock(io: &mut impl Registers) {
    let v = io.read16(CMU + 0x3fe);
    io.write16(CMU + 0x3fe, (v & !1) | 0xa500);
}
fn wait(io: &mut impl Registers, mask: u8, expected: u8) -> Result<(), ()> {
    for _ in 0..POLLS {
        if io.read8(CMU + 0x3c) & mask == expected {
            return Ok(());
        }
    }
    Err(())
}

// Mirrors stock's save/park/restore envelope around CMU divider/source writes.
// None writes the fixed bus dividers; Some selects the requested source.
fn transition(io: &mut impl Registers, source: Option<u8>) {
    let gates = [
        io.read32(PWC),
        io.read32(PWC + 4),
        io.read32(PWC + 8),
        io.read32(PWC + 12),
    ];
    io.write32(PWC + 0x10, 0xa5a50001);
    if io.read8(CMU + 0x26) & 7 == 5 || source == Some(5) {
        io.write32(PWC, 0xfffffaee);
        for n in 1..4 {
            io.write32(PWC + n * 4, u32::MAX);
        }
        io.settle();
    }
    unlock(io);
    if let Some(source) = source {
        let v = io.read8(CMU + 0x26);
        io.write8(CMU + 0x26, (v & !7) | source);
    } else {
        io.write32(CMU + 0x20, DIVIDERS);
    }
    lock(io);
    io.settle();
    for (n, value) in gates.into_iter().enumerate() {
        io.write32(PWC + n * 4, value);
    }
    io.write32(PWC + 0x10, 0xa5a50000);
}

fn setup(io: &mut impl Registers) -> Result<(), ()> {
    if io.read8(CMU + 0x26) & 7 > 5 {
        return Err(());
    }
    // PLLSRC is shared with UPLL. Do not change an active USB PLL's source.
    if io.read32(CMU + 0x100) & 0x80 == 0 && io.read8(CMU + 0x2e) & 1 == 0 {
        return Err(());
    }
    // Never reduce inherited latency before knowing the entry clock. Five
    // waits cover the documented <=200 MHz range; exit low-power read modes.
    // These keys unlock register configuration, not flash programming/erase.
    io.write32(EFM, 0x123);
    io.write32(EFM, 0x3210);
    let v = io.read32(EFM + 8);
    let waits = ((v >> 4) & 0xf).max(5);
    io.write32(EFM + 8, (v & !0x1f1) | (waits << 4));
    if io.read32(EFM) == 1 {
        io.write32(EFM, 0x3210);
        io.write32(EFM, 0x3210);
    }
    if io.read32(EFM + 8) & 0x1f1 != waits << 4 {
        return Err(());
    }
    unlock(io);
    let v = io.read8(CMU + 0x36);
    io.write8(CMU + 0x36, v & !1); // HRC enable.
    let ready = wait(io, 1, 1);
    lock(io);
    ready?;
    transition(io, Some(0));
    if io.read8(CMU + 0x26) & 7 != 0 {
        return Err(());
    }
    unlock(io);
    let v = io.read8(CMU + 0x2a);
    io.write8(CMU + 0x2a, v | 1); // MPLL stop, now that CPU runs from HRC.
    let stopped = wait(io, 0x20, 0);
    lock(io);
    stopped?;
    transition(io, None);
    if io.read32(CMU + 0x20) != DIVIDERS {
        return Err(());
    }
    // Preserve stock LRC enabling and protection/access width.
    unlock(io);
    let v = io.read8(CMU + 0x427);
    io.write8(CMU + 0x427, v & !1);
    io.settle();
    lock(io);
    unlock(io);
    let v = io.read32(CMU + 0x100);
    if v & 0x80 == 0 && io.read8(CMU + 0x3c) & 0x40 != 0 {
        lock(io);
        return Err(()); // UPLL has not yet stopped; shared source unsafe.
    }
    io.write32(CMU + 0x100, (v & !PLL_MASK) | PLL_FIELDS);
    lock(io);
    if io.read32(CMU + 0x100) & PLL_MASK != PLL_FIELDS {
        return Err(());
    }
    unlock(io);
    let v = io.read8(CMU + 0x2a);
    io.write8(CMU + 0x2a, v & !1);
    let ready = wait(io, 0x20, 0x20);
    lock(io);
    ready?;
    transition(io, Some(5));
    unlock(io);
    let v = io.read16(CMU + 0x10);
    io.write16(CMU + 0x10, v & !0xf);
    lock(io);
    // Fail-stop on unexpected readback rather than publish a guessed clock.
    if io.read8(CMU + 0x26) & 7 != 5
        || io.read32(CMU + 0x20) != DIVIDERS
        || io.read32(CMU + 0x100) & PLL_MASK != PLL_FIELDS
    {
        return Err(());
    }

    let hrc = if io.read32(0x40010684) & 1 != 0 {
        16_000_000
    } else {
        20_000_000
    };
    io.write32(0x1fff82a4, hrc);
    io.write32(0x1fff82a8, (hrc / 2) * 30 / 2);
    Ok(())
}

struct Hardware;
// SAFETY: callers use documented MMIO addresses and matching register widths
// during exclusive startup, before peripheral drivers or interrupts run.
impl Registers for Hardware {
    #[inline(always)]
    fn read8(&mut self, a: usize) -> u8 {
        unsafe { (a as *const u8).read_volatile() }
    }
    #[inline(always)]
    fn read16(&mut self, a: usize) -> u16 {
        unsafe { (a as *const u16).read_volatile() }
    }
    #[inline(always)]
    fn read32(&mut self, a: usize) -> u32 {
        unsafe { (a as *const u32).read_volatile() }
    }
    #[inline(always)]
    fn write8(&mut self, a: usize, v: u8) {
        unsafe {
            (a as *mut u8).write_volatile(v);
        }
    }
    #[inline(always)]
    fn write16(&mut self, a: usize, v: u16) {
        unsafe {
            (a as *mut u16).write_volatile(v);
        }
    }
    #[inline(always)]
    fn write32(&mut self, a: usize, v: u32) {
        unsafe {
            (a as *mut u32).write_volatile(v);
        }
    }
    fn settle(&mut self) {
        for _ in 0..512 {
            unsafe {
                core::arch::asm!(
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    "nop",
                    options(nostack, preserves_flags)
                );
            }
        }
    }
}

fn divide(field: u32) -> u32 {
    match field {
        0..=6 => 1 << field,
        _ => 1,
    }
}

/// Configure the system clock on HRC + MPLL and publish the core frequency.
///
/// # Safety
/// Must only be called once during startup, before other drivers use PCLK3.
pub unsafe fn init() -> Result<Clocks, ()> {
    setup(&mut Hardware)?;
    // SAFETY: setup wrote the clock bookkeeping word; startup owns these reads.
    let sys_clk = unsafe { (0x1fff82a8 as *const u32).read_volatile() };
    let scfgr = (CMU + 0x20) as *const u32;
    let v = unsafe { scfgr.read_volatile() };
    let d = |shift: u32| divide((v >> shift) & 0xf);
    Ok(Clocks {
        sys_clk,
        hclk: sys_clk / d(24),
        exclk: sys_clk / d(20),
        pclk: [
            sys_clk / d(0),
            sys_clk / d(4),
            sys_clk / d(8),
            sys_clk / d(12),
            sys_clk / d(16),
        ],
    })
}
