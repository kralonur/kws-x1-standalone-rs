//! Clock startup with checked HRC/MPLL transitions and inherited bus gates.
//! Normalize onto HRC before changing PLL factors, then verify the final clock
//! state before publishing frequencies to the HAL.
//!
//! Register authority: [RM] Rev1.5. Sections below describe hardware encodings,
//! not the choice of the inherited clock profile. Undocumented compatibility
//! assumptions and uncalibrated software delays are identified separately.
//!
//! [RM]: https://oss-nc-beijing-2.cecloudcs.com/doc-rm/RM_HC32F460_F45x_A460SeriesReferenceManual_Rev1.5.pdf

use embassy_hc32::clock_decode::{HRC_16MHZ, HRC_20MHZ};
pub use embassy_hc32::clocks::Clocks;

// RM 4.11: CMU clock-register base; retain the existing raw access widths.
const CMU: usize = 0x40054000;
// RM 5.7: PWC function-clock gate register base.
const PWC: usize = 0x40048000;
// RM 7.9: EFM base, also the 32-bit FAPRT register address.
const EFM_FAPRT: usize = 0x40010400;
// RM 4.11 register table / 4.11.23: 16-bit AD/TRNG clock selector offset.
const CMU_PERICKSEL: usize = CMU + 0x10;
// RM 4.11 register table / 4.11.21: 32-bit clock divider register offset.
const CMU_SCFGR: usize = CMU + 0x20;
// RM 4.11 register table / 4.11.20: 8-bit system-clock source selector offset.
const CMU_CKSWR: usize = CMU + 0x26;
// RM 4.11 register table / 4.11.16: 8-bit MPLL control register offset.
const CMU_PLLCR: usize = CMU + 0x2a;
// RM 4.11 register table / 4.11.18: 8-bit UPLL control register offset.
const CMU_UPLLCR: usize = CMU + 0x2e;
// RM 4.11 register table / 4.11.10: 8-bit HRC control register offset.
const CMU_HRCCR: usize = CMU + 0x36;
// RM 4.11 register table / 4.11.19: 8-bit oscillator status register offset.
const CMU_OSCSTBSR: usize = CMU + 0x3c;
// RM 4.11 register table / 4.11.15: 32-bit shared PLL source/MPLL factors offset.
const CMU_PLLCFGR: usize = CMU + 0x100;
// RM 5.7.18: 16-bit function protection register in the CMU address region;
// its address is also confirmed by the dependency's PAC register layout.
const PWC_FPRC: usize = CMU + 0x3fe;
// RM 4.11 register table / 4.11.14: 8-bit LRC control register offset.
const CMU_LRCCR: usize = CMU + 0x427;
// RM 5.7 register table / 5.7.13-16: four consecutive 32-bit clock gates.
const FCG_COUNT: usize = 4;
// RM 5.7 register table: gate-register spacing, in bytes; preserve access order.
const FCG_STRIDE: usize = core::mem::size_of::<u32>();
// RM 5.7 register table / 5.7.17: 32-bit gate write-protection register offset.
const PWC_FCG0PC: usize = PWC + 0x10;
// RM 7.9 register table / 7.9.3: 32-bit flash read-mode configuration offset.
const EFM_FRMC: usize = EFM_FAPRT + 8;

// RM 5.7.18: FPRCWE[15:8] must contain 0xa5 to update protection bits.
const CMU_WRITE_KEY: u16 = 0xa500;
// RM Table 5-9: FPRCB0 grants writes to protected CMU clock registers.
const CMU_WRITE_ENABLE: u16 = 1;
// RM 5.7.17: FCG0PCWE[31:16] must contain 0xa5a5 to update PRT0.
const FCG_WRITE_KEY: u32 = 0xa5a5_0000;
// RM 5.7.17: FCG0PC.PRT0, bit 0, enables FCG0 writes.
const FCG_WRITE_ENABLE: u32 = 1;
// Inherited transition policy: FCG0 parking mask. No vendor requirement for
// this exact selection is established; preserve it without retuning.
const PARKED_FCG0: u32 = 0xfffffaee;
// RM 7.9.1: first FAPRT unlock key; retain the existing 32-bit write width.
const EFM_UNLOCK_FIRST: u32 = 0x0123;
// RM 7.9.1: second unlock key. Any write when unlocked re-enables protection;
// the inherited relock sequence deliberately uses this same value twice.
const EFM_UNLOCK_SECOND: u32 = 0x3210;
// RM 7.9.1: FAPRT reads one when access protection is disabled.
const EFM_UNPROTECTED: u32 = 1;
// RM 7.9.3: FRMC.FLWT is the four-bit field starting at bit 4.
const FLASH_WAIT_SHIFT: u32 = 4;
// RM 7.9.3: unshifted mask for FRMC.FLWT[3:0].
const FLASH_WAIT_MASK: u32 = 0xf;
// RM 7.9.3: FRMC.SLPMD (bit 0) and LVM (bit 8) enable low-power read modes.
const FLASH_LOW_POWER_MASK: u32 = (1 << 0) | (1 << 8);
// RM Table 7-1: five wait cycles cover HCLK up to 200 MHz in normal read mode.
// Preserve higher inherited latency because the entry clock is not yet known.
const MIN_FLASH_WAITS: u32 = 5;
// Only change/read back the latency and low-power fields, not cache controls.
const FLASH_READ_MODE_MASK: u32 = (FLASH_WAIT_MASK << FLASH_WAIT_SHIFT) | FLASH_LOW_POWER_MASK;

// RM 4.11.20: CKSW[2:0] selects the system-clock source.
const CLOCK_SOURCE_MASK: u8 = 0b111;
// RM 4.11.20: CKSW=0 selects HRC.
const CLOCK_SOURCE_HRC: u8 = 0;
// RM 4.11.20: CKSW=5 selects MPLL; larger encodings are prohibited.
const CLOCK_SOURCE_MPLL: u8 = 5;
// RM 4.11.16, 4.11.18: PLLCR.MPLLOFF and UPLLCR.UPLLOFF are both bit 0.
const PLL_STOP: u8 = 1;
// RM 4.11.10, 4.11.14: HRCCR.HRCSTP and LRCCR.LRCSTP are both bit 0.
const RC_STOP: u8 = 1;
// RM 4.11.19: OSCSTBSR.HRCSTBF is bit 0.
const HRC_STABLE: u8 = 1;
// RM 4.11.19: OSCSTBSR.MPLLSTBF is bit 5.
const MPLL_STABLE: u8 = 1 << 5;
// RM 4.11.19: OSCSTBSR.UPLLSTBF is bit 6.
const UPLL_STABLE: u8 = 1 << 6;
// RM 4.11.23: PERICKSEL is a four-bit AD/TRNG clock selector.
const PERIPHERAL_CLOCK_MASK: u16 = 0xf;

// Inherited clock-profile input division; RM 4.11.15 stores the factor minus one.
const MPLL_INPUT_DIVIDER: u32 = 2;
// Inherited profile multiplier: HRC / 2 * 30 gives a 240/300 MHz VCO.
// RM 4.11.15 permits 240-480 MHz; this is a profile choice, not a required rate.
const MPLL_MULTIPLIER: u32 = 30;
// Inherited profile P/Q/R divide by two, giving a 120/150 MHz system clock.
// Keep these rates to preserve peripheral timing; RM 4.11.15 defines the encoding.
const MPLL_OUTPUT_DIVIDER: u32 = 2;
// RM 4.11.15: PLLCFGR.MPLLM occupies bits 4:0.
const MPLL_INPUT_MASK: u32 = 0x1f;
// RM 4.11.15: PLLCFGR.MPLLN begins at bit 8.
const MPLL_MULTIPLIER_SHIFT: u32 = 8;
// RM 4.11.15: PLLCFGR.MPLLN occupies nine bits.
const MPLL_MULTIPLIER_MASK: u32 = 0x1ff;
// RM 4.11.15: PLLCFGR.MPLLP starts at bit 28.
const MPLL_P_SHIFT: u32 = 28;
// RM 4.11.15: PLLCFGR.MPLLQ starts at bit 24.
const MPLL_Q_SHIFT: u32 = 24;
// RM 4.11.15: PLLCFGR.MPLLR starts at bit 20.
const MPLL_R_SHIFT: u32 = 20;
// RM 4.11.15: each P/Q/R output divider field is four bits wide.
const MPLL_OUTPUT_MASK: u32 = 0xf;
// RM 4.11.15: shared PLLSRC, bit 7, selects HRC for MPLL and UPLL.
const PLL_SOURCE_HRC: u32 = 1 << 7;
// Fields deliberately changed by startup; leave unrelated/reserved bits untouched.
const PLL_MASK: u32 = (MPLL_OUTPUT_MASK << MPLL_P_SHIFT)
    | (MPLL_OUTPUT_MASK << MPLL_Q_SHIFT)
    | (MPLL_OUTPUT_MASK << MPLL_R_SHIFT)
    | (MPLL_MULTIPLIER_MASK << MPLL_MULTIPLIER_SHIFT)
    | MPLL_INPUT_MASK
    | PLL_SOURCE_HRC;
// RM 4.11.15 encodes all factors minus one; use the same profile for bookkeeping.
const PLL_FIELDS: u32 = ((MPLL_OUTPUT_DIVIDER - 1) << MPLL_P_SHIFT)
    | ((MPLL_OUTPUT_DIVIDER - 1) << MPLL_Q_SHIFT)
    | ((MPLL_OUTPUT_DIVIDER - 1) << MPLL_R_SHIFT)
    | ((MPLL_MULTIPLIER - 1) << MPLL_MULTIPLIER_SHIFT)
    | (MPLL_INPUT_DIVIDER - 1)
    | PLL_SOURCE_HRC;
// Inherited SCFGR profile, encoded per RM 4.11.21: HCLK /1, EXCLK /2,
// PCLK0 /1, PCLK1/2/4 /2 and PCLK3 /4. Preserve existing peripheral timing.
const DIVIDERS: u32 = 0x00112110;
// RM 4.11.21: PCLK0..4 selectors start every four bits.
const PCLK_FIELD_STRIDE: u32 = 4;
// RM 4.11.21: EXCKS starts at bit 20.
const EXCLK_SHIFT: u32 = 20;
// RM 4.11.21: HCLKS starts at bit 24.
const HCLK_SHIFT: u32 = 24;
// Legacy extraction includes the reserved fourth bit; retain the nibble mask
// in this behavior-preserving pass. The documented selector itself is three bits.
const DIVIDER_READ_MASK: u32 = 0xf;
// RM 4.11.21: valid divider selectors 0..=6 encode /1 through /64.
const MAX_DIVIDER_SELECTOR: u32 = 6;

// Inherited, NOT vendor-documented: EFM-region frequency readback word.
// Bit 0 set is interpreted as nominal 16 MHz, clear as 20 MHz; this is an assumption.
const HRC_FREQUENCY_READBACK: usize = 0x40010684;
// Inherited readback selector, bit 0. Reuse the HAL's nominal frequencies
// (ICG1.HRCFREQSEL mapping, RM 6.2.2); these are not measured frequencies.
const HRC_16MHZ_SELECTED: u32 = 1;
// Inherited startup contract: HRC frequency word in RAM, in Hz.
// This compatibility location is not a vendor register or a measured value.
const HRC_FREQUENCY_WORD: usize = 0x1fff82a4;
// Inherited startup contract: system frequency word follows the HRC word.
const SYSTEM_FREQUENCY_WORD: usize = HRC_FREQUENCY_WORD + core::mem::size_of::<u32>();
// Inherited software bound, in polls rather than milliseconds. No calibration
// evidence is recorded; preserve the finite fail-stop limit instead of retuning it.
const POLLS: usize = 100_000;
// Inherited settling policy: 512 iterations of five explicit NOPs. Not a
// calibrated duration or proof of a hardware minimum; preserve it unchanged.
const SETTLE_ITERATIONS: usize = 512;

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
    let v = io.read16(PWC_FPRC);
    io.write16(PWC_FPRC, v | CMU_WRITE_KEY | CMU_WRITE_ENABLE);
}
fn lock(io: &mut impl Registers) {
    let v = io.read16(PWC_FPRC);
    io.write16(PWC_FPRC, (v & !CMU_WRITE_ENABLE) | CMU_WRITE_KEY);
}
fn wait(io: &mut impl Registers, mask: u8, expected: u8) -> Result<(), ()> {
    for _ in 0..POLLS {
        if io.read8(CMU_OSCSTBSR) & mask == expected {
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
        io.read32(PWC + FCG_STRIDE),
        io.read32(PWC + 2 * FCG_STRIDE),
        io.read32(PWC + 3 * FCG_STRIDE),
    ];
    io.write32(PWC_FCG0PC, FCG_WRITE_KEY | FCG_WRITE_ENABLE);
    if io.read8(CMU_CKSWR) & CLOCK_SOURCE_MASK == CLOCK_SOURCE_MPLL
        || source == Some(CLOCK_SOURCE_MPLL)
    {
        io.write32(PWC, PARKED_FCG0);
        for n in 1..FCG_COUNT {
            io.write32(PWC + n * FCG_STRIDE, u32::MAX);
        }
        io.settle();
    }
    unlock(io);
    if let Some(source) = source {
        let v = io.read8(CMU_CKSWR);
        io.write8(CMU_CKSWR, (v & !CLOCK_SOURCE_MASK) | source);
    } else {
        io.write32(CMU_SCFGR, DIVIDERS);
    }
    lock(io);
    io.settle();
    for (n, value) in gates.into_iter().enumerate() {
        io.write32(PWC + n * FCG_STRIDE, value);
    }
    io.write32(PWC_FCG0PC, FCG_WRITE_KEY);
}

fn setup(io: &mut impl Registers) -> Result<(), ()> {
    if io.read8(CMU_CKSWR) & CLOCK_SOURCE_MASK > CLOCK_SOURCE_MPLL {
        return Err(());
    }
    // PLLSRC is shared with UPLL. Do not change an active USB PLL's source.
    if io.read32(CMU_PLLCFGR) & PLL_SOURCE_HRC == 0 && io.read8(CMU_UPLLCR) & PLL_STOP == 0 {
        return Err(());
    }
    // Never reduce inherited latency before knowing the entry clock. Five
    // waits cover the documented <=200 MHz range; exit low-power read modes.
    // These keys unlock register configuration, not flash programming/erase.
    io.write32(EFM_FAPRT, EFM_UNLOCK_FIRST);
    io.write32(EFM_FAPRT, EFM_UNLOCK_SECOND);
    let v = io.read32(EFM_FRMC);
    let waits = ((v >> FLASH_WAIT_SHIFT) & FLASH_WAIT_MASK).max(MIN_FLASH_WAITS);
    io.write32(
        EFM_FRMC,
        (v & !FLASH_READ_MODE_MASK) | (waits << FLASH_WAIT_SHIFT),
    );
    if io.read32(EFM_FAPRT) == EFM_UNPROTECTED {
        io.write32(EFM_FAPRT, EFM_UNLOCK_SECOND);
        io.write32(EFM_FAPRT, EFM_UNLOCK_SECOND);
    }
    if io.read32(EFM_FRMC) & FLASH_READ_MODE_MASK != waits << FLASH_WAIT_SHIFT {
        return Err(());
    }
    unlock(io);
    let v = io.read8(CMU_HRCCR);
    io.write8(CMU_HRCCR, v & !RC_STOP); // HRC enable.
    let ready = wait(io, HRC_STABLE, HRC_STABLE);
    lock(io);
    ready?;
    transition(io, Some(CLOCK_SOURCE_HRC));
    if io.read8(CMU_CKSWR) & CLOCK_SOURCE_MASK != CLOCK_SOURCE_HRC {
        return Err(());
    }
    unlock(io);
    let v = io.read8(CMU_PLLCR);
    io.write8(CMU_PLLCR, v | PLL_STOP); // MPLL stop, now that CPU runs from HRC.
    let stopped = wait(io, MPLL_STABLE, 0);
    lock(io);
    stopped?;
    transition(io, None);
    if io.read32(CMU_SCFGR) != DIVIDERS {
        return Err(());
    }
    // Preserve stock LRC enabling and protection/access width.
    unlock(io);
    let v = io.read8(CMU_LRCCR);
    io.write8(CMU_LRCCR, v & !RC_STOP);
    io.settle();
    lock(io);
    unlock(io);
    let v = io.read32(CMU_PLLCFGR);
    if v & PLL_SOURCE_HRC == 0 && io.read8(CMU_OSCSTBSR) & UPLL_STABLE != 0 {
        lock(io);
        return Err(()); // UPLL has not yet stopped; shared source unsafe.
    }
    io.write32(CMU_PLLCFGR, (v & !PLL_MASK) | PLL_FIELDS);
    lock(io);
    if io.read32(CMU_PLLCFGR) & PLL_MASK != PLL_FIELDS {
        return Err(());
    }
    unlock(io);
    let v = io.read8(CMU_PLLCR);
    io.write8(CMU_PLLCR, v & !PLL_STOP);
    let ready = wait(io, MPLL_STABLE, MPLL_STABLE);
    lock(io);
    ready?;
    transition(io, Some(CLOCK_SOURCE_MPLL));
    unlock(io);
    let v = io.read16(CMU_PERICKSEL);
    io.write16(CMU_PERICKSEL, v & !PERIPHERAL_CLOCK_MASK);
    lock(io);
    // Fail-stop on unexpected readback rather than publish a guessed clock.
    if io.read8(CMU_CKSWR) & CLOCK_SOURCE_MASK != CLOCK_SOURCE_MPLL
        || io.read32(CMU_SCFGR) != DIVIDERS
        || io.read32(CMU_PLLCFGR) & PLL_MASK != PLL_FIELDS
    {
        return Err(());
    }

    let hrc = if io.read32(HRC_FREQUENCY_READBACK) & HRC_16MHZ_SELECTED != 0 {
        HRC_16MHZ
    } else {
        HRC_20MHZ
    };
    io.write32(HRC_FREQUENCY_WORD, hrc);
    io.write32(
        SYSTEM_FREQUENCY_WORD,
        (hrc / MPLL_INPUT_DIVIDER) * MPLL_MULTIPLIER / MPLL_OUTPUT_DIVIDER,
    );
    Ok(())
}

struct Hardware;
// SAFETY: exclusive startup uses documented register addresses/widths, the
// inherited HRC readback address and aligned clock bookkeeping RAM words.
// Peripheral drivers and interrupts must not access this state concurrently.
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
        for _ in 0..SETTLE_ITERATIONS {
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
        0..=MAX_DIVIDER_SELECTOR => 1 << field,
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
    let sys_clk = unsafe { (SYSTEM_FREQUENCY_WORD as *const u32).read_volatile() };
    let scfgr = CMU_SCFGR as *const u32;
    // SAFETY: startup exclusively owns the aligned, documented 32-bit SCFGR register.
    let v = unsafe { scfgr.read_volatile() };
    let d = |shift: u32| divide((v >> shift) & DIVIDER_READ_MASK);
    Ok(Clocks {
        sys_clk,
        hclk: sys_clk / d(HCLK_SHIFT),
        exclk: sys_clk / d(EXCLK_SHIFT),
        pclk: [
            sys_clk / d(0),
            sys_clk / d(PCLK_FIELD_STRIDE),
            sys_clk / d(2 * PCLK_FIELD_STRIDE),
            sys_clk / d(3 * PCLK_FIELD_STRIDE),
            sys_clk / d(4 * PCLK_FIELD_STRIDE),
        ],
    })
}
