//! Application clock-profile selection; the HAL owns detection and programming.
//! Vendor reference links and hardware requirements are in [`embassy_hc32::clocks`].
use embassy_hc32::clocks::{self, Clocks, Config, Divider, Dividers, Error, Mpll};

// Retained profile choice: divide nominal HRC by two before VCO multiplication.
const PLL_INPUT_DIVIDER: u8 = 2;
// Retained profile choice: HRC / 2 * 30 gives a 240/300 MHz VCO, within
// RM 4.11.15's 240-480 MHz range.
const PLL_MULTIPLIER: u16 = 30;
// Retained P/Q/R profile: divide VCO by two, preserving 120/150 MHz SYSCLK.
const PLL_OUTPUT_DIVIDERS: [u8; 3] = [2; 3];
// Preserve HCLK, EXCLK and the active peripheral rates. PCLK2 uses /4 rather
// than /2: RM Table 4-1 limits it to 60 MHz, so 150 MHz /2 is invalid.
// /4 yields 30/37.5 MHz; PCLK3 (I2C) remains /4 and PCLK1 (SPI) remains /2.
const BUS_DIVIDERS: Dividers = Dividers {
    hclk: Divider::Div1,
    exclk: Divider::Div2,
    pclk: [
        Divider::Div1,
        Divider::Div2,
        Divider::Div4,
        Divider::Div4,
        Divider::Div2,
    ],
};

/// Apply the application profile and return checked nominal clock frequencies.
///
/// # Safety
/// Call once during exclusive startup, after SRAM timing is configured and
/// before interrupts or peripheral drivers run. See `clocks::configure`.
pub unsafe fn init() -> Result<Clocks, Error> {
    let config = Config {
        mpll: Mpll::new(PLL_INPUT_DIVIDER, PLL_MULTIPLIER, PLL_OUTPUT_DIVIDERS)?,
        dividers: BUS_DIVIDERS,
    };
    // SAFETY: the caller establishes the HAL's exclusive-startup/SRAM contract.
    unsafe { clocks::configure(config) }
}
