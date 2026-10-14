//! Independent ADC1/PGA acquisition. ADC2 remains owned by the line task;
//! the bus monitor is not used for ripple.

use embassy_hc32::adc::{Adc1, AmplifiedPin, CaptureError, SampleTime};
use embassy_hc32::dma::{self, Unit};
use embassy_hc32::gpio::Port;
use embassy_time::{Duration, Timer};
use firmware_core::buttons::View;
use firmware_core::ripple::{SAMPLE_COUNT, analyze};

/// Dedicated ripple DMA controller; the panel uses DMA2.
pub const DMA_UNIT: Unit = Unit::Dma1;
/// The established acquisition uses DMA1 channel 0.
pub const DMA_CHANNEL: usize = 0;
/// Established ripple pickup: ADC1_IN2 / PA2, amplified at twofold gain.
const INPUT_PIN: u8 = 2;
/// ADC sampling cycles: ceil((1 kΩ + 6 kΩ) * 7 pF * ln(2^14) *
/// 38.25 MHz + 1) = 20, from datasheet table 3-38's settling formula.
/// The 1 kΩ effective-source budget is an assumption, not a PGA guarantee;
/// 38.25 MHz includes +2% HRC tolerance on the faster 37.5 MHz profile.
const SAMPLE_CYCLES: u8 = 20;
const SAMPLE_TIME: SampleTime = SampleTime::new(SAMPLE_CYCLES).expect("20 cycles is in range");
/// Successive comparison takes 13 ADCLK cycles at 12 bits (RM 16.3.8).
const COMPARISON_CYCLES: u16 = 13;
/// A chosen generous capture bound, in milliseconds, not a sample-rate claim.
const CAPTURE_TIMEOUT: Duration = Duration::from_millis(20);
/// UI capture cadence, in milliseconds; no acquisition between captures.
const UPDATE_PERIOD: Duration = Duration::from_millis(100);

/// Nominal time per division for the same sampling configuration used below.
pub fn nominal_tenths_us_per_div(adc_clock_hz: u32) -> Option<u32> {
    firmware_core::ripple::nominal_tenths_us_per_div(
        adc_clock_hz,
        u16::from(SAMPLE_CYCLES) + COMPARISON_CYCLES,
    )
}

/// # Safety
/// Call once during startup. Hands ADC1, PA2, PWC FCG3.ADC1, DMA1 and its
/// trigger registers to this task. No other code may configure them.
pub async unsafe fn new() -> Result<Adc1, CaptureError> {
    // SAFETY: forwarded from this function's exclusive-startup contract.
    unsafe {
        let channels = dma::split(DMA_UNIT)?;
        let [channel, ..] = channels;
        let pin = AmplifiedPin::new(Port::A, INPUT_PIN)?;
        Ok(Adc1::new(
            hc32f460_pac::Adc1::steal(),
            pin,
            channel,
            SAMPLE_TIME,
            CAPTURE_TIMEOUT,
        )
        .await?)
    }
}

#[embassy_executor::task]
pub async fn run(mut adc: Option<Adc1>) {
    let sender = crate::RIPPLE.sender();
    let mut samples = [0u16; SAMPLE_COUNT];
    loop {
        if crate::BUTTON_STATE
            .try_get()
            .is_some_and(|input| input.page.view() == View::Ripple && !input.ripple_hold)
        {
            let reading = match adc.as_mut() {
                Some(adc) => {
                    // SAFETY: always awaited, never leaked; no access to the
                    // task-owned buffer until capture has quiesced DMA.
                    unsafe { adc.capture(&mut samples).await }
                        .ok()
                        .map(|()| analyze(&samples))
                }
                None => None,
            };
            sender.send(reading);
        }
        Timer::after(UPDATE_PERIOD).await;
    }
}
