//! Independent ADC1/PGA acquisition. ADC2 remains owned by the line task;
//! the bus monitor is not used for ripple.

use embassy_hc32::adc::{Adc1, AmplifiedPin, CaptureError, SampleTime};
use embassy_hc32::dma::{self, Unit};
use embassy_hc32::gpio::Port;
use embassy_time::{Duration, Timer};
use firmware_core::buttons::View;
use firmware_core::ripple::{
    MIN_SAMPLE_CYCLES, PeakHistory, Reading, SAMPLE_COUNT, Sampling, Timebase, analyze,
};

/// Dedicated ripple DMA controller; the panel uses DMA2.
pub const DMA_UNIT: Unit = Unit::Dma1;
/// The established acquisition uses DMA1 channel 0.
pub const DMA_CHANNEL: usize = 0;
/// Established ripple pickup: ADC1_IN2 / PA2, amplified at twofold gain.
const INPUT_PIN: u8 = 2;
/// Start at the fastest sampling: 20 cycles = ceil((1 kΩ + 6 kΩ) * 7 pF *
/// ln(2^14) * 38.25 MHz + 1), from datasheet table 3-38's settling formula.
/// The 1 kΩ effective-source budget is an assumption, not a PGA guarantee;
/// 38.25 MHz includes +2% HRC tolerance on the faster 37.5 MHz profile.
/// Slower timebases only lengthen it.
const FAST_SAMPLE_TIME: SampleTime =
    SampleTime::new(MIN_SAMPLE_CYCLES).expect("20 cycles is in range");
/// A chosen generous capture bound, in milliseconds, not a sample-rate claim.
/// The slowest capture, 1024 samples of 268 cycles at 30 MHz, takes 9.2 ms.
const CAPTURE_TIMEOUT: Duration = Duration::from_millis(20);
/// UI capture cadence, in milliseconds; no acquisition between captures.
const UPDATE_PERIOD: Duration = Duration::from_millis(100);

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
            FAST_SAMPLE_TIME,
            CAPTURE_TIMEOUT,
        )
        .await?)
    }
}

/// `clock_hz` is ADC1's clock, which sets each timebase's sampling.
#[embassy_executor::task]
pub async fn run(mut adc: Option<Adc1>, clock_hz: u32) {
    let sender = crate::RIPPLE.sender();
    let mut samples = [0u16; SAMPLE_COUNT];
    let mut sample_cycles = MIN_SAMPLE_CYCLES;
    let mut timebase = Timebase::default();
    let mut history = PeakHistory::default();
    loop {
        let live = crate::BUTTON_STATE
            .try_get()
            .filter(|input| input.page.view() == View::Ripple && !input.ripple_hold);
        match live {
            Some(input) => {
                // Peaks at another sample rate are not comparable.
                if input.ripple_timebase != timebase {
                    timebase = input.ripple_timebase;
                    history.clear();
                }
                let reading = match (adc.as_mut(), timebase.sampling(clock_hz)) {
                    (Some(adc), Some(sampling)) => {
                        capture(adc, &mut samples, &mut sample_cycles, timebase, sampling)
                            .await
                            .map(|mut reading| {
                                history.add(&mut reading);
                                reading
                            })
                    }
                    _ => None,
                };
                sender.send(reading);
            }
            // A smoothed readout restarts with the next live capture.
            None => history.clear(),
        }
        Timer::after(UPDATE_PERIOD).await;
    }
}

/// Capture with `sampling`, first switching the sample time if it changed.
async fn capture(
    adc: &mut Adc1,
    samples: &mut [u16; SAMPLE_COUNT],
    sample_cycles: &mut u8,
    timebase: Timebase,
    sampling: Sampling,
) -> Option<Reading> {
    if *sample_cycles != sampling.sample_cycles {
        let sample_time = SampleTime::new(sampling.sample_cycles)?;
        adc.set_sample_time(sample_time).ok()?;
        *sample_cycles = sampling.sample_cycles;
    }
    // SAFETY: always awaited, never leaked; no access to the task-owned
    // buffer until capture has quiesced DMA.
    unsafe { adc.capture(samples).await }.ok()?;
    Some(analyze(samples, timebase, sampling.window))
}
