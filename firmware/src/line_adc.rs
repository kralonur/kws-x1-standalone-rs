//! D+, D−, CC1 and CC2 line voltages, measured by ADC2 against the internal
//! reference (RM 16.3.2). ADC2 is independent of the INA226 bus monitor and
//! the PD controller; nothing here drives the lines. Between scans, D−'s
//! edges can also be watched for another device's FCP/SCP traffic.

use embassy_hc32::adc::{
    Adc2, AdcPin, Average, FULL_SCALE, Input, InternalReference, SampleTime, Source,
};
use embassy_hc32::gpio::Port;
use embassy_hc32::intc::{EirqTrigger, Line, Route};
use embassy_time::{Duration, Timer};

use crate::dm_sniff;

/// The four line voltages in mV, scaled by the measured reference
/// ([`REFERENCE_MV`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LineVoltages {
    pub dp_mv: u16,
    pub dm_mv: u16,
    pub cc1_mv: u16,
    pub cc2_mv: u16,
}

/// The line pins, as the source firmware reads them.
const CC1_PIN: (Port, u8) = (Port::A, 6);
const CC2_PIN: (Port, u8) = (Port::A, 7);
const DM_PIN: (Port, u8) = (Port::B, 0);
const DP_PIN: (Port, u8) = (Port::B, 1);

/// Sampling times in ADC clock cycles (RM 16.4.11), the source firmware's:
/// long for the divided CC inputs, shorter for D+/D− and the reference.
/// Checked when compiled: an out-of-range value fails the build.
const CC_SAMPLE_TIME: SampleTime = SampleTime::new(255).expect("255 cycles is in range");
const DATA_SAMPLE_TIME: SampleTime = SampleTime::new(40).expect("40 cycles is in range");
const REFERENCE_SAMPLE_TIME: SampleTime = SampleTime::new(80).expect("80 cycles is in range");
/// Each result is the mean of four conversions, as the source firmware sets.
const AVERAGE: Average = Average::Four;
/// One averaged scan is about 3000 ADC clocks (four times the sampling
/// cycles above plus 12-bit conversions), 0.1 ms at a 30 MHz PCLK2, so this
/// is a wide bound.
const SCAN_TIMEOUT: Duration = Duration::from_millis(3);
/// While D− is watched, a scan first waits for a gap in its traffic,
/// checking this often and at most this many times. Chosen: a few
/// transactions' length, so steady traffic delays a scan by 50 ms at most.
const QUIET_POLL: Duration = Duration::from_millis(1);
const QUIET_POLLS: u32 = 50;

/// Internal reference in mV. The manual gives about 1100 mV (RM 16.3.2),
/// with no tolerance and no per-chip calibration value. The stock firmware
/// uses 1083 mV, which a multimeter check on one unit matched: D+ and D−
/// driven at 0.55-3.23 V fit a gain-only scale (no offset) of 1083.7 mV,
/// every point then within 5 mV. Other units' references may differ by a
/// few percent.
pub const REFERENCE_MV: u16 = 1083;
/// Software scale of each line: the CC inputs read half the line voltage
/// (the source firmware doubles them), D+/D− are direct.
const CC_SCALE: u32 = 2;
const DATA_SCALE: u32 = 1;

/// Owns ADC2, its four analog line pins and the internal reference path.
pub struct LineAdc {
    /// CC1, CC2, D−, D+, then the internal reference.
    adc: Adc2<5>,
    /// The NVIC line and priority D−'s edge interrupt uses.
    edge_line: Line,
    edge_priority: u8,
    /// D−'s edge interrupt route, while its edges are watched.
    edges: Option<Route>,
}

impl LineAdc {
    /// Make the line pins analog inputs, route the internal reference to
    /// ADC2 and set up one averaged scan of the lines and the reference.
    ///
    /// # Errors
    /// Returns the HAL error if the inherited ADC conversion cannot be stopped
    /// or the scan configuration is invalid.
    ///
    /// # Safety
    /// Call once during startup; route ADC2's end-of-scan event before the
    /// first [`Self::read`]. The caller gives up ADC2, these four pins, PWC
    /// FCG3's ADC2/CMP gates, PWCMR.ADBUFE and CMP RVADC to this owner; no
    /// other code may configure them. It also gives up external-interrupt
    /// channel [`dm_sniff::EIRQ`] and `edge_line`, whose handler must call
    /// [`dm_sniff::on_interrupt`].
    pub async unsafe fn new(
        adc: hc32f460_pac::Adc2,
        edge_line: Line,
        edge_priority: u8,
    ) -> Result<Self, embassy_hc32::adc::Error> {
        // SAFETY: forwarded from this function's contract.
        unsafe {
            let pin = |(port, pin), sample_time| Input {
                source: Source::Pin(
                    AdcPin::new(port, pin).expect("each line pin is an ADC12 input"),
                ),
                sample_time,
            };
            let reference = Input {
                source: Source::Internal(InternalReference::new().await),
                sample_time: REFERENCE_SAMPLE_TIME,
            };
            let inputs = [
                pin(CC1_PIN, CC_SAMPLE_TIME),
                pin(CC2_PIN, CC_SAMPLE_TIME),
                pin(DM_PIN, DATA_SAMPLE_TIME),
                pin(DP_PIN, DATA_SAMPLE_TIME),
                reference,
            ];
            Ok(Self {
                adc: Adc2::new(adc, inputs, Some(AVERAGE), SCAN_TIMEOUT)?,
                edge_line,
                edge_priority,
                edges: None,
            })
        }
    }

    /// Watch D−'s edges for another device's FCP/SCP traffic, or stop.
    /// Scans and [`Self::dm_input`] stop watching while they run.
    pub fn listen(&mut self, on: bool) {
        if on == self.edges.is_some() {
            return;
        }
        // Without a decoder, nothing would catch an edge storm.
        if !on || !dm_sniff::start() {
            // Errata 2.4.2: the NVIC first (dropping the route), then the
            // pin's interrupt.
            self.edges = None;
            dm_analog();
            return;
        }
        // Errata 2.4.2: the pin's interrupt first, then the NVIC. RM 9.4.11:
        // digital input on, EXTI on; output and pull-up stay off.
        dm_control(|w| {
            w.ddis().clear_bit();
            w.inte().set_bit()
        });
        embassy_hc32::intc::configure_eirq(dm_sniff::EIRQ, EirqTrigger::Both);
        embassy_hc32::intc::eirq_clear(dm_sniff::EIRQ);
        // SAFETY: this owner holds the channel and `edge_line`, whose handler
        // the constructor's caller installed.
        let route = unsafe {
            Route::new(
                u16::from(dm_sniff::EIRQ),
                self.edge_line,
                self.edge_priority,
            )
        };
        match route {
            Ok(route) => self.edges = Some(route),
            // Never leave the pin digital without a watcher.
            Err(_) => dm_analog(),
        }
    }

    /// Temporarily borrow the idle D− ADC pin as a fast digital receiver.
    /// The borrow prevents conversions until the saved pin mode is restored.
    pub fn dm_input(&mut self) -> DmInput<'_> {
        // This meter is about to drive D−; its own edges are not traffic.
        self.listen(false);
        // SAFETY: this owner already exclusively owns PB0. Only that pin's
        // control and the shared, interrupt-guarded write lock are accessed.
        let gpio = unsafe { hc32f460_pac::Gpio::steal() };
        let saved = gpio.pcrb0().read().bits();
        // RM 9.4.11: enable the digital input with normal polarity; keep the
        // output, pull-up and EXTI off. Other pin controls are preserved.
        with_unlocked(&gpio, |gpio| {
            gpio.pcrb0().modify(|_, w| {
                w.ddis().clear_bit();
                w.inve().clear_bit();
                w.poute().clear_bit();
                w.puu().clear_bit();
                w.inte().clear_bit()
            });
        });
        DmInput {
            gpio,
            saved,
            _adc: core::marker::PhantomData,
        }
    }

    /// Run one scan, returning line voltages and the reference count for
    /// ripple normalization. `None` on timeout or an unusable reference.
    pub async fn read(&mut self) -> Option<(LineVoltages, u16)> {
        let listening = self.edges.is_some();
        if listening {
            // Pausing mid-transaction would lose it: wait for a gap.
            for _ in 0..QUIET_POLLS {
                if dm_sniff::quiet() {
                    break;
                }
                Timer::after(QUIET_POLL).await;
            }
        }
        self.listen(false);
        let scan = self.adc.read().await;
        self.listen(listening);
        let [cc1, cc2, dm, dp, reference] = scan.ok()?;
        let line = |count: u16, scale| millivolts(count, scale, reference);
        Some((
            LineVoltages {
                dp_mv: line(dp, DATA_SCALE)?,
                dm_mv: line(dm, DATA_SCALE)?,
                cc1_mv: line(cc1, CC_SCALE)?,
                cc2_mv: line(cc2, CC_SCALE)?,
            },
            reference,
        ))
    }
}

/// Modify PB0's (D−'s) pin control.
fn dm_control(
    change: impl FnOnce(&mut hc32f460_pac::gpio::pcrb0::W) -> &mut hc32f460_pac::gpio::pcrb0::W,
) {
    // SAFETY: the line ADC exclusively owns PB0 and its interrupt channel.
    // Only that pin's control and the shared, interrupt-guarded write lock
    // are accessed.
    let gpio = unsafe { hc32f460_pac::Gpio::steal() };
    with_unlocked(&gpio, |gpio| {
        gpio.pcrb0().modify(|_, w| change(w));
    });
}

/// Return D− to its analog input mode, unwatched: RM 16.5.4 wants the
/// digital input off for the ADC.
fn dm_analog() {
    dm_control(|w| {
        w.inte().clear_bit();
        w.ddis().set_bit()
    });
    dm_sniff::stop();
}

/// GPIO write-protect key (RM 9.4.10, `PWPR.WP`); `PWPR.WE` set unlocks.
const GPIO_WRITE_KEY: u8 = 0xa5;

pub struct DmInput<'a> {
    gpio: hc32f460_pac::Gpio,
    saved: u16,
    _adc: core::marker::PhantomData<&'a mut LineAdc>,
}

impl DmInput<'_> {
    pub fn high(&self) -> bool {
        // PB0 is D−.
        self.gpio.pidrb().read().pin00().bit_is_set()
    }
}

/// Run `change` with the GPIO configuration registers unlocked.
fn with_unlocked(gpio: &hc32f460_pac::Gpio, change: impl FnOnce(&hc32f460_pac::Gpio)) {
    cortex_m::interrupt::free(|_| {
        // SAFETY: the key is the documented `WP` value; the critical section
        // keeps this unlock from interleaving with the HAL's GPIO unlocks.
        gpio.pwpr()
            .write(|w| unsafe { w.wp().bits(GPIO_WRITE_KEY) }.we().set_bit());
        change(gpio);
        // SAFETY: as above; `WE` clear relocks.
        gpio.pwpr()
            .write(|w| unsafe { w.wp().bits(GPIO_WRITE_KEY) });
    });
}

impl Drop for DmInput<'_> {
    fn drop(&mut self) {
        let saved = self.saved;
        // SAFETY: restores the exact control word read in `dm_input`.
        with_unlocked(&self.gpio, |gpio| {
            gpio.pcrb0().write(|w| unsafe { w.bits(saved) });
        });
    }
}

/// `count * scale` in mV, normalized by the internal reference count and
/// rounded to the nearest mV: `count * scale * REFERENCE_MV / reference`.
/// A zero or saturated reference gives `None`, not a plausible-looking value.
fn millivolts(count: u16, scale: u32, reference: u16) -> Option<u16> {
    if reference == 0 || reference >= FULL_SCALE {
        return None;
    }
    let reference = u32::from(reference);
    let scaled = u32::from(count.min(FULL_SCALE)) * scale * u32::from(REFERENCE_MV);
    u16::try_from((scaled + reference / 2) / reference).ok()
}
