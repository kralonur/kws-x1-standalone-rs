//! Async ST7735 transport over the exclusive SPI3 HAL owner. No transfer DMA.
//! Panel commands, pin routing and wire order follow the owner-confirmed driver.
//! Complete command/frame operations hold one mutex; D/C changes fence wire idle.
use embassy_hc32::gpio::{AlternateMode, AlternatePin, Level, OutputPin, Port};
use kws_x1_core::panel;

const BUDGET: u32 = 200_000;
trait Bus {
    async fn idle(&mut self) -> bool;
    fn dc(&mut self, data: bool);
    async fn bytes(&mut self, bytes: &[u8]) -> bool;
}
struct Spi {
    driver: embassy_hc32::spi::Spi3,
    dc: OutputPin,
    reset: OutputPin,
    _sck: AlternatePin,
    _mosi: AlternatePin,
    _backlight_pin: AlternatePin,
}
static DISPLAY: embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    Option<Spi>,
> = embassy_sync::mutex::Mutex::new(None);
impl Bus for Spi {
    async fn idle(&mut self) -> bool {
        self.driver.flush().await.is_ok()
    }
    fn dc(&mut self, data: bool) {
        if data {
            self.dc.set_high();
        } else {
            self.dc.set_low();
        }
    }
    async fn bytes(&mut self, bytes: &[u8]) -> bool {
        self.driver.write(bytes).await.is_ok()
    }
}

async fn command(bus: &mut impl Bus, cmd: u8, bytes: &[u8]) -> bool {
    if !bus.idle().await {
        return false;
    }
    bus.dc(false);
    if !bus.bytes(&[cmd]).await {
        return false;
    }
    if !bytes.is_empty() {
        if !bus.idle().await {
            return false;
        }
        bus.dc(true);
        if !bus.bytes(bytes).await {
            return false;
        }
    }
    true
}

async fn sequence<F: core::future::Future<Output = bool>>(
    bus: &mut impl Bus,
    mut delay: impl FnMut(u32) -> F,
) -> bool {
    for &(cmd, data) in panel::INIT {
        if !command(bus, cmd, data).await {
            return false;
        }
        if cmd == 0x11 && !delay(120).await {
            return false;
        }
    }
    command(bus, 0x29, &[]).await && command(bus, 0x36, &[0x60]).await && bus.idle().await
}
async fn delay(ms: u32) -> bool {
    embassy_time::Timer::after_millis(u64::from(ms)).await;
    true
}

fn dma0_idle(enable: u32, status: u32) -> bool {
    enable & 1 == 0 && status & (1 << 16) == 0
}

/// Stop only display DMA2 channel0 if its clock is already enabled. RM14.3.11
/// says disabling terminates after the current read/write; wait for CHACT0 too.
unsafe fn stop_display_dma() -> bool {
    unsafe {
        if (0x40048000 as *const u32).read_volatile() & (1 << 15) != 0 {
            return true;
        }
        let enable = 0x4005341c as *mut u32;
        enable.write_volatile(enable.read_volatile() & !1);
        for _ in 0..BUDGET {
            if dma0_idle(
                enable.read_volatile(),
                (0x40053424 as *const u32).read_volatile(),
            ) {
                return true;
            }
        }
        false
    }
}

/// # Safety
/// Requires privileged initialized clock/PORT and running Embassy time driver.
/// PB5/PB6/PB7/PA15, PH2 GPIO drive and SPI3 are configured. Backlight timer
/// setup is separate. `backlight_pin` must be the owned PH2 TimerA4/function4
/// pin; its mux is preserved. Inherited display DMA2 channel0 is stopped if clocked.
pub async unsafe fn init(mut backlight_pin: AlternatePin) -> bool {
    let mut owner = DISPLAY.lock().await;
    if owner.is_some() {
        return false;
    }
    unsafe {
        if !stop_display_dma() {
            return false;
        }
        let sck = AlternatePin::new(Port::B, 5, 0x28, AlternateMode::Digital);
        let mosi = AlternatePin::new(Port::B, 6, 0x2b, AlternateMode::Digital);
        let dc = OutputPin::new_high_drive(Port::B, 7, Level::High);
        let reset = OutputPin::new_high_drive(Port::A, 15, Level::High);
        // Preserve PH2's TimerA4 mux and legacy INTE=1 (not INVE/inversion).
        backlight_pin.configure_high_drive_output(Level::Low, true);
        let Ok(driver) = embassy_hc32::spi::Spi3::new(
            hc32f460_pac::Spi3::steal(),
            embassy_hc32::spi::Config {
                cfg1: 0x5000_0010,
                cfg2: 0xe403,
                control: 0x4b,
            },
        ) else {
            return false;
        };
        let mut bus = Spi {
            driver,
            dc,
            reset,
            _sck: sck,
            _mosi: mosi,
            _backlight_pin: backlight_pin,
        };
        bus.reset.set_low();
        if !delay(10).await {
            return false;
        }
        bus.reset.set_high();
        if !delay(10).await {
            return false;
        }
        if !sequence(&mut bus, delay).await {
            return false;
        }
        *owner = Some(bus);
        true
    }
}

/// # Safety
/// `frame` must point to 32400 readable RGB565 wire-swapped halfwords. Callback
/// may sample inputs/update model but must not modify the frame during transfer.
async unsafe fn stream(
    bus: &mut impl Bus,
    frame: *const u16,
    mut sample: impl FnMut(),
    orientation: u8,
) -> bool {
    let raset: [u8; 4] = if orientation == 0 {
        [0, 53, 0, 187]
    } else {
        [0, 52, 0, 186]
    };
    if !command(bus, 0x2a, &[0, 40, 1, 23]).await
        || !command(bus, 0x2b, &raset).await
        || !command(bus, 0x2c, &[]).await
        || !bus.idle().await
    {
        return false;
    }
    bus.dc(true);
    let mut wire = [0u8; 480];
    for row in 0..135 {
        sample();
        for column in 0..240 {
            let pixel = unsafe { frame.add(row * 240 + column).read_volatile() };
            wire[2 * column..2 * column + 2].copy_from_slice(&pixel.to_le_bytes());
        }
        if !bus.bytes(&wire).await {
            return false;
        }
    }
    bus.idle().await
}

/// Change ST7735 memory access control (MADCTL) orientation:
/// 0: normal landscape (0x60)
/// 1: 180-degree inverted landscape (0xa0)
pub async unsafe fn set_orientation(orientation: u8) -> bool {
    let mut owner = DISPLAY.lock().await;
    let Some(bus) = owner.as_mut() else {
        return false;
    };
    let madctl = if orientation == 0 { 0x60 } else { 0xa0 };
    command(bus, 0x36, &[madctl]).await && bus.idle().await
}

/// # Safety
/// Same initialized hardware/frame requirements as init/stream.
pub async unsafe fn flush(frame: *const u16, sample: impl FnMut(), orientation: u8) -> bool {
    let mut owner = DISPLAY.lock().await;
    let Some(bus) = owner.as_mut() else {
        return false;
    };
    unsafe { stream(bus, frame, sample, orientation).await }
}
