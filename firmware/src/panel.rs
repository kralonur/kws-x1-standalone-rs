//! Async ST7789V transport over the exclusive SPI3 HAL owner. No transfer DMA.
//! Panel commands, pin routing and wire order follow the owner-confirmed driver.
//! Complete command/frame operations hold one mutex; D/C changes fence wire idle.
use embassy_hc32::gpio::{AlternateMode, AlternatePin, Level, OutputPin, Port};
use kws_x1_core::panel;

/// Polls of the DMA channel state before the stop is treated as failed. The
/// channel stops after its current read/write (RM 14.3.11), so this is a
/// generous chosen bound, not a measured or specified time.
const DMA_STOP_POLLS: u32 = 200_000;
trait Bus {
    async fn idle(&mut self) -> bool;
    fn dc(&mut self, data: bool);
    async fn bytes(&mut self, bytes: &[u8]) -> bool;
}
struct Spi {
    driver: embassy_hc32::spi::Spi3,
    dc: OutputPin,
    reset: OutputPin,
    _mosi: AlternatePin,
    _sck: AlternatePin,
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
        if cmd == panel::CMD_SLEEP_OUT && !delay(panel::SLEEP_OUT_MS).await {
            return false;
        }
    }
    command(bus, panel::CMD_DISPLAY_ON, &[]).await
        && command(bus, panel::CMD_MEMORY_ACCESS_CONTROL, &[panel::MADCTL]).await
        && bus.idle().await
}
async fn delay(ms: u32) -> bool {
    embassy_time::Timer::after_millis(u64::from(ms)).await;
    true
}

/// PWC_FCG0 and its DMA2 bit: set while DMA2's function clock is stopped (RM 5.7.13).
const PWC_FCG0: *const u32 = 0x4004_8000 as *const u32;
const FCG0_DMA2_STOPPED: u32 = 1 << 15;
/// DMA2 channel enable and channel state registers: base 0x4005_3400 plus
/// 0x1c and 0x24 (RM 14.5).
const DMA2_CHEN: *mut u32 = 0x4005_341c as *mut u32;
const DMA2_CHSTAT: *const u32 = 0x4005_3424 as *const u32;
/// CHEN[0] and CHSTAT CHACT[0]: channel 0 enabled and active (RM 14.5.8, 14.5.11).
const CHEN0: u32 = 1 << 0;
const CHACT0: u32 = 1 << 16;

fn dma0_idle(enable: u32, status: u32) -> bool {
    enable & CHEN0 == 0 && status & CHACT0 == 0
}

/// Stop only display DMA2 channel0 if its clock is already enabled. RM 14.3.11
/// says disabling terminates after the current read/write; wait for CHACT0 too.
unsafe fn stop_display_dma() -> bool {
    unsafe {
        if PWC_FCG0.read_volatile() & FCG0_DMA2_STOPPED != 0 {
            return true;
        }
        DMA2_CHEN.write_volatile(DMA2_CHEN.read_volatile() & !CHEN0);
        for _ in 0..DMA_STOP_POLLS {
            if dma0_idle(DMA2_CHEN.read_volatile(), DMA2_CHSTAT.read_volatile()) {
                return true;
            }
        }
        false
    }
}

/// PB5 and PB6 pin functions: Func40 SPI3_MOSI and Func43 SPI3_SCK in
/// Func_Grp2 (HC32 Datasheet Tables 2-1 and 2-2).
const SPI3_MOSI: u8 = 40;
const SPI3_SCK: u8 = 43;

/// SPI_CR1: SPE | MSTR | TXMDS | SPIMDS, an enabled 3-wire master that only
/// transmits (RM 27.12.2).
const SPI_CR1: u32 = 1 << 6 | 1 << 3 | 1 << 1 | 1 << 0;
/// SPI_CFG1: MIDI = 101b, 6 SCK + 2 PCLK between accesses; bit 4 is reserved
/// and written as 1 (RM 27.12.3).
const SPI_CFG1: u32 = 0b101 << 28 | 1 << 4;
/// SPI_CFG2: MSSIE | MSSDLE | MIDIE use the CFG1 delays, DSIZE = 0100b (8-bit),
/// MSB first, SS0, MBR = 000b (PCLK1 / 2), CPOL = CPHA = 1 (RM 27.12.5).
const SPI_CFG2: u32 = 1 << 15 | 1 << 14 | 1 << 13 | 0b0100 << 8 | 1 << 1 | 1 << 0;

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
        let mosi = AlternatePin::new(Port::B, 5, SPI3_MOSI, AlternateMode::Digital);
        let sck = AlternatePin::new(Port::B, 6, SPI3_SCK, AlternateMode::Digital);
        let dc = OutputPin::new_high_drive(Port::B, 7, Level::High);
        let reset = OutputPin::new_high_drive(Port::A, 15, Level::High);
        // Preserve PH2's TimerA4 mux and inherited INTE=1 (not INVE/inversion).
        backlight_pin.configure_high_drive_output(Level::Low, true);
        let Ok(driver) = embassy_hc32::spi::Spi3::new(
            hc32f460_pac::Spi3::steal(),
            embassy_hc32::spi::Config {
                cfg1: SPI_CFG1,
                cfg2: SPI_CFG2,
                control: SPI_CR1,
            },
        ) else {
            return false;
        };
        let mut bus = Spi {
            driver,
            dc,
            reset,
            _mosi: mosi,
            _sck: sck,
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

/// Panel size in pixels, as frame buffer indices.
const WIDTH: usize = panel::WIDTH as usize;
const HEIGHT: usize = panel::HEIGHT as usize;

/// CASET/RASET parameters: first and last address, big-endian (ST 9.1.20, 9.1.21).
const fn span(start: u16, length: u16) -> [u8; 4] {
    let [start_high, start_low] = start.to_be_bytes();
    let [end_high, end_low] = (start + length - 1).to_be_bytes();
    [start_high, start_low, end_high, end_low]
}
const COLUMNS: [u8; 4] = span(panel::X_OFFSET, panel::WIDTH);
const ROWS: [u8; 4] = span(panel::Y_OFFSET, panel::HEIGHT);
const ROWS_INVERTED: [u8; 4] = span(panel::Y_OFFSET_INVERTED, panel::HEIGHT);

/// # Safety
/// `frame` must point to 32400 readable RGB565 wire-swapped halfwords. Callback
/// may sample inputs/update model but must not modify the frame during transfer.
async unsafe fn stream(
    bus: &mut impl Bus,
    frame: *const u16,
    mut sample: impl FnMut(),
    orientation: u8,
) -> bool {
    let raset = if orientation == 0 {
        ROWS
    } else {
        ROWS_INVERTED
    };
    if !command(bus, panel::CMD_COLUMN_ADDRESS, &COLUMNS).await
        || !command(bus, panel::CMD_ROW_ADDRESS, &raset).await
        || !command(bus, panel::CMD_MEMORY_WRITE, &[]).await
        || !bus.idle().await
    {
        return false;
    }
    bus.dc(true);
    let mut wire = [0u8; WIDTH * 2];
    for row in 0..HEIGHT {
        sample();
        for column in 0..WIDTH {
            let pixel = unsafe { frame.add(row * WIDTH + column).read_volatile() };
            wire[2 * column..2 * column + 2].copy_from_slice(&pixel.to_le_bytes());
        }
        if !bus.bytes(&wire).await {
            return false;
        }
    }
    bus.idle().await
}

/// Change ST7789V memory access control (MADCTL) orientation:
/// 0: normal landscape ([`panel::MADCTL`])
/// 1: 180-degree inverted landscape ([`panel::MADCTL_INVERTED`])
pub async unsafe fn set_orientation(orientation: u8) -> bool {
    let mut owner = DISPLAY.lock().await;
    let Some(bus) = owner.as_mut() else {
        return false;
    };
    let madctl = if orientation == 0 {
        panel::MADCTL
    } else {
        panel::MADCTL_INVERTED
    };
    command(bus, panel::CMD_MEMORY_ACCESS_CONTROL, &[madctl]).await && bus.idle().await
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
