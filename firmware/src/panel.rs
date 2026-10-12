//! Async ST7789V transport over the exclusive SPI3 HAL owner. Commands go
//! out byte by byte, pixels through a DMA channel ([`DMA_UNIT`] channel
//! [`DMA_CHANNEL`], the one the original firmware used for the panel).
//! Panel commands, pin routing and wire order follow the owner-confirmed driver.
//! Complete command/frame operations hold one mutex; D/C changes fence wire idle.
use embassy_hc32::dma::{self, Unit};
use embassy_hc32::gpio::{AlternateMode, AlternatePin, Level, OutputPin, Port};
use firmware_core::panel;

/// The DMA unit and channel that send pixels. The channel's completion
/// event and the unit's error event must be routed to handlers that call
/// [`dma::on_interrupt`] with the unit.
pub const DMA_UNIT: Unit = Unit::Dma2;
pub const DMA_CHANNEL: usize = 0;

trait Bus {
    async fn idle(&mut self) -> bool;
    fn dc(&mut self, data: bool);
    async fn bytes(&mut self, bytes: &[u8]) -> bool;
    /// Send pixel data without the CPU copying each byte.
    async fn pixels(&mut self, bytes: &[u8]) -> bool;
}
struct Spi {
    driver: embassy_hc32::spi::Spi3,
    dma: dma::Channel,
    dc: OutputPin,
    reset: OutputPin,
    _mosi: AlternatePin,
    _sck: AlternatePin,
    _backlight_pin: AlternatePin,
    /// [`row_hash`] of each row the panel holds, or `None` when the panel's
    /// contents are unknown and the next flush sends every row.
    shown: Option<[u32; HEIGHT]>,
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
    async fn pixels(&mut self, bytes: &[u8]) -> bool {
        self.driver.write_dma(&mut self.dma, bytes).await.is_ok()
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

/// PB5 and PB6 pin functions: Func40 SPI3_MOSI and Func43 SPI3_SCK in
/// Func_Grp2 (HC32 Datasheet Tables 2-1 and 2-2).
const SPI3_MOSI: u8 = 40;
const SPI3_SCK: u8 = 43;

/// SPI_CR1: SPE | MSTR | TXMDS | SPIMDS, an enabled 3-wire master that only
/// transmits (RM 27.12.2).
const SPI_CR1: u32 = 1 << 6 | 1 << 3 | 1 << 1 | 1 << 0;
/// SPI_CFG1: no access delay (MIDI) is set, as CFG2 leaves it off; bit 4 is
/// reserved and written as 1 (RM 27.12.3).
const SPI_CFG1: u32 = 1 << 4;
/// SPI_CFG2: MSSIE | MSSDLE use the CFG1 delays, DSIZE = 0100b (8-bit),
/// MSB first, SS0, MBR = 000b (PCLK1 / 2), CPOL = CPHA = 1 (RM 27.12.5).
/// MIDIE is clear for the shortest gap between frames (1 SCK + 2 PCLK, at
/// least 3 SCK + 2 PCLK per Errata 4.3.1); the original 6 SCK gap made
/// the panel's full redraws slower, and the ST7789V needs no gap.
const SPI_CFG2: u32 = 1 << 15 | 1 << 14 | 0b0100 << 8 | 1 << 1 | 1 << 0;

/// Reset pulse and settle time, in ms. Retained from the original driver;
/// the ST7789V needs a 10 µs pulse and 5 ms before the next command.
const RESET_MS: u32 = 10;

/// Pulse the panel's reset line and wait for it to settle.
async fn hard_reset(bus: &mut Spi) {
    bus.reset.set_low();
    delay(RESET_MS).await;
    bus.reset.set_high();
    delay(RESET_MS).await;
}

/// # Safety
/// Requires privileged initialized clock/PORT and running Embassy time driver.
/// PB5/PB6/PB7/PA15, PH2 GPIO drive and SPI3 are configured. Backlight timer
/// setup is separate. `backlight_pin` must be the owned PH2 TimerA4/function4
/// pin; its mux is preserved. [`DMA_UNIT`] is given to the panel, which stops
/// any channel an earlier image left running before SPI3 is touched.
pub async unsafe fn init(mut backlight_pin: AlternatePin) -> bool {
    let mut owner = DISPLAY.lock().await;
    if owner.is_some() {
        return false;
    }
    unsafe {
        let Some(channel) = dma::split(DMA_UNIT)
            .ok()
            .and_then(|channels| channels.into_iter().nth(DMA_CHANNEL))
        else {
            return false;
        };
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
            dma: channel,
            dc,
            reset,
            _mosi: mosi,
            _sck: sck,
            _backlight_pin: backlight_pin,
            shown: None,
        };
        hard_reset(&mut bus).await;
        if !sequence(&mut bus, delay).await {
            return false;
        }
        *owner = Some(bus);
        true
    }
}

/// Bytes in one RGB565 panel row.
const ROW_BYTES: usize = panel::WIDTH as usize * 2;
// `row_hash` reads whole 32-bit words; a partial word at a row's end would be skipped.
const _: () = assert!(ROW_BYTES.is_multiple_of(4));
/// Panel rows, as frame row indices.
const HEIGHT: usize = panel::HEIGHT as usize;

/// FNV-1a 32-bit offset basis and prime (FNV specification,
/// draft-eastlake-fnv), applied a 32-bit word at a time instead of a byte:
/// byte steps measured 13 ms per frame. Each step is a bijection (XOR, then
/// multiplying by an odd prime), so a row with one changed word always hashes
/// differently; other changes match the old hash only by a 1 in 2^32 chance,
/// and then the row stays stale only until it changes again.
const FNV_OFFSET: u32 = 0x811c_9dc5;
const FNV_PRIME: u32 = 0x0100_0193;

/// Word-wise FNV-1a hash of one row's wire bytes.
fn row_hash(row: &[u8]) -> u32 {
    let (words, _) = row.as_chunks::<4>();
    words.iter().fold(FNV_OFFSET, |hash, &word| {
        (hash ^ u32::from_le_bytes(word)).wrapping_mul(FNV_PRIME)
    })
}

/// CASET/RASET parameters: first and last address, big-endian (ST 9.1.20, 9.1.21).
const fn span(start: u16, length: u16) -> [u8; 4] {
    let [start_high, start_low] = start.to_be_bytes();
    let [end_high, end_low] = (start + length - 1).to_be_bytes();
    [start_high, start_low, end_high, end_low]
}
const COLUMNS: [u8; 4] = span(panel::X_OFFSET, panel::WIDTH);

/// Landscape orientation of the displayed frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Orientation {
    /// Normal landscape ([`panel::MADCTL`], rows from [`panel::Y_OFFSET`]).
    Normal,
    /// 180-degree inverted landscape ([`panel::MADCTL_INVERTED`]).
    Inverted,
}

impl Orientation {
    pub fn flipped(self) -> Self {
        match self {
            Self::Normal => Self::Inverted,
            Self::Inverted => Self::Normal,
        }
    }

    fn madctl(self) -> u8 {
        match self {
            Self::Normal => panel::MADCTL,
            Self::Inverted => panel::MADCTL_INVERTED,
        }
    }

    /// Controller row address of the frame's first row.
    fn first_row(self) -> u16 {
        match self {
            Self::Normal => panel::Y_OFFSET,
            Self::Inverted => panel::Y_OFFSET_INVERTED,
        }
    }
}

/// Write `rows` (whole wire-order rows) to the panel from controller row
/// address `first_row`.
async fn send_rows(bus: &mut impl Bus, first_row: u16, rows: &[u8]) -> bool {
    let count = (rows.len() / ROW_BYTES) as u16;
    if !command(bus, panel::CMD_COLUMN_ADDRESS, &COLUMNS).await
        || !command(bus, panel::CMD_ROW_ADDRESS, &span(first_row, count)).await
        || !command(bus, panel::CMD_MEMORY_WRITE, &[]).await
        || !bus.idle().await
    {
        return false;
    }
    bus.dc(true);
    bus.pixels(rows).await
}

/// Send the rows whose [`row_hash`] differs from `shown`, in runs of
/// adjacent changed rows, and leave `shown` holding the frame's hashes.
/// Every row goes out while `shown` is `None`, and a failed transfer leaves
/// it `None`.
///
/// # Safety
/// `frame` must point to [`panel::PIXELS`] readable RGB565 wire-swapped
/// halfwords that nothing modifies during the transfer.
async unsafe fn stream(
    bus: &mut impl Bus,
    frame: *const u16,
    orientation: Orientation,
    shown: &mut Option<[u32; HEIGHT]>,
) -> bool {
    // SAFETY: the caller guarantees `frame` holds PIXELS halfwords that stay
    // unmodified for this transfer. They are already in wire byte order, so
    // their memory bytes go out unchanged.
    let wire = unsafe { core::slice::from_raw_parts(frame.cast::<u8>(), panel::PIXELS * 2) };
    let previous = shown.take();
    let mut hashes = [0; HEIGHT];
    for (hash, row) in hashes.iter_mut().zip(wire.chunks(ROW_BYTES)) {
        *hash = row_hash(row);
    }
    let changed = |row: usize| previous.is_none_or(|previous| previous[row] != hashes[row]);
    let mut row = 0;
    while row < HEIGHT {
        if !changed(row) {
            row += 1;
            continue;
        }
        let start = row;
        while row < HEIGHT && changed(row) {
            row += 1;
        }
        let first_row = orientation.first_row() + start as u16;
        if !send_rows(bus, first_row, &wire[start * ROW_BYTES..row * ROW_BYTES]).await {
            return false;
        }
    }
    if !bus.idle().await {
        return false;
    }
    *shown = Some(hashes);
    true
}

/// Change ST7789V memory access control (MADCTL) orientation.
pub async unsafe fn set_orientation(orientation: Orientation) -> bool {
    let mut owner = DISPLAY.lock().await;
    let Some(bus) = owner.as_mut() else {
        return false;
    };
    // The new orientation shows the old contents flipped: send every row.
    bus.shown = None;
    command(
        bus,
        panel::CMD_MEMORY_ACCESS_CONTROL,
        &[orientation.madctl()],
    )
    .await
        && bus.idle().await
}

/// # Safety
/// Same initialized hardware/frame requirements as init/stream.
pub async unsafe fn flush(frame: *const u16, orientation: Orientation) -> bool {
    let mut owner = DISPLAY.lock().await;
    let Some(bus) = owner.as_mut() else {
        return false;
    };
    let mut shown = bus.shown.take();
    let sent = unsafe { stream(bus, frame, orientation, &mut shown).await };
    bus.shown = shown;
    if !sent {
        restart(bus, orientation).await;
    }
    sent
}

/// Bring the panel back after a failed transfer, so the next flush draws
/// again: clear the SPI fault, then reset and re-initialize the panel, which
/// may hold a partial byte or command. Leaves `shown` unknown.
async fn restart(bus: &mut Spi, orientation: Orientation) -> bool {
    bus.shown = None;
    if bus.driver.recover().await.is_err() {
        return false;
    }
    hard_reset(bus).await;
    sequence(bus, delay).await
        && command(
            bus,
            panel::CMD_MEMORY_ACCESS_CONTROL,
            &[orientation.madctl()],
        )
        .await
        && bus.idle().await
}
