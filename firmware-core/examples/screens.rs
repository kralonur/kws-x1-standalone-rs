//! Draws the README screenshots with the firmware's own page code, included
//! from `firmware/src` unchanged. Each page gets the values a photo of the
//! meter showed, so a screenshot shows what the meter showed then, laid out
//! as the current firmware draws it. Calibration, with no photo, gets
//! example values.
//!
//! `cargo run -p firmware-core --example screens --locked -- <directory>`
//! writes one lossless WebP per page into `<directory>`.

use std::convert::Infallible;
use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicU32;

use anyhow::{Context, Result};
use embedded_graphics::pixelcolor::{Rgb565, Rgb888};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use firmware_core::buttons::{
    Detail, Level, Navigation, Page, QcOffer, RippleFocus, SettingsFocus, Tab,
};
use firmware_core::calibration::{Calibration, Quantity, Step, Wizard};
use firmware_core::energy::Totals;
use firmware_core::panel::{HEIGHT, WIDTH};
use firmware_core::pd_cable::{CableReport, CableStatus};
use firmware_core::pd_cable_modes::{Discovery, Status as ModesStatus};
use firmware_core::pd_rx::{AdvertisedPdo, Advertisements};
use firmware_core::pd_sniff::{Entry, Rp};
use firmware_core::pd_spr::{EPR_OBJECTS, EprOffer, SPR_OBJECTS, Supply};
use firmware_core::pd_svids::{Progress, Store};
use firmware_core::power_history::{GraphTrace, GraphWindow, History, Sample};
use firmware_core::ripple::{self, PeakHistory, SAMPLE_COUNT, Scale, Timebase};
use firmware_core::settings::{EnergyMemory, Settings};
use firmware_core::spectrum::Spectrum;
use image_webp::{ColorType, WebPEncoder};
use qc_protocol::detect::{Outcome, Report};
use qc_protocol::qc2::Voltage;
use u8g2_fonts::FontRenderer;
use u8g2_fonts::fonts;
use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};

/// The firmware's page code. A directory path keeps `pd`'s own submodules
/// where the firmware has them. Each page here uses only some of it.
#[path = "../../firmware/src"]
#[allow(dead_code, unused_imports)]
mod firmware {
    pub mod pd;
    pub mod protocol;
    pub mod render;
    pub mod ui;
}
use firmware::pd::{Observation, Phase, SniffObservation, SprObservation};
use firmware::protocol::{Sources, identify};
use firmware::render::{Counters, system_lines};
use firmware::ui::{self, Header, MAX_OBJECTS, Meter, Peaks, Qc2, Qc3, QcMode, Reading};
use firmware::{pd, render};

/// Stand-ins for the firmware modules the page code names: the same types,
/// without the hardware around them.
mod line_adc {
    /// Internal reference in mV, as `firmware/src/line_adc.rs` sets it.
    pub const REFERENCE_MV: u16 = 1083;

    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub struct LineVoltages {
        pub dp_mv: u16,
        pub dm_mv: u16,
        pub cc1_mv: u16,
        pub cc2_mv: u16,
    }
}
#[path = "../../firmware/src/scp_drive/status.rs"]
#[allow(dead_code)]
mod scp_drive;
mod qc_drive {
    use qc_protocol::detect::{Outcome, Report, Stage};
    use qc_protocol::qc2::{Failure, Voltage};

    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    #[allow(dead_code)]
    pub enum Status {
        #[default]
        Off,
        Entering,
        Failed(Failure),
        Active(Voltage, Option<Outcome>),
        Stepping(u16),
        Detecting(Stage),
        Detected(Report),
        Scp(u8, crate::scp_drive::Status),
    }
}
mod energy_store {
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    #[allow(dead_code)]
    pub enum Status {
        Ready,
        Occupied,
        SaveFailed,
    }
}
mod settings_store {
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    #[allow(dead_code)]
    pub enum Status {
        #[default]
        Saved,
        NoSpace,
        WriteFailed,
    }
}
use line_adc::LineVoltages;
use qc_drive::Status as QcStatus;

/// The system page's timing peaks in ms, which `render` reads from the
/// crate root (System photo).
static FRAME_MS: AtomicU32 = AtomicU32::new(6);
static BUTTON_GAP_MS: AtomicU32 = AtomicU32::new(18);

/// Each panel pixel becomes a square this many pixels wide, so the small
/// fonts stay sharp at the README's display width.
const SCALE: u32 = 4;
/// The heartbeat dot's colour on a healthy bus (`ui`'s cyan).
const HEARTBEAT: Rgb565 = Rgb565::new(6, 54, 31);

/// Power Data Object fields (USB PD R3.2 Tables 6.9, 6.13 and 6.15): a
/// fixed object's voltage in 50 mV units at bits 19..10 and current in
/// 10 mA units at 9..0, and in the first object EPR Mode Capable at bit 23;
/// an APDO's type at bits 31..28 (PPS 1100, SPR AVS 1110); PPS maximum and
/// minimum voltage in 100 mV units at 24..17 and 15..8 and current in
/// 50 mA units at 6..0; SPR AVS maximum currents for 9-15 V and 15-20 V in
/// 10 mA units at 19..10 and 9..0.
const FIXED_MV_UNIT: u32 = 50;
const FIXED_MA_UNIT: u32 = 10;
const FIXED_FIELD: u32 = 0x3ff;
const EPR_CAPABLE: u32 = 1 << 23;
const PPS_APDO: u32 = 0b1100 << 28;
const PPS_MV_UNIT: u32 = 100;
const PPS_MA_UNIT: u32 = 50;
const AVS_APDO: u32 = 0b1110 << 28;
const AVS_MA_UNIT: u32 = 10;

const fn fixed(mv: u32, ma: u32) -> u32 {
    ((mv / FIXED_MV_UNIT) << 10) | (ma / FIXED_MA_UNIT)
}

const fn pps(min_mv: u32, max_mv: u32, ma: u32) -> u32 {
    PPS_APDO | ((max_mv / PPS_MV_UNIT) << 17) | ((min_mv / PPS_MV_UNIT) << 8) | (ma / PPS_MA_UNIT)
}

const fn avs(ma_15v: u32, ma_20v: u32) -> u32 {
    AVS_APDO | ((ma_15v / AVS_MA_UNIT) << 10) | (ma_20v / AVS_MA_UNIT)
}

/// The charger in the PD photos: its Source_Capabilities (PD Session `N6`,
/// Charger objects 1-6), and the eighth object of the EPR offer the
/// Charger photo's full scan received.
const CHARGER: [u32; 6] = [
    fixed(5_000, 3_000) | EPR_CAPABLE,
    fixed(9_000, 3_000),
    fixed(12_000, 3_000),
    fixed(15_000, 3_000),
    fixed(20_000, 5_000),
    pps(3_300, 21_000, 5_000),
];
const EPR_28V: u32 = fixed(28_000, 5_000);
/// The charger in the AVS photo: an SPR AVS range of 9-20 V, 3 A up to
/// 15 V and 5 A above. Its first object is not in the photo: every source
/// offers vSafe5V as its first object (USB PD 6.4.1), assumed at 3 A.
const AVS_CHARGER: [u32; 2] = [fixed(5_000, 3_000), avs(3_000, 5_000)];
/// PD 3.0 as the session records the charger's revision.
const PD3: u8 = 3;
/// Source_Capabilities message header (USB PD Table 6.1): data message 1,
/// from a Source and DFP, PD 3.0 (`10` at bits 7..6), its object count at
/// bits 14..12.
const SOURCE_CAPABILITIES: u16 = 1 | 1 << 5 | 0b10 << 6 | 1 << 8;
const OBJECTS_SHIFT: u16 = 12;
/// Messages the PD Sniff photo's sniffer had heard.
const SNIFFED_MESSAGES: usize = 150;

/// The USB4 cable in the Cable photo: its Discover Identity reply.
const USB4_CABLE: [u32; 5] = [
    0xFF00_A841,
    0x1C60_2E99,
    0x0000_1520,
    0x0001_0000,
    0x000A_4643,
];
/// The reply's PD revision field: 2 is PD 3.0.
const CABLE_PD3_REVISION: u8 = 2;
/// Intel's vendor ID, which identifies Thunderbolt Alt Mode.
const THUNDERBOLT_SVID: u16 = firmware_core::cable_summary::THUNDERBOLT_SVID;

/// ADC1's clock on the slower clock profile, in Hz (`ripple::Timebase`).
const RIPPLE_CLOCK_HZ: u32 = 30_000_000;
/// An internal reference count near the measured 1083 mV on a 3.3 V,
/// 12-bit scale (`line_adc::REFERENCE_MV`).
const RIPPLE_REFERENCE: u16 = 1344;
/// The ripple's frequency in Hz, a charger's switching frequency: it puts
/// the spectrum's three peaks at the Ripple photo's 27.9k, 55.7k and 83.6k
/// (issue #10's last test photo).
const RIPPLE_HZ: f32 = 27_860.0;
/// The photo's sawtooth around a mid-scale mean: its amplitude in ADC
/// counts, the shares of each period it rises and stays at the top, and
/// the span of the noise on it in counts, chosen so the page reads the
/// photo's peak-to-peak and RMS.
const RIPPLE_MEAN: f32 = 2048.0;
const RIPPLE_AMPLITUDE: f32 = 77.0;
const RIPPLE_RISE: f32 = 0.15;
const RIPPLE_TOP: f32 = 0.12;
const RIPPLE_NOISE: f32 = 6.0;

/// One mAh in mA·ms and one mWh in µW·ms, for the energy totals.
const MA_MS_PER_MAH: u64 = 3_600_000;
const UW_MS_PER_MWH: u64 = 3_600_000_000;

/// The firmware's package manifest, for its version.
const FIRMWARE_MANIFEST: &str = include_str!("../../firmware/Cargo.toml");
/// Where the settings page draws the version: compiled in here it would
/// show this crate's, so it is drawn again. `ui`'s font, colour, right edge
/// and baseline, and the area its digits cover.
const VERSION_FONT: FontRenderer = FontRenderer::new::<fonts::u8g2_font_profont12_mr>();
const VERSION_COLOR: Rgb565 = Rgb565::new(21, 43, 22);
const VERSION_RIGHT: i32 = 228;
const VERSION_BASELINE: i32 = 117;
const VERSION_AREA: Rectangle = Rectangle::new(Point::new(150, 105), Size::new(82, 14));

/// One frame, as the panel shows it.
struct Canvas(Vec<Rgb565>);

impl Canvas {
    fn new() -> Self {
        Self(vec![
            Rgb565::BLACK;
            usize::from(WIDTH) * usize::from(HEIGHT)
        ])
    }

    /// The frame enlarged by [`SCALE`], as RGB bytes.
    fn rgb(&self) -> Vec<u8> {
        let (width, scale) = (usize::from(WIDTH), SCALE as usize);
        let mut bytes = Vec::with_capacity(self.0.len() * scale * scale * 3);
        for row in self.0.chunks(width) {
            for _ in 0..scale {
                for &pixel in row {
                    let color = Rgb888::from(pixel);
                    for _ in 0..scale {
                        bytes.extend([color.r(), color.g(), color.b()]);
                    }
                }
            }
        }
        bytes
    }
}

impl OriginDimensions for Canvas {
    fn size(&self) -> Size {
        Size::new(u32::from(WIDTH), u32::from(HEIGHT))
    }
}

impl DrawTarget for Canvas {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I: IntoIterator<Item = Pixel<Rgb565>>>(
        &mut self,
        pixels: I,
    ) -> Result<(), Self::Error> {
        let (width, height) = (i32::from(WIDTH), i32::from(HEIGHT));
        for Pixel(point, color) in pixels {
            if (0..width).contains(&point.x) && (0..height).contains(&point.y) {
                self.0[(point.y * width + point.x) as usize] = color;
            }
        }
        Ok(())
    }
}

/// The SVIDs the Cable photos' cable listed: Thunderbolt only (`MODES TBT`).
struct CableSvids;

impl Store for CableSvids {
    fn clear(&self) {}
    fn response(&self, _: &[u32]) -> Option<Progress> {
        None
    }
    fn at(&self, index: u16) -> Option<u16> {
        (index == 0).then_some(THUNDERBOLT_SVID)
    }
    fn contains(&self, svid: u16) -> bool {
        svid == THUNDERBOLT_SVID
    }
}

/// Draws one page.
type Draw = fn(&mut Canvas) -> Result<()>;

/// The header for `shown` with Left/Right at `level`, as the firmware
/// builds it.
fn tabs(shown: Tab, level: Level, passthrough: bool, tabs: &mut Vec<Tab>) -> Header<'_> {
    let group = shown.group();
    tabs.extend(group.available(passthrough));
    Header::Tabs {
        current: tabs.iter().position(|tab| tab.same(shown)).unwrap_or(0),
        tabs,
        group: group.position(passthrough),
        level,
        passthrough,
    }
}

/// `objects` as this meter's session received them.
fn advertisements(objects: &[u32]) -> Advertisements {
    let mut caps = Advertisements {
        objects: [AdvertisedPdo {
            raw: 0,
            fixed: None,
        }; MAX_OBJECTS],
        count: objects.len() as u8,
    };
    for (slot, &raw) in caps.objects.iter_mut().zip(objects) {
        let fixed = (raw >> 30 == 0).then(|| {
            (
                ((raw >> 10 & FIXED_FIELD) * FIXED_MV_UNIT) as u16,
                ((raw & FIXED_FIELD) * FIXED_MA_UNIT) as u16,
            )
        });
        *slot = AdvertisedPdo { raw, fixed };
    }
    caps
}

/// A sink session on CC1 that agreed `supply` at `mv` and `ma` with a
/// charger offering `objects`.
fn session(objects: &[u32], supply: Supply, mv: u16, ma: u16) -> Observation {
    let mut observation = Observation::new();
    observation.phase = Phase::Negotiating;
    observation.selected_cc = Some(1);
    observation.capabilities = Some(advertisements(objects));
    observation.spr = Some(SprObservation {
        supply,
        selected_supply: supply,
        target_mv: mv,
        target_ma: ma,
        request_ma: ma,
        selected_mv: Some(mv),
        selected_ma: Some(ma),
        peer_revision: Some(PD3),
        tx_written: true,
        tx_goodcrc: true,
        tx_sent_irq: true,
        accept_seen: true,
        psrdy_seen: true,
        contract_seen: true,
        ..SprObservation::pending()
    });
    observation
}

/// The PD photos' 5 V contract, with the PD Session photo's details.
fn five_volt_session() -> Observation {
    let mut observation = session(&CHARGER, Supply::Fixed, 5_000, 3_000);
    if let Some(spr) = observation.spr.as_mut() {
        spr.request_id = Some(7);
        spr.elapsed_ack_ms = Some(4);
        spr.elapsed_accept_ms = Some(5);
        spr.elapsed_psrdy_ms = Some(129);
        spr.vbus_live_mv = Some(5_130);
        // `CHARGER BUSY: OVERRIDDEN`.
        spr.sink_tx_cause = b'Q';
    }
    observation
}

/// The passthrough sniffer on CC1 behind a 3 A Type-C port, having heard
/// the charger repeat its offer (PD Sniff photo).
fn sniffing() -> Observation {
    let mut sniff = SniffObservation::new();
    sniff.cc = Some(1);
    sniff.rp = Some(Rp::Current3A0);
    let data: Vec<u8> = CHARGER.iter().flat_map(|pdo| pdo.to_le_bytes()).collect();
    let header = SOURCE_CAPABILITIES | (CHARGER.len() as u16) << OBJECTS_SHIFT;
    for _ in 0..SNIFFED_MESSAGES {
        sniff.log.push(Entry::classify(header, &data, &CHARGER));
    }
    sniff.caps[..CHARGER.len()].copy_from_slice(&CHARGER);
    sniff.caps_count = CHARGER.len() as u8;
    let mut observation = Observation::new();
    observation.pc13_high = Some(true);
    observation.sniff = Some(sniff);
    observation
}

/// The Quick Charge probe's findings in the QC photos: 9 V, 12 V and
/// steps, no 20 V.
fn probed() -> QcOffer {
    let mut navigation = Navigation::new(Page::default(), Settings::default());
    navigation.finish_probe(Report {
        v9: true,
        v12: true,
        v20: false,
        steps: true,
    });
    navigation.update(0).0.qc
}

fn overview_page(
    canvas: &mut Canvas,
    observation: &Observation,
    reading: Reading,
    peaks: &[Reading],
    passthrough: bool,
) {
    let mut strip = Vec::new();
    let mut max = Peaks::default();
    for &peak in peaks {
        max.update(peak);
    }
    ui::draw_meter(
        canvas,
        &tabs(Tab::Overview, Level::Groups, passthrough, &mut strip),
        &Meter {
            reading: Some(reading),
            peaks: max,
            protocol: identify(&Sources {
                observation,
                lines: None,
                driven: None,
                qc: QcStatus::Off,
                findings: None,
                scp: None,
                vbus_mv: Some(reading.mv),
            }),
            cc: pd::attached_cc(observation),
            flipped: false,
            alive: HEARTBEAT,
            pulses: 0,
            reset_hold: 0,
        },
    );
}

/// Overview photo: the 5 V contract, nothing drawing current.
fn overview(canvas: &mut Canvas) -> Result<()> {
    let reading = Reading { mv: 5_130, ma: 0 };
    overview_page(canvas, &five_volt_session(), reading, &[reading], false);
    Ok(())
}

/// Passthrough Overview photo: a phone charging behind the meter, the
/// charger on the plug side (current flowing plug to receptacle is
/// negative), 2.57 W now; at most 5.07 V before the load and 3.58 W at
/// 0.71 A.
fn passthrough_overview(canvas: &mut Canvas) -> Result<()> {
    overview_page(
        canvas,
        &sniffing(),
        Reading {
            mv: 5_055,
            ma: -509,
        },
        &[
            Reading { mv: 5_070, ma: 0 },
            Reading {
                mv: 5_045,
                ma: -710,
            },
        ],
        true,
    );
    Ok(())
}

/// Lines photo.
fn lines(canvas: &mut Canvas) -> Result<()> {
    let mut strip = Vec::new();
    ui::draw_lines(
        canvas,
        &tabs(Tab::Lines, Level::Tabs, false, &mut strip),
        Some(LineVoltages {
            dp_mv: 2_500,
            dm_mv: 2_470,
            cc1_mv: 610,
            cc2_mv: 0,
        }),
    );
    Ok(())
}

/// Energy photo: 0.014 Ah and 0.073 Wh over 1:44, kept in RAM only. Half a
/// unit more keeps each value clear of the next.
fn energy(canvas: &mut Canvas) -> Result<()> {
    let mut strip = Vec::new();
    ui::draw_energy(
        canvas,
        &tabs(Tab::Energy, Level::Tabs, false, &mut strip),
        Totals {
            ma_ms: 14 * MA_MS_PER_MAH + MA_MS_PER_MAH / 2,
            uw_ms: 73 * UW_MS_PER_MWH + UW_MS_PER_MWH / 2,
            elapsed_ms: 104_000,
        },
        EnergyMemory::Off,
        energy_store::Status::Ready,
        0,
    );
    Ok(())
}

/// Graph photo: two minutes of power, a few idle seconds at 5.13 V, then a
/// load starting at 0.71 A (3.58 W) easing to 0.485 A (2.45 W) at 5.06 V.
fn graph(canvas: &mut Canvas) -> Result<()> {
    const SECONDS: u64 = 120;
    const IDLE: u64 = 14;
    let mut history = History::default();
    for second in 0..=SECONDS {
        let sample = if second < IDLE {
            Sample {
                mv: 5_130,
                ma: 0,
                mw: 0,
            }
        } else {
            let progress = (second - IDLE) as f32 / (SECONDS - IDLE) as f32;
            let ease = (1.0 - (-3.0 * progress).exp()) / (1.0 - (-3.0f32).exp());
            let ma = (710.0 - 225.0 * ease).round() as u16;
            let mv = (5_045.0 + 15.0 * ease).round() as u16;
            Sample {
                mv,
                ma,
                mw: u32::from(mv) * u32::from(ma) / 1000,
            }
        };
        history.sample(second * 1000, Some(sample));
    }
    // The next second's first reading stores the last one.
    history.sample((SECONDS + 1) * 1000, None);
    let mut strip = Vec::new();
    ui::draw_graph(
        canvas,
        &tabs(Tab::Graph, Level::Tabs, false, &mut strip),
        &history,
        GraphTrace::Power,
        GraphWindow::Min2,
        None,
        0,
    );
    Ok(())
}

/// Ripple photo (issue #10): 64 mV peak-to-peak and 19.6 mV RMS at 27.9 kHz
/// on 4.880 V and 2.792 A, held at 20 us/div, automatic 20 mV/div: a
/// sawtooth rising fast and falling slowly.
fn ripple(canvas: &mut Canvas) -> Result<()> {
    let timebase = Timebase::Us20;
    let sampling = timebase
        .sampling(RIPPLE_CLOCK_HZ)
        .context("the timebase at the example clock")?;
    let period = sampling.rate_hz(RIPPLE_CLOCK_HZ) as f32 / RIPPLE_HZ;
    let mut noise = 7u32;
    let samples: [u16; SAMPLE_COUNT] = std::array::from_fn(|index| {
        noise = noise.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let jitter = ((noise >> 24) as f32 / 255.0 - 0.5) * RIPPLE_NOISE;
        let phase = (index as f32 / period).fract();
        // From the bottom to the top in the rise, along the top, then
        // back down.
        let fall = RIPPLE_RISE + RIPPLE_TOP;
        let wave = if phase < RIPPLE_RISE {
            phase / RIPPLE_RISE * 2.0 - 1.0
        } else if phase < fall {
            1.0
        } else {
            1.0 - (phase - fall) / (1.0 - fall) * 2.0
        };
        (RIPPLE_MEAN + RIPPLE_AMPLITUDE * wave + jitter).round() as u16
    });
    let mut reading = ripple::analyze(&samples, timebase, sampling.window);
    PeakHistory::default().add(&mut reading);
    reading.spectrum = Spectrum::new(&samples, &mut [0.0; SAMPLE_COUNT]);
    let mut strip = Vec::new();
    ui::draw_ripple(
        canvas,
        &tabs(Tab::Ripple, Level::Page, false, &mut strip),
        &ui::Ripple {
            reading: Some(&reading),
            reference: Some(RIPPLE_REFERENCE),
            bus: Some(Reading {
                mv: 4_880,
                ma: 2_792,
            }),
            hold: true,
            timebase,
            scale: Scale::Auto,
            focus: Some(RippleFocus::Capture),
            clock_hz: Some(RIPPLE_CLOCK_HZ),
        },
    );
    Ok(())
}

/// PD Profiles photo: the charger's fixed profiles, 20 V agreed.
fn pd_profiles(canvas: &mut Canvas) -> Result<()> {
    let observation = session(&CHARGER, Supply::Fixed, 20_000, 5_000);
    let mut strip = Vec::new();
    ui::draw_fixed(
        canvas,
        &tabs(Tab::Fixed, Level::Tabs, false, &mut strip),
        &pd::offer(&observation),
        false,
    );
    Ok(())
}

/// PPS photo: 3.30 V at up to 5 A, the output reading 3.38 V.
fn pps_page(canvas: &mut Canvas) -> Result<()> {
    let position = CHARGER.len() as u8;
    let observation = session(&CHARGER, Supply::Pps(position), 3_300, 5_000);
    let mut strip = Vec::new();
    ui::draw_pps(
        canvas,
        &tabs(Tab::Pps, Level::Tabs, false, &mut strip),
        &pd::offer(&observation),
        None,
        Some(Reading { mv: 3_380, ma: 0 }),
    );
    Ok(())
}

/// AVS photo: 15 V at 3 A from a 9-20 V range, the output reading 15.08 V.
fn avs_page(canvas: &mut Canvas) -> Result<()> {
    let position = AVS_CHARGER.len() as u8;
    let observation = session(&AVS_CHARGER, Supply::Avs(position), 15_000, 3_000);
    let mut strip = Vec::new();
    ui::draw_avs(
        canvas,
        &tabs(Tab::Avs, Level::Tabs, false, &mut strip),
        &pd::offer(&observation),
        false,
        Some(Reading { mv: 15_080, ma: 0 }),
    );
    Ok(())
}

/// QC 2.0 photo: 9 V requested and followed.
fn qc2(canvas: &mut Canvas) -> Result<()> {
    let mut strip = Vec::new();
    ui::draw_qc2(
        canvas,
        &tabs(Tab::Qc2, Level::Tabs, false, &mut strip),
        &Qc2 {
            preview: None,
            request: Some(Voltage::V9),
            mode: QcMode::Plain,
            status: QcStatus::Active(Voltage::V9, Some(Outcome::Followed)),
            qc: probed(),
        },
    );
    Ok(())
}

/// Experimental SCP layout fixtures, not hardware measurements.
fn scp_fixture_adapter() -> scp_protocol::Adapter {
    scp_protocol::Adapter {
        modes: scp_protocol::Modes::Both,
        type0: Some(0xb8),
        type1: None,
        capabilities: Some(scp_protocol::Capabilities {
            min_mv: 5_500,
            max_mv: 11_000,
            min_ma: 1_000,
            max_ma: 3_000,
            vstep_mv: Some(20),
            istep_ma: Some(50),
            max_power_dw: Some(330),
        }),
    }
}

fn scp(canvas: &mut Canvas) -> Result<()> {
    let mut strip = Vec::new();
    ui::draw_scp(
        canvas,
        &tabs(Tab::Scp, Level::Tabs, false, &mut strip),
        &ui::Scp {
            probing: false,
            mode: QcMode::Plain,
            findings: Some(firmware_core::settings::ScpFindings::Confirmed),
            status: Some(scp_drive::Status::Detected(scp_fixture_adapter())),
            selection: firmware_core::scp::Selection::new(scp_fixture_adapter()),
            focus: None,
            active: false,
            reading: None,
        },
    );
    Ok(())
}

fn charger_scp(canvas: &mut Canvas) -> Result<()> {
    detail(
        canvas,
        Detail::Charger,
        false,
        [ui::scp_charger_lines(
            Some(firmware_core::settings::ScpFindings::Confirmed),
            Some(scp_drive::Status::Detected(scp_fixture_adapter())),
        )],
        0,
    );
    Ok(())
}

/// QC 3.0 photo: stepped to 7.00 V, the output reading 7.07 V.
fn qc3(canvas: &mut Canvas) -> Result<()> {
    let mut strip = Vec::new();
    ui::draw_qc3(
        canvas,
        &tabs(Tab::Qc3, Level::Tabs, false, &mut strip),
        &Qc3 {
            target_mv: Some(7_000),
            adjusting: false,
            mode: QcMode::Plain,
            status: QcStatus::Stepping(7_000),
            qc: probed(),
            lines: Some(LineVoltages {
                dp_mv: 560,
                dm_mv: 3_090,
                cc1_mv: 0,
                cc2_mv: 0,
            }),
            reading: Some(Reading { mv: 7_070, ma: 0 }),
        },
    );
    Ok(())
}

/// A detail list opened from row `top`, as the photos show them.
fn detail<const W: usize>(
    canvas: &mut Canvas,
    page: Detail,
    passthrough: bool,
    sections: impl IntoIterator<Item = [[u8; W]; 8]>,
    top: u16,
) {
    let mut strip = Vec::new();
    ui::draw_detail(
        canvas,
        &tabs(Tab::detail(page), Level::Page, passthrough, &mut strip),
        page,
        sections,
        top,
        true,
    );
}

/// The PD page's sections, as the firmware lists them.
fn pd_detail(canvas: &mut Canvas, observation: Observation, passthrough: bool) {
    detail(
        canvas,
        Detail::Pd,
        passthrough,
        std::iter::once(pd::lines(observation))
            .chain(pd::startup_lines(observation))
            .chain(pd::caps_lines(observation)),
        0,
    );
}

/// PD Session photo.
fn pd_session(canvas: &mut Canvas) -> Result<()> {
    pd_detail(canvas, five_volt_session(), false);
    Ok(())
}

/// PD Sniff photo.
fn pd_sniff(canvas: &mut Canvas) -> Result<()> {
    pd_detail(canvas, sniffing(), true);
    Ok(())
}

/// Charger photo: the same charger after the full scan received its EPR
/// offer.
fn charger(canvas: &mut Canvas) -> Result<()> {
    let mut observation = five_volt_session();
    observation.scan_pd = true;
    if let Some(spr) = observation.spr.as_mut() {
        let mut objects = [0; EPR_OBJECTS];
        objects[..CHARGER.len()].copy_from_slice(&CHARGER);
        objects[SPR_OBJECTS] = EPR_28V;
        spr.epr_offer = EprOffer::Received(objects);
    }
    let partner = pd::partner_sections(observation);
    let offer = partner.min(pd::CAPABILITY_SECTIONS);
    detail(
        canvas,
        Detail::Charger,
        false,
        (0..offer)
            .map(|section| pd::partner_lines(observation, section))
            .chain(
                (0..ui::qc_charger_sections(None))
                    .map(|section| ui::qc_charger_lines(None, QcMode::Pd, section)),
            )
            .chain(core::iter::once(ui::scp_charger_lines(None, None)))
            .chain((offer..partner).map(|section| pd::partner_lines(observation, section))),
        0,
    );
    Ok(())
}

/// The Cable photos' list: the USB4 cable's E-marker, read under PD 3.0,
/// and its SVID list, Thunderbolt only. Shown from the first row starting
/// with `heading`.
fn cable_page(canvas: &mut Canvas, heading: &str) -> Result<()> {
    let mut objects = [0; 7];
    objects[..USB4_CABLE.len()].copy_from_slice(&USB4_CABLE);
    let mut observation = Observation::new();
    observation.spr = Some(SprObservation {
        cable_report: Some(CableReport {
            status: CableStatus::Identity,
            objects,
            count: USB4_CABLE.len() as u8,
            revision: CABLE_PD3_REVISION,
            next_tx_id: 0,
            last_rx_id: None,
        }),
        cable_modes: Discovery {
            status: ModesStatus::Ack,
            count: 1,
            ..Discovery::EMPTY
        },
        peer_revision: Some(PD3),
        ..SprObservation::pending()
    });
    let sections: Vec<_> = (0..pd::cable_sections(observation))
        .map(|section| {
            let svid =
                pd::cable_svid_index(observation, section).and_then(|index| CableSvids.at(index));
            pd::cable_lines(observation, section, svid, &CableSvids, true)
        })
        .collect();
    let top = shown_rows(&sections)
        .position(|row| row.starts_with(heading))
        .with_context(|| format!("no {heading} row on the Cable page"))?;
    detail(canvas, Detail::Cable, false, sections, top as u16);
    Ok(())
}

/// The rows a detail list shows, in order: `ui::draw_detail` leaves out
/// blank ones.
fn shown_rows<const W: usize>(sections: &[[[u8; W]; 8]]) -> impl Iterator<Item = String> + '_ {
    sections
        .iter()
        .flatten()
        .map(|row| String::from_utf8_lossy(row).trim_end().to_owned())
        .filter(|row| !row.is_empty())
}

/// First Cable photo: the summary.
fn cable(canvas: &mut Canvas) -> Result<()> {
    cable_page(canvas, "CABLE SUMMARY")
}

/// Second Cable photo: scrolled to the raw identity.
fn cable_details(canvas: &mut Canvas) -> Result<()> {
    cable_page(canvas, "E-MARKER DETAILS")
}

/// System photo: no I2C faults, 1334 reads, scrolled down to the counters.
fn system(canvas: &mut Canvas) -> Result<()> {
    let counters = Counters {
        step: 0,
        sr: 0,
        cr1: 0,
        cr4: 0,
        stack_peak: 0,
        stack_size: 0,
        reg_failures: 0,
        errors: 0,
        reads: 1_334,
        reset_flags: 0,
    };
    detail(
        canvas,
        Detail::System,
        false,
        system_lines(counters, false),
        2,
    );
    Ok(())
}

/// The firmware's version, from its manifest's first `version` line.
fn firmware_version() -> Result<&'static str> {
    FIRMWARE_MANIFEST
        .lines()
        .find_map(|line| line.strip_prefix("version = "))
        .map(|version| version.trim_matches('"'))
        .context("no version in firmware/Cargo.toml")
}

/// Settings photo: full brightness, automatic rotation, the energy memory
/// interval being changed at 30 s.
fn settings(canvas: &mut Canvas) -> Result<()> {
    let settings = Settings {
        brightness: 4,
        energy_memory: EnergyMemory::Seconds30,
        ..Settings::default()
    };
    ui::draw_settings(
        canvas,
        &Header::Settings,
        settings,
        SettingsFocus::EnergyMemory,
        settings_store::Status::Saved,
        0,
    );
    VERSION_AREA
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
        .draw(canvas)?;
    VERSION_FONT
        .render_aligned(
            firmware_version()?,
            Point::new(VERSION_RIGHT, VERSION_BASELINE),
            VerticalPosition::Baseline,
            HorizontalAlignment::Right,
            FontColor::Transparent(VERSION_COLOR),
            canvas,
        )
        .ok()
        .context("drawing the version")?;
    Ok(())
}

/// No photo yet: the current step, a reference meter reading 2.000 A while
/// this one reads 2.016 A.
fn calibration(canvas: &mut Canvas) -> Result<()> {
    let live = Reading {
        mv: 5_040,
        ma: 2_016,
    };
    let wizard = Wizard {
        step: Step::Entry {
            quantity: Quantity::Current,
            start: Some(live.ma.unsigned_abs()),
            adjust: -16,
        },
        ..Wizard::new(Calibration::default())
    };
    ui::draw_calibration(canvas, &Header::Settings, &wizard, Some(live));
    Ok(())
}

fn save(directory: &Path, name: &str, canvas: &Canvas) -> Result<()> {
    let path = directory.join(format!("{name}.webp"));
    let file = File::create(&path).with_context(|| format!("creating {}", path.display()))?;
    WebPEncoder::new(BufWriter::new(file))
        .encode(
            &canvas.rgb(),
            u32::from(WIDTH) * SCALE,
            u32::from(HEIGHT) * SCALE,
            ColorType::Rgb8,
        )
        .with_context(|| format!("encoding {}", path.display()))
}

fn main() -> Result<()> {
    let directory = PathBuf::from(
        std::env::args()
            .nth(1)
            .context("usage: screens <output directory>")?,
    );
    let pages: [(&str, Draw); 21] = [
        ("overview_screen", overview),
        ("lines_screen", lines),
        ("energy_screen", energy),
        ("graph_screen", graph),
        ("ripple_screen", ripple),
        ("pd_profiles_screen", pd_profiles),
        ("pps_screen", pps_page),
        ("avs_screen", avs_page),
        ("qc2_screen", qc2),
        ("qc3_screen", qc3),
        ("scp_screen", scp),
        ("charger_scp_screen", charger_scp),
        ("pd_session_screen", pd_session),
        ("charger_screen", charger),
        ("cable_screen", cable),
        ("cable_details_screen", cable_details),
        ("system_screen", system),
        ("settings_screen", settings),
        ("calibration_screen", calibration),
        ("passthrough_overview_screen", passthrough_overview),
        ("pd_sniff_screen", pd_sniff),
    ];
    for (name, draw) in pages {
        let mut canvas = Canvas::new();
        draw(&mut canvas)?;
        save(&directory, name, &canvas)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use firmware_core::settings::ScpFindings;

    #[test]
    fn charger_keeps_pd_query_tabs_and_asks_for_pd_without_an_offer() {
        let mut observation = Observation::new();
        observation.spr = Some(SprObservation::pending());
        observation.phase = Phase::NoCapabilities;
        assert_eq!(pd::partner_sections(observation), 5);
        for section in pd::CAPABILITY_SECTIONS..pd::CAPABILITY_SECTIONS + 3 {
            let rows = pd::partner_lines(observation, section);
            let text = String::from_utf8(rows.into_iter().flatten().collect()).unwrap();
            assert!(text.contains("CHARGER MUST SUPPORT PD"));
            assert!(!text.contains("WAITING"));
        }
        // A real PD offer must still expose its identity/status sections.
        assert_eq!(pd::partner_sections(five_volt_session()), 5);
    }

    #[test]
    fn scp_display_distinguishes_evidence_from_an_output_contract() {
        for (findings, expected) in [
            (None, "NOT TESTED YET"),
            (Some(ScpFindings::Unverified), "SUPPORT NOT DETERMINED"),
            (Some(ScpFindings::Confirmed), "TYPE REGISTER VERIFIED"),
        ] {
            let rows = ui::scp_charger_lines(findings, None);
            let text = String::from_utf8(rows.into_iter().flatten().collect()).unwrap();
            assert!(text.contains("EXPERIMENTAL READ-ONLY"));
            assert!(text.contains(expected));
            assert!(!text.contains("NOT SUPPORTED"));
        }
        let observation = Observation::new();
        let mut sources = Sources {
            observation: &observation,
            lines: None,
            driven: None,
            qc: QcStatus::Off,
            findings: None,
            scp: Some(ScpFindings::Confirmed),
            vbus_mv: Some(5_000),
        };
        assert_eq!(identify(&sources), ui::Protocol::ScpDetected);
        sources.vbus_mv = Some(9_000);
        assert_ne!(identify(&sources), ui::Protocol::ScpDetected);
        let pd = five_volt_session();
        sources.observation = &pd;
        sources.vbus_mv = Some(5_000);
        assert!(matches!(identify(&sources), ui::Protocol::Pd { .. }));
    }
}
