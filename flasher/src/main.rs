mod image;
mod protocol;

use anyhow::{Context, Result, bail, ensure};
use clap::Parser;
use firmware_image::APP_BASE;
use hidapi::{HidApi, HidDevice};
use std::path::PathBuf;

// USB vendor/product IDs identifying the supported bootloader HID interface.
const TARGET_VID: u16 = 0x1105;
const TARGET_PID: u16 = 0x000a;
// Full erase-and-write passes per run. The bootloader's final check (code 4)
// has failed intermittently with an image whose every batch it accepted, and
// passed on the next attempt; the bootloader itself is never erased.
const FLASH_ATTEMPTS: u32 = 2;

#[derive(Parser)]
#[command(about = "Flash an exact hash-confirmed application image over USB HID")]
struct Args {
    #[arg(long, required_unless_present = "info", conflicts_with = "info")]
    image: Option<PathBuf>,
    #[arg(long, required_unless_present = "info", conflicts_with = "info")]
    confirm_sha256: Option<String>,
    /// Validate and describe only; never discovers or opens hardware.
    #[arg(long, conflicts_with = "info")]
    dry_run: bool,
    /// Query mode without flashing. Performs device I/O.
    #[arg(long)]
    info: bool,
    /// Exact HIDAPI path (not necessarily a Linux /dev path).
    #[arg(long)]
    device: Option<String>,
}

fn open_device(requested: Option<&str>) -> Result<(HidDevice, String)> {
    let api = HidApi::new().context("Initializing HID access")?;
    let matches: Vec<_> = api
        .device_list()
        .filter(|device| device.vendor_id() == TARGET_VID && device.product_id() == TARGET_PID)
        .collect();
    let selected = if let Some(path) = requested {
        matches
            .iter()
            .copied()
            .find(|device| device.path().to_string_lossy() == path)
            .context("Device override is not a matching bootloader HID path")?
    } else {
        ensure!(
            !matches.is_empty(),
            "No device found; hold Back+OK while connecting USB"
        );
        if matches.len() != 1 {
            let paths: Vec<_> = matches
                .iter()
                .map(|device| device.path().to_string_lossy())
                .collect();
            bail!("Multiple matching HID paths; select one with --device: {paths:?}");
        }
        matches[0]
    };
    let path = selected.path().to_string_lossy().into_owned();
    let device = api
        .open_path(selected.path())
        .context("Opening HID device; on Linux check hidraw permissions or install a udev rule")?;
    Ok((device, path))
}

fn run(args: Args) -> Result<()> {
    if args.info {
        let (device, path) = open_device(args.device.as_deref())?;
        let mode = protocol::query_info(&device)?;
        tracing::info!(device = path, mode, "Device mode (0 = bootloader)");
        return Ok(());
    }
    let image = args.image.context("--image is required")?;
    let confirmed = args
        .confirm_sha256
        .context("--confirm-sha256 is required")?;
    let plan = image::Plan::load(&image, &confirmed)?;
    let end = APP_BASE + u32::from(plan.sectors) * image::SECTOR_BYTES;
    tracing::info!(image = %image.display(), sha256 = plan.digest, payload_bytes = plan.payload.len(),
        sectors = plan.sectors, erase_start = APP_BASE, erase_end = end,
        "Validated image; bootloader and bytes above the application erase envelope remain protected");
    if args.dry_run {
        tracing::info!("Dry run: no device was discovered or opened and nothing was written");
        return Ok(());
    }
    let (device, _) = open_device(args.device.as_deref())?;
    for attempt in 1..=FLASH_ATTEMPTS {
        match protocol::flash(&plan, &device)? {
            protocol::Finish::Accepted => return Ok(()),
            protocol::Finish::VerifyFailed if attempt < FLASH_ATTEMPTS => {
                tracing::warn!(
                    attempt,
                    "Bootloader verification failed (code 4); erasing and writing again"
                );
            }
            protocol::Finish::VerifyFailed => {}
        }
    }
    bail!(
        "Bootloader verification failed (code 4) on every attempt; the bootloader is intact. \
         Unplug the meter, hold Back+OK while reconnecting and run the same command again"
    )
}

fn main() -> std::process::ExitCode {
    tracing_subscriber::fmt()
        .with_target(false)
        .without_time()
        .with_ansi(false)
        .with_writer(std::io::stderr)
        .init();
    match run(Args::parse()) {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(error) => {
            tracing::error!("{error:#}");
            std::process::ExitCode::FAILURE
        }
    }
}
