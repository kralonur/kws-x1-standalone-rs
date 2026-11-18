mod image;
mod link;
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
    /// Show detailed image and USB diagnostics.
    #[arg(short, long)]
    verbose: bool,
}

fn bootloaders(api: &HidApi) -> impl Iterator<Item = &hidapi::DeviceInfo> {
    api.device_list()
        .filter(|device| device.vendor_id() == TARGET_VID && device.product_id() == TARGET_PID)
}

fn open_device(api: &HidApi, requested: Option<&str>) -> Result<(HidDevice, String)> {
    let matches: Vec<_> = bootloaders(api).collect();
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
        let api = HidApi::new().context("Initializing HID access")?;
        let (device, path) = open_device(&api, args.device.as_deref())?;
        let mode = protocol::query_info(&device)?;
        tracing::info!("Device mode: {mode} (0 = bootloader) on {path}");
        return Ok(());
    }
    let image = args.image.context("--image is required")?;
    let confirmed = args
        .confirm_sha256
        .context("--confirm-sha256 is required")?;
    let plan = image::Plan::load(&image, &confirmed)?;
    let end = APP_BASE + u32::from(plan.sectors) * image::SECTOR_BYTES;
    let stages = if plan.usb.is_some() { 2 } else { 1 };
    let kind = if stages == 2 {
        "two-stage USB"
    } else {
        "standalone"
    };
    tracing::info!("Image verified: {} ({kind})", image.display());
    tracing::debug!(
        sha256 = plan.digest,
        payload_bytes = plan.payload.len(),
        sectors = plan.sectors,
        erase_start = APP_BASE,
        erase_end = end,
        "Validated image details"
    );
    if args.dry_run {
        if let Some(usb) = &plan.usb {
            tracing::info!(
                "  Stage 2 will send {} bytes to the USB installer if needed.",
                usb.upper.len()
            );
        }
        tracing::info!("Dry run: no device was opened and nothing was written.");
        return Ok(());
    }
    let mut api = HidApi::new().context("Initializing HID access")?;
    if let Some(usb) = &plan.usb
        && let Some(path) = link::resume(
            &link::installers(&mut api)?,
            bootloaders(&api).next().is_some(),
            args.device.as_deref(),
        )?
    {
        tracing::info!("[2/2] USB installer (resuming)");
        tracing::debug!(device = path, "Found waiting installer");
        finish(link::install(&link::open(&api, &path)?, usb)?);
        return Ok(());
    }
    tracing::info!("[1/{stages}] Factory bootloader");
    let (device, path) = open_device(&api, args.device.as_deref())?;
    tracing::debug!(device = path, "Opened bootloader");
    for attempt in 1..=FLASH_ATTEMPTS {
        match protocol::flash(&plan, &device)? {
            protocol::Finish::Accepted => {
                if let Some(usb) = &plan.usb {
                    drop(device);
                    tracing::info!("[2/2] USB installer");
                    finish(link::run(&mut api, usb)?);
                } else {
                    keep_connected("The meter is restarting.");
                }
                return Ok(());
            }
            protocol::Finish::VerifyFailed if attempt < FLASH_ATTEMPTS => {
                tracing::warn!(
                    "  Warning: bootloader verification failed (code 4). Retrying once."
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

fn finish(outcome: link::Outcome) {
    match outcome {
        link::Outcome::Sent => tracing::info!("  Transfer complete: verified by the meter."),
        link::Outcome::AlreadyInstalled => {
            tracing::info!("  Transfer skipped: this data is already on the meter.")
        }
    }
    keep_connected("The meter is still writing the main firmware.");
}

fn keep_connected(status: &str) {
    tracing::info!("{status} Keep it connected until the measurement screen appears.");
    tracing::info!("If the backlight blinks, enter bootloader mode and flash again.");
}

/// Report at most four progress lines per transfer, using acknowledged data only.
fn report_progress(label: &str, previous: usize, done: usize, total: usize) {
    const MILESTONES: usize = 4;
    if done * MILESTONES / total > previous * MILESTONES / total {
        tracing::info!("  {label}: {:>3}%", done * 100 / total);
    }
}

fn main() -> std::process::ExitCode {
    let args = Args::parse();
    let level = if args.verbose {
        tracing::Level::DEBUG
    } else {
        tracing::Level::INFO
    };
    tracing_subscriber::fmt()
        .with_max_level(level)
        .with_level(false)
        .with_target(false)
        .without_time()
        .with_ansi(false)
        .with_writer(std::io::stderr)
        .init();
    match run(args) {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(error) => {
            tracing::error!("Error: {error:#}");
            std::process::ExitCode::FAILURE
        }
    }
}
