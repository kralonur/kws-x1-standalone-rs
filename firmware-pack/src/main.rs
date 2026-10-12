use anyhow::{Context, Result, ensure};
use clap::{Parser, Subcommand};
use firmware_image::{
    self as image, APP_BASE, APP_LIMIT, CONTAINER_PREFIX, ERASE_CEILING, EXTENDED_PAYLOAD_LEN,
    SECTOR_BYTES,
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{fs, io::Write, mem::size_of, path::PathBuf};

// Runtime vector table: 16 exceptions followed by 144 external interrupts.
const VECTOR_COUNT: usize = 160;
const FIRST_IRQ: usize = 16;
const SYSTICK_EXCEPTION: usize = 15;
// Firmware startup's permitted non-default external interrupt vector indices.
const CUSTOM_IRQS: [usize; 14] = [4, 8, 13, 31, 38, 39, 86, 87, 88, 110, 111, 112, 113, 116];
// The startup stack must lie strictly inside this linker-defined RAM region.
const STACK_RAM_START: u32 = 0x20000000;
const STACK_RAM_END: u32 = 0x20010000;

#[derive(Parser)]
#[command(about = "Generate and audit firmware containers offline; never accesses USB")]
struct Args {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Generate a standalone container; never overwrites an existing output.
    Pack {
        #[arg(long)]
        app: PathBuf,
        #[arg(long)]
        out: PathBuf,
    },
    /// Audit the exact hash-confirmed container against its built application.
    Audit {
        #[arg(long)]
        app: PathBuf,
        #[arg(long)]
        image: PathBuf,
        #[arg(long)]
        confirm_sha256: String,
    },
}

#[derive(Serialize)]
struct Application {
    initial_sp: u32,
    reset_vector: u32,
    systick_vector: u32,
    custom_irq_lines: Vec<usize>,
}

#[derive(Serialize)]
struct Report {
    sha256: String,
    payload_bytes: usize,
    erase_sectors: u32,
    erase_envelope: [u32; 2],
    #[serde(flatten)]
    application: Application,
    application_bytes: usize,
    application_sha256: String,
    unused_content_bytes: usize,
    descriptor_bytes: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    candidate: Option<PathBuf>,
    status: &'static str,
}

fn digest(data: &[u8]) -> String {
    Sha256::digest(data)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn check_application(app: &[u8]) -> Result<Application> {
    ensure!(
        (VECTOR_COUNT * size_of::<u32>()..=APP_LIMIT).contains(&app.len()),
        "Application is truncated or overlaps the container descriptor"
    );
    let vectors: [u32; VECTOR_COUNT] = std::array::from_fn(|index| {
        // The length check above guarantees every vector's four bytes exist.
        let offset = index * size_of::<u32>();
        u32::from_le_bytes([
            app[offset],
            app[offset + 1],
            app[offset + 2],
            app[offset + 3],
        ])
    });
    ensure!(
        STACK_RAM_START < vectors[0] && vectors[0] < STACK_RAM_END,
        "Initial stack pointer is not strictly inside the startup RAM region"
    );
    for index in [1, SYSTICK_EXCEPTION]
        .into_iter()
        .chain(FIRST_IRQ..VECTOR_COUNT)
    {
        let address = vectors[index];
        ensure!(
            address & 1 != 0 && APP_BASE < address && address < APP_BASE + app.len() as u32,
            "Vector {index} is not a Thumb address inside the application"
        );
    }
    let default = vectors[FIRST_IRQ];
    let custom_irq_lines: Vec<_> = (FIRST_IRQ..VECTOR_COUNT)
        .filter(|index| vectors[*index] != default)
        .map(|index| index - FIRST_IRQ)
        .collect();
    ensure!(
        custom_irq_lines.as_slice() == CUSTOM_IRQS,
        "Unexpected custom IRQ lines: {custom_irq_lines:?}"
    );
    ensure!(
        vectors[SYSTICK_EXCEPTION] != default,
        "SysTick uses the default interrupt handler"
    );
    Ok(Application {
        initial_sp: vectors[0],
        reset_vector: vectors[1],
        systick_vector: vectors[SYSTICK_EXCEPTION],
        custom_irq_lines,
    })
}

fn prepare(app: &[u8]) -> Result<(Vec<u8>, Report)> {
    let application = check_application(app)?;
    let sectors = image::erase_sectors_for(EXTENDED_PAYLOAD_LEN);
    let end = APP_BASE + sectors * SECTOR_BYTES;
    ensure!(
        end <= ERASE_CEILING,
        "Erase envelope exceeds the application ceiling"
    );
    let mut container = vec![0; CONTAINER_PREFIX.len() + EXTENDED_PAYLOAD_LEN];
    image::build_container(app, &mut container)
        .map_err(|error| anyhow::anyhow!("Building container: {error:?}"))?;
    image::verify_container(&container)
        .map_err(|error| anyhow::anyhow!("Invalid generated container: {error:?}"))?;
    let report = Report {
        sha256: digest(&container),
        payload_bytes: EXTENDED_PAYLOAD_LEN,
        erase_sectors: sectors,
        erase_envelope: [APP_BASE, end],
        application,
        application_bytes: app.len(),
        application_sha256: digest(app),
        unused_content_bytes: APP_LIMIT - app.len(),
        descriptor_bytes: image::DESCRIPTOR.len(),
        candidate: None,
        status: "prepared, not flashed",
    };
    Ok((container, report))
}

fn run(args: Args) -> Result<()> {
    let report = match args.command {
        Command::Pack { app, out } => {
            let app = fs::read(app).context("Reading application")?;
            let (container, mut report) = prepare(&app)?;
            if let Some(parent) = out.parent() {
                fs::create_dir_all(parent).context("Creating output directory")?;
            }
            let mut output = fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&out)
                .context("Creating output; existing files are never overwritten")?;
            output.write_all(&container).context("Writing container")?;
            report.candidate = Some(out);
            report
        }
        Command::Audit {
            app,
            image,
            confirm_sha256,
        } => {
            let candidate = fs::read(image).context("Reading image")?;
            let actual = digest(&candidate);
            ensure!(
                actual == confirm_sha256,
                "Image SHA-256 is {actual}, not the confirmed {confirm_sha256}"
            );
            let app = fs::read(app).context("Reading application")?;
            let (expected, mut report) = prepare(&app)?;
            image::verify_container(&candidate)
                .map_err(|error| anyhow::anyhow!("Invalid container: {error:?}"))?;
            ensure!(
                candidate == expected,
                "Container differs from the canonical layout for the supplied application"
            );
            report.status = "structural audit passed; hardware not validated";
            report
        }
    };
    let mut stdout = std::io::stdout().lock();
    serde_json::to_writer_pretty(&mut stdout, &report).context("Writing audit report")?;
    writeln!(stdout).context("Finishing audit report")?;
    Ok(())
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
