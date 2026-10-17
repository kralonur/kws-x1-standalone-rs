use anyhow::{Context, Result, ensure};
use clap::{Parser, Subcommand};
use firmware_image::{
    self as image, APP_BASE, APP_LIMIT, CONTAINER_PREFIX, ERASE_CEILING, EXTENDED_PAYLOAD_LEN,
    SECTOR_BYTES, UPPER_BASE, UPPER_END, UPPER_ID_MAGIC, UPPER_ID_UNSET,
    packed::{
        DICT_BYTES, PACKED_MAGIC, PACKED_UNSET, Packed, Stream, WINDOW_BYTES, filter_thumb_bl,
        inflate_sectors, main_erase_margin,
    },
    region_hash,
};
use object::{
    Endianness, elf,
    read::elf::{ElfFile32, ProgramHeader},
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    fs,
    io::Write,
    mem::size_of,
    path::{Path, PathBuf},
};

// Runtime vector table: 16 exceptions followed by 144 external interrupts.
const VECTOR_COUNT: usize = 160;
const FIRST_IRQ: usize = 16;
const SYSTICK_EXCEPTION: usize = 15;
// Firmware startup's permitted non-default external interrupt vector indices,
// including the D− edge on 1 and ripple DMA1 completion/error on 40/41.
const CUSTOM_IRQS: [usize; 16] = [
    1, 4, 13, 31, 38, 39, 40, 41, 86, 87, 88, 110, 111, 112, 113, 116,
];
// The startup stack must lie strictly inside this linker-defined RAM region.
const STACK_RAM_START: u32 = 0x20000000;
const STACK_RAM_END: u32 = 0x20010000;
// The unpacker's stack lies strictly inside its RAM (`firmware/unpacker.x`).
const UNPACKER_RAM_END: u32 = 0x20020000;
// LZMA2 preset level and Thumb-aligned literal/position context bits:
// lp = 1 aligns literals to 16-bit Thumb halfwords, lc = 2 keeps lc + lp = 3
// (12 KiB literal probability model), and pb = 2 aligns matches to 32-bit words.
const LZMA_PRESET: u32 = 9;
const LZMA_LC: u32 = 2;
const LZMA_LP: u32 = 1;
const LZMA_PB: u32 = 2;

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
        #[command(flatten)]
        source: Source,
        #[arg(long)]
        out: PathBuf,
    },
    /// Audit the exact hash-confirmed container against what it was built from.
    Audit {
        #[command(flatten)]
        source: Source,
        #[arg(long)]
        image: PathBuf,
        #[arg(long)]
        confirm_sha256: String,
    },
}

/// What the container holds: a plain application image, or a release
/// built from the firmware and unpacker ELFs (`firmware_image::packed`).
#[derive(clap::Args)]
#[group(required = true, multiple = true)]
struct Source {
    /// A plain application image without an upper region.
    #[arg(long, conflicts_with_all = ["firmware", "unpacker"])]
    app: Option<PathBuf>,
    /// The firmware ELF (`target/.../release/firmware`).
    #[arg(long, requires = "unpacker")]
    firmware: Option<PathBuf>,
    /// The unpacker ELF (`target/.../release/unpacker`).
    #[arg(long, requires = "firmware")]
    unpacker: Option<PathBuf>,
}

#[derive(Serialize)]
struct Application {
    initial_sp: u32,
    reset_vector: u32,
    systick_vector: u32,
    custom_irq_lines: Vec<usize>,
}

/// How a release image's regions were packed.
#[derive(Serialize)]
struct Release {
    unpacker_bytes: usize,
    main_bytes: usize,
    main_packed_bytes: usize,
    upper_bytes: usize,
    upper_packed_bytes: usize,
    upper_unused_bytes: usize,
    /// The least distance between a main firmware sector the unpacker
    /// erases and the stream bytes it has yet to read.
    erase_margin_bytes: usize,
}

#[derive(Serialize)]
struct Report {
    sha256: String,
    payload_bytes: usize,
    erase_sectors: u32,
    erase_envelope: [u32; 2],
    /// The firmware that runs: the application, or a release's main firmware.
    #[serde(flatten)]
    application: Application,
    application_bytes: usize,
    application_sha256: String,
    unused_content_bytes: usize,
    descriptor_bytes: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    release: Option<Release>,
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

/// Little-endian bytes of record words.
fn words(words: &[u32]) -> Vec<u8> {
    words.iter().flat_map(|word| word.to_le_bytes()).collect()
}

/// Fill the record in `app` that is `magic` followed by `fields.len()` unset
/// words (a bare copy of the magic, such as a compared constant, is not a
/// record); `Ok(false)` when there is none.
fn fill_record(app: &mut [u8], magic: &[u32; 2], unset: u32, fields: &[u32]) -> Result<bool> {
    let pattern = words(&[magic.as_slice(), &vec![unset; fields.len()]].concat());
    let mut found = app
        .windows(pattern.len())
        .enumerate()
        .filter(|(_, window)| *window == pattern.as_slice())
        .map(|(offset, _)| offset);
    let Some(offset) = found.next() else {
        return Ok(false);
    };
    ensure!(found.next().is_none(), "A record appears more than once");
    let at = offset + size_of_val(magic);
    let filled = words(fields);
    app[at..at + filled.len()].copy_from_slice(&filled);
    Ok(true)
}

/// Flash images of an ELF's loadable bytes, laid out from their load
/// addresses with zero gaps: the application from `APP_BASE` and the upper
/// region from `UPPER_BASE` (empty when it has none).
fn split(elf_bytes: &[u8]) -> Result<(Vec<u8>, Vec<u8>)> {
    let file = ElfFile32::<Endianness>::parse(elf_bytes).context("Parsing ELF")?;
    let endian = file.endian();
    let (mut app, mut upper) = (Vec::new(), Vec::new());
    for header in file.elf_program_headers() {
        if header.p_type(endian) != elf::PT_LOAD || header.p_filesz(endian) == 0 {
            continue;
        }
        let address = header.p_paddr(endian);
        let data = header
            .data(endian, elf_bytes)
            .map_err(|()| anyhow::anyhow!("Segment at {address:#x} lies outside the ELF"))?;
        let (image, base, end) = if (APP_BASE..ERASE_CEILING).contains(&address) {
            (&mut app, APP_BASE, ERASE_CEILING)
        } else if (UPPER_BASE..UPPER_END).contains(&address) {
            (&mut upper, UPPER_BASE, UPPER_END)
        } else {
            anyhow::bail!("Segment at {address:#x} is outside the application and upper region");
        };
        let at = (address - base) as usize;
        let stop = at + data.len();
        ensure!(
            stop <= (end - base) as usize,
            "Segment at {address:#x} crosses the end of its region"
        );
        if image.len() < stop {
            image.resize(stop, 0);
        }
        image[at..stop].copy_from_slice(data);
    }
    Ok((app, upper))
}

/// Filter Thumb-2 `BL` displacements and compress as a raw LZMA2 stream.
fn deflate(bytes: &[u8]) -> Result<Vec<u8>> {
    let mut filtered = bytes.to_vec();
    filter_thumb_bl(&mut filtered, 0, true);
    let mut options = lzma_rust2::Lzma2Options::with_preset(LZMA_PRESET);
    options.lzma_options.dict_size = DICT_BYTES;
    options.lzma_options.lc = LZMA_LC;
    options.lzma_options.lp = LZMA_LP;
    options.lzma_options.pb = LZMA_PB;
    let mut compressed = Vec::new();
    let mut writer = lzma_rust2::Lzma2Writer::new(&mut compressed, options);
    writer
        .write_all(&filtered)
        .context("Compressing release stream")?;
    writer.finish().context("Finishing release stream")?;
    Ok(compressed)
}

/// Inflate `packed` as the unpacker will, requiring exactly `expected`, and
/// return the least erase margin when `record` is given (the main stream).
fn rehearse(packed: &[u8], expected: &[u8], record: Option<&Packed>) -> Result<usize> {
    let mut window = vec![0; WINDOW_BYTES];
    let window: &mut [u8; WINDOW_BYTES] = window.as_mut_slice().try_into()?;
    let (mut inflated, mut margin) = (Vec::with_capacity(expected.len()), usize::MAX);
    inflate_sectors(packed, window, |chunk| {
        inflated.extend_from_slice(chunk.bytes);
        match record {
            None => true,
            Some(record) => main_erase_margin(record, chunk.offset, chunk.consumed)
                .map(|left| margin = margin.min(left))
                .is_some(),
        }
    })
    .map_err(|error| anyhow::anyhow!("Rehearsing the unpacker: {error:?}"))?;
    ensure!(
        inflated == expected,
        "A stream does not inflate to its region"
    );
    Ok(margin)
}

/// A whole-word region's stream record.
fn stream(region: &[u8], packed: &[u8]) -> Result<Stream> {
    Ok(Stream {
        len: u32::try_from(region.len())?,
        hash: region_hash(region),
        packed: u32::try_from(packed.len())?,
    })
}

/// Pad to whole words with zeros; the unpacker writes words.
fn whole_words(mut bytes: Vec<u8>) -> Vec<u8> {
    bytes.resize(bytes.len().next_multiple_of(size_of::<u32>()), 0);
    bytes
}

/// A release's application image: the unpacker, its record filled in, then
/// the upper and main streams. Returns it with the main firmware's checks.
fn release(firmware: &[u8], unpacker: &[u8]) -> Result<(Vec<u8>, Application, Release)> {
    let (main, upper) = split(firmware).context("Splitting the firmware")?;
    let (mut main, upper) = (whole_words(main), whole_words(upper));
    ensure!(!upper.is_empty(), "The firmware has no upper region");
    ensure!(
        upper.len() <= (UPPER_END - UPPER_BASE) as usize,
        "The upper region exceeds its flash range"
    );
    let upper_id = [u32::try_from(upper.len())?, region_hash(&upper)];
    ensure!(
        fill_record(&mut main, &UPPER_ID_MAGIC, UPPER_ID_UNSET, &upper_id)?,
        "The firmware has no upper record"
    );
    let application = check_application(&main)?;

    let (mut content, unpacker_upper) = split(unpacker).context("Splitting the unpacker")?;
    ensure!(
        unpacker_upper.is_empty(),
        "The unpacker must not run code from the upper region"
    );
    let unpacker_bytes = content.len();
    check_unpacker(&content)?;
    let (upper_packed, main_packed) = (deflate(&upper)?, deflate(&main)?);
    let record = Packed {
        magic: PACKED_MAGIC,
        data: APP_BASE + u32::try_from(unpacker_bytes.next_multiple_of(size_of::<u32>()))?,
        upper: stream(&upper, &upper_packed)?,
        main: stream(&main, &main_packed)?,
    };
    let fields = [
        record.data,
        record.upper.len,
        record.upper.hash,
        record.upper.packed,
        record.main.len,
        record.main.hash,
        record.main.packed,
    ];
    ensure!(
        fill_record(&mut content, &PACKED_MAGIC, PACKED_UNSET, &fields)?,
        "The unpacker has no record"
    );
    content.resize((record.data - APP_BASE) as usize, 0);
    content.extend_from_slice(&upper_packed);
    content.extend_from_slice(&main_packed);
    ensure!(
        content.len() <= APP_LIMIT,
        "The release exceeds the application space by {} bytes",
        content.len() - APP_LIMIT
    );
    rehearse(&upper_packed, &upper, None)?;
    let erase_margin_bytes = rehearse(&main_packed, &main, Some(&record))?;
    let release = Release {
        unpacker_bytes,
        main_bytes: main.len(),
        main_packed_bytes: main_packed.len(),
        upper_bytes: upper.len(),
        upper_packed_bytes: upper_packed.len(),
        upper_unused_bytes: (UPPER_END - UPPER_BASE) as usize - upper.len(),
        erase_margin_bytes,
    };
    Ok((content, application, release))
}

/// The unpacker starts on its own stack in its RAM, at a Thumb reset
/// handler inside its image.
fn check_unpacker(unpacker: &[u8]) -> Result<()> {
    let word = |index: usize| -> Result<u32> {
        let bytes = unpacker
            .get(index * size_of::<u32>()..(index + 1) * size_of::<u32>())
            .context("The unpacker is truncated")?;
        Ok(u32::from_le_bytes(bytes.try_into()?))
    };
    let (stack, reset) = (word(0)?, word(1)?);
    ensure!(
        STACK_RAM_START < stack && stack < UNPACKER_RAM_END,
        "The unpacker's stack is not strictly inside its RAM"
    );
    ensure!(
        reset & 1 != 0 && APP_BASE < reset && reset < APP_BASE + unpacker.len() as u32,
        "The unpacker's reset vector is not a Thumb address inside it"
    );
    Ok(())
}

/// The container for `source`, with its report.
fn prepare(source: &Source) -> Result<(Vec<u8>, Report)> {
    let (content, application, release) = match source {
        Source { app: Some(app), .. } => {
            let mut app = fs::read(app).context("Reading application")?;
            ensure!(
                !fill_record(
                    &mut app,
                    &UPPER_ID_MAGIC,
                    UPPER_ID_UNSET,
                    &[UPPER_ID_UNSET; 2]
                )?,
                "This build runs code from the upper region; pack its ELF with --firmware and --unpacker"
            );
            let application = check_application(&app)?;
            (app, application, None)
        }
        Source {
            firmware: Some(firmware),
            unpacker: Some(unpacker),
            ..
        } => {
            let firmware = fs::read(firmware).context("Reading firmware ELF")?;
            let unpacker = fs::read(unpacker).context("Reading unpacker ELF")?;
            let (content, application, release) = release(&firmware, &unpacker)?;
            (content, application, Some(release))
        }
        _ => anyhow::bail!("Pass --app, or --firmware with --unpacker"),
    };
    let sectors = image::erase_sectors_for(EXTENDED_PAYLOAD_LEN);
    let end = APP_BASE + sectors * SECTOR_BYTES;
    ensure!(
        end <= ERASE_CEILING,
        "Erase envelope exceeds the application ceiling"
    );
    let mut container = vec![0; CONTAINER_PREFIX.len() + EXTENDED_PAYLOAD_LEN];
    image::build_container(&content, &mut container)
        .map_err(|error| anyhow::anyhow!("Building container: {error:?}"))?;
    image::verify_container(&container)
        .map_err(|error| anyhow::anyhow!("Invalid generated container: {error:?}"))?;
    let report = Report {
        sha256: digest(&container),
        payload_bytes: EXTENDED_PAYLOAD_LEN,
        erase_sectors: sectors,
        erase_envelope: [APP_BASE, end],
        application,
        application_bytes: content.len(),
        application_sha256: digest(&content),
        unused_content_bytes: APP_LIMIT - content.len(),
        descriptor_bytes: image::DESCRIPTOR.len(),
        release,
        candidate: None,
        status: "prepared, not flashed",
    };
    Ok((container, report))
}

/// Write `bytes` to a new file at `path`, creating its directory.
fn write_new(path: &Path, bytes: &[u8]) -> Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).context("Creating output directory")?;
    }
    fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .and_then(|mut output| output.write_all(bytes))
        .context("Creating output; existing files are never overwritten")
}

fn run(args: Args) -> Result<()> {
    let report = match args.command {
        Command::Pack { source, out } => {
            let (container, mut report) = prepare(&source)?;
            write_new(&out, &container)?;
            report.candidate = Some(out);
            report
        }
        Command::Audit {
            source,
            image,
            confirm_sha256,
        } => {
            let candidate = fs::read(image).context("Reading image")?;
            let actual = digest(&candidate);
            ensure!(
                actual == confirm_sha256,
                "Image SHA-256 is {actual}, not the confirmed {confirm_sha256}"
            );
            let (expected, mut report) = prepare(&source)?;
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
