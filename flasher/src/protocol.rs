use crate::image::{CHUNK_BYTES, Plan};
use anyhow::{Result, bail, ensure};
use firmware_image::CONTAINER_PREFIX;
use std::{
    thread,
    time::{Duration, Instant},
};

// Command header, codes and batch size from the established bootloader protocol.
const HEADER: [u8; 3] = [0x05, 0x11, 0x09];
const INFO: u8 = 100;
const ERASE: u8 = 107;
const ANNOUNCE: u8 = 101;
const CHECKSUM: u8 = 103;
const FINISH: u8 = 105;
const REBOOT: u8 = 255;
const MODE_RESPONSE: u8 = 0;
const BOOTLOADER: u8 = 0;
const SUCCESS: u8 = 1;
const CHECKSUM_ACCEPTED: u8 = 2;
const VERIFY_FAILED: u8 = 4;
const ERROR_START: u8 = 240;
const BATCH_CHUNKS: usize = 128;
// Existing transport pacing and response deadlines, in milliseconds/seconds.
const CHUNK_DELAY: Duration = Duration::from_millis(5);
// Erase settle margin in milliseconds, exercised in successful Linux flashes.
const ERASE_DELAY: Duration = Duration::from_millis(250);
// Final programming settle margin in milliseconds before read-back verification,
// exercised in successful Linux flashes of generated and stock images.
const FINISH_DELAY: Duration = Duration::from_millis(1000);
const DRAIN_TIMEOUT: Duration = Duration::from_millis(200);
const DRAIN_LIMIT: usize = 64;
const INFO_TIMEOUT: Duration = Duration::from_secs(2);
const ERASE_TIMEOUT: Duration = Duration::from_secs(5);
const ANNOUNCE_TIMEOUT: Duration = Duration::from_secs(2);
const CHECKSUM_TIMEOUT: Duration = Duration::from_secs(3);
const FINISH_TIMEOUT: Duration = Duration::from_secs(5);

pub trait Transport {
    fn write(&self, payload: &[u8]) -> Result<()>;
    fn read(&self, timeout: Duration) -> Result<Option<[u8; CHUNK_BYTES]>>;
}

impl Transport for hidapi::HidDevice {
    fn write(&self, payload: &[u8]) -> Result<()> {
        ensure!(payload.len() <= CHUNK_BYTES, "Report larger than 64 bytes");
        // HIDAPI requires a report ID byte; this protocol uses unnumbered reports.
        let mut report = [0; CHUNK_BYTES + 1];
        report[1..1 + payload.len()].copy_from_slice(payload);
        let written = self.write(&report)?;
        ensure!(
            written == report.len(),
            "Short HID write: {written}/65 bytes; stop without retry"
        );
        Ok(())
    }

    fn read(&self, timeout: Duration) -> Result<Option<[u8; CHUNK_BYTES]>> {
        let mut report = [0; CHUNK_BYTES];
        let millis = i32::try_from(timeout.as_millis().max(1))?;
        let read = self.read_timeout(&mut report, millis)?;
        Ok((read == CHUNK_BYTES && report[..HEADER.len()] == HEADER).then_some(report))
    }
}

fn packet(command: u8, arguments: &[u8]) -> Result<[u8; CHUNK_BYTES]> {
    // Header, command and final checksum leave 59 argument bytes.
    const MAX_ARGUMENTS: usize = CHUNK_BYTES - HEADER.len() - 2;
    ensure!(
        arguments.len() <= MAX_ARGUMENTS,
        "Arguments do not fit a control report"
    );
    let mut report = [0; CHUNK_BYTES];
    report[..HEADER.len()].copy_from_slice(&HEADER);
    report[HEADER.len()] = command;
    report[HEADER.len() + 1..HEADER.len() + 1 + arguments.len()].copy_from_slice(arguments);
    report[CHUNK_BYTES - 1] = arguments
        .iter()
        .fold(command, |sum, byte| sum.wrapping_add(*byte));
    Ok(report)
}

fn response(
    device: &impl Transport,
    timeout: Duration,
    codes: &[u8],
    what: &str,
) -> Result<[u8; CHUNK_BYTES]> {
    let deadline = Instant::now() + timeout;
    while let Some(remaining) = deadline.checked_duration_since(Instant::now()) {
        let Some(report) = device.read(remaining)? else {
            continue;
        };
        let code = report[HEADER.len()];
        // Control responses checksum the command and arguments, excluding the header.
        let checksum = report[HEADER.len()..CHUNK_BYTES - 1]
            .iter()
            .fold(0u8, |sum, byte| sum.wrapping_add(*byte));
        ensure!(
            checksum == report[CHUNK_BYTES - 1],
            "{what}: invalid response checksum; stop without retry"
        );
        ensure!(
            code < ERROR_START,
            "{what}: device reported error code {code}"
        );
        if codes.contains(&code) {
            return Ok(report);
        }
        tracing::info!(code, "{what}: ignoring informational response");
    }
    bail!("{what}: no response within {:.1}s", timeout.as_secs_f32())
}

pub fn query_info(device: &impl Transport) -> Result<u8> {
    let deadline = Instant::now() + DRAIN_TIMEOUT;
    for _ in 0..DRAIN_LIMIT {
        let Some(remaining) = deadline.checked_duration_since(Instant::now()) else {
            break;
        };
        let _ = device.read(remaining)?;
    }
    device.write(&packet(INFO, &[])?)?;
    let report = response(device, INFO_TIMEOUT, &[MODE_RESPONSE], "info")?;
    Ok(report[HEADER.len() + 1])
}

pub fn flash(plan: &Plan, device: &impl Transport) -> Result<()> {
    ensure!(
        query_info(device)? == BOOTLOADER,
        "Device is not in Bootloader mode"
    );
    tracing::info!(sectors = plan.sectors, "Erasing application sectors");
    let arguments = [plan.sectors, CONTAINER_PREFIX[0], CONTAINER_PREFIX[1]];
    device.write(&packet(ERASE, &arguments)?)?;
    response(device, ERASE_TIMEOUT, &[SUCCESS], "erase")?;
    thread::sleep(ERASE_DELAY);
    for (index, batch) in plan.payload.chunks(CHUNK_BYTES * BATCH_CHUNKS).enumerate() {
        let number = index + 1;
        device.write(&packet(
            ANNOUNCE,
            &[batch.len().div_ceil(CHUNK_BYTES) as u8, number as u8],
        )?)?;
        response(device, ANNOUNCE_TIMEOUT, &[SUCCESS], "batch announcement")?;
        for chunk in batch.chunks(CHUNK_BYTES) {
            device.write(chunk)?;
            thread::sleep(CHUNK_DELAY);
        }
        let checksum = batch
            .iter()
            .fold(0u16, |sum, byte| sum.wrapping_add(u16::from(*byte)));
        device.write(&packet(CHECKSUM, &checksum.to_le_bytes())?)?;
        response(
            device,
            CHECKSUM_TIMEOUT,
            &[CHECKSUM_ACCEPTED],
            "batch checksum",
        )?;
        tracing::info!(batch = number, "Batch written and checksum accepted");
    }
    thread::sleep(FINISH_DELAY);
    device.write(&packet(FINISH, &[])?)?;
    let report = response(device, FINISH_TIMEOUT, &[SUCCESS, VERIFY_FAILED], "finish")?;
    ensure!(
        report[HEADER.len()] != VERIFY_FAILED,
        "Bootloader verification failed (code 4). Stop without reboot, retry or restoration; recovery requires separate exact-image authorization"
    );
    device.write(&packet(REBOOT, &[])?)?;
    tracing::info!("Bootloader accepted the image; reboot command sent");
    Ok(())
}
