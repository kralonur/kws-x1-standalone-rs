use anyhow::{Context, Result, ensure};
use firmware_image::{self as image, APP_BASE, CONTAINER_PREFIX, MARKER_OFFSET};
use sha2::{Digest, Sha256};
use std::{fs, path::Path};

// Existing HID chunk size and application erase limits from tools/package_app.py.
pub const CHUNK_BYTES: usize = 64;
pub const ERASE_CEILING: u32 = 0x44000;
pub const SECTOR_BYTES: u32 = 0x2000;

pub struct Plan {
    pub payload: Vec<u8>,
    pub digest: String,
    pub sectors: u8,
}

impl Plan {
    pub fn load(path: &Path, confirmed: &str) -> Result<Self> {
        ensure!(path.is_file(), "Image must be a regular file");
        let data = fs::read(path).context("Reading image")?;
        let digest: String = Sha256::digest(&data)
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect();
        ensure!(
            digest == confirmed,
            "Image SHA-256 is {digest}, not the confirmed {confirmed}"
        );
        image::verify_container(&data)
            .map_err(|error| anyhow::anyhow!("Invalid container: {error:?}"))?;
        let payload = &data[CONTAINER_PREFIX.len()..];
        ensure!(
            payload.len() <= (ERASE_CEILING - APP_BASE) as usize,
            "Payload exceeds the application erase envelope"
        );
        let sectors = image::erase_sectors_for(payload.len());
        ensure!(
            APP_BASE + sectors * SECTOR_BYTES <= ERASE_CEILING,
            "Erase envelope exceeds the protected application ceiling"
        );
        ensure!(
            payload[MARKER_OFFSET + 1..].iter().all(|byte| *byte == 0),
            "Nonzero trailer/alignment bytes"
        );
        Ok(Self {
            payload: payload.to_vec(),
            digest,
            sectors: u8::try_from(sectors)?,
        })
    }
}
