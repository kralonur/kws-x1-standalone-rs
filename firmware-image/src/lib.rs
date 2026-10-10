//! Application image trailer and container framing.
//!
//! The byte layout mirrors the preserved original container exactly:
//!
//! ```text
//! container := 0x00 0x0a ++ payload
//! payload   := content[0x3340c] ++ checksum[4] ++ 0x80 ++ 0x00[15]
//! checksum  := big-endian sum of content bytes, modulo 2^32
//! ```
//!
//! Bootloader checksum necessity has not been isolated experimentally. The original
//! application never reads it; error 245 is handled during erase/preparation, not
//! final verification. This crate reproduces the original layout instead of inventing one.

#![no_std]

/// Original content region plus checksum and trailer (`0x33420`).
pub const PAYLOAD_LEN: usize = 0x33420;
/// Bootloader HID payload bytes per report, excluding the report ID.
pub const CHUNK_BYTES: usize = 64;
/// Bootloader application erase sector size in bytes (8 KiB).
pub const SECTOR_BYTES: u32 = 0x2000;
/// Exclusive flash ceiling for application erasure, protecting higher storage.
pub const ERASE_CEILING: u32 = 0x44000;
/// Original length rounded up to a 64-byte chunk multiple, zero padded.
/// This is a conservative tooling policy, not an acceptance guarantee or a proven
/// universal bootloader requirement; see `docs/SAFETY.md`.
pub const EXTENDED_PAYLOAD_LEN: usize = PAYLOAD_LEN.div_ceil(64) * 64;
/// Application base address in flash (`0x10000`), where the bootloader enters it.
pub const APP_BASE: u32 = 0x10000;
/// First byte after the application content: the stored checksum.
pub const CHECKSUM_OFFSET: usize = 0x3340c;
/// Byte holding the `0x80` marker, followed by fifteen zero bytes.
pub const MARKER_OFFSET: usize = 0x33410;
pub const MARKER: u8 = 0x80;
/// Opaque container prefix; the host flasher passes it to bootloader command 107 unchanged.
pub const CONTAINER_PREFIX: [u8; 2] = [0x00, 0x0a];
/// Filler for unused content space between the application and the checksum.
pub const FILLER: u8 = 0xff;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageError {
    EmptyContent,
    ContentTooLarge { len: usize },
    WrongBufferLength { len: usize },
    WrongPrefix,
    BadChecksum { stored: u32, computed: u32 },
    BadMarker { found: u8 },
    NonZeroTrailer,
}

/// Additive checksum: the sum of the content bytes, modulo 2^32.
pub fn additive_checksum(content: &[u8]) -> u32 {
    content
        .iter()
        .fold(0u32, |sum, byte| sum.wrapping_add(*byte as u32))
}

/// Sector count the host flasher derives from a payload length.
///
/// This mirrors the host flasher (`chunks // 128 + 1`, 64-byte HID chunks) and is
/// a host-side convention, not a documented bootloader rule.
pub fn erase_sectors_for(payload_len: usize) -> u32 {
    let chunks = payload_len.div_ceil(CHUNK_BYTES);
    (chunks / (SECTOR_BYTES as usize / CHUNK_BYTES)) as u32 + 1
}

fn check_content(content: &[u8]) -> Result<(), ImageError> {
    if content.is_empty() {
        return Err(ImageError::EmptyContent);
    }
    if content.len() > CHECKSUM_OFFSET {
        return Err(ImageError::ContentTooLarge { len: content.len() });
    }
    Ok(())
}

/// Write a full payload (content, filler, checksum, trailer, zero alignment) into `out`.
pub fn build_payload(content: &[u8], out: &mut [u8]) -> Result<(), ImageError> {
    check_content(content)?;
    if out.len() != EXTENDED_PAYLOAD_LEN {
        return Err(ImageError::WrongBufferLength { len: out.len() });
    }
    out.fill(FILLER);
    out[..content.len()].copy_from_slice(content);
    let checksum = additive_checksum(&out[..CHECKSUM_OFFSET]);
    out[CHECKSUM_OFFSET..CHECKSUM_OFFSET + 4].copy_from_slice(&checksum.to_be_bytes());
    out[MARKER_OFFSET] = MARKER;
    // Marker, fifteen zero bytes, filler to the original length, then the zero
    // alignment bytes required by tooling policy.
    out[MARKER_OFFSET + 1..PAYLOAD_LEN].fill(0);
    out[PAYLOAD_LEN..].fill(0);
    Ok(())
}

/// Write a full container (prefix plus payload) into `out`.
pub fn build_container(content: &[u8], out: &mut [u8]) -> Result<(), ImageError> {
    if out.len() != EXTENDED_PAYLOAD_LEN + CONTAINER_PREFIX.len() {
        return Err(ImageError::WrongBufferLength { len: out.len() });
    }
    let (prefix, payload) = out.split_at_mut(CONTAINER_PREFIX.len());
    prefix.copy_from_slice(&CONTAINER_PREFIX);
    build_payload(content, payload)
}

/// Validate a payload built by [`build_payload`], or the preserved original payload.
pub fn verify_payload(payload: &[u8]) -> Result<(), ImageError> {
    if payload.len() < PAYLOAD_LEN {
        return Err(ImageError::WrongBufferLength { len: payload.len() });
    }
    let Some(&stored) = payload[CHECKSUM_OFFSET..].first_chunk() else {
        return Err(ImageError::WrongBufferLength { len: payload.len() });
    };
    let stored = u32::from_be_bytes(stored);
    let computed = additive_checksum(&payload[..CHECKSUM_OFFSET]);
    if stored != computed {
        return Err(ImageError::BadChecksum { stored, computed });
    }
    if payload[MARKER_OFFSET] != MARKER {
        return Err(ImageError::BadMarker {
            found: payload[MARKER_OFFSET],
        });
    }
    if payload[MARKER_OFFSET + 1..PAYLOAD_LEN]
        .iter()
        .any(|byte| *byte != 0)
    {
        return Err(ImageError::NonZeroTrailer);
    }
    Ok(())
}

/// Validate a container (prefix plus payload).
pub fn verify_container(container: &[u8]) -> Result<(), ImageError> {
    if container.len() < PAYLOAD_LEN + CONTAINER_PREFIX.len() {
        return Err(ImageError::WrongBufferLength {
            len: container.len(),
        });
    }
    if container[..CONTAINER_PREFIX.len()] != CONTAINER_PREFIX {
        return Err(ImageError::WrongPrefix);
    }
    verify_payload(&container[CONTAINER_PREFIX.len()..])
}
