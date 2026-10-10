//! Application image trailer and container framing.
//!
//! Generated layout accepted by the bootloader in a supervised Linux flash:
//!
//! ```text
//! container := 0x00 0x0a ++ payload
//! payload   := application ++ zero_fill ++ descriptor[16] ++ checksum[4] ++ 0x80 ++ zeros[559]
//! descriptor starts at 0x333fc; checksum starts at 0x3340c
//! checksum  := big-endian sum of bytes before the checksum, modulo 2^32
//! ```
//!
//! The descriptor's fields are not decoded; no baseline application code or data
//! is embedded. Acceptance of other application builds still requires validation.

#![no_std]

/// Observed 16-byte footer descriptor immediately before the checksum.
/// Retained as protocol metadata; its individual fields have not been isolated.
pub const DESCRIPTOR: [u8; 16] = [
    0xcb, 0x6e, 0xda, 0x43, 0xc3, 0xfa, 0x35, 0x1c, 0x9f, 0x57, 0x6a, 0x3c, 0x01, 0x00, 0x00, 0x00,
];
/// Footer descriptor start, derived from the established checksum offset.
pub const DESCRIPTOR_OFFSET: usize = CHECKSUM_OFFSET - DESCRIPTOR.len();
/// Application ceiling protecting the descriptor; matches `firmware/memory.x`.
pub const APP_LIMIT: usize = DESCRIPTOR_OFFSET;
/// Original content region plus checksum and trailer (`0x33420`).
pub const PAYLOAD_LEN: usize = 0x33420;
/// Bootloader HID payload bytes per report, excluding the report ID.
pub const CHUNK_BYTES: usize = 64;
/// Bootloader application erase sector size in bytes (8 KiB).
pub const SECTOR_BYTES: u32 = 0x2000;
/// Exclusive flash ceiling for application erasure, protecting higher storage.
pub const ERASE_CEILING: u32 = 0x44000;
/// Extended payload length matching the hardware-qualified container (`0x33640` / 210,496 bytes).
pub const EXTENDED_PAYLOAD_LEN: usize = 0x33640;
/// Application base address in flash (`0x10000`), where the bootloader enters it.
pub const APP_BASE: u32 = 0x10000;
/// First byte after the application content: the stored checksum.
pub const CHECKSUM_OFFSET: usize = 0x3340c;
/// Byte holding the `0x80` marker, followed by zero alignment bytes.
pub const MARKER_OFFSET: usize = 0x33410;
pub const MARKER: u8 = 0x80;
/// Opaque container prefix; the host flasher passes it to bootloader command 107 unchanged.
pub const CONTAINER_PREFIX: [u8; 2] = [0x00, 0x0a];

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
    if content.len() > APP_LIMIT {
        return Err(ImageError::ContentTooLarge { len: content.len() });
    }
    Ok(())
}

/// Write a full payload (content, zero fill, descriptor, checksum and trailer) into `out`.
pub fn build_payload(content: &[u8], out: &mut [u8]) -> Result<(), ImageError> {
    check_content(content)?;
    if out.len() != EXTENDED_PAYLOAD_LEN {
        return Err(ImageError::WrongBufferLength { len: out.len() });
    }
    out.fill(0);
    out[..content.len()].copy_from_slice(content);
    out[DESCRIPTOR_OFFSET..CHECKSUM_OFFSET].copy_from_slice(&DESCRIPTOR);
    let checksum = additive_checksum(&out[..CHECKSUM_OFFSET]);
    out[CHECKSUM_OFFSET..CHECKSUM_OFFSET + 4].copy_from_slice(&checksum.to_be_bytes());
    out[MARKER_OFFSET] = MARKER;
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
