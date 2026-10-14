//! Black-box regressions for destructive output handling and canonical image auditing.
//! Synthetic files only; this executable has no USB access.

use anyhow::Result;
use sha2::{Digest, Sha256};
use std::{fs, path::PathBuf, process::Command};
use tempfile::TempDir;

// Independent encoding of the established checksum/marker offsets, canonical
// 64-byte-aligned payload, executable ceiling, and an unused content location.
const CHECKSUM: usize = 0x3340c;
const MARKER: usize = 0x33410;
const PAYLOAD_BYTES: usize = 0x33640;
const APP_LIMIT: usize = 0x333fc;
const UNUSED_CONTENT: usize = 0x24000;
// Independent encoding of the observed footer, so builder changes cannot alter the fixture.
const DESCRIPTOR: [u8; 16] = [
    0xcb, 0x6e, 0xda, 0x43, 0xc3, 0xfa, 0x35, 0x1c, 0x9f, 0x57, 0x6a, 0x3c, 0x01, 0x00, 0x00, 0x00,
];
// The fixture has 160 vectors and the firmware's permitted custom IRQ lines.
const VECTOR_COUNT: usize = 160;
const CUSTOM_IRQS: [usize; 15] = [
    4, 13, 31, 38, 39, 40, 41, 86, 87, 88, 110, 111, 112, 113, 116,
];

fn digest(data: &[u8]) -> String {
    Sha256::digest(data)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn container(payload: &mut [u8]) -> Vec<u8> {
    let checksum = payload[..CHECKSUM]
        .iter()
        .fold(0u32, |sum, byte| sum.wrapping_add(u32::from(*byte)));
    payload[CHECKSUM..CHECKSUM + 4].copy_from_slice(&checksum.to_be_bytes());
    let mut image = vec![0x00, 0x0a];
    image.extend_from_slice(payload);
    image
}

struct Fixture {
    _directory: TempDir,
    app: PathBuf,
    image: PathBuf,
}

impl Fixture {
    fn new() -> Result<Self> {
        let directory = TempDir::new()?;
        let app = directory.path().join("app.bin");
        let image = directory.path().join("candidate.X1C");
        // A 1 KiB synthetic application with Thumb vectors inside its own code.
        let mut content = vec![0; 1024];
        let mut vectors = [0x10301u32; VECTOR_COUNT];
        vectors[0] = 0x2000ef00;
        vectors[1..16].fill(0);
        vectors[1] = 0x10281;
        vectors[15] = 0x10305;
        for line in CUSTOM_IRQS {
            vectors[16 + line] = 0x10303;
        }
        for (index, vector) in vectors.iter().enumerate() {
            content[index * 4..(index + 1) * 4].copy_from_slice(&vector.to_le_bytes());
        }
        fs::write(&app, content)?;
        Ok(Self {
            _directory: directory,
            app,
            image,
        })
    }

    fn pack(&self) -> Result<std::process::Output> {
        Ok(Command::new(env!("CARGO_BIN_EXE_firmware-pack"))
            .args(["pack", "--app"])
            .arg(&self.app)
            .arg("--out")
            .arg(&self.image)
            .output()?)
    }

    fn audit(&self, hash: &str) -> Result<std::process::Output> {
        Ok(Command::new(env!("CARGO_BIN_EXE_firmware-pack"))
            .args(["audit", "--app"])
            .arg(&self.app)
            .arg("--image")
            .arg(&self.image)
            .args(["--confirm-sha256", hash])
            .output()?)
    }
}

#[test]
fn generated_container_matches_the_standalone_layout() -> Result<()> {
    let fixture = Fixture::new()?;
    let result = fixture.pack()?;
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let app = fs::read(&fixture.app)?;
    let mut payload = vec![0; PAYLOAD_BYTES];
    payload[..app.len()].copy_from_slice(&app);
    payload[APP_LIMIT..CHECKSUM].copy_from_slice(&DESCRIPTOR);
    payload[MARKER] = 0x80;
    let image = fs::read(&fixture.image)?;
    assert_eq!(image, container(&mut payload));
    let report: serde_json::Value = serde_json::from_slice(&result.stdout)?;
    assert_eq!(report["sha256"], digest(&image));
    let audit = fixture.audit(&digest(&image))?;
    assert!(
        audit.status.success(),
        "{}",
        String::from_utf8_lossy(&audit.stderr)
    );
    let report: serde_json::Value = serde_json::from_slice(&audit.stdout)?;
    assert_eq!(
        report["status"],
        "structural audit passed; hardware not validated"
    );
    Ok(())
}

// Protect the reserved descriptor from an application reaching the linker ceiling.
#[test]
fn application_at_size_limit_preserves_footer() -> Result<()> {
    let fixture = Fixture::new()?;
    let mut app = fs::read(&fixture.app)?;
    app.resize(APP_LIMIT, 1);
    fs::write(&fixture.app, &app)?;
    assert!(fixture.pack()?.status.success());
    let image = fs::read(&fixture.image)?;
    assert_eq!(&image[2..2 + APP_LIMIT], app.as_slice());
    assert_eq!(&image[2 + APP_LIMIT..2 + CHECKSUM], &DESCRIPTOR);
    assert!(fixture.audit(&digest(&image))?.status.success());
    Ok(())
}

#[test]
fn audit_rejects_an_unconfirmed_image() -> Result<()> {
    let fixture = Fixture::new()?;
    assert!(fixture.pack()?.status.success());
    // SHA-256 is 64 hex digits; this deliberately disagrees with the fixture.
    let result = fixture.audit(&"0".repeat(64))?;
    assert!(!result.status.success());
    assert!(String::from_utf8_lossy(&result.stderr).contains("not the confirmed"));
    Ok(())
}

#[test]
fn invalid_binary_is_rejected_before_writing() -> Result<()> {
    let fixture = Fixture::new()?;
    let good = fs::read(&fixture.app)?;
    let mut oversized = good.clone();
    oversized.resize(APP_LIMIT + 1, 0);
    let mut wrong_irq = good.clone();
    let offset = (16 + CUSTOM_IRQS[0]) * 4;
    wrong_irq[offset..offset + 4].copy_from_slice(&0x10301u32.to_le_bytes());
    for app in [good[..VECTOR_COUNT * 4 - 1].to_vec(), oversized, wrong_irq] {
        fs::write(&fixture.app, app)?;
        assert!(!fixture.pack()?.status.success());
        assert!(!fixture.image.exists());
    }
    Ok(())
}

#[test]
fn audit_rejects_malformed_or_overlong_containers() -> Result<()> {
    let fixture = Fixture::new()?;
    assert!(fixture.pack()?.status.success());
    let good = fs::read(&fixture.image)?;
    let mut bad_prefix = good.clone();
    bad_prefix[..2].copy_from_slice(b"xx");
    let mut bad_checksum = good.clone();
    bad_checksum[2 + CHECKSUM] ^= 1;
    let mut bad_trailer = good.clone();
    bad_trailer[good.len() - 1] = 1;
    let mut extra_chunk = good.clone();
    extra_chunk.extend_from_slice(&[0; 64]);
    let mut beyond_erase_ceiling = good.clone();
    // Four extra KiB force another erase sector beyond the permitted ceiling.
    beyond_erase_ceiling.extend_from_slice(&[0; 4096]);
    for image in [
        bad_prefix,
        good[..good.len() - 1].to_vec(),
        bad_checksum,
        bad_trailer,
        extra_chunk,
        beyond_erase_ceiling,
    ] {
        fs::write(&fixture.image, &image)?;
        assert!(!fixture.audit(&digest(&image))?.status.success());
    }
    Ok(())
}

#[test]
fn existing_output_is_not_overwritten() -> Result<()> {
    let fixture = Fixture::new()?;
    fs::write(&fixture.image, b"keep this file")?;
    assert!(!fixture.pack()?.status.success());
    assert_eq!(fs::read(&fixture.image)?, b"keep this file");
    Ok(())
}

#[test]
fn audit_rejects_application_change_with_valid_checksum_and_hash() -> Result<()> {
    let fixture = Fixture::new()?;
    assert!(fixture.pack()?.status.success());
    let image = fs::read(&fixture.image)?;
    let mut payload = image[2..].to_vec();
    // First byte after the independent fixture's vector table.
    payload[VECTOR_COUNT * 4] ^= 1;
    let modified = container(&mut payload);
    fs::write(&fixture.image, &modified)?;
    assert!(!fixture.audit(&digest(&modified))?.status.success());
    Ok(())
}

#[test]
fn audit_rejects_unused_content_change_with_valid_checksum_and_hash() -> Result<()> {
    let fixture = Fixture::new()?;
    assert!(fixture.pack()?.status.success());
    let image = fs::read(&fixture.image)?;
    let mut payload = image[2..].to_vec();
    payload[UNUSED_CONTENT] ^= 1;
    let modified = container(&mut payload);
    fs::write(&fixture.image, &modified)?;
    assert!(!fixture.audit(&digest(&modified))?.status.success());
    Ok(())
}
