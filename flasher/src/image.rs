use anyhow::{Context, Result, ensure};
use firmware_image::{self as image, APP_BASE, CONTAINER_PREFIX, MARKER_OFFSET, link};
use sha2::{Digest, Sha256};
use std::{fs, path::Path};

pub use firmware_image::{CHUNK_BYTES, ERASE_CEILING, SECTOR_BYTES};

pub struct Plan {
    pub payload: Vec<u8>,
    pub digest: String,
    pub sectors: u8,
    /// A USB install file's record words and upper region.
    pub usb: Option<crate::link::Part>,
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
        let (container, section) = link::split(&data).map_err(|error| match error {
            link::SectionError::Missing => anyhow::anyhow!(
                "USB install image without its appended upper region; it cannot complete"
            ),
            link::SectionError::Invalid => {
                anyhow::anyhow!("Invalid USB install section after the container")
            }
        })?;
        image::verify_container(container)
            .map_err(|error| anyhow::anyhow!("Invalid container: {error:?}"))?;
        let payload = &container[CONTAINER_PREFIX.len()..];
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
            usb: section.map(|section| crate::link::Part {
                identity: section.identity,
                upper: section.upper.to_vec(),
            }),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firmware_image::{
        EXTENDED_PAYLOAD_LEN, build_container,
        packed::{PACKED_MAGIC, UPPER_OVER_USB},
        region_hash,
    };
    use std::process;

    fn load(name: &str, file: &[u8]) -> Result<Plan> {
        let path = std::env::temp_dir().join(format!("kws-x1-{name}-{}.X1C", process::id()));
        fs::write(&path, file)?;
        let digest: String = Sha256::digest(file)
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect();
        let plan = Plan::load(&path, &digest);
        fs::remove_file(&path)?;
        plan
    }

    /// A container whose record has `upper_packed`, and an upper region for it.
    fn container(upper_packed: u32) -> (Vec<u8>, [u32; 7], Vec<u8>) {
        let upper: Vec<u8> = (0..4096u32).map(|index| (index * 7 % 253) as u8).collect();
        let identity = [
            APP_BASE + 0x6000,
            upper.len() as u32,
            region_hash(&upper),
            upper_packed,
            0x2_8000,
            0x1234_5678,
            0x1_8000,
        ];
        let mut content = vec![0x5a; 0x7000];
        let record: Vec<u8> = PACKED_MAGIC
            .iter()
            .chain(&identity)
            .flat_map(|word| word.to_le_bytes())
            .collect();
        content[0x400..0x400 + record.len()].copy_from_slice(&record);
        let mut out = vec![0; CONTAINER_PREFIX.len() + EXTENDED_PAYLOAD_LEN];
        build_container(&content, &mut out).unwrap();
        (out, identity, upper)
    }

    #[test]
    fn rejects_a_usb_install_image_without_its_upper_region() {
        let (file, _, _) = container(UPPER_OVER_USB);
        let error = load("missing", &file).err().unwrap().to_string();
        assert!(
            error.contains("without its appended upper region"),
            "{error}"
        );
    }

    #[test]
    fn loads_usb_install_and_normal_images() {
        let (mut file, identity, upper) = container(UPPER_OVER_USB);
        file.extend_from_slice(&link::section_header(&identity));
        file.extend_from_slice(&upper);
        let usb = load("usb", &file).unwrap().usb.unwrap();
        assert_eq!((usb.identity, usb.upper), (identity, upper));
        let (file, _, _) = container(0x1_0000);
        assert!(load("normal", &file).unwrap().usb.is_none());
    }
}
