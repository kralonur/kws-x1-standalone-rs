use firmware_core::{charging_log as storage, selective_log};

#[test]
fn brief_dip_or_reverse_current_spike_survives_a_quiet_average() {
    use selective_log::{Summary, Window};
    let mut w = Window::default();
    let steady = (5000, -1000);
    for at in (0..1000).step_by(100) {
        assert!(w.observe(at, Some(steady)).is_none());
    }
    let baseline = w.observe(1000, Some((4600, -1000))).unwrap();
    assert!(baseline.keep(None));
    for at in (1100..2000).step_by(100) {
        w.observe(at, Some(steady));
    }
    let dip = w.observe(2000, Some((5000, -1500))).unwrap();
    assert_eq!(
        dip,
        Summary {
            at: 1900,
            avg_mv: 4960,
            avg_ma: -1000,
            min_mv: 4600,
            peak_ma: -1000
        }
    );
    assert!(dip.keep(Some(baseline)));
    for at in (2100..3000).step_by(100) {
        w.observe(at, Some(steady));
    }
    let spike = w.observe(3000, None).unwrap();
    assert_eq!((spike.avg_ma, spike.peak_ma), (-1050, -1500));
    assert!(spike.keep(Some(baseline)));
    assert!(w.observe(9000, None).is_none());
    w.observe(9100, Some((5000, i16::MIN)));
    assert_eq!(w.finish().unwrap().peak_ma, i16::MIN);
    assert!(w.finish().is_none());
    assert!(
        !Summary {
            at: baseline.at + 59_999,
            ..baseline
        }
        .keep(Some(baseline))
    );
    assert!(
        Summary {
            at: baseline.at + 60_000,
            ..baseline
        }
        .keep(Some(baseline))
    );
}

#[test]
fn only_exact_complete_sop_routine_is_suppressed() {
    for id in 0..8u32 {
        for role in [0, 0x20, 0x100, 0x120] {
            let h = (7 << 16) | (id << 9) | role;
            assert!(selective_log::routine(&[h | 1]));
            for payload in [0x0003_8002, 0x0004_8002] {
                assert!(selective_log::routine(&[h | 0x9010, payload]));
            }
        }
    }
    for sop in 0..7 {
        assert!(!selective_log::routine(&[(sop << 16) | 1]));
        assert!(!selective_log::routine(&[
            (sop << 16) | 0x9010,
            0x0003_8002
        ]));
    }
    for words in [
        vec![],
        vec![0x70001, 0],
        vec![0x71001],
        vec![0x79010],
        vec![0x79010, 0x0003_8003],
        vec![0x79010, 0x0003_8002, 0],
        vec![0x70003],
        vec![0x70006],
        vec![0x72009, 1, 2],
        vec![0x7700f, 1, 2, 3, 4, 5, 6, 7],
    ] {
        assert!(!selective_log::routine(&words), "{words:X?}");
    }
}

fn put(dst: &mut [u8], record: [u32; 4]) {
    for (b, w) in dst.as_chunks_mut::<4>().0.iter_mut().zip(record) {
        b.copy_from_slice(&w.to_le_bytes());
    }
}

fn export(disk: &storage::Disk<'_>, name: &[u8; 11]) -> Vec<u8> {
    let mut block = [0; storage::BLOCK_BYTES];
    disk.read(0, &mut block);
    assert_eq!(&block[54..62], b"FAT12   ");
    let root = u16::from_le_bytes([block[14], block[15]]) as u32
        + u32::from(block[16]) * u16::from_le_bytes([block[22], block[23]]) as u32;
    let root_blocks = (u16::from_le_bytes([block[17], block[18]]) as usize * 32)
        .div_ceil(storage::BLOCK_BYTES) as u32;
    let data = root + root_blocks;
    for lba in root..data {
        disk.read(lba, &mut block);
        for entry in block.as_chunks::<32>().0 {
            if &entry[..11] == name {
                let first = data + u32::from(u16::from_le_bytes([entry[26], entry[27]])) - 2;
                let size = u32::from_le_bytes(entry[28..32].try_into().unwrap()) as usize;
                let mut bytes = Vec::new();
                // The logger allocates contiguous clusters.
                for lba in first..first + size.div_ceil(storage::BLOCK_BYTES) as u32 {
                    disk.read(lba, &mut block);
                    bytes.extend_from_slice(&block);
                }
                bytes.truncate(size);
                return bytes;
            }
        }
    }
    panic!("file not found: {name:?}");
}

// Protect the journal tags, signed CSV fields, and session splitting on tail reuse.
#[test]
fn journal_round_trips_to_read_only_disk() {
    let mut vi = [0xff; storage::SECTOR];
    let mut pd = [0xff; storage::SECTOR];
    let blank = [0xff; storage::SECTOR];
    let row = storage::extrema_record(990, 5010, -125, 4700, -832);
    assert_eq!(
        storage::valid_record(&row.map(u32::to_le_bytes).concat()),
        Some([990, 0xff83_1392, 0xfcc0_125c,])
    );
    put(&mut vi[..16], storage::header(1, 0, false));
    put(&mut vi[16..32], row);
    put(&mut vi[32..48], storage::session_marker(2));
    put(
        &mut vi[48..64],
        storage::extrema_record(50, 5000, 11, 4990, 11),
    );
    put(&mut pd[..16], storage::header(2, 0, true));
    put(&mut pd[16..32], storage::record(71, 256, 0x70003));
    assert_eq!(&vi[..4], b"XSL2");
    assert_eq!(&pd[..4], b"XSP1");
    assert_eq!(storage::latest_session(&vi), 2);
    assert_eq!(storage::reusable_tail(&vi, false), Some(3));
    let disk = storage::Disk::new([&vi, &pd, &blank, &blank, &blank, &blank]);
    let vi_header =
        "session,elapsed_ms,avg_voltage_mV,avg_current_mA,min_voltage_mV,peak_current_mA\r\n";
    assert_eq!(
        export(&disk, b"VI000102CSV"),
        format!("{vi_header}0000000001,0000000990,05010,-00125,04700,-00832\r\n").as_bytes()
    );
    assert_eq!(
        export(&disk, b"VI000202CSV"),
        format!("{vi_header}0000000002,0000000050,05000,+00011,04990,+00011\r\n").as_bytes()
    );
    assert_eq!(
        export(&disk, b"PD000201CSV"),
        b"session,elapsed_ms,event_tag,raw_u32\r\n0000000002,0000000071,0000000256,00000458755\r\n"
    );
    assert!(export(&disk, b"STATUS  TXT").starts_with(b"LOG STORAGE\r\nFREE SECTORS: 4/6"));
    let raw = export(&disk, b"STORAGE BIN");
    assert_eq!(raw, [&vi[..], &pd, &blank, &blank, &blank, &blank].concat());
}

// A power cut before the checksum must seal the sector, even if a later slot is valid.
#[test]
fn torn_record_exports_only_committed_prefix() {
    let mut vi = [0xff; storage::SECTOR];
    let blank = [0xff; storage::SECTOR];
    put(&mut vi[..16], storage::header(1, 0, false));
    put(
        &mut vi[16..32],
        storage::extrema_record(10, 5000, 0, 5000, 0),
    );
    put(&mut vi[32..44], storage::session_marker(2));
    put(
        &mut vi[48..64],
        storage::extrema_record(11, 5001, 1, 5001, 1),
    );
    assert_eq!(storage::committed_slots(&vi), 1);
    assert_eq!(storage::latest_session(&vi), 1);
    assert_eq!(storage::reusable_tail(&vi, false), None);
    let disk = storage::Disk::new([&vi, &blank, &blank, &blank, &blank, &blank]);
    assert_eq!(
        export(&disk, b"VI000102CSV").split(|&b| b == b'\n').count(),
        3
    );
}
