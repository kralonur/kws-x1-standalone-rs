//! The self-unpacking release: one application image holding the unpacker
//! and two raw LZMA2 streams, the upper region's then the main firmware's.
//!
//! ```text
//! APP_BASE: unpacker ++ upper stream ++ main stream   (Packed::data -> upper stream)
//! ```
//!
//! On its first boot the unpacker inflates both streams and checks their
//! lengths and hashes, then writes the upper region and finally the main
//! firmware over itself, sector by sector, from RAM. The main firmware's
//! sectors are erased only below the stream bytes still to be read
//! ([`main_erase_margin`]); `firmware-pack` runs the same [`inflate_sectors`]
//! to prove that before packing.

use lzma_rust2::{Action, Lzma2Stream, Status};

use crate::{APP_BASE, ImageError, SECTOR_BYTES};

/// One region in the image: its inflated length and hash
/// ([`crate::region_hash`]) and its compressed stream's length, in bytes.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Stream {
    pub len: u32,
    pub hash: u32,
    pub packed: u32,
}

/// The unpacker's record, filled in by `firmware-pack`: where the upper
/// stream starts (the main stream follows it) and both streams.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Packed {
    pub magic: [u32; 2],
    pub data: u32,
    pub upper: Stream,
    pub main: Stream,
}

/// Marks the unpacker's record for `firmware-pack`; random, unique.
pub const PACKED_MAGIC: [u32; 2] = [0x554e_504b, 0x6a09_e667];
/// An unfilled record field.
pub const PACKED_UNSET: u32 = u32::MAX;
/// Inflated output is handed over in flash-sector chunks.
pub const CHUNK_BYTES: usize = SECTOR_BYTES as usize;
/// LZMA2 sliding dictionary size in bytes (64 KiB).
pub const DICT_BYTES: u32 = 64 * 1024;
/// Unpacker working buffer size in bytes: one 8 KiB sector buffer plus 80 KiB
/// for the 64 KiB LZMA2 dictionary and 12 KiB literal probability model.
pub const WINDOW_BYTES: usize = 88 * 1024;

/// One chunk of inflated output: its offset in the region, its bytes (a full
/// sector except possibly the last) and the stream bytes consumed so far.
pub struct Chunk<'a> {
    pub offset: usize,
    pub bytes: &'a [u8],
    pub consumed: usize,
}

/// ARMv7-M Thumb-2 32-bit `BL` instruction fields (ARMv7-M ARM A7.7.18, T1):
/// first halfword `11110 : S : imm10`, second `11 : J1 : 1 : J2 : imm11`,
/// encoding a 24-bit halfword displacement `S : I1 : I2 : imm10 : imm11`
/// where `I1 = !(J1 ^ S)` and `I2 = !(J2 ^ S)`.
const BL_HW1_MASK: u16 = 0xf800;
const BL_HW1_PREFIX: u16 = 0xf000;
const BL_HW2_MASK: u16 = 0xd000;
const BL_HW2_PREFIX: u16 = 0xd000;
const BL_S_SHIFT: u32 = 10;
const BL_J1_SHIFT: u32 = 13;
const BL_J2_SHIFT: u32 = 11;
const BL_IMM10_MASK: u16 = 0x03ff;
const BL_IMM11_MASK: u16 = 0x07ff;
const BL_S_BIT: u32 = 23;
const BL_I1_BIT: u32 = 22;
const BL_I2_BIT: u32 = 21;
const BL_IMM10_BIT: u32 = 11;
const BL_OFFSET_MASK: u32 = 0x00ff_ffff;

/// Convert Thumb-2 `BL` relative displacements within each sector chunk to
/// absolute halfword targets (`encode = true`) or back (`encode = false`).
pub fn filter_thumb_bl(bytes: &mut [u8], base_offset: usize, encode: bool) {
    for (index, sector) in bytes.chunks_mut(CHUNK_BYTES).enumerate() {
        let sector_offset = base_offset + index * CHUNK_BYTES;
        let mut at = 0;
        while at + 4 <= sector.len() {
            let hw1 = u16::from_le_bytes([sector[at], sector[at + 1]]);
            let hw2 = u16::from_le_bytes([sector[at + 2], sector[at + 3]]);
            if hw1 & BL_HW1_MASK == BL_HW1_PREFIX && hw2 & BL_HW2_MASK == BL_HW2_PREFIX {
                let s = u32::from((hw1 >> BL_S_SHIFT) & 1);
                let j1 = u32::from((hw2 >> BL_J1_SHIFT) & 1);
                let j2 = u32::from((hw2 >> BL_J2_SHIFT) & 1);
                let i1 = (j1 ^ s) ^ 1;
                let i2 = (j2 ^ s) ^ 1;
                let imm10 = u32::from(hw1 & BL_IMM10_MASK);
                let imm11 = u32::from(hw2 & BL_IMM11_MASK);
                let raw = (s << BL_S_BIT)
                    | (i1 << BL_I1_BIT)
                    | (i2 << BL_I2_BIT)
                    | (imm10 << BL_IMM10_BIT)
                    | imm11;
                let pc = ((sector_offset + at) / 2) as u32;
                let converted = if encode {
                    raw.wrapping_add(pc)
                } else {
                    raw.wrapping_sub(pc)
                } & BL_OFFSET_MASK;
                let s = (converted >> BL_S_BIT) & 1;
                let i1 = (converted >> BL_I1_BIT) & 1;
                let i2 = (converted >> BL_I2_BIT) & 1;
                let j1 = (i1 ^ s) ^ 1;
                let j2 = (i2 ^ s) ^ 1;
                let out1 = BL_HW1_PREFIX
                    | ((s as u16) << BL_S_SHIFT)
                    | (((converted >> BL_IMM10_BIT) as u16) & BL_IMM10_MASK);
                let out2 = BL_HW2_PREFIX
                    | ((j1 as u16) << BL_J1_SHIFT)
                    | ((j2 as u16) << BL_J2_SHIFT)
                    | ((converted as u16) & BL_IMM11_MASK);
                sector[at..at + 2].copy_from_slice(&out1.to_le_bytes());
                sector[at + 2..at + 4].copy_from_slice(&out2.to_le_bytes());
                at += 4;
            } else {
                at += 2;
            }
        }
    }
}

#[cfg(target_os = "none")]
mod arena {
    use core::{
        alloc::{GlobalAlloc, Layout},
        cell::UnsafeCell,
        ptr::null_mut,
    };

    struct Bump {
        start: UnsafeCell<usize>,
        end: UnsafeCell<usize>,
    }

    // SAFETY: `unpacker` disables interrupts at entry (`main`), and `firmware`
    // never calls `inflate_sectors` or allocates on the heap.
    unsafe impl Sync for Bump {}

    #[global_allocator]
    static ALLOCATOR: Bump = Bump {
        start: UnsafeCell::new(0),
        end: UnsafeCell::new(0),
    };

    pub(super) struct Scope;

    impl Scope {
        pub(super) fn new(bytes: &mut [u8]) -> Self {
            let range = bytes.as_mut_ptr_range();
            // SAFETY: single-threaded execution in `unpacker` with interrupts off.
            unsafe {
                *ALLOCATOR.start.get() = range.start as usize;
                *ALLOCATOR.end.get() = range.end as usize;
            }
            Self
        }
    }

    impl Drop for Scope {
        fn drop(&mut self) {
            // SAFETY: single-threaded execution in `unpacker` with interrupts off.
            unsafe {
                *ALLOCATOR.start.get() = 0;
                *ALLOCATOR.end.get() = 0;
            }
        }
    }

    unsafe impl GlobalAlloc for Bump {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            let start = unsafe { *self.start.get() };
            let end = unsafe { *self.end.get() };
            let aligned = start.next_multiple_of(layout.align());
            let Some(next) = aligned.checked_add(layout.size()) else {
                return null_mut();
            };
            if start == 0 || next > end {
                return null_mut();
            }
            unsafe { *self.start.get() = next };
            aligned as *mut u8
        }

        unsafe fn dealloc(&self, _ptr: *mut u8, _layout: Layout) {}
    }
}

/// Inflate the raw LZMA2 `stream`, handing `sink` each chunk in order;
/// `sink` returning false stops with [`ImageError::Stopped`]. Returns the
/// inflated length.
pub fn inflate_sectors(
    stream: &[u8],
    window: &mut [u8; WINDOW_BYTES],
    mut sink: impl FnMut(Chunk) -> bool,
) -> Result<usize, ImageError> {
    let (sector, arena) = window.split_at_mut(CHUNK_BYTES);
    #[cfg(target_os = "none")]
    let _scope = arena::Scope::new(arena);
    #[cfg(not(target_os = "none"))]
    let _ = arena;
    let mut decoder = Lzma2Stream::new_mem_limit(DICT_BYTES, u32::MAX);
    let (mut read, mut flushed, mut filled) = (0, 0, 0);
    loop {
        let result = decoder
            .process(&stream[read..], &mut sector[filled..], Action::Finish)
            .map_err(|_| ImageError::BadStream)?;
        read += result.bytes_consumed;
        filled += result.bytes_produced;
        let done = result.status == Status::StreamEnd;
        if !done && result.bytes_consumed == 0 && result.bytes_produced == 0 {
            return Err(ImageError::BadStream);
        }
        if filled == CHUNK_BYTES || (done && filled > 0) {
            filter_thumb_bl(&mut sector[..filled], flushed, false);
            let chunk = Chunk {
                offset: flushed,
                bytes: &sector[..filled],
                consumed: read,
            };
            if !sink(chunk) {
                return Err(ImageError::Stopped);
            }
            flushed += filled;
            filled = 0;
        }
        if done {
            if read != stream.len() {
                return Err(ImageError::BadStream);
            }
            return Ok(flushed);
        }
    }
}

/// How far the main firmware's sector holding `chunk_offset` ends below the
/// first main stream byte not yet consumed; `None` when it reaches into
/// those bytes and must not be erased.
pub fn main_erase_margin(packed: &Packed, chunk_offset: usize, consumed: usize) -> Option<usize> {
    let sector_end = APP_BASE as usize + chunk_offset + CHUNK_BYTES;
    let unread = packed.data as usize + packed.upper.packed as usize + consumed;
    unread.checked_sub(sector_end)
}
