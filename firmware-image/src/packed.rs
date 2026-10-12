//! The self-unpacking release: one application image holding the unpacker
//! and two raw deflate streams, the upper region's then the main firmware's.
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

use miniz_oxide::inflate::{
    TINFLStatus,
    core::{DecompressorOxide, decompress},
};

use crate::{APP_BASE, ImageError, SECTOR_BYTES};

/// One region in the image: its inflated length and hash
/// ([`crate::region_hash`]) and its deflate stream's length, in bytes.
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
/// The deflate dictionary size (RFC 1951 section 2): the wrapping output
/// window the decompressor needs.
pub const WINDOW_BYTES: usize = 32 * 1024;

/// One chunk of inflated output: its offset in the region, its bytes (a full
/// sector except possibly the last) and the stream bytes consumed so far.
pub struct Chunk<'a> {
    pub offset: usize,
    pub bytes: &'a [u8],
    pub consumed: usize,
}

/// Inflate the raw deflate `stream`, handing `sink` each chunk in order;
/// `sink` returning false stops with [`ImageError::Stopped`]. Returns the
/// inflated length.
pub fn inflate_sectors(
    stream: &[u8],
    window: &mut [u8; WINDOW_BYTES],
    mut sink: impl FnMut(Chunk) -> bool,
) -> Result<usize, ImageError> {
    const _: () = assert!(WINDOW_BYTES.is_multiple_of(CHUNK_BYTES));
    let mut state = DecompressorOxide::new();
    let (mut read, mut written, mut flushed) = (0, 0, 0);
    loop {
        // Raw deflate, all input present, output wrapping in `window`: a
        // call stops at the window's end, which is a chunk boundary.
        let (status, used, made) = decompress(
            &mut state,
            &stream[read..],
            window,
            written % WINDOW_BYTES,
            0,
        );
        read += used;
        written += made;
        let done = match status {
            TINFLStatus::Done => true,
            TINFLStatus::HasMoreOutput => false,
            _ => return Err(ImageError::BadStream),
        };
        while written - flushed >= CHUNK_BYTES || (done && written > flushed) {
            let len = (written - flushed).min(CHUNK_BYTES);
            let start = flushed % WINDOW_BYTES;
            let chunk = Chunk {
                offset: flushed,
                bytes: &window[start..start + len],
                consumed: read,
            };
            if !sink(chunk) {
                return Err(ImageError::Stopped);
            }
            flushed += len;
        }
        if done {
            return Ok(written);
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
