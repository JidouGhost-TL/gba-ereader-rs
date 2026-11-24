//! Printable Olympus/Nintendo DotCode strip renderer (zero-dependency bitmap output).
//!
//! Renders a hardware `.raw` strip (`1,872` bytes = 18 physical blocks for a
//! short strip, or `2,912` bytes = 28 physical blocks for a long strip) into
//! the real Olympus/Nintendo optical dot matrix:
//!
//! - **Strip height**: [`STRIP_ROWS`] = `40` dot rows.
//! - **Physical block pitch**: [`BLOCK_COLS`] = `35` dot columns per block:
//!   - Columns `0..5`: `5×5` rounded corner finder/anchor marks at rows `0..5`
//!     (top) and `35..40` (bottom), plus the vertical sync/address bar in column `2`.
//!   - Columns `5..7`: 2-column left guard zone.
//!   - Columns `7..33`: `26` data columns × `40` rows = `1,040` dot cells,
//!     holding all `104` physical block bytes modulated through
//!     [`crate::dotcode::modulate`] (`4` 10-bit symbols per column, MSB-first).
//!   - Columns `33..35`: 2-column right guard zone.
//! - A trailing 5-column anchor bar closes the final block on the right edge,
//!   surrounded by a 4-module quiet zone on all four sides.
//! - Exports as binary PBM (`P4`), PGM (`P5`), or uncompressed 1-bit BMP at
//!   300, 600, or 1200 DPI.

#[cfg(feature = "alloc")]
use crate::dotcode::{
    modulate, LONG_STRIP_PHYSICAL_BLOCKS, LONG_STRIP_RAW_LEN, PHYSICAL_BLOCK_LEN,
    SHORT_STRIP_PHYSICAL_BLOCKS, SHORT_STRIP_RAW_LEN,
};
#[cfg(feature = "alloc")]
use crate::error::{Error, Result};

/// Modules (dot cells) per inch at the nominal strip density.
pub const MODULES_PER_INCH: u32 = 150;
/// Supported print resolutions in dots per inch.
pub const SUPPORTED_DPI: [u32; 3] = [300, 600, 1200];
/// Dot rows in a rendered Olympus/Nintendo DotCode strip (`40`).
pub const STRIP_ROWS: usize = 40;
/// Dot columns per physical block (`35`: 5 anchor + 2 guard + 26 data + 2 guard).
pub const BLOCK_COLS: usize = 35;
/// Width/height in dot cells of each rounded corner anchor mark (`5`).
pub const ANCHOR_SIZE: usize = 5;
/// Quiet zone margin in dot cells around the rendered strip (`4`).
pub const QUIET_ZONE: usize = 4;

/// A 1-bit bitmap of a rendered strip (row-major, `true` = black dot).
#[cfg(feature = "alloc")]
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct StripBitmap {
    /// Width in pixels.
    pub width: usize,
    /// Height in pixels.
    pub height: usize,
    /// Row-major pixels, one byte per pixel (0 = white, 1 = black).
    pub pixels: alloc::vec::Vec<u8>,
}

#[cfg(feature = "alloc")]
impl StripBitmap {
    /// Pixel at `(x, y)`, or `false` outside the bitmap.
    #[must_use]
    #[inline]
    pub fn get(&self, x: usize, y: usize) -> bool {
        if x >= self.width || y >= self.height {
            false
        } else {
            self.pixels[y * self.width + x] != 0
        }
    }

    /// Serialize as binary PBM (`P4`).
    #[must_use]
    pub fn to_pbm(&self) -> alloc::vec::Vec<u8> {
        let header = alloc::format!("P4\n{} {}\n", self.width, self.height);
        let row_bytes = self.width.div_ceil(8);
        let hdr_len = header.len();
        let mut out = alloc::vec![0u8; hdr_len + self.height * row_bytes];
        out[..hdr_len].copy_from_slice(header.as_bytes());
        for y in 0..self.height {
            let row = &self.pixels[y * self.width..(y + 1) * self.width];
            let dst = &mut out[hdr_len + y * row_bytes..hdr_len + (y + 1) * row_bytes];
            let (chunks, rem) = row.as_chunks::<8>();
            let mut bx = 0usize;
            for chunk in chunks {
                dst[bx] = (((chunk[0] != 0) as u8) << 7)
                    | (((chunk[1] != 0) as u8) << 6)
                    | (((chunk[2] != 0) as u8) << 5)
                    | (((chunk[3] != 0) as u8) << 4)
                    | (((chunk[4] != 0) as u8) << 3)
                    | (((chunk[5] != 0) as u8) << 2)
                    | (((chunk[6] != 0) as u8) << 1)
                    | ((chunk[7] != 0) as u8);
                bx += 1;
            }
            if !rem.is_empty() {
                let mut byte = 0u8;
                for (k, &p) in rem.iter().enumerate() {
                    if p != 0 {
                        byte |= 1 << (7 - k);
                    }
                }
                dst[bx] = byte;
            }
        }
        out
    }

    /// Serialize as binary PGM (`P5`, 8-bit grayscale, black = 0).
    #[must_use]
    pub fn to_pgm(&self) -> alloc::vec::Vec<u8> {
        let header = alloc::format!("P5\n{} {}\n255\n", self.width, self.height);
        let mut out = alloc::vec::Vec::with_capacity(header.len() + self.pixels.len());
        out.extend_from_slice(header.as_bytes());
        out.extend(self.pixels.iter().map(|&p| if p != 0 { 0u8 } else { 255u8 }));
        out
    }

    /// Serialize as an uncompressed 1-bit BMP.
    #[must_use]
    pub fn to_bmp(&self) -> alloc::vec::Vec<u8> {
        let row_bytes = self.width.div_ceil(32) * 4;
        let img_size = row_bytes * self.height;
        let file_size = 62 + img_size;
        let mut out = alloc::vec![0u8; file_size];
        out[0] = b'B';
        out[1] = b'M';
        out[2..6].copy_from_slice(&(file_size as u32).to_le_bytes());
        out[10..14].copy_from_slice(&62u32.to_le_bytes());
        out[14..18].copy_from_slice(&40u32.to_le_bytes());
        out[18..22].copy_from_slice(&(self.width as i32).to_le_bytes());
        out[22..26].copy_from_slice(&(self.height as i32).to_le_bytes());
        out[26..28].copy_from_slice(&1u16.to_le_bytes());
        out[28..30].copy_from_slice(&1u16.to_le_bytes());
        out[34..38].copy_from_slice(&(img_size as u32).to_le_bytes());
        // Palette: white then black.
        out[54..58].copy_from_slice(&[255, 255, 255, 0]);
        out[58..62].copy_from_slice(&[0, 0, 0, 0]);
        for y in 0..self.height {
            let src_y = self.height - 1 - y; // BMP rows are bottom-up.
            let row = &self.pixels[src_y * self.width..(src_y + 1) * self.width];
            let dst = &mut out[62 + y * row_bytes..62 + (y + 1) * row_bytes];
            let (chunks, rem) = row.as_chunks::<8>();
            let mut bx = 0usize;
            for chunk in chunks {
                // Palette index 0 = white (bit 1), 1 = black (bit 0).
                dst[bx] = (((chunk[0] == 0) as u8) << 7)
                    | (((chunk[1] == 0) as u8) << 6)
                    | (((chunk[2] == 0) as u8) << 5)
                    | (((chunk[3] == 0) as u8) << 4)
                    | (((chunk[4] == 0) as u8) << 3)
                    | (((chunk[5] == 0) as u8) << 2)
                    | (((chunk[6] == 0) as u8) << 1)
                    | ((chunk[7] == 0) as u8);
                bx += 1;
            }
            if !rem.is_empty() {
                let mut byte = 0u8;
                for (k, &p) in rem.iter().enumerate() {
                    if p == 0 {
                        byte |= 1 << (7 - k);
                    }
                }
                dst[bx] = byte;
            }
        }
        out
    }
}

/// Pixels per module (dot cell) at `dpi` (at least 1).
#[must_use]
pub const fn module_px(dpi: u32) -> usize {
    let px = (dpi / MODULES_PER_INCH) as usize;
    if px < 1 { 1 } else { px }
}

/// Draw a 5×5 rounded Olympus DotCode anchor mark on the 1x module grid.
#[cfg(feature = "alloc")]
fn draw_anchor_1x(ox: usize, oy: usize, cols: usize, grid: &mut [u8]) {
    // Row 0 and row 4: omit extreme corners (columns 1..4).
    grid[oy * cols + ox + 1..oy * cols + ox + 4].fill(1);
    grid[(oy + 4) * cols + ox + 1..(oy + 4) * cols + ox + 4].fill(1);
    // Rows 1..4: all 5 columns.
    for dy in 1..4 {
        let base = (oy + dy) * cols + ox;
        grid[base..base + ANCHOR_SIZE].fill(1);
    }
}

/// Render a hardware `.raw` strip (`1,872` or `2,912` bytes) into a printable
/// Olympus/Nintendo DotCode bitmap at `dpi`.
#[cfg(feature = "alloc")]
pub fn render_strip(raw: &[u8], dpi: u32) -> Result<StripBitmap> {
    let phys_blocks = match raw.len() {
        SHORT_STRIP_RAW_LEN => SHORT_STRIP_PHYSICAL_BLOCKS,
        LONG_STRIP_RAW_LEN => LONG_STRIP_PHYSICAL_BLOCKS,
        _ => return Err(Error::InvalidStrip("raw strip length must be 1872 (short) or 2912 (long)")),
    };
    if !SUPPORTED_DPI.contains(&dpi) {
        return Err(Error::InvalidHeader("unsupported DPI"));
    }
    let scale = module_px(dpi);
    let cols = phys_blocks * BLOCK_COLS + ANCHOR_SIZE + QUIET_ZONE * 2;
    let rows = STRIP_ROWS + QUIET_ZONE * 2;

    // Step 1: Render 1x module grid (fits in L1/L2 cache: <= 48 KiB).
    let mut grid = alloc::vec![0u8; cols * rows];
    for pb in 0..phys_blocks {
        let bx = QUIET_ZONE + pb * BLOCK_COLS;
        draw_anchor_1x(bx, QUIET_ZONE, cols, &mut grid);
        draw_anchor_1x(bx, QUIET_ZONE + STRIP_ROWS - ANCHOR_SIZE, cols, &mut grid);

        // Sync dot (row 7) and 16-bit block address bar (rows 10..26) in col 2.
        grid[(QUIET_ZONE + 7) * cols + bx + 2] = 1;
        let addr_word = ((pb as u16) + 1) ^ 0xA500;
        for bit in 0..16 {
            if (addr_word >> (15 - bit)) & 1 == 1 {
                grid[(QUIET_ZONE + 10 + bit) * cols + bx + 2] = 1;
            }
        }

        // 26 data columns x 40 rows = 104 bytes x 10 modulated bits.
        let blk = &raw[pb * PHYSICAL_BLOCK_LEN..(pb + 1) * PHYSICAL_BLOCK_LEN];
        for col in 0..26 {
            let cx = bx + 7 + col;
            for b_idx in 0..4 {
                let sym = modulate(blk[col * 4 + b_idx]);
                let base_y = QUIET_ZONE + b_idx * 10;
                for k in 0..10 {
                    if (sym >> (9 - k)) & 1 == 1 {
                        grid[(base_y + k) * cols + cx] = 1;
                    }
                }
            }
        }
    }

    // Closing right-edge anchor pair.
    let end_x = QUIET_ZONE + phys_blocks * BLOCK_COLS;
    draw_anchor_1x(end_x, QUIET_ZONE, cols, &mut grid);
    draw_anchor_1x(end_x, QUIET_ZONE + STRIP_ROWS - ANCHOR_SIZE, cols, &mut grid);

    // Step 2: Expand 1x module grid to `scale` x `scale` pixels sequentially.
    let width = cols * scale;
    let height = rows * scale;
    let mut pixels = alloc::vec![0u8; width * height];
    for r in 0..rows {
        let src_row = &grid[r * cols..(r + 1) * cols];
        let first_dst_start = r * scale * width;
        {
            let dst_row = &mut pixels[first_dst_start..first_dst_start + width];
            match scale {
                2 => {
                    for (c, &p) in src_row.iter().enumerate() {
                        if p != 0 {
                            let x = c * 2;
                            dst_row[x] = 1;
                            dst_row[x + 1] = 1;
                        }
                    }
                }
                4 => {
                    for (c, &p) in src_row.iter().enumerate() {
                        if p != 0 {
                            let x = c * 4;
                            dst_row[x..x + 4].fill(1);
                        }
                    }
                }
                8 => {
                    for (c, &p) in src_row.iter().enumerate() {
                        if p != 0 {
                            let x = c * 8;
                            dst_row[x..x + 8].fill(1);
                        }
                    }
                }
                _ => {
                    for (c, &p) in src_row.iter().enumerate() {
                        if p != 0 {
                            dst_row[c * scale..(c + 1) * scale].fill(1);
                        }
                    }
                }
            }
        }
        // Duplicate the expanded scanline `scale - 1` times.
        for sy in 1..scale {
            let dst_start = (r * scale + sy) * width;
            pixels.copy_within(first_dst_start..first_dst_start + width, dst_start);
        }
    }

    Ok(StripBitmap { width, height, pixels })
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;

    #[test]
    fn render_formats_short_and_long() {
        let raw_s = crate::dotcode::bin_to_raw(
            &crate::dotcode::build_bin(1, 0, 0, 1, b"abc").unwrap(),
        )
        .unwrap();
        for dpi in SUPPORTED_DPI {
            let bmp = render_strip(&raw_s, dpi).unwrap();
            let scale = module_px(dpi);
            assert_eq!(bmp.width, (SHORT_STRIP_PHYSICAL_BLOCKS * BLOCK_COLS + ANCHOR_SIZE + QUIET_ZONE * 2) * scale);
            assert_eq!(bmp.height, (STRIP_ROWS + QUIET_ZONE * 2) * scale);
            assert_eq!(bmp.pixels.len(), bmp.width * bmp.height);
            assert!(bmp.to_pbm().starts_with(b"P4\n"));
            assert!(bmp.to_pgm().starts_with(b"P5\n"));
            let b = bmp.to_bmp();
            assert_eq!(&b[0..2], b"BM");
            assert_eq!(u32::from_le_bytes([b[2], b[3], b[4], b[5]]) as usize, b.len());
        }
        assert_eq!(module_px(300), 2);
        assert_eq!(module_px(600), 4);
        assert_eq!(module_px(1200), 8);
        assert_eq!(render_strip(&raw_s, 150).err(), Some(Error::InvalidHeader("unsupported DPI")));
    }
}
