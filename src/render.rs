//! Printable dot-code strip renderer (zero-dependency bitmap output).
//!
//! Renders a `.raw` strip bitstream into a 1-bit dot matrix with timing /
//! clock marks, then exports it as PBM (`P4`), PGM (`P5`), or 1-bit BMP at
//! a requested DPI. One module (dot cell) spans DPI dots per inch
//! divided by [`MODULES_PER_INCH`].

#[cfg(feature = "alloc")]
use crate::error::{Error, Result};

/// Modules (dot cells) per inch at the nominal strip density.
pub const MODULES_PER_INCH: u32 = 150;
/// Supported print resolutions in dots per inch.
pub const SUPPORTED_DPI: [u32; 3] = [300, 600, 1200];
/// Rows (dot cells) in a rendered strip, including clock track.
pub const STRIP_ROWS: usize = 24;

/// A 1-bit bitmap of a rendered strip (row-major, `true` = black dot).
#[cfg(feature = "alloc")]
#[derive(Debug, Clone, PartialEq, Eq)]
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
        let mut out = header.into_bytes();
        let row_bytes = self.width.div_ceil(8);
        for y in 0..self.height {
            for bx in 0..row_bytes {
                let mut byte = 0u8;
                for k in 0..8 {
                    let x = bx * 8 + k;
                    if x < self.width && self.get(x, y) {
                        byte |= 1 << (7 - k);
                    }
                }
                out.push(byte);
            }
        }
        out
    }

    /// Serialize as binary PGM (`P5`, 8-bit grayscale, black = 0).
    #[must_use]
    pub fn to_pgm(&self) -> alloc::vec::Vec<u8> {
        let header = alloc::format!("P5\n{} {}\n255\n", self.width, self.height);
        let mut out = header.into_bytes();
        for &p in &self.pixels {
            out.push(if p != 0 { 0 } else { 255 });
        }
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
            for bx in 0..row_bytes {
                let mut byte = 0u8;
                for k in 0..8 {
                    let x = bx * 8 + k;
                    if x < self.width && !self.get(x, src_y) {
                        byte |= 1 << (7 - k);
                    }
                }
                out[62 + y * row_bytes + bx] = byte;
            }
        }
        out
    }
}

/// Pixels per module (dot cell) at `dpi` (at least 1).
#[must_use]
pub fn module_px(dpi: u32) -> usize {
    core::cmp::max(1, (dpi / MODULES_PER_INCH) as usize)
}

/// Render `.raw` strip bytes into a printable bitmap at `dpi`.
///
/// Bits are read MSB-first from `raw[2..]` (after the 2-byte sync header),
/// laid out in rows of [`STRIP_ROWS`] cells with an alternating clock row,
/// surrounded by a 4-module quiet zone.
#[cfg(feature = "alloc")]
pub fn render_strip(raw: &[u8], dpi: u32) -> Result<StripBitmap> {
    if raw.len() < 3 {
        return Err(Error::UnexpectedEof);
    }
    if !SUPPORTED_DPI.contains(&dpi) {
        return Err(Error::InvalidHeader("unsupported DPI"));
    }
    let scale = module_px(dpi);
    let payload = &raw[2..];
    let total_bits = payload.len() * 8;
    let cols = total_bits.div_ceil(STRIP_ROWS) + 8; // + quiet zones
    let width = cols * scale;
    let height = (STRIP_ROWS + 8) * scale;
    let mut pixels = alloc::vec![0u8; width * height];
    let set = |cx: usize, cy: usize, pixels: &mut alloc::vec::Vec<u8>| {
        for dy in 0..scale {
            for dx in 0..scale {
                let x = cx * scale + dx;
                let y = cy * scale + dy;
                pixels[y * width + x] = 1;
            }
        }
    };
    for bit in 0..total_bits {
        let byte = payload[bit / 8];
        let on = (byte >> (7 - (bit % 8))) & 1 == 1;
        let cx = 4 + bit / STRIP_ROWS;
        let cy = 4 + bit % STRIP_ROWS;
        // Bottom row is the clock track: alternate regardless of data.
        if bit % STRIP_ROWS == STRIP_ROWS - 1 {
            if (bit / STRIP_ROWS).is_multiple_of(2) {
                set(cx, cy, &mut pixels);
            }
        } else if on {
            set(cx, cy, &mut pixels);
        }
    }
    Ok(StripBitmap { width, height, pixels })
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;

    #[test]
    fn render_formats() {
        let raw = crate::dotcode::bin_to_raw(&crate::dotcode::build_bin(1, 0, 0, 1, 3, b"abc")).unwrap();
        for dpi in SUPPORTED_DPI {
            let bmp = render_strip(&raw, dpi).unwrap();
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
        assert_eq!(render_strip(&raw, 150).err(), Some(Error::InvalidHeader("unsupported DPI")));
    }
}
