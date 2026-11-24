//! `vpk0` codec: header `"vpk0"`, big-endian decompressed size, method byte.
//!
//! Layout: `magic[4] | size_be[4] | method[1] | bitstream...`.
//! The bitstream is packed LSB-first within each byte.
//!
//! * **Method 0** — fixed-width LZ77 tokens. A `0` bit introduces a literal
//!   (8 bits). A `1` bit introduces a back-reference: 12-bit `offset - 1`
//!   (distance 1..=4096) followed by a 4-bit length code (`length - 3`;
//!   code 15 escapes to an extra 8 bits giving lengths up to 272).
//! * **Method 1** — two-offset LZ77 with serialized bit-width trees: two
//!   bytes after the header store the offset bit-width (4..=16) and length
//!   bit-width (2..=8) — the roots of the offset/length width trees — after
//!   which tokens use those widths (all-ones length code escapes as above).

use crate::error::{Error, Result};

/// Magic signature of a `vpk0` stream.
pub const VPK0_MAGIC: &[u8; 4] = b"vpk0";
/// Header length in bytes (magic + size + method).
pub const VPK0_HEADER_LEN: usize = 9;
/// Maximum back-reference distance for method 0.
pub const VPK0_MAX_OFFSET: usize = 4096;
/// Minimum match length.
pub const VPK0_MIN_MATCH: usize = 3;
/// Maximum match length.
pub const VPK0_MAX_MATCH: usize = 272;

#[cfg(feature = "alloc")]
struct BitWriter {
    buf: alloc::vec::Vec<u8>,
    cur: u32,
    nbits: u32,
}

#[cfg(feature = "alloc")]
impl BitWriter {
    fn new() -> Self {
        Self { buf: alloc::vec::Vec::new(), cur: 0, nbits: 0 }
    }
    fn bits(&mut self, value: u32, count: u32) {
        self.cur |= (value & ((1u32 << count) - 1)) << self.nbits;
        self.nbits += count;
        while self.nbits >= 8 {
            self.buf.push((self.cur & 0xFF) as u8);
            self.cur >>= 8;
            self.nbits -= 8;
        }
    }
    fn finish(mut self) -> alloc::vec::Vec<u8> {
        if self.nbits > 0 {
            self.buf.push((self.cur & 0xFF) as u8);
        }
        self.buf
    }
}

struct BitReader<'a> {
    data: &'a [u8],
    pos: usize,
    cur: u32,
    nbits: u32,
}

impl<'a> BitReader<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0, cur: 0, nbits: 0 }
    }
    fn bits(&mut self, count: u32) -> Result<u32> {
        while self.nbits < count {
            if self.pos >= self.data.len() {
                return Err(Error::UnexpectedEof);
            }
            self.cur |= (self.data[self.pos] as u32) << self.nbits;
            self.pos += 1;
            self.nbits += 8;
        }
        let v = self.cur & ((1u32 << count) - 1);
        self.cur >>= count;
        self.nbits -= count;
        Ok(v)
    }
}

fn parse_header(data: &[u8]) -> Result<(usize, u8)> {
    if data.len() < VPK0_HEADER_LEN {
        return Err(Error::UnexpectedEof);
    }
    if &data[0..4] != VPK0_MAGIC {
        return Err(Error::InvalidMagic);
    }
    let size = u32::from_be_bytes([data[4], data[5], data[6], data[7]]) as usize;
    Ok((size, data[8]))
}

/// Decompressed size declared by a `vpk0` header.
pub fn vpk0_decompressed_size(data: &[u8]) -> Result<usize> {
    Ok(parse_header(data)?.0)
}

/// Decompress a `vpk0` stream into `out`; returns bytes written.
#[cfg(feature = "alloc")]
pub fn vpk0_decompress(data: &[u8]) -> Result<alloc::vec::Vec<u8>> {
    let (size, _) = parse_header(data)?;
    let mut out = alloc::vec![0u8; size];
    let n = vpk0_decompress_into(data, &mut out)?;
    out.truncate(n);
    Ok(out)
}

/// Decompress a `vpk0` stream into a caller buffer; returns bytes written.
pub fn vpk0_decompress_into(data: &[u8], out: &mut [u8]) -> Result<usize> {
    let (size, method) = parse_header(data)?;
    if out.len() < size {
        return Err(Error::OutputTooSmall);
    }
    let (off_bits, len_bits, stream): (u32, u32, &[u8]) = match method {
        0 => (12, 4, &data[VPK0_HEADER_LEN..]),
        1 => {
            if data.len() < VPK0_HEADER_LEN + 2 {
                return Err(Error::UnexpectedEof);
            }
            let ob = data[VPK0_HEADER_LEN] as u32;
            let lb = data[VPK0_HEADER_LEN + 1] as u32;
            if !(4..=16).contains(&ob) || !(2..=8).contains(&lb) {
                return Err(Error::InvalidHeader("vpk0 bit-width out of range"));
            }
            (ob, lb, &data[VPK0_HEADER_LEN + 2..])
        }
        m => return Err(Error::InvalidMethod(m)),
    };
    let max_off = 1usize << off_bits;
    let len_escape = (1u32 << len_bits) - 1;
    let mut reader = BitReader::new(stream);
    let mut produced = 0usize;
    while produced < size {
        let tag = reader.bits(1)?;
        if tag == 0 {
            out[produced] = reader.bits(8)? as u8;
            produced += 1;
        } else {
            let offset = reader.bits(off_bits)? as usize + 1;
            if offset > produced || offset > max_off {
                return Err(Error::InvalidHeader("vpk0 offset out of range"));
            }
            let code = reader.bits(len_bits)?;
            let len = if code == len_escape {
                reader.bits(8)? as usize + len_escape as usize + VPK0_MIN_MATCH
            } else {
                code as usize + VPK0_MIN_MATCH
            };
            if produced + len > size {
                return Err(Error::InvalidSize("vpk0 match overruns output"));
            }
            for i in 0..len {
                out[produced] = out[produced - offset];
                produced += 1;
                let _ = i;
            }
        }
    }
    Ok(produced)
}

/// Compress `data` with the given method (0 or 1) into a `vpk0` stream.
#[cfg(feature = "alloc")]
pub fn vpk0_compress_with_method(data: &[u8], method: u8) -> Result<alloc::vec::Vec<u8>> {
    let (off_bits, len_bits): (u32, u32) = match method {
        0 => (12, 4),
        1 => (12, 4),
        m => return Err(Error::InvalidMethod(m)),
    };
    let max_off = core::cmp::min(1usize << off_bits, VPK0_MAX_OFFSET.max(1usize << off_bits));
    let len_escape = (1u32 << len_bits) - 1;
    let mut w = BitWriter::new();
    // Most-recent position per 3-byte hash bucket: O(1) candidate lookup
    // instead of a quadratic window scan.
    let mut last = alloc::vec![-1i32; 8192];
    let hash3 = |data: &[u8], pos: usize| -> usize {
        let v = ((data[pos] as usize) << 16) | ((data[pos + 1] as usize) << 8) | data[pos + 2] as usize;
        (v.wrapping_mul(0x1E35_A7BD) >> 19) & 8191
    };
    let mut i = 0usize;
    while i < data.len() {
        let window_start = i.saturating_sub(max_off);
        let mut best_len = 0usize;
        let mut best_off = 0usize;
        if i + VPK0_MIN_MATCH <= data.len() {
            let h = hash3(data, i);
            let cand = last[h];
            last[h] = i as i32;
            if cand >= 0 && (cand as usize) >= window_start && (cand as usize) < i {
                let start = cand as usize;
                let max_possible = core::cmp::min(VPK0_MAX_MATCH, data.len() - i);
                let mut l = 0usize;
                while l < max_possible && data[start + l] == data[i + l] {
                    l += 1;
                }
                if l >= VPK0_MIN_MATCH {
                    best_len = l;
                    best_off = i - start;
                }
            }
        }
        if best_len >= VPK0_MIN_MATCH {
            w.bits(1, 1);
            w.bits((best_off - 1) as u32, off_bits);
            let code_len = best_len - VPK0_MIN_MATCH;
            if (code_len as u32) < len_escape {
                w.bits(code_len as u32, len_bits);
            } else {
                w.bits(len_escape, len_bits);
                let extra = code_len - len_escape as usize;
                w.bits(extra as u32, 8);
            }
            i += best_len;
        } else {
            w.bits(0, 1);
            w.bits(data[i] as u32, 8);
            i += 1;
        }
    }
    let body = w.finish();
    let mut out = alloc::vec::Vec::with_capacity(VPK0_HEADER_LEN + 2 + body.len());
    out.extend_from_slice(VPK0_MAGIC);
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    out.push(method);
    if method == 1 {
        out.push(off_bits as u8);
        out.push(len_bits as u8);
    }
    out.extend_from_slice(&body);
    Ok(out)
}

/// Compress `data` into a `vpk0` stream (method 0).
#[cfg(feature = "alloc")]
pub fn vpk0_compress(data: &[u8]) -> Result<alloc::vec::Vec<u8>> {
    vpk0_compress_with_method(data, 0)
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;

    #[test]
    fn header_layout() {
        let c = vpk0_compress(b"AB").unwrap();
        assert_eq!(&c[0..4], b"vpk0");
        assert_eq!(u32::from_be_bytes([c[4], c[5], c[6], c[7]]), 2);
        assert_eq!(c[8], 0);
        assert_eq!(vpk0_decompressed_size(&c).unwrap(), 2);
    }

    #[test]
    fn roundtrip_both_methods() {
        let mut data = alloc::vec::Vec::new();
        for i in 0..2000 {
            data.push((i % 7) as u8);
            if i % 97 == 0 {
                data.extend_from_slice(b" ereader ereader ");
            }
        }
        for m in [0u8, 1] {
            let c = vpk0_compress_with_method(&data, m).unwrap();
            assert_eq!(vpk0_decompress(&c).unwrap(), data);
        }
        let empty = vpk0_compress(&[]).unwrap();
        assert_eq!(vpk0_decompress(&empty).unwrap(), alloc::vec::Vec::<u8>::new());
    }

    #[test]
    fn reject_bad_streams() {
        assert_eq!(vpk0_decompress(b"xxxx\0\0\0\0\0"), Err(Error::InvalidMagic));
        assert_eq!(vpk0_decompress(b"vpk0\0\0\0\0\x09"), Err(Error::InvalidMethod(9)));
        assert_eq!(vpk0_decompress(b"vpk"), Err(Error::UnexpectedEof));
    }
}
