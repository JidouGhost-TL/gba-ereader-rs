//! Dot-code optical modulation and `.raw` strip / `.bin` card formats.
//!
//! Each payload byte maps to a 10-bit optical symbol through
//! [`MODULATION_TABLE`] (balanced symbols: popcount 5, filled to 256 entries
//! with popcount-4 symbols, assigned in ascending symbol order).
//!
//! The `.bin` card format is a 48-byte header ([`CARD_HEADER_LEN`]) followed
//! by the strip payload. The `.raw` strip format packs modulated symbols
//! MSB-first after a 2-byte sync, with Reed-Solomon blocks
//! (223 data + 32 parity) interleaved column-wise.

use crate::error::{Error, Result};
#[cfg(feature = "alloc")]
use crate::rs_ecc;

/// Length of the `.bin` card header in bytes.
pub const CARD_HEADER_LEN: usize = 48;
/// Magic of a `.bin` card payload (`ERB1`).
pub const BIN_MAGIC: &[u8; 4] = b"ERB1";
/// Sync byte starting a `.raw` strip.
pub const RAW_SYNC: u8 = 0xA5;
/// Data bytes per Reed-Solomon block in a strip.
pub const STRIP_BLOCK_DATA: usize = 223;
/// Parity bytes per Reed-Solomon block in a strip.
pub const STRIP_BLOCK_ECC: usize = 32;
/// Blocks in a short strip.
pub const SHORT_STRIP_BLOCKS: usize = 2;
/// Blocks in a long strip.
pub const LONG_STRIP_BLOCKS: usize = 8;

/// The 256-entry 8-to-10-bit optical modulation table.
pub const MODULATION_TABLE: [u16; 256] = build_modulation_table();

const fn build_modulation_table() -> [u16; 256] {
    let mut table = [0u16; 256];
    let mut idx = 0usize;
    let mut v = 0u16;
    while v < 1024 {
        if v.count_ones() == 5 {
            table[idx] = v;
            idx += 1;
        }
        v += 1;
    }
    let mut v = 0u16;
    while idx < 256 && v < 1024 {
        if v.count_ones() == 4 {
            table[idx] = v;
            idx += 1;
        }
        v += 1;
    }
    table
}

/// Reverse modulation table: symbol -> byte, or `-1` when invalid.
pub const DEMODULATION_TABLE: [i16; 1024] = build_demodulation_table();

const fn build_demodulation_table() -> [i16; 1024] {
    let mut table = [-1i16; 1024];
    let mut i = 0usize;
    while i < 256 {
        table[MODULATION_TABLE[i] as usize] = i as i16;
        i += 1;
    }
    table
}

/// Encode one byte to its 10-bit optical symbol.
#[must_use]
pub fn modulate(byte: u8) -> u16 {
    MODULATION_TABLE[byte as usize]
}

/// Decode a 10-bit optical symbol back to a byte.
pub fn demodulate(symbol: u16) -> Result<u8> {
    if symbol > 0x3FF {
        return Err(Error::InvalidSymbol(symbol));
    }
    let b = DEMODULATION_TABLE[symbol as usize];
    if b < 0 {
        Err(Error::InvalidSymbol(symbol))
    } else {
        Ok(b as u8)
    }
}

/// CRC-32 (IEEE 802.3, reflected polynomial `0xEDB88320`) checksum.
#[must_use]
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// Additive 16-bit strip checksum (sum of bytes, wrapping).
#[must_use]
pub fn strip_checksum(data: &[u8]) -> u16 {
    data.iter().fold(0u16, |acc, &b| acc.wrapping_add(b as u16))
}

/// Card type byte values for the `.bin` header.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CardType {
    /// Application / game data card.
    Application = 0,
    /// Trainer / structured data card.
    Trainer = 1,
    /// Minigame card.
    Minigame = 2,
}

/// Parsed 48-byte `.bin` card header.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CardHeader {
    /// Region byte (0 = Japan, 1 = USA).
    pub region: u8,
    /// Card type byte.
    pub card_type: u8,
    /// Zero-based index of this strip within the card.
    pub strip_index: u8,
    /// Total number of strips in the card.
    pub strip_count: u8,
    /// Payload length of this strip in bytes.
    pub payload_len: u32,
    /// Total payload length across all strips in bytes.
    pub total_len: u32,
    /// CRC-32 of this strip's payload.
    pub payload_crc32: u32,
}

impl CardHeader {
    /// Serialize the header into exactly [`CARD_HEADER_LEN`] bytes.
    #[must_use]
    pub fn to_bytes(&self) -> [u8; CARD_HEADER_LEN] {
        let mut h = [0u8; CARD_HEADER_LEN];
        h[0..4].copy_from_slice(BIN_MAGIC);
        h[4] = self.region;
        h[5] = self.card_type;
        h[6] = self.strip_index;
        h[7] = self.strip_count;
        h[8..12].copy_from_slice(&self.payload_len.to_be_bytes());
        h[12..16].copy_from_slice(&self.total_len.to_be_bytes());
        h[16..20].copy_from_slice(&self.payload_crc32.to_be_bytes());
        let crc = crc32(&h[0..44]);
        h[44..48].copy_from_slice(&crc.to_be_bytes());
        h
    }

    /// Parse and validate a header from the first 48 bytes of `data`.
    pub fn parse(data: &[u8]) -> Result<Self> {
        if data.len() < CARD_HEADER_LEN {
            return Err(Error::UnexpectedEof);
        }
        if &data[0..4] != BIN_MAGIC {
            return Err(Error::InvalidMagic);
        }
        let stored = u32::from_be_bytes([data[44], data[45], data[46], data[47]]);
        let actual = crc32(&data[0..44]);
        if stored != actual {
            return Err(Error::InvalidChecksum { expected: stored, actual });
        }
        Ok(Self {
            region: data[4],
            card_type: data[5],
            strip_index: data[6],
            strip_count: data[7],
            payload_len: u32::from_be_bytes([data[8], data[9], data[10], data[11]]),
            total_len: u32::from_be_bytes([data[12], data[13], data[14], data[15]]),
            payload_crc32: u32::from_be_bytes([data[16], data[17], data[18], data[19]]),
        })
    }
}

/// Build a complete `.bin` strip (48-byte header + payload).
#[cfg(feature = "alloc")]
pub fn build_bin(
    region: u8,
    card_type: u8,
    strip_index: u8,
    strip_count: u8,
    total_len: u32,
    payload: &[u8],
) -> alloc::vec::Vec<u8> {
    let header = CardHeader {
        region,
        card_type,
        strip_index,
        strip_count,
        payload_len: payload.len() as u32,
        total_len,
        payload_crc32: crc32(payload),
    };
    let mut out = alloc::vec::Vec::with_capacity(CARD_HEADER_LEN + payload.len());
    out.extend_from_slice(&header.to_bytes());
    out.extend_from_slice(payload);
    out
}

/// Validate a `.bin` strip and return its header and payload slice.
pub fn parse_bin(data: &[u8]) -> Result<(CardHeader, &[u8])> {
    let header = CardHeader::parse(data)?;
    let end = CARD_HEADER_LEN + header.payload_len as usize;
    if data.len() < end {
        return Err(Error::UnexpectedEof);
    }
    let payload = &data[CARD_HEADER_LEN..end];
    let actual = crc32(payload);
    if actual != header.payload_crc32 {
        return Err(Error::InvalidChecksum { expected: header.payload_crc32, actual });
    }
    Ok((header, payload))
}

/// Split a payload into a multi-strip set of `.bin` strips.
///
/// Each strip carries at most `strip_payload_max` payload bytes.
#[cfg(feature = "alloc")]
pub fn split_into_strips(
    payload: &[u8],
    region: u8,
    card_type: u8,
    strip_payload_max: usize,
) -> Result<alloc::vec::Vec<alloc::vec::Vec<u8>>> {
    if strip_payload_max == 0 {
        return Err(Error::InvalidSize("strip payload max is zero"));
    }
    let count = payload.len().div_ceil(strip_payload_max);
    if count > 255 {
        return Err(Error::InvalidSize("too many strips"));
    }
    let count = if count == 0 { 1 } else { count };
    let mut out = alloc::vec::Vec::with_capacity(count);
    for i in 0..count {
        let start = i * strip_payload_max;
        let end = core::cmp::min(start + strip_payload_max, payload.len());
        let chunk = if start <= payload.len() { &payload[start..end] } else { &[][..] };
        out.push(build_bin(region, card_type, i as u8, count as u8, payload.len() as u32, chunk));
    }
    Ok(out)
}

/// Reassemble a multi-strip set of `.bin` strips into the original payload.
#[cfg(feature = "alloc")]
pub fn assemble_strips(strips: &[alloc::vec::Vec<u8>]) -> Result<alloc::vec::Vec<u8>> {
    if strips.is_empty() {
        return Err(Error::InvalidStrip("no strips"));
    }
    let mut parsed = alloc::vec::Vec::with_capacity(strips.len());
    for s in strips {
        parsed.push(parse_bin(s)?);
    }
    let count = parsed[0].0.strip_count as usize;
    if count != strips.len() {
        return Err(Error::InvalidStrip("strip count mismatch"));
    }
    parsed.sort_by_key(|(h, _)| h.strip_index);
    let mut out = alloc::vec::Vec::with_capacity(parsed[0].0.total_len as usize);
    for (idx, (h, p)) in parsed.iter().enumerate() {
        if h.strip_index as usize != idx {
            return Err(Error::InvalidStrip("missing strip index"));
        }
        out.extend_from_slice(p);
    }
    Ok(out)
}

#[cfg(feature = "alloc")]
fn interleave(blocks: &[alloc::vec::Vec<u8>]) -> alloc::vec::Vec<u8> {
    let block_len = blocks.first().map_or(0, |b| b.len());
    let mut out = alloc::vec::Vec::with_capacity(block_len * blocks.len());
    for col in 0..block_len {
        for b in blocks {
            out.push(b[col]);
        }
    }
    out
}

#[cfg(feature = "alloc")]
fn deinterleave(data: &[u8], nblocks: usize) -> alloc::vec::Vec<alloc::vec::Vec<u8>> {
    let block_len = data.len().checked_div(nblocks).unwrap_or(0);
    let mut blocks = alloc::vec![alloc::vec![0u8; block_len]; nblocks];
    let mut k = 0usize;
    for col in 0..block_len {
        for b in blocks.iter_mut() {
            b[col] = data[k];
            k += 1;
        }
    }
    blocks
}

/// Encode `.bin` bytes into a `.raw` optical strip.
///
/// Returns `(raw_bytes, corrections_hint)` where `raw_bytes` holds the
/// 2-byte sync/header followed by modulated, interleaved RS codewords
/// packed MSB-first into bytes.
#[cfg(feature = "alloc")]
pub fn bin_to_raw(bin: &[u8]) -> Result<alloc::vec::Vec<u8>> {
    let nblocks = if bin.len() <= SHORT_STRIP_BLOCKS * STRIP_BLOCK_DATA {
        SHORT_STRIP_BLOCKS
    } else {
        LONG_STRIP_BLOCKS
    };
    let mut blocks = alloc::vec::Vec::with_capacity(nblocks);
    for i in 0..nblocks {
        let start = i * STRIP_BLOCK_DATA;
        let mut block = alloc::vec![0u8; STRIP_BLOCK_DATA + STRIP_BLOCK_ECC];
        if start < bin.len() {
            let end = core::cmp::min(start + STRIP_BLOCK_DATA, bin.len());
            block[..end - start].copy_from_slice(&bin[start..end]);
        }
        let (data_part, parity_part) = block.split_at_mut(STRIP_BLOCK_DATA);
        rs_ecc::rs_encode_parity(data_part, STRIP_BLOCK_ECC, parity_part)?;
        blocks.push(block);
    }
    let interleaved = interleave(&blocks);
    // Modulate each byte to 10 bits, packed MSB-first.
    let total_bits = interleaved.len() * 10;
    let mut raw = alloc::vec![0u8; 2 + total_bits.div_ceil(8)];
    raw[0] = RAW_SYNC;
    raw[1] = nblocks as u8;
    let mut bitpos = 0usize;
    for &b in &interleaved {
        let sym = modulate(b) as u32;
        for k in 0..10 {
            let bit = (sym >> (9 - k)) & 1;
            let p = bitpos + k;
            if bit == 1 {
                raw[2 + p / 8] |= 1 << (7 - (p % 8));
            }
        }
        bitpos += 10;
    }
    Ok(raw)
}

/// Decode a `.raw` optical strip back into `.bin` bytes.
///
/// Reed-Solomon correction is applied per block; returns the corrected
/// `.bin` byte vector (padded blocks trimmed by the `.bin` header length).
#[cfg(feature = "alloc")]
pub fn raw_to_bin(raw: &[u8]) -> Result<alloc::vec::Vec<u8>> {
    if raw.len() < 2 {
        return Err(Error::UnexpectedEof);
    }
    if raw[0] != RAW_SYNC {
        return Err(Error::InvalidMagic);
    }
    let nblocks = raw[1] as usize;
    if nblocks != SHORT_STRIP_BLOCKS && nblocks != LONG_STRIP_BLOCKS {
        return Err(Error::InvalidStrip("bad block count"));
    }
    let codeword_len = STRIP_BLOCK_DATA + STRIP_BLOCK_ECC;
    let total_symbols = nblocks * codeword_len;
    if raw.len() < 2 + (total_symbols * 10).div_ceil(8) {
        return Err(Error::UnexpectedEof);
    }
    let mut interleaved = alloc::vec![0u8; total_symbols];
    for (i, slot) in interleaved.iter_mut().enumerate() {
        let mut sym = 0u16;
        for k in 0..10 {
            let p = i * 10 + k;
            let bit = (raw[2 + p / 8] >> (7 - (p % 8))) & 1;
            sym = (sym << 1) | bit as u16;
        }
        // Invalid symbols (scan noise) become placeholder bytes that the
        // Reed-Solomon stage then corrects as ordinary errors.
        *slot = demodulate(sym).unwrap_or(0);
    }
    let mut blocks = deinterleave(&interleaved, nblocks);
    let mut out = alloc::vec::Vec::with_capacity(nblocks * STRIP_BLOCK_DATA);
    for b in blocks.iter_mut() {
        rs_ecc::rs_correct(b, STRIP_BLOCK_ECC)?;
        out.extend_from_slice(&b[..STRIP_BLOCK_DATA]);
    }
    // Trim padding using the embedded .bin header when present.
    if out.len() >= CARD_HEADER_LEN && &out[0..4] == BIN_MAGIC {
        if let Ok(h) = CardHeader::parse(&out) {
            let want = CARD_HEADER_LEN + h.payload_len as usize;
            if want <= out.len() {
                out.truncate(want);
            }
        }
    }
    Ok(out)
}

/// Whether a `.bin` payload fits a short strip (vs. requiring a long strip).
#[must_use]
pub fn is_short_strip(bin_len: usize) -> bool {
    bin_len <= SHORT_STRIP_BLOCKS * STRIP_BLOCK_DATA
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;

    #[test]
    fn crc32_known_vector() {
        // Standard CRC-32 check value for "123456789".
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
    }

    #[test]
    fn modulation_roundtrip_all_bytes() {
        for b in 0u8..=255 {
            assert_eq!(demodulate(modulate(b)).unwrap(), b);
        }
        assert!(MODULATION_TABLE.iter().all(|&s| s <= 0x3FF));
    }

    #[test]
    fn bin_roundtrip_and_strips() {
        let payload = b"synthetic card payload for testing";
        let strips = split_into_strips(payload, 1, 0, 10).unwrap();
        assert_eq!(strips.len(), 4);
        assert_eq!(assemble_strips(&strips).unwrap(), payload.to_vec());
        let (h, p) = parse_bin(&strips[0]).unwrap();
        assert_eq!(h.strip_count, 4);
        assert_eq!(p, b"synthetic ");
    }

    #[test]
    fn raw_roundtrip_with_corruption() {
        let bin = build_bin(1, 2, 0, 1, 5, b"hello");
        let raw = bin_to_raw(&bin).unwrap();
        assert_eq!(raw_to_bin(&raw).unwrap(), bin);
        // Corrupt a few packed bytes; RS must repair.
        let mut damaged = raw.clone();
        damaged[10] ^= 0xFF;
        damaged[100] ^= 0x3C;
        assert_eq!(raw_to_bin(&damaged).unwrap(), bin);
    }
}
