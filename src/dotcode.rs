//! Nintendo/Olympus DotCode optical modulation, `.raw` strip format, and
//! 48-byte `.bin` card header verified against retail e-Reader ROMs.
//!
//! Every constant and layout in this module is verified byte-for-byte against
//! `ST Decoder Ver.1.10 for CARDe` (`0x08711ad0` in `PSAE`, `0x083f0c50` in
//! `PEAJ`, `0x086c750c` in `PSAJ`) and the `PSAE` scan-mode IWRAM demodulation
//! table at `0x03006214`:
//!
//! - **8-to-10-bit modulation**: Exact 4-to-5-bit nibble split
//!   ([`NIBBLE_MODULATION`]), ensuring no three consecutive black dots (`111`)
//!   occur anywhere in a modulated bitstream.
//! - **`.raw` strip layout**:
//!   - Short strip: `18` physical blocks × `104` bytes = **`1,872` bytes**
//!     (`0x750`), carrying `28` interleaved `RS(64, 48)` codewords (`1,344`
//!     decoded bytes = 48-byte header + up to `1,296` payload bytes).
//!   - Long strip: `28` physical blocks × `104` bytes = **`2,912` bytes**
//!     (`0xB60`), carrying `44` interleaved `RS(64, 48)` codewords (`2,112`
//!     decoded bytes = 48-byte header + up to `2,064` payload bytes).
//! - **24-byte DotCode header**: `RS(24, 8)` with inverted parity (`^ 0xFF`),
//!   spread 2 bytes per physical block across blocks `0..12` (repeated in
//!   subsequent 12-block groups).
//! - **48-byte Card Header** (block 0 of the decoded `.bin` stream):
//!   big-endian `0x0030` header length, `0x10` version, `0x12` subheader
//!   offset, mandatory `"NINTENDO"` TLV (`0x0000`, len `8`), 16-bit inverted
//!   data checksum (`0x13..0x15`), and 8-bit inverted global checksum (`0x2F`).

use crate::error::{Error, Result};
#[cfg(feature = "alloc")]
use crate::rs_ecc;

/// Length of the `.bin` card header in bytes (`0x0030`).
pub const CARD_HEADER_LEN: usize = 48;
/// Mandatory TLV vendor signature in the 48-byte card header (`"NINTENDO"`).
pub const CARD_VENDOR_MAGIC: &[u8; 8] = b"NINTENDO";
/// Physical block size in bytes in a `.raw` strip (`0x68`).
pub const PHYSICAL_BLOCK_LEN: usize = 104;
/// DotCode header bytes carried at the start of each physical block.
pub const PHYSICAL_BLOCK_HEADER_BYTES: usize = 2;
/// Interleaved RS payload/parity bytes carried per physical block (`104 - 2`).
pub const PHYSICAL_BLOCK_DATA_BYTES: usize = 102;
/// Length of the `RS(24, 8)` DotCode strip header in bytes.
pub const DOTCODE_HEADER_LEN: usize = 24;
/// Physical blocks per DotCode header cycle (`12 * 2 = 24` bytes).
pub const DOTCODE_HEADER_BLOCKS: usize = 12;

/// Physical blocks in a short `.raw` strip (`18 * 104 = 1,872` bytes).
pub const SHORT_STRIP_PHYSICAL_BLOCKS: usize = 18;
/// Physical blocks in a long `.raw` strip (`28 * 104 = 2,912` bytes).
pub const LONG_STRIP_PHYSICAL_BLOCKS: usize = 28;
/// Exact byte length of a short `.raw` strip (`0x750`).
pub const SHORT_STRIP_RAW_LEN: usize = SHORT_STRIP_PHYSICAL_BLOCKS * PHYSICAL_BLOCK_LEN;
/// Exact byte length of a long `.raw` strip (`0xB60`).
pub const LONG_STRIP_RAW_LEN: usize = LONG_STRIP_PHYSICAL_BLOCKS * PHYSICAL_BLOCK_LEN;

/// Data bytes per Reed-Solomon codeword (`RS(64, 48)`).
pub const STRIP_BLOCK_DATA: usize = 48;
/// Parity bytes per Reed-Solomon codeword (`RS(64, 48)`).
pub const STRIP_BLOCK_ECC: usize = 16;
/// Total bytes per Reed-Solomon codeword (`48 + 16`).
pub const STRIP_BLOCK_LEN: usize = STRIP_BLOCK_DATA + STRIP_BLOCK_ECC;
/// Interleaved `RS(64, 48)` blocks in a short strip (`28 * 48 = 1,344` bytes).
pub const SHORT_STRIP_BLOCKS: usize = 28;
/// Interleaved `RS(64, 48)` blocks in a long strip (`44 * 48 = 2,112` bytes).
pub const LONG_STRIP_BLOCKS: usize = 44;
/// Maximum payload bytes in a short strip (`27 * 48 = 1,296`, `0x510`).
pub const SHORT_STRIP_MAX_PAYLOAD: usize = (SHORT_STRIP_BLOCKS - 1) * STRIP_BLOCK_DATA;
/// Maximum payload bytes in a long strip (`43 * 48 = 2,064`, `0x810`).
pub const LONG_STRIP_MAX_PAYLOAD: usize = (LONG_STRIP_BLOCKS - 1) * STRIP_BLOCK_DATA;

/// Exact 4-to-5-bit Olympus/Nintendo optical nibble modulation table
/// (extracted from `PSAE` scan-mode IWRAM `0x03006214`).
pub const NIBBLE_MODULATION: [u8; 16] = [
    0x00, 0x01, 0x02, 0x12, 0x04, 0x05, 0x06, 0x16,
    0x08, 0x09, 0x0A, 0x14, 0x0C, 0x0D, 0x11, 0x10,
];

/// The 256-entry 8-to-10-bit optical modulation table (`(hi5 << 5) | lo5`).
pub const MODULATION_TABLE: [u16; 256] = build_modulation_table();

const fn build_modulation_table() -> [u16; 256] {
    let mut table = [0u16; 256];
    let mut b = 0usize;
    while b < 256 {
        let hi = NIBBLE_MODULATION[b >> 4] as u16;
        let lo = NIBBLE_MODULATION[b & 0x0F] as u16;
        table[b] = (hi << 5) | lo;
        b += 1;
    }
    table
}

/// Reverse modulation table: 10-bit symbol -> byte (`0..=255`), or `-1` when invalid.
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

/// Encode one byte to its 10-bit Olympus/Nintendo optical symbol.
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

/// CRC-32 (IEEE 802.3, reflected polynomial `0xEDB88320`) utility checksum.
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

/// Hardware 16-bit big-endian word data checksum (inverted sum of big-endian
/// `u16` words across the 48-byte-aligned strip payload, verified against
/// `PSAE` `FUN_0871237c` at `0x08712658..0x08712758`).
#[must_use]
pub fn data_checksum(padded_payload: &[u8]) -> u16 {
    let mut sum = 0u16;
    let mut i = 0usize;
    while i < padded_payload.len() {
        let hi = padded_payload[i] as u16;
        let lo = if i + 1 < padded_payload.len() {
            padded_payload[i + 1] as u16
        } else {
            0
        };
        sum = sum.wrapping_add((hi << 8) | lo);
        i += 2;
    }
    !sum
}

/// Hardware 8-bit global checksum stored at header offset `0x2F`:
/// `!((sum(header[0..47]) + sum_{b} xor(block_b[0..48])) & 0xFF)`
/// (verified against `PSAE` `FUN_08712768` + `FUN_0871237c` at `0x08712674..0x08712708`).
#[must_use]
pub fn global_checksum(header_first_47: &[u8], padded_payload: &[u8]) -> u8 {
    let mut hdr_sum = 0u8;
    for &b in header_first_47.iter().take(47) {
        hdr_sum = hdr_sum.wrapping_add(b);
    }
    let mut blk_xor_sum = 0u8;
    for chunk in padded_payload.chunks(STRIP_BLOCK_DATA) {
        let mut x = 0u8;
        for &b in chunk {
            x ^= b;
        }
        blk_xor_sum = blk_xor_sum.wrapping_add(x);
    }
    !hdr_sum.wrapping_add(blk_xor_sum)
}

/// Additive 16-bit strip checksum (sum of bytes, wrapping).
#[must_use]
pub fn strip_checksum(data: &[u8]) -> u16 {
    data.iter().fold(0u16, |acc, &b| acc.wrapping_add(b as u16))
}

/// Card type byte values for the 48-byte `.bin` header.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CardType {
    /// Application / game data card.
    Application = 0,
    /// Trainer / structured data card.
    Trainer = 1,
    /// Minigame card.
    Minigame = 2,
}

/// Parsed 48-byte Nintendo/Olympus DotCode card header (block 0 of `.bin`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CardHeader {
    /// Region byte (`0` = Japan, `1` = USA).
    pub region: u8,
    /// Card / strip type byte (offset `0x03`).
    pub card_type: u8,
    /// Zero-based index of this strip within a multi-strip card (offset `0x0D`).
    pub strip_index: u8,
    /// Total number of strips in the card (offset `0x0E`).
    pub strip_count: u8,
    /// Payload length of this strip in bytes (offset `0x06..0x08`, u16 BE).
    pub payload_len: u16,
    /// Header flags byte (offset `0x02`, bit 0 = global checksum enabled).
    pub flags: u8,
    /// Stored 16-bit inverted data checksum (offset `0x13..0x15`, u16 BE).
    pub data_checksum: u16,
    /// Stored 8-bit inverted global checksum (offset `0x2F`).
    pub global_checksum: u8,
}

impl CardHeader {
    /// Serialize the header into exactly [`CARD_HEADER_LEN`] (48) bytes,
    /// computing both the 16-bit data checksum and 8-bit global checksum over
    /// `padded_payload` (which must be padded to a multiple of 48 bytes).
    #[must_use]
    pub fn to_bytes(&self, padded_payload: &[u8]) -> [u8; CARD_HEADER_LEN] {
        let mut h = [0u8; CARD_HEADER_LEN];
        // 0x00..0x02: header length = 0x0030 (48)
        h[0x00..0x02].copy_from_slice(&(CARD_HEADER_LEN as u16).to_be_bytes());
        // 0x02: flags (bit 0 = global checksum enabled)
        h[0x02] = self.flags | 0x01;
        // 0x03: card/strip type
        h[0x03] = self.card_type;
        // 0x06..0x08: payload length (u16 BE)
        h[0x06..0x08].copy_from_slice(&self.payload_len.to_be_bytes());
        // 0x0A: format version = 0x10 (mandatory in FUN_08712768)
        h[0x0A] = 0x10;
        // 0x0B: subheader offset = 0x12 (mandatory in FUN_08712768)
        h[0x0B] = 0x12;
        // 0x0C..0x12: metadata
        h[0x0C] = self.region;
        h[0x0D] = self.strip_index;
        h[0x0E] = self.strip_count;
        // 0x12: enable 16-bit data checksum (0x10)
        h[0x12] = 0x10;
        let dcs = data_checksum(padded_payload);
        h[0x13..0x15].copy_from_slice(&dcs.to_be_bytes());
        // 0x15: TLV length = 12 (Tag 0x0000, Len 0x0008, "NINTENDO")
        h[0x15] = 12;
        h[0x16..0x18].copy_from_slice(&0x0000u16.to_be_bytes());
        h[0x18..0x1A].copy_from_slice(&0x0008u16.to_be_bytes());
        h[0x1A..0x22].copy_from_slice(CARD_VENDOR_MAGIC);
        // 0x2F: global checksum
        h[0x2F] = global_checksum(&h[0..47], padded_payload);
        h
    }

    /// Parse and validate the 48-byte DotCode card header from `data`.
    pub fn parse(data: &[u8]) -> Result<Self> {
        if data.len() < CARD_HEADER_LEN {
            return Err(Error::UnexpectedEof);
        }
        let hdr_len = u16::from_be_bytes([data[0x00], data[0x01]]) as usize;
        if hdr_len != CARD_HEADER_LEN {
            return Err(Error::InvalidHeader("header length is not 0x0030"));
        }
        let flags = data[0x02];
        if flags & !0x01 != 0 && (flags & 0x01) == 0 {
            return Err(Error::InvalidHeader("invalid header flags"));
        }
        if data[0x0A] != 0x10 {
            return Err(Error::InvalidHeader("format version byte is not 0x10"));
        }
        if data[0x0B] != 0x12 {
            return Err(Error::InvalidHeader("subheader offset byte is not 0x12"));
        }
        if data[0x12] != 0x00 && data[0x12] != 0x10 {
            return Err(Error::InvalidHeader("invalid data checksum flag"));
        }
        let tlv_len = data[0x15] as usize;
        if 0x16 + tlv_len > CARD_HEADER_LEN {
            return Err(Error::InvalidHeader("TLV section exceeds 48-byte header"));
        }
        // Verify mandatory "NINTENDO" TLV (tag 0x0000, len 8).
        let mut pos = 0x16usize;
        let end = 0x16 + tlv_len;
        let mut found_vendor = false;
        while pos + 4 <= end {
            let tag = u16::from_be_bytes([data[pos], data[pos + 1]]);
            let len = u16::from_be_bytes([data[pos + 2], data[pos + 3]]) as usize;
            pos += 4;
            if pos + len > end {
                return Err(Error::InvalidHeader("truncated TLV entry"));
            }
            if tag == 0x0000 {
                if len != CARD_VENDOR_MAGIC.len() || &data[pos..pos + len] != CARD_VENDOR_MAGIC {
                    return Err(Error::InvalidMagic);
                }
                found_vendor = true;
            }
            pos += len;
        }
        if !found_vendor {
            return Err(Error::InvalidMagic);
        }
        Ok(Self {
            region: data[0x0C],
            card_type: data[0x03],
            strip_index: data[0x0D],
            strip_count: data[0x0E],
            payload_len: u16::from_be_bytes([data[0x06], data[0x07]]),
            flags,
            data_checksum: u16::from_be_bytes([data[0x13], data[0x14]]),
            global_checksum: data[0x2F],
        })
    }
}

/// Build a complete `.bin` strip (48-byte hardware header + 48-byte-padded payload).
///
/// The resulting buffer length is `1,344` bytes (`28 * 48`, short strip) when
/// `payload.len() <= 1,296`, or `2,112` bytes (`44 * 48`, long strip) when
/// `payload.len() <= 2,064`.
#[cfg(feature = "alloc")]
pub fn build_bin(
    region: u8,
    card_type: u8,
    strip_index: u8,
    strip_count: u8,
    payload: &[u8],
) -> Result<alloc::vec::Vec<u8>> {
    let max_payload = if payload.len() <= SHORT_STRIP_MAX_PAYLOAD {
        SHORT_STRIP_MAX_PAYLOAD
    } else if payload.len() <= LONG_STRIP_MAX_PAYLOAD {
        LONG_STRIP_MAX_PAYLOAD
    } else {
        return Err(Error::InvalidSize("payload exceeds long strip capacity (2064 bytes)"));
    };
    let mut padded = alloc::vec![0u8; max_payload];
    padded[..payload.len()].copy_from_slice(payload);
    let header = CardHeader {
        region,
        card_type,
        strip_index,
        strip_count,
        payload_len: payload.len() as u16,
        flags: 0x01,
        data_checksum: data_checksum(&padded),
        global_checksum: 0,
    };
    let hdr_bytes = header.to_bytes(&padded);
    let mut out = alloc::vec::Vec::with_capacity(CARD_HEADER_LEN + max_payload);
    out.extend_from_slice(&hdr_bytes);
    out.extend_from_slice(&padded);
    Ok(out)
}

/// Validate a `.bin` strip (48-byte header, 16-bit data checksum, and 8-bit
/// global checksum) and return its header and trimmed payload slice.
pub fn parse_bin(data: &[u8]) -> Result<(CardHeader, &[u8])> {
    let header = CardHeader::parse(data)?;
    let padded_payload = &data[CARD_HEADER_LEN..];
    if header.payload_len as usize > padded_payload.len() {
        return Err(Error::UnexpectedEof);
    }
    if data[0x12] == 0x10 {
        let actual_dcs = data_checksum(padded_payload);
        if actual_dcs != header.data_checksum {
            return Err(Error::InvalidChecksum {
                expected: header.data_checksum as u32,
                actual: actual_dcs as u32,
            });
        }
    }
    if header.flags & 0x01 != 0 {
        let actual_gcs = global_checksum(&data[0..47], padded_payload);
        if actual_gcs != header.global_checksum {
            return Err(Error::InvalidChecksum {
                expected: header.global_checksum as u32,
                actual: actual_gcs as u32,
            });
        }
    }
    Ok((header, &padded_payload[..header.payload_len as usize]))
}

/// Split a payload into a multi-strip set of `.bin` strips.
///
/// Each strip carries at most `strip_payload_max` payload bytes (capped at
/// [`LONG_STRIP_MAX_PAYLOAD`] = 2,064).
#[cfg(feature = "alloc")]
pub fn split_into_strips(
    payload: &[u8],
    region: u8,
    card_type: u8,
    strip_payload_max: usize,
) -> Result<alloc::vec::Vec<alloc::vec::Vec<u8>>> {
    if strip_payload_max == 0 || strip_payload_max > LONG_STRIP_MAX_PAYLOAD {
        return Err(Error::InvalidSize("invalid strip payload max"));
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
        out.push(build_bin(region, card_type, i as u8, count as u8, chunk)?);
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
    let mut out = alloc::vec::Vec::new();
    for (idx, (h, p)) in parsed.iter().enumerate() {
        if h.strip_index as usize != idx {
            return Err(Error::InvalidStrip("missing strip index"));
        }
        out.extend_from_slice(p);
    }
    Ok(out)
}

/// Encode `.bin` strip bytes into a hardware-exact `.raw` strip
/// (`1,872` bytes for short strips, `2,912` bytes for long strips).
///
/// The resulting `.raw` strip is byte-for-byte compatible with the retail
/// e-Reader `ST Decoder Ver.1.10 for CARDe` (`PSAE` `0x08711ad0`).
#[cfg(feature = "alloc")]
pub fn bin_to_raw(bin: &[u8]) -> Result<alloc::vec::Vec<u8>> {
    let (phys_blocks, interleave) = if bin.len() <= SHORT_STRIP_BLOCKS * STRIP_BLOCK_DATA {
        (SHORT_STRIP_PHYSICAL_BLOCKS, SHORT_STRIP_BLOCKS)
    } else if bin.len() <= LONG_STRIP_BLOCKS * STRIP_BLOCK_DATA {
        (LONG_STRIP_PHYSICAL_BLOCKS, LONG_STRIP_BLOCKS)
    } else {
        return Err(Error::InvalidSize("bin exceeds long strip size (2112 bytes)"));
    };

    // Build 24-byte DotCode header: RS(24, 8) with inverted parity.
    let mut dc_hdr = [0u8; DOTCODE_HEADER_LEN];
    dc_hdr[0] = 0x00;
    dc_hdr[1] = 0x01;
    dc_hdr[2..4].copy_from_slice(&1u16.to_be_bytes());
    dc_hdr[4] = STRIP_BLOCK_LEN as u8;
    dc_hdr[5] = STRIP_BLOCK_ECC as u8;
    dc_hdr[6..8].copy_from_slice(&(interleave as u16).to_be_bytes());
    let (hdr_data, hdr_par) = dc_hdr.split_at_mut(8);
    rs_ecc::rs_encode_parity_inverted(hdr_data, 16, hdr_par)?;

    // Encode `interleave` RS(64, 48) codewords with inverted parity.
    let mut blocks = alloc::vec![[0u8; STRIP_BLOCK_LEN]; interleave];
    for (i, blk) in blocks.iter_mut().enumerate() {
        let start = i * STRIP_BLOCK_DATA;
        if start < bin.len() {
            let end = core::cmp::min(start + STRIP_BLOCK_DATA, bin.len());
            blk[..end - start].copy_from_slice(&bin[start..end]);
        }
        let (data_part, parity_part) = blk.split_at_mut(STRIP_BLOCK_DATA);
        rs_ecc::rs_encode_parity_inverted(data_part, STRIP_BLOCK_ECC, parity_part)?;
    }

    // Interleave column-wise across all 64 rows.
    let mut interleaved = alloc::vec![0u8; interleave * STRIP_BLOCK_LEN];
    let mut k = 0usize;
    for row in 0..STRIP_BLOCK_LEN {
        for blk in &blocks {
            interleaved[k] = blk[row];
            k += 1;
        }
    }

    // Pack into physical blocks of 104 bytes (2 DotCode header bytes + 102 data bytes).
    let mut raw = alloc::vec![0u8; phys_blocks * PHYSICAL_BLOCK_LEN];
    let mut data_pos = 0usize;
    for pb in 0..phys_blocks {
        let base = pb * PHYSICAL_BLOCK_LEN;
        let hb = (pb % DOTCODE_HEADER_BLOCKS) * PHYSICAL_BLOCK_HEADER_BYTES;
        raw[base] = dc_hdr[hb];
        raw[base + 1] = dc_hdr[hb + 1];
        let take = core::cmp::min(PHYSICAL_BLOCK_DATA_BYTES, interleaved.len() - data_pos);
        if take > 0 {
            raw[base + PHYSICAL_BLOCK_HEADER_BYTES..base + PHYSICAL_BLOCK_HEADER_BYTES + take]
                .copy_from_slice(&interleaved[data_pos..data_pos + take]);
            data_pos += take;
        }
    }
    Ok(raw)
}

/// Decode and Reed-Solomon-correct a hardware `.raw` strip (`1,872` or `2,912`
/// bytes) back into its `.bin` bytes (`1,344` or `2,112` bytes: 48-byte card
/// header + padded payload).
///
/// Mirrors `ST Decoder Ver.1.10 for CARDe` (`PSAE` `0x08711ad0`):
/// 1. Extracts and RS-corrects the 24-byte `RS(24, 8)` DotCode header.
/// 2. Deinterleaves and RS-corrects all `28` (short) or `44` (long) `RS(64, 48)`
///    blocks.
/// 3. Validates the 48-byte card header, `"NINTENDO"` TLV, 16-bit data checksum,
///    and 8-bit global checksum.
#[cfg(feature = "alloc")]
pub fn raw_to_bin(raw: &[u8]) -> Result<alloc::vec::Vec<u8>> {
    let phys_blocks = match raw.len() {
        SHORT_STRIP_RAW_LEN => SHORT_STRIP_PHYSICAL_BLOCKS,
        LONG_STRIP_RAW_LEN => LONG_STRIP_PHYSICAL_BLOCKS,
        _ => return Err(Error::InvalidStrip("raw strip length must be 1872 (short) or 2912 (long)")),
    };

    // Extract 24-byte DotCode header from first 12 physical blocks (2 bytes each)
    // and repair with RS(24, 8) inverted-parity ECC.
    let mut dc_hdr = [0u8; DOTCODE_HEADER_LEN];
    for pb in 0..DOTCODE_HEADER_BLOCKS {
        let base = pb * PHYSICAL_BLOCK_LEN;
        dc_hdr[pb * 2] = raw[base];
        dc_hdr[pb * 2 + 1] = raw[base + 1];
    }
    rs_ecc::rs_correct_inverted(&mut dc_hdr, 16)?;

    let rs_n = dc_hdr[4] as usize;
    let rs_nsym = dc_hdr[5] as usize;
    let interleave = u16::from_be_bytes([dc_hdr[6], dc_hdr[7]]) as usize;
    if rs_n != STRIP_BLOCK_LEN || rs_nsym != STRIP_BLOCK_ECC {
        return Err(Error::InvalidHeader("unsupported RS block geometry in DotCode header"));
    }
    if interleave == 0 || interleave * rs_n > phys_blocks * PHYSICAL_BLOCK_DATA_BYTES {
        return Err(Error::InvalidHeader("invalid interleave count in DotCode header"));
    }

    // Gather interleaved payload+parity stream from the 102 data bytes of each physical block.
    let total_stream = interleave * rs_n;
    let mut interleaved = alloc::vec::Vec::with_capacity(total_stream);
    for pb in 0..phys_blocks {
        let base = pb * PHYSICAL_BLOCK_LEN + PHYSICAL_BLOCK_HEADER_BYTES;
        let rem = total_stream - interleaved.len();
        if rem == 0 {
            break;
        }
        let take = core::cmp::min(PHYSICAL_BLOCK_DATA_BYTES, rem);
        interleaved.extend_from_slice(&raw[base..base + take]);
    }

    // Deinterleave and RS(64, 48)-correct every block.
    let mut out = alloc::vec::Vec::with_capacity(interleave * STRIP_BLOCK_DATA);
    let mut cw = [0u8; STRIP_BLOCK_LEN];
    for col in 0..interleave {
        for (row, slot) in cw.iter_mut().enumerate() {
            *slot = interleaved[row * interleave + col];
        }
        rs_ecc::rs_correct_inverted(&mut cw, STRIP_BLOCK_ECC)?;
        out.extend_from_slice(&cw[..STRIP_BLOCK_DATA]);
    }

    // Validate the 48-byte card header and both hardware checksums.
    let _ = parse_bin(&out)?;
    Ok(out)
}

/// Whether a payload of `payload_len` bytes fits a short strip (<= 1,296 bytes).
#[must_use]
pub fn is_short_strip(payload_len: usize) -> bool {
    payload_len <= SHORT_STRIP_MAX_PAYLOAD
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;

    #[test]
    fn crc32_known_vector() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
    }

    #[test]
    fn modulation_roundtrip_all_bytes_no_three_ones() {
        for b in 0u8..=255 {
            let sym = modulate(b);
            assert!(sym <= 0x3FF);
            assert_eq!(demodulate(sym).unwrap(), b);
            // Olympus RLL property: no 3 consecutive 1-bits inside any symbol.
            assert_eq!(sym & (sym >> 1) & (sym >> 2), 0);
        }
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
    fn raw_short_and_long_roundtrip_with_corruption() {
        // Short strip
        let bin_s = build_bin(1, 2, 0, 1, b"hello short strip").unwrap();
        assert_eq!(bin_s.len(), SHORT_STRIP_BLOCKS * STRIP_BLOCK_DATA);
        let raw_s = bin_to_raw(&bin_s).unwrap();
        assert_eq!(raw_s.len(), SHORT_STRIP_RAW_LEN);
        assert_eq!(raw_to_bin(&raw_s).unwrap(), bin_s);

        // Corrupt 4 DotCode header bytes + 8 data bytes; RS must repair.
        let mut damaged = raw_s.clone();
        damaged[0] ^= 0xFF;
        damaged[2 * 104 + 1] ^= 0x55;
        damaged[5 * 104] ^= 0xAA;
        damaged[9 * 104 + 1] ^= 0x3C;
        for i in 0..8 {
            damaged[(i * 2) * 104 + 10 + i] ^= 0x77;
        }
        assert_eq!(raw_to_bin(&damaged).unwrap(), bin_s);

        // Long strip
        let long_pay = alloc::vec![0x5Au8; 1800];
        let bin_l = build_bin(1, 0, 0, 1, &long_pay).unwrap();
        assert_eq!(bin_l.len(), LONG_STRIP_BLOCKS * STRIP_BLOCK_DATA);
        let raw_l = bin_to_raw(&bin_l).unwrap();
        assert_eq!(raw_l.len(), LONG_STRIP_RAW_LEN);
        assert_eq!(raw_to_bin(&raw_l).unwrap(), bin_l);
    }
}
