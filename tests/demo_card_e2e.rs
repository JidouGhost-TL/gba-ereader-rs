//! Clean-room end-to-end GBA executable e-Reader card pipeline test.
//!
//! Builds a 100% clean-room ARM7TDMI GBA Mode 3 demo payload, compresses it
//! with `vpk0`, wraps it in a retail-compatible `PSAE` (USA e-Reader) Card
//! Type `0x0E` 48-byte DotCode header (including `"NINTENDO"` TLV `0x0000`,
//! application TLV `0x0022`, 16-bit data checksum, 8-bit header XOR checksum,
//! and 8-bit global checksum), encodes it to a 1,872-byte `.raw` strip,
//! renders it to printable Olympus/Nintendo DotCode BMP & PBM bitmaps,
//! verifies optical dot geometry round-trip, injects RS header + data
//! corruption, repairs via `raw_to_bin`, and verifies bit-for-bit payload
//! recovery.

use gba_ereader::dotcode::{
    bin_to_raw, data_checksum, demodulate, global_checksum, parse_bin, raw_to_bin,
    CARD_HEADER_LEN, PHYSICAL_BLOCK_LEN, SHORT_STRIP_MAX_PAYLOAD, SHORT_STRIP_PHYSICAL_BLOCKS,
    SHORT_STRIP_RAW_LEN,
};
use gba_ereader::render::{
    module_px, render_strip, ANCHOR_SIZE, BLOCK_COLS, QUIET_ZONE, STRIP_ROWS,
};
use gba_ereader::vpk0::{vpk0_compress, vpk0_decompress};

/// Build the clean-room ARM7TDMI GBA Mode 3 demo binary (entry at `0x02000000`).
fn build_cleanroom_demo_gba() -> Vec<u8> {
    const GLYPHS: &[(u8, [u8; 5])] = &[
        (b' ', [0x00, 0x00, 0x00, 0x00, 0x00]),
        (b'+', [0x08, 0x08, 0x3E, 0x08, 0x08]),
        (b'-', [0x08, 0x08, 0x08, 0x08, 0x08]),
        (b'0', [0x3E, 0x51, 0x49, 0x45, 0x3E]),
        (b'7', [0x01, 0x71, 0x09, 0x05, 0x03]),
        (b'A', [0x7E, 0x11, 0x11, 0x11, 0x7E]),
        (b'B', [0x7F, 0x49, 0x49, 0x49, 0x36]),
        (b'C', [0x3E, 0x41, 0x41, 0x41, 0x22]),
        (b'D', [0x7F, 0x41, 0x41, 0x22, 0x1C]),
        (b'E', [0x7F, 0x49, 0x49, 0x49, 0x41]),
        (b'G', [0x3E, 0x41, 0x49, 0x49, 0x7A]),
        (b'I', [0x00, 0x41, 0x7F, 0x41, 0x00]),
        (b'K', [0x7F, 0x08, 0x14, 0x22, 0x41]),
        (b'L', [0x7F, 0x40, 0x40, 0x40, 0x40]),
        (b'M', [0x7F, 0x02, 0x0C, 0x02, 0x7F]),
        (b'N', [0x7F, 0x04, 0x08, 0x10, 0x7F]),
        (b'O', [0x3E, 0x41, 0x41, 0x41, 0x3E]),
        (b'P', [0x7F, 0x09, 0x09, 0x09, 0x06]),
        (b'R', [0x7F, 0x09, 0x19, 0x29, 0x46]),
        (b'S', [0x46, 0x49, 0x49, 0x49, 0x31]),
        (b'T', [0x01, 0x01, 0x7F, 0x01, 0x01]),
        (b'V', [0x1F, 0x20, 0x40, 0x20, 0x1F]),
    ];

    fn encode_str(s: &[u8]) -> Vec<u8> {
        let mut out = Vec::with_capacity(s.len() + 1);
        for &ch in s {
            let idx = GLYPHS.iter().position(|(c, _)| *c == ch).expect("glyph");
            out.push(idx as u8);
        }
        out.push(0xFF);
        out
    }

    let mut data = Vec::new();
    let off_font = data.len();
    for (_, cols) in GLYPHS {
        data.extend_from_slice(cols);
    }
    let off_str1 = data.len();
    data.extend_from_slice(&encode_str(b"GBA-EREADER-RS"));
    let off_str2 = data.len();
    data.extend_from_slice(&encode_str(b"SCAN + VPK0 OK"));
    let off_str3 = data.len();
    data.extend_from_slice(&encode_str(b"CLEAN-ROOM ARM7TDMI CARD"));
    while data.len() % 4 != 0 {
        data.push(0);
    }
    let off_bars = data.len();
    let bars: [u16; 8] = [0x7FFF, 0x03FF, 0x7FE0, 0x03E0, 0x7C1F, 0x001F, 0x7C00, 0x39CE];
    for c in bars {
        data.extend_from_slice(&c.to_le_bytes());
    }

    struct Asm {
        words: Vec<u32>,
        literals: Vec<(usize, u32)>,
        branches: Vec<(usize, &'static str, u32)>,
        labels: Vec<(&'static str, usize)>,
    }
    impl Asm {
        fn new() -> Self {
            Self {
                words: Vec::new(),
                literals: Vec::new(),
                branches: Vec::new(),
                labels: Vec::new(),
            }
        }
        fn emit(&mut self, w: u32) {
            self.words.push(w);
        }
        fn label(&mut self, name: &'static str) {
            self.labels.push((name, self.words.len()));
        }
        fn ldr_lit(&mut self, rd: u32, val: u32) {
            let idx = self.words.len();
            self.literals.push((idx, val));
            self.words.push(0xE59F_0000 | (rd << 12));
        }
        fn b(&mut self, cond: u32, link: bool, target: &'static str) {
            let idx = self.words.len();
            let op = (cond << 28) | (if link { 0x0B00_0000 } else { 0x0A00_0000 });
            self.branches.push((idx, target, op));
            self.words.push(op);
        }
        fn finalize(mut self, data: &[u8], base_addr: u32) -> (Vec<u8>, u32) {
            for (idx, target, op) in self.branches {
                let tgt = self.labels.iter().find(|(n, _)| *n == target).expect("label").1;
                let off = (tgt as isize) - (idx as isize) - 2;
                self.words[idx] = op | ((off as u32) & 0x00FF_FFFF);
            }
            let pool_start_word = self.words.len();
            let mut pool_vals: Vec<u32> = Vec::new();
            for (idx, val) in self.literals {
                let pos = if let Some(p) = pool_vals.iter().position(|&v| v == val) {
                    p
                } else {
                    pool_vals.push(val);
                    pool_vals.len() - 1
                };
                let lit_word = pool_start_word + pos;
                let byte_off = ((lit_word as isize) - (idx as isize) - 2) * 4;
                assert!((0..4096).contains(&byte_off));
                self.words[idx] |= byte_off as u32;
            }
            for v in pool_vals {
                self.words.push(v);
            }
            let data_base = base_addr + (self.words.len() * 4) as u32;
            let mut out = Vec::with_capacity(self.words.len() * 4 + data.len());
            for w in self.words {
                out.extend_from_slice(&w.to_le_bytes());
            }
            out.extend_from_slice(data);
            (out, data_base)
        }
    }

    let base = 0x0200_0000u32;
    let mut data_base = base + 0x200;
    for _ in 0..2 {
        let mut a = Asm::new();
        a.ldr_lit(0, 0x0400_0208);
        a.emit(0xE3A0_1000);
        a.emit(0xE1C0_10B0);
        a.ldr_lit(0, 0x0400_0050);
        a.emit(0xE1C0_10B0);
        a.ldr_lit(0, 0x0400_0000);
        a.ldr_lit(1, 0x0403);
        a.emit(0xE1C0_10B0);

        a.emit(0xE3A0_0000);
        a.emit(0xE3A0_1000);
        a.emit(0xE3A0_20F0);
        a.emit(0xE3A0_30A0);
        a.ldr_lit(4, 0x2882);
        a.b(0xE, true, "fill_rect");

        a.emit(0xE3A0_0004);
        a.emit(0xE3A0_1004);
        a.emit(0xE3A0_20E8);
        a.emit(0xE3A0_3098);
        a.ldr_lit(4, 0x7F40);
        a.b(0xE, true, "fill_rect");

        a.emit(0xE3A0_0006);
        a.emit(0xE3A0_1006);
        a.emit(0xE3A0_20E4);
        a.emit(0xE3A0_3094);
        a.ldr_lit(4, 0x2882);
        a.b(0xE, true, "fill_rect");

        a.emit(0xE3A0_000A);
        a.emit(0xE3A0_100C);
        a.emit(0xE3A0_20DC);
        a.emit(0xE3A0_301C);
        a.ldr_lit(4, 0x4140);
        a.b(0xE, true, "fill_rect");

        a.emit(0xE3A0_000E);
        a.emit(0xE3A0_1030);
        a.emit(0xE3A0_20D4);
        a.emit(0xE3A0_3034);
        a.ldr_lit(4, 0x7FFF);
        a.b(0xE, true, "fill_rect");

        a.ldr_lit(8, data_base + off_bars as u32);
        a.emit(0xE3A0_9000);
        a.emit(0xE3A0_A010);
        a.label("bar_loop");
        a.emit(0xE1A0_000A);
        a.emit(0xE3A0_1032);
        a.emit(0xE3A0_201A);
        a.emit(0xE3A0_3030);
        a.emit(0xE0D8_40B2);
        a.b(0xE, true, "fill_rect");
        a.emit(0xE28A_A01A);
        a.emit(0xE289_9001);
        a.emit(0xE359_0008);
        a.b(0x1, false, "bar_loop");

        a.emit(0xE3A0_0024);
        a.emit(0xE3A0_1013);
        a.ldr_lit(2, data_base + off_str1 as u32);
        a.emit(0xE3A0_3002);
        a.ldr_lit(4, 0x7FFF);
        a.b(0xE, true, "draw_str");

        a.emit(0xE3A0_0024);
        a.emit(0xE3A0_106E);
        a.ldr_lit(2, data_base + off_str2 as u32);
        a.emit(0xE3A0_3002);
        a.ldr_lit(4, 0x2FE4);
        a.b(0xE, true, "draw_str");

        a.emit(0xE3A0_0030);
        a.emit(0xE3A0_1086);
        a.ldr_lit(2, data_base + off_str3 as u32);
        a.emit(0xE3A0_3001);
        a.ldr_lit(4, 0x7F70);
        a.b(0xE, true, "draw_str");

        a.label("halt");
        a.b(0xE, false, "halt");

        a.label("fill_rect");
        a.emit(0xE92D_40F0);
        a.emit(0xE3A0_50F0);
        a.emit(0xE020_0591);
        a.emit(0xE1A0_0080);
        a.emit(0xE280_0406);
        a.emit(0xE1A0_6003);
        a.label("fr_row");
        a.emit(0xE1A0_5000);
        a.emit(0xE1A0_7002);
        a.label("fr_col");
        a.emit(0xE0C5_40B2);
        a.emit(0xE257_7001);
        a.b(0x1, false, "fr_col");
        a.emit(0xE280_0F78);
        a.emit(0xE256_6001);
        a.b(0x1, false, "fr_row");
        a.emit(0xE8BD_80F0);

        a.label("draw_str");
        a.emit(0xE92D_4FF0);
        a.emit(0xE1A0_8000);
        a.emit(0xE1A0_9001);
        a.emit(0xE1A0_A002);
        a.emit(0xE1A0_B003);
        a.label("ds_char");
        a.emit(0xE4DA_5001);
        a.emit(0xE355_00FF);
        a.b(0x0, false, "ds_done");
        a.ldr_lit(6, data_base + off_font as u32);
        a.emit(0xE086_6005);
        a.emit(0xE086_6105);
        a.emit(0xE3A0_7000);
        a.label("ds_col");
        a.emit(0xE7D6_C007);
        a.emit(0xE3A0_5000);
        a.label("ds_row");
        a.emit(0xE31C_0001);
        a.b(0x0, false, "ds_skip");
        a.emit(0xE92D_10E0);
        a.emit(0xE020_8B97);
        a.emit(0xE021_9B95);
        a.emit(0xE1A0_200B);
        a.emit(0xE1A0_300B);
        a.b(0xE, true, "fill_rect");
        a.emit(0xE8BD_10E0);
        a.label("ds_skip");
        a.emit(0xE1A0_C0AC);
        a.emit(0xE285_5001);
        a.emit(0xE355_0007);
        a.b(0x1, false, "ds_row");
        a.emit(0xE287_7001);
        a.emit(0xE357_0005);
        a.b(0x1, false, "ds_col");
        a.emit(0xE088_808B);
        a.emit(0xE088_810B);
        a.b(0xE, false, "ds_char");
        a.label("ds_done");
        a.emit(0xE8BD_8FF0);

        let (bytes, actual_data_base) = a.finalize(&data, base);
        if actual_data_base == data_base {
            return bytes;
        }
        data_base = actual_data_base;
    }
    unreachable!()
}

/// Package a GBA executable binary into a single-strip USA (`PSAE`) Card Type
/// `0x0E` `.bin` strip (1,344 bytes) accepted by retail e-Reader firmware.
fn build_psae_gba_card_bin(gba_bin: &[u8]) -> Vec<u8> {
    let vpk = vpk0_compress(gba_bin).expect("vpk0_compress");
    let mut payload = Vec::new();
    payload.extend_from_slice(&[0u8; 33]); // Empty 33-byte title -> immediate boot
    payload.extend_from_slice(&(vpk.len() as u16).to_le_bytes());
    payload.extend_from_slice(&0u32.to_le_bytes()); // GBA-type 4-byte NULL marker
    payload.extend_from_slice(&vpk);
    assert!(payload.len() <= SHORT_STRIP_MAX_PAYLOAD);

    let mut bin = vec![0u8; CARD_HEADER_LEN + SHORT_STRIP_MAX_PAYLOAD];
    bin[CARD_HEADER_LEN..CARD_HEADER_LEN + payload.len()].copy_from_slice(&payload);

    let mut h = [0u8; CARD_HEADER_LEN];
    h[0x00..0x02].copy_from_slice(&(CARD_HEADER_LEN as u16).to_be_bytes());
    h[0x02] = 0x01; // global checksum enabled
    h[0x03] = 0x02; // Primary Type
    h[0x06..0x08].copy_from_slice(&(payload.len() as u16).to_be_bytes());
    h[0x0A] = 0x10; // format version
    h[0x0B] = 0x12; // subheader offset
    h[0x0C] = 0xE0; // Card Type 0x0E (Dotcode Application, 33-byte title)
    h[0x0D] = 0x01; // Region 1 (USA)
    h[0x0E] = 0x02; // Short strip
    h[0x12] = 0x10; // 16-bit data checksum enabled
    let dcs = data_checksum(&bin[CARD_HEADER_LEN..]);
    h[0x13..0x15].copy_from_slice(&dcs.to_be_bytes());
    h[0x15] = 0x19; // 25 bytes of TLVs (Tag 0x0000 "NINTENDO" + Tag 0x0022 App Header)
    h[0x16..0x18].copy_from_slice(&0x0000u16.to_be_bytes());
    h[0x18..0x1A].copy_from_slice(&0x0008u16.to_be_bytes());
    h[0x1A..0x22].copy_from_slice(b"NINTENDO");
    h[0x22..0x26].copy_from_slice(&[0x00, 0x22, 0x00, 0x09]);

    let vpk_section_len = (2 + 4 + vpk.len()) as u32;
    let size_info: u32 = (1 << 1) | (1 << 5) | (vpk_section_len << 9);
    let flags: u32 = 2; // start immediately, no subtitle, GBA/Z80
    h[0x26..0x2A].copy_from_slice(&size_info.to_le_bytes());
    h[0x2A..0x2E].copy_from_slice(&flags.to_le_bytes());
    let mut hcs = 0u8;
    for &idx in &[0x0C, 0x0D, 0x10, 0x11, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D] {
        hcs ^= h[idx];
    }
    h[0x2E] = hcs;
    h[0x2F] = global_checksum(&h[0..47], &bin[CARD_HEADER_LEN..]);
    bin[..CARD_HEADER_LEN].copy_from_slice(&h);
    bin
}

#[test]
fn cleanroom_demo_card_full_e2e_pipeline_and_optical_roundtrip() {
    let gba_bin = build_cleanroom_demo_gba();
    assert_eq!(gba_bin.len() % 4, 0);

    // 1. Build retail-compatible PSAE .bin and encode to 1,872-byte .raw strip.
    let bin = build_psae_gba_card_bin(&gba_bin);
    assert_eq!(bin.len(), CARD_HEADER_LEN + SHORT_STRIP_MAX_PAYLOAD);
    let raw = bin_to_raw(&bin).expect("bin_to_raw");
    assert_eq!(raw.len(), SHORT_STRIP_RAW_LEN);

    // 2. Render printable DotCode strip at 600 DPI and verify BMP + PBM headers.
    let bmp = render_strip(&raw, 600).expect("render_strip");
    let scale = module_px(600);
    assert_eq!(scale, 4);
    assert_eq!(
        bmp.width,
        (SHORT_STRIP_PHYSICAL_BLOCKS * BLOCK_COLS + ANCHOR_SIZE + QUIET_ZONE * 2) * scale
    );
    assert_eq!(bmp.height, (STRIP_ROWS + QUIET_ZONE * 2) * scale);
    let bmp_bytes = bmp.to_bmp();
    assert_eq!(&bmp_bytes[0..2], b"BM");
    // Quiet-zone corner pixel (top-left, last row in bottom-up BMP) must be white (palette index 0).
    let bmp_row_bytes = bmp.width.div_ceil(32) * 4;
    let top_row_off = 62 + (bmp.height - 1) * bmp_row_bytes;
    assert_eq!(bmp_bytes[top_row_off] & 0x80, 0);
    let pbm_bytes = bmp.to_pbm();
    assert!(pbm_bytes.starts_with(b"P4\n"));

    // 3. Optical geometry round-trip: sample modulated dot columns back from the
    //    rendered 600 DPI bitmap and demodulate every single byte of the .raw strip!
    let mut optically_recovered_raw = Vec::with_capacity(SHORT_STRIP_RAW_LEN);
    for pb in 0..SHORT_STRIP_PHYSICAL_BLOCKS {
        let bx = QUIET_ZONE + pb * BLOCK_COLS;
        for col in 0..26 {
            let cx = (bx + 7 + col) * scale + scale / 2;
            for b_idx in 0..4 {
                let mut sym = 0u16;
                for k in 0..10 {
                    let cy = (QUIET_ZONE + b_idx * 10 + k) * scale + scale / 2;
                    sym = (sym << 1) | u16::from(bmp.get(cx, cy));
                }
                optically_recovered_raw.push(demodulate(sym).expect("valid 10-bit symbol"));
            }
        }
    }
    assert_eq!(optically_recovered_raw, raw);

    // 4. Inject simultaneous DotCode header errors (4 bytes) + RS payload errors
    //    (8 bytes across distinct codewords) and verify RS ECC repairs bit-for-bit.
    let mut damaged = optically_recovered_raw;
    damaged[0] ^= 0xFF;
    damaged[3 * PHYSICAL_BLOCK_LEN + 1] ^= 0xA5;
    damaged[7 * PHYSICAL_BLOCK_LEN] ^= 0x5A;
    damaged[11 * PHYSICAL_BLOCK_LEN + 1] ^= 0x3C;
    for i in 0..8 {
        damaged[(i * 2) * PHYSICAL_BLOCK_LEN + 10 + i] ^= 0x77;
    }

    let repaired_bin = raw_to_bin(&damaged).expect("RS ECC repair");
    assert_eq!(repaired_bin, bin);

    // 5. Parse 48-byte hardware header, extract vpk0 stream, and decompress bit-for-bit.
    let (_, payload) = parse_bin(&repaired_bin).expect("parse_bin");
    let vpk_len = u16::from_le_bytes([payload[33], payload[34]]) as usize;
    assert_eq!(&payload[35..39], &[0, 0, 0, 0]); // GBA-type NULL marker
    let vpk_slice = &payload[39..39 + vpk_len];
    assert_eq!(&vpk_slice[0..4], b"vpk0");
    let recovered_gba = vpk0_decompress(vpk_slice).expect("vpk0_decompress");
    assert_eq!(recovered_gba, gba_bin);
}
