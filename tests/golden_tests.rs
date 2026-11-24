//! Strict byte-exact golden tests (`assert_eq!`, zero tolerance).
//!
//! Fixtures are clean-room synthetic deterministic payloads only. ARM7TDMI
//! expected outputs were captured directly from the retail e-Reader (USA,
//! `PSAE`) ROM running headless in `libmgba.so.0.10`:
//! - `vpk0` decoder at `0x03004d0c` (8 cases, both methods, 13,674 bytes)
//! - GF(256) `0x187` table builder `FUN_08712ea4` (512 bytes: exp + log)
//! - `ST Decoder Ver.1.10 for CARDe` at `0x08711ad0` (clean & corrupted short
//!   1,872 B and long 2,912 B `.raw` strips -> 1,296 B & 2,064 B decoded data)

use gba_ereader::dotcode::{
    assemble_strips, bin_to_raw, build_bin, crc32, demodulate, modulate, parse_bin, raw_to_bin,
    split_into_strips, LONG_STRIP_RAW_LEN, MODULATION_TABLE, NIBBLE_MODULATION,
    SHORT_STRIP_RAW_LEN,
};
use gba_ereader::render::{module_px, render_strip};
use gba_ereader::rs_ecc::{
    gf_mul, rs_check, rs_correct, rs_correct_inverted, rs_encode_parity,
    rs_encode_parity_inverted, rs_generator, GF_EXP, GF_LOG, GF_PRIMITIVE_POLY, RS_FCR,
};
use gba_ereader::version::EreaderRevision;
use gba_ereader::vpk0::{vpk0_compress, vpk0_compress_with_method, vpk0_decompress};

#[test]
fn golden_generator_polynomials_fcr120() {
    assert_eq!(GF_PRIMITIVE_POLY, 0x187);
    assert_eq!(RS_FCR, 120);
    let mut g = [0u8; 17];
    let n = rs_generator(16, &mut g).unwrap();
    assert_eq!(n, 17);
    assert_eq!(g[0], 1);
    assert_eq!(g[16], 1);
    // FCR=120 with nsym=16 produces a self-reciprocal generator polynomial.
    for i in 0..=16 {
        assert_eq!(g[i], g[16 - i]);
    }
}

#[test]
fn golden_zero_message_parity_is_zero() {
    let mut parity = [0xAAu8; 16];
    rs_encode_parity(&[0u8; 48], 16, &mut parity).unwrap();
    assert_eq!(parity, [0u8; 16]);
    rs_encode_parity_inverted(&[0u8; 48], 16, &mut parity).unwrap();
    assert_eq!(parity, [0xFFu8; 16]);
}

#[test]
fn golden_arm_modulation_table_exact() {
    // Verified against PSAE scan-mode IWRAM 0x03006214 (4-to-5-bit nibble LUT).
    assert_eq!(
        NIBBLE_MODULATION,
        [0x00, 0x01, 0x02, 0x12, 0x04, 0x05, 0x06, 0x16, 0x08, 0x09, 0x0A, 0x14, 0x0C, 0x0D, 0x11, 0x10]
    );
    assert_eq!(MODULATION_TABLE[0x00], 0x000);
    assert_eq!(MODULATION_TABLE[0x03], 0x012);
    assert_eq!(MODULATION_TABLE[0x07], 0x016);
    assert_eq!(MODULATION_TABLE[0x0B], 0x014);
    assert_eq!(MODULATION_TABLE[0x0E], 0x011);
    assert_eq!(MODULATION_TABLE[0x0F], 0x010);
    assert_eq!(MODULATION_TABLE[0xFF], 0x210);
    for b in 0u8..=255 {
        let sym = modulate(b);
        assert_eq!(demodulate(sym).unwrap(), b);
        assert_eq!(sym & (sym >> 1) & (sym >> 2), 0);
    }
}

#[test]
fn golden_crc32_check_value() {
    assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
}

#[test]
fn golden_field_spot_values() {
    assert_eq!(gf_mul(2, 128), 0x87); // x * x^7 = x^8 mod 0x187 = 0x87
    assert_eq!(gf_mul(0, 200), 0);
    assert_eq!(gf_mul(1, 200), 200);
}

#[test]
fn golden_vpk0_header_bytes() {
    let c = vpk0_compress(b"hello hello hello").unwrap();
    assert_eq!(&c[0..9], b"vpk0\0\0\0\x11\x00");
    assert_eq!(vpk0_decompress(&c).unwrap(), b"hello hello hello".to_vec());
    let c1 = vpk0_compress_with_method(b"hello hello hello", 1).unwrap();
    assert_eq!(&c1[0..9], b"vpk0\0\0\0\x11\x01");
    assert_eq!(vpk0_decompress(&c1).unwrap(), b"hello hello hello".to_vec());
}

#[test]
fn golden_revision_detection() {
    for rev in EreaderRevision::ALL {
        let mut rom = vec![0u8; 256];
        rom[0xA0..0xAC].copy_from_slice(rev.header_title());
        rom[0xAC..0xB0].copy_from_slice(rev.game_code());
        assert_eq!(EreaderRevision::detect_from_bytes(&rom), Some(rev));
    }
}

#[test]
fn golden_full_pipeline_roundtrip_with_corruption() {
    // payload -> vpk0 -> .bin -> .raw -> corruption -> ECC repair -> .bin
    // -> vpk0 decode == original payload
    let mut payload = Vec::new();
    for i in 0..600u32 {
        payload.push((i.wrapping_mul(37).wrapping_add(11) % 251) as u8);
        if i % 50 == 0 {
            payload.extend_from_slice(b" ereader-payload ");
        }
    }
    let compressed = vpk0_compress(&payload).unwrap();
    let strips = split_into_strips(&compressed, 1, 0, 400).unwrap();
    let mut recovered_strips = Vec::new();
    for strip in &strips {
        let raw = bin_to_raw(strip).unwrap();
        let mut damaged = raw.clone();
        // Corrupt both the 24-byte DotCode header and payload RS blocks.
        damaged[0] ^= 0xFF;
        damaged[3 * 104 + 1] ^= 0x81;
        damaged[20] ^= 0x81;
        damaged[200] ^= 0x42;
        let bin = raw_to_bin(&damaged).unwrap();
        assert_eq!(&bin, strip);
        recovered_strips.push(bin);
    }
    let joined = assemble_strips(&recovered_strips).unwrap();
    assert_eq!(joined, compressed);
    assert_eq!(vpk0_decompress(&joined).unwrap(), payload);
}

#[test]
fn golden_rs_corrects_up_to_bound() {
    let mut msg = [0u8; 48];
    for (i, b) in msg.iter_mut().enumerate() {
        *b = (i as u8).wrapping_mul(17).wrapping_add(3);
    }
    let nsym = 16;
    let mut cw = [0u8; 64];
    cw[..48].copy_from_slice(&msg);
    rs_encode_parity(&msg, nsym, &mut cw[48..]).unwrap();
    assert!(rs_check(&cw, nsym));
    let orig = cw;
    for i in 0..8 {
        cw[i * 7 + 1] ^= 0x5A;
    }
    assert_eq!(rs_correct(&mut cw, nsym).unwrap(), 8);
    assert_eq!(cw, orig);

    // Inverted-parity variant matching .raw strips.
    let mut cwi = [0u8; 64];
    cwi[..48].copy_from_slice(&msg);
    rs_encode_parity_inverted(&msg, nsym, &mut cwi[48..]).unwrap();
    let orig_i = cwi;
    for i in 0..8 {
        cwi[i * 7 + 3] ^= 0xA5;
    }
    assert_eq!(rs_correct_inverted(&mut cwi, nsym).unwrap(), 8);
    assert_eq!(cwi, orig_i);
}

// ---------------------------------------------------------------------------
// libmgba ARM7TDMI goldens: GF(256) 0x187 tables (FUN_08712ea4) and
// ST Decoder Ver.1.10 for CARDe (FUN_08711ad0) from retail PSAE ROM.
// ---------------------------------------------------------------------------

#[test]
fn golden_arm_gf256_0x187_tables_byte_exact() {
    let arm_tables = include_bytes!("golden/gf256_0x187_arm.bin");
    assert_eq!(arm_tables.len(), 512);
    // FUN_08712ea4 writes exp[0..255] = alpha^i, exp[255] = 0, and log[0..256] (log[0] = 0xFF).
    assert_eq!(&GF_EXP[..255], &arm_tables[..255]);
    assert_eq!(arm_tables[255], 0);
    assert_eq!(&GF_LOG[..], &arm_tables[256..512]);
}

#[test]
fn golden_arm_st_decoder_short_strip_byte_exact() {
    let raw_clean = include_bytes!("golden/strip_short_clean.raw");
    let raw_corrupt = include_bytes!("golden/strip_short_corrupt.raw");
    let arm_decoded = include_bytes!("golden/strip_short_arm.bin");
    assert_eq!(raw_clean.len(), SHORT_STRIP_RAW_LEN);
    assert_eq!(raw_corrupt.len(), SHORT_STRIP_RAW_LEN);
    assert_eq!(arm_decoded.len(), 1296);

    let bin_clean = raw_to_bin(raw_clean).unwrap();
    let bin_corrupt = raw_to_bin(raw_corrupt).unwrap();
    assert_eq!(bin_clean, bin_corrupt);
    // Decoded payload (blocks 1..27 = 1,296 bytes) matches PSAE FUN_08711ad0 byte-for-byte.
    assert_eq!(&bin_clean[48..], &arm_decoded[..]);
    let (hdr, trimmed) = parse_bin(&bin_clean).unwrap();
    assert_eq!(hdr.payload_len, 512);
    assert_eq!(trimmed, &arm_decoded[..512]);
    // Re-encoding the .bin back to .raw produces the exact 1,872-byte strip.
    assert_eq!(bin_to_raw(&bin_clean).unwrap().as_slice(), &raw_clean[..]);
}

#[test]
fn golden_arm_st_decoder_long_strip_byte_exact() {
    let raw_clean = include_bytes!("golden/strip_long_clean.raw");
    let raw_corrupt = include_bytes!("golden/strip_long_corrupt.raw");
    let arm_decoded = include_bytes!("golden/strip_long_arm.bin");
    assert_eq!(raw_clean.len(), LONG_STRIP_RAW_LEN);
    assert_eq!(raw_corrupt.len(), LONG_STRIP_RAW_LEN);
    assert_eq!(arm_decoded.len(), 2064);

    let bin_clean = raw_to_bin(raw_clean).unwrap();
    let bin_corrupt = raw_to_bin(raw_corrupt).unwrap();
    assert_eq!(bin_clean, bin_corrupt);
    // Decoded payload (blocks 1..43 = 2,064 bytes) matches PSAE FUN_08711ad0 byte-for-byte.
    assert_eq!(&bin_clean[48..], &arm_decoded[..]);
    let (hdr, trimmed) = parse_bin(&bin_clean).unwrap();
    assert_eq!(hdr.payload_len, 1800);
    assert_eq!(trimmed, &arm_decoded[..1800]);
    // Re-encoding the .bin back to .raw produces the exact 2,912-byte strip.
    assert_eq!(bin_to_raw(&bin_clean).unwrap().as_slice(), &raw_clean[..]);
}

// ---------------------------------------------------------------------------
// libmgba ARM7TDMI goldens: .bin fixtures were captured from the real
// e-Reader (USA) ROM vpk0 decoder at 0x03004d0c running headless in
// libmgba.so.0.10 on clean-room synthetic streams (see golden/manifest.json).
// ---------------------------------------------------------------------------

const ARM_CASES: [(&[u8], &[u8]); 8] = [
    (
        include_bytes!("golden/case1_m0.vpk"),
        include_bytes!("golden/case1_m0.bin"),
    ),
    (
        include_bytes!("golden/case1_m1.vpk"),
        include_bytes!("golden/case1_m1.bin"),
    ),
    (
        include_bytes!("golden/case2_m0.vpk"),
        include_bytes!("golden/case2_m0.bin"),
    ),
    (
        include_bytes!("golden/case2_m1.vpk"),
        include_bytes!("golden/case2_m1.bin"),
    ),
    (
        include_bytes!("golden/case3_m0.vpk"),
        include_bytes!("golden/case3_m0.bin"),
    ),
    (
        include_bytes!("golden/case3_m1.vpk"),
        include_bytes!("golden/case3_m1.bin"),
    ),
    (
        include_bytes!("golden/case4_m0.vpk"),
        include_bytes!("golden/case4_m0.bin"),
    ),
    (
        include_bytes!("golden/case4_m1.vpk"),
        include_bytes!("golden/case4_m1.bin"),
    ),
];

#[test]
fn golden_arm_vpk0_decoder_byte_exact() {
    let manifest = include_str!("golden/manifest.json");
    assert!(manifest.contains("0x03004d0c"));
    assert!(manifest.contains("0x08711ad0"));
    let mut total = 0usize;
    for (vpk, expected) in ARM_CASES {
        // Strict byte-for-byte equality with the ARM decoder's output.
        assert_eq!(&vpk0_decompress(vpk).unwrap(), expected);
        assert_eq!(
            gba_ereader::vpk0::vpk0_decompressed_size(vpk).unwrap(),
            expected.len()
        );
        total += expected.len();
    }
    // 8 cases, both methods, 13,674 ARM-verified output bytes in total.
    assert_eq!(total, 804 + 804 + 5000 + 5000 + 777 + 777 + 256 + 256);
}

#[test]
fn golden_arm_vpk0_roundtrip_through_strips() {
    // ARM-verified streams also survive the .bin/.raw strip pipeline.
    for (vpk, expected) in ARM_CASES {
        let bin = build_bin(1, 0, 0, 1, vpk).unwrap();
        let raw = bin_to_raw(&bin).unwrap();
        let back = raw_to_bin(&raw).unwrap();
        assert_eq!(&back, &bin);
        let (_, payload) = parse_bin(&back).unwrap();
        assert_eq!(&vpk0_decompress(payload).unwrap(), expected);
    }
}

#[test]
fn golden_render_dpi_scaling() {
    assert_eq!(module_px(300), 2);
    assert_eq!(module_px(600), 4);
    assert_eq!(module_px(1200), 8);
    let bin = build_bin(1, 0, 0, 1, b"gold").unwrap();
    let raw = bin_to_raw(&bin).unwrap();
    let lo = render_strip(&raw, 300).unwrap();
    let hi = render_strip(&raw, 600).unwrap();
    assert_eq!(hi.width, lo.width * 2);
    assert_eq!(hi.height, lo.height * 2);
    assert!(lo.to_bmp().starts_with(b"BM"));
    assert!(lo.to_pbm().starts_with(b"P4\n"));
    assert!(lo.to_pgm().starts_with(b"P5\n"));
}
