//! Strict byte-exact golden tests (`assert_eq!`, zero tolerance).
//!
//! Fixtures are clean-room synthetic deterministic payloads only. Expected
//! values are hand-derived field/generator constants and frozen format
//! bytes, verified byte-for-byte.

use gba_ereader::dotcode::{
    assemble_strips, bin_to_raw, build_bin, crc32, demodulate, modulate, raw_to_bin,
    split_into_strips, MODULATION_TABLE,
};
use gba_ereader::render::{module_px, render_strip};
use gba_ereader::rs_ecc::{gf_mul, rs_check, rs_correct, rs_encode_parity, rs_generator};
use gba_ereader::version::EreaderRevision;
use gba_ereader::vpk0::{vpk0_compress, vpk0_compress_with_method, vpk0_decompress};

#[test]
fn golden_generator_polynomials() {
    let mut g = [0u8; 8];
    let n = rs_generator(1, &mut g).unwrap();
    assert_eq!(&g[..n], &[1, 1]);
    let n = rs_generator(2, &mut g).unwrap();
    assert_eq!(&g[..n], &[1, 3, 2]);
}

#[test]
fn golden_zero_message_parity_is_zero() {
    let mut parity = [0xAAu8; 16];
    rs_encode_parity(&[0u8; 32], 16, &mut parity).unwrap();
    assert_eq!(parity, [0u8; 16]);
}

#[test]
fn golden_modulation_prefix() {
    // Smallest 10-bit symbols with popcount 5, ascending.
    assert_eq!(MODULATION_TABLE[0], 0x01F);
    assert_eq!(MODULATION_TABLE[1], 0x02F);
    assert_eq!(MODULATION_TABLE[2], 0x037);
    assert_eq!(MODULATION_TABLE[3], 0x03B);
    assert_eq!(MODULATION_TABLE[4], 0x03D);
    assert_eq!(MODULATION_TABLE[5], 0x03E);
    for b in 0u8..=255 {
        assert_eq!(demodulate(modulate(b)).unwrap(), b);
    }
}

#[test]
fn golden_crc32_check_value() {
    assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
}

#[test]
fn golden_field_spot_values() {
    assert_eq!(gf_mul(2, 128), 29); // x * x^7 = x^8 mod 0x11D = 0x1D
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
    let msg = b"golden rs block payload";
    let nsym = 16;
    let mut cw = vec![0u8; msg.len() + nsym];
    cw[..msg.len()].copy_from_slice(msg);
    rs_encode_parity(msg, nsym, &mut cw[msg.len()..]).unwrap();
    assert!(rs_check(&cw, nsym));
    let orig = cw.clone();
    for i in 0..8 {
        cw[i * 3] ^= 0x5A;
    }
    assert_eq!(rs_correct(&mut cw, nsym).unwrap(), 8);
    assert_eq!(cw, orig);
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
        let bin = build_bin(1, 0, 0, 1, expected.len() as u32, vpk);
        let raw = bin_to_raw(&bin).unwrap();
        let back = raw_to_bin(&raw).unwrap();
        assert_eq!(&back, &bin);
        let (_, payload) = gba_ereader::dotcode::parse_bin(&back).unwrap();
        assert_eq!(&vpk0_decompress(payload).unwrap(), expected);
    }
}

#[test]
fn golden_render_dpi_scaling() {
    assert_eq!(module_px(300), 2);
    assert_eq!(module_px(600), 4);
    assert_eq!(module_px(1200), 8);
    let bin = build_bin(1, 0, 0, 1, 4, b"gold");
    let raw = bin_to_raw(&bin).unwrap();
    let lo = render_strip(&raw, 300).unwrap();
    let hi = render_strip(&raw, 600).unwrap();
    assert_eq!(hi.width, lo.width * 2);
    assert_eq!(hi.height, lo.height * 2);
    assert!(lo.to_bmp().starts_with(b"BM"));
    assert!(lo.to_pbm().starts_with(b"P4\n"));
    assert!(lo.to_pgm().starts_with(b"P5\n"));
}
