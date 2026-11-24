use std::hint::black_box;
use std::time::Instant;

use gba_ereader::dotcode::{bin_to_raw, build_bin, raw_to_bin};
use gba_ereader::rs_ecc::{rs_correct_inverted, rs_encode_parity_inverted};
use gba_ereader::vpk0::{vpk0_compress, vpk0_decompress};

fn bench(name: &str, iters: usize, bytes: usize, mut f: impl FnMut()) {
    for _ in 0..20 {
        f();
    }
    let start = Instant::now();
    for _ in 0..iters {
        f();
    }
    let elapsed = start.elapsed();
    let ns = elapsed.as_nanos() as f64 / iters as f64;
    let mbps = (bytes as f64 * iters as f64 / elapsed.as_secs_f64()) / 1_000_000.0;
    println!("{name:<44} {ns:10.1} ns/iter ({mbps:8.2} MB/s)");
}

fn main() {
    println!("=== gba-ereader Performance Benchmarks ===");
    let mut payload = Vec::new();
    for i in 0..8192u32 {
        payload.push((i.wrapping_mul(31).wrapping_add(7) % 253) as u8);
        if i % 64 == 0 {
            payload.extend_from_slice(b" ereader ereader ");
        }
    }
    let compressed = vpk0_compress(&payload).unwrap();
    bench("vpk0 decode (8 KiB payload)", 2000, payload.len(), || {
        black_box(vpk0_decompress(black_box(&compressed)).unwrap());
    });
    bench("vpk0 encode (8 KiB payload)", 200, payload.len(), || {
        black_box(vpk0_compress(black_box(&payload)).unwrap());
    });

    let msg = [0xA5u8; 48];
    let mut parity = [0u8; 16];
    bench("RS(64,48) encode (GF 0x187, FCR=120)", 50000, 64, || {
        rs_encode_parity_inverted(black_box(&msg), 16, &mut parity).unwrap();
        black_box(parity);
    });
    let mut cw = [0u8; 64];
    cw[..48].copy_from_slice(&msg);
    rs_encode_parity_inverted(&msg, 16, &mut cw[48..]).unwrap();
    bench("RS(64,48) decode clean", 50000, 64, || {
        let mut c = cw;
        black_box(rs_correct_inverted(&mut c, 16).unwrap());
    });
    let mut damaged = cw;
    for i in 0..8 {
        damaged[i * 7 + 1] ^= 0xFF;
    }
    bench("RS(64,48) decode 8 errors (max t=8)", 10000, 64, || {
        let mut c = damaged;
        black_box(rs_correct_inverted(&mut c, 16).unwrap());
    });

    let bin_s = build_bin(1, 0, 0, 1, &payload[..512]).unwrap();
    let raw_s = bin_to_raw(&bin_s).unwrap();
    bench("bin -> raw short strip (1872 B)", 2000, raw_s.len(), || {
        black_box(bin_to_raw(black_box(&bin_s)).unwrap());
    });
    bench("raw -> bin short strip (1872 B)", 2000, raw_s.len(), || {
        black_box(raw_to_bin(black_box(&raw_s)).unwrap());
    });

    let bin_l = build_bin(1, 0, 0, 1, &payload[..1800]).unwrap();
    let raw_l = bin_to_raw(&bin_l).unwrap();
    bench("bin -> raw long strip (2912 B)", 2000, raw_l.len(), || {
        black_box(bin_to_raw(black_box(&bin_l)).unwrap());
    });
    bench("raw -> bin long strip (2912 B)", 2000, raw_l.len(), || {
        black_box(raw_to_bin(black_box(&raw_l)).unwrap());
    });
}
