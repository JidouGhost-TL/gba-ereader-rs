use std::hint::black_box;
use std::time::Instant;

use gba_ereader::dotcode::{bin_to_raw, build_bin, raw_to_bin};
use gba_ereader::rs_ecc::{rs_correct, rs_encode_parity};
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

    let msg = [0xA5u8; 223];
    let mut parity = [0u8; 32];
    bench("RS encode (223+32)", 20000, 255, || {
        rs_encode_parity(black_box(&msg), 32, &mut parity).unwrap();
        black_box(parity);
    });
    let mut cw = [0u8; 255];
    cw[..223].copy_from_slice(&msg);
    rs_encode_parity(&msg, 32, &mut cw[223..]).unwrap();
    bench("RS decode/correct clean (223+32)", 20000, 255, || {
        let mut c = cw;
        black_box(rs_correct(&mut c, 32).unwrap());
    });
    let mut damaged = cw;
    for i in 0..16 {
        damaged[i * 13] ^= 0xFF;
    }
    bench("RS decode/correct 16 errors (223+32)", 5000, 255, || {
        let mut c = damaged;
        black_box(rs_correct(&mut c, 32).unwrap());
    });

    let bin = build_bin(1, 0, 0, 1, payload.len() as u32, &payload[..400]);
    let raw = bin_to_raw(&bin).unwrap();
    bench("bin -> raw strip", 2000, bin.len(), || {
        black_box(bin_to_raw(black_box(&bin)).unwrap());
    });
    bench("raw -> bin strip", 2000, bin.len(), || {
        black_box(raw_to_bin(black_box(&raw)).unwrap());
    });
}
