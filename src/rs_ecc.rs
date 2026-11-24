//! Reed-Solomon ECC over GF(256) matching Nintendo GBA e-Reader hardware.
//!
//! Verified against `ST Decoder Ver.1.10 for CARDe` in all three retail
//! e-Reader ROMs (`PEAJ`, `PSAE`, `PSAJ`):
//!
//! - **Field**: GF(2^8) with primitive polynomial `x^8 + x^7 + x^2 + x + 1`
//!   (`0x187`) and primitive element `2` (verified against `PSAE` `FUN_08712ea4`
//!   and the primitive polynomial table at `0x087141b0`).
//! - **First Consecutive Root (FCR)**: `120` (`0x78`), so the generator
//!   polynomial is `g(x) = (x - a^120)(x - a^121)...(x - a^(120 + nsym - 1))`
//!   (verified against `PSAE` `FUN_08712f60`).
//! - **Codewords**: Systematic (message bytes first, followed by `nsym` parity
//!   bytes). In e-Reader `.raw` DotCode strips, parity bytes are stored
//!   bitwise-inverted (`^ 0xFF`), both for the 24-byte `RS(24, 8)` DotCode
//!   header and the interleaved `RS(64, 48)` data blocks (verified against
//!   `PSAE` `FUN_08712d30` and `FUN_0871237c`).
//! - **Correction bound**: A codeword with `nsym` parity bytes corrects up to
//!   `nsym / 2` unknown byte errors (up to `t = 8` errors for `nsym = 16`).

use crate::error::{Error, Result};

/// Primitive polynomial for GF(256): `0x187` (`x^8 + x^7 + x^2 + x + 1`).
pub const GF_PRIMITIVE_POLY: u16 = 0x187;
/// Primitive polynomial for GF(32) address blocks: `0x25` (`x^5 + x^2 + 1`).
pub const GF32_PRIMITIVE_POLY: u8 = 0x25;
/// Primitive element of GF(256).
pub const GF_PRIMITIVE_ELEMENT: u8 = 2;
/// First consecutive root (FCR) of the e-Reader RS generator polynomial: `120` (`0x78`).
pub const RS_FCR: usize = 120;
/// Maximum supported parity length.
pub const MAX_NSYM: usize = 64;

/// Exponent (antilog) table, duplicated to 512 entries for wrap-free multiply.
pub const GF_EXP: [u8; 512] = build_exp();
/// Logarithm table (`GF_LOG[0]` is `0xFF`, matching `FUN_08712ea4`).
pub const GF_LOG: [u8; 256] = build_log();

const fn build_exp() -> [u8; 512] {
    let mut exp = [0u8; 512];
    let mut x: u16 = 1;
    let mut i = 0;
    while i < 255 {
        exp[i] = x as u8;
        x <<= 1;
        if x & 0x100 != 0 {
            x ^= GF_PRIMITIVE_POLY;
        }
        i += 1;
    }
    let mut j = 255;
    while j < 512 {
        exp[j] = exp[j - 255];
        j += 1;
    }
    exp
}

const fn build_log() -> [u8; 256] {
    let mut log = [0xFFu8; 256];
    let mut i = 0;
    while i < 255 {
        log[GF_EXP[i] as usize] = i as u8;
        i += 1;
    }
    log
}

/// Multiply two GF(256) field elements.
#[must_use]
pub fn gf_mul(x: u8, y: u8) -> u8 {
    if x == 0 || y == 0 {
        0
    } else {
        GF_EXP[GF_LOG[x as usize] as usize + GF_LOG[y as usize] as usize]
    }
}

/// Divide `x` by `y` (`y` must be nonzero; returns 0 when `x` is 0).
#[must_use]
pub fn gf_div(x: u8, y: u8) -> u8 {
    if x == 0 {
        0
    } else {
        GF_EXP[GF_LOG[x as usize] as usize + 255 - GF_LOG[y as usize] as usize]
    }
}

/// Raise `x` to the power `n`.
#[must_use]
pub fn gf_pow(x: u8, n: i32) -> u8 {
    if x == 0 {
        return 0;
    }
    let mut e = (GF_LOG[x as usize] as i32 * n) % 255;
    if e < 0 {
        e += 255;
    }
    GF_EXP[e as usize]
}

/// Multiplicative inverse of `x` (0 maps to 0 by convention).
#[must_use]
pub fn gf_inverse(x: u8) -> u8 {
    if x == 0 {
        0
    } else {
        GF_EXP[255 - GF_LOG[x as usize] as usize]
    }
}

/// Evaluate a polynomial (coefficients highest-degree first) at `x`.
#[must_use]
pub fn gf_poly_eval(poly: &[u8], x: u8) -> u8 {
    let mut y = 0u8;
    for &c in poly {
        y = gf_mul(y, x) ^ c;
    }
    y
}

/// Compute the e-Reader RS generator polynomial of degree `nsym` into `out`.
///
/// Roots are `a^(RS_FCR + i)` for `i = 0..nsym` (`RS_FCR = 120`).
/// `out` must hold at least `nsym + 1` bytes. Returns the length written.
pub fn rs_generator(nsym: usize, out: &mut [u8]) -> Result<usize> {
    if nsym == 0 || nsym > MAX_NSYM || out.len() < nsym + 1 {
        return Err(Error::InvalidSize("generator output too small"));
    }
    let mut g = [0u8; MAX_NSYM + 1];
    let mut glen = 1usize;
    g[0] = 1;
    for i in 0..nsym {
        let a = GF_EXP[(RS_FCR + i) % 255];
        let mut ng = [0u8; MAX_NSYM + 1];
        for j in 0..glen {
            ng[j] ^= g[j];
            ng[j + 1] ^= gf_mul(g[j], a);
        }
        g = ng;
        glen += 1;
    }
    out[..glen].copy_from_slice(&g[..glen]);
    Ok(glen)
}

/// Compute `nsym` systematic parity bytes for `msg` into `parity`.
pub fn rs_encode_parity(msg: &[u8], nsym: usize, parity: &mut [u8]) -> Result<()> {
    if nsym == 0 || nsym > MAX_NSYM || parity.len() < nsym {
        return Err(Error::InvalidSize("parity output too small"));
    }
    let mut gen = [0u8; MAX_NSYM + 1];
    let glen = rs_generator(nsym, &mut gen)?;
    let gen = &gen[..glen];
    parity[..nsym].fill(0);
    for &b in msg {
        let coef = b ^ parity[0];
        parity.copy_within(1..nsym, 0);
        parity[nsym - 1] = 0;
        if coef != 0 {
            for j in 0..nsym {
                parity[j] ^= gf_mul(gen[j + 1], coef);
            }
        }
    }
    Ok(())
}

/// Compute `nsym` systematic parity bytes and bitwise-invert (`^ 0xFF`) them,
/// matching the e-Reader DotCode `.raw` strip representation.
pub fn rs_encode_parity_inverted(msg: &[u8], nsym: usize, parity: &mut [u8]) -> Result<()> {
    rs_encode_parity(msg, nsym, parity)?;
    for p in parity[..nsym].iter_mut() {
        *p ^= 0xFF;
    }
    Ok(())
}

/// Compute syndromes of `codeword` (message + non-inverted parity) into `out`.
///
/// Returns `true` when all syndromes are zero (valid codeword).
pub fn rs_syndromes(codeword: &[u8], nsym: usize, out: &mut [u8]) -> Result<bool> {
    if nsym == 0 || nsym > MAX_NSYM || out.len() < nsym {
        return Err(Error::InvalidSize("syndrome output too small"));
    }
    let mut all_zero = true;
    for (i, slot) in out[..nsym].iter_mut().enumerate() {
        let root = GF_EXP[(RS_FCR + i) % 255];
        let s = gf_poly_eval(codeword, root);
        *slot = s;
        if s != 0 {
            all_zero = false;
        }
    }
    Ok(all_zero)
}

/// Check whether `codeword` is a valid RS codeword with `nsym` parity bytes.
#[must_use]
pub fn rs_check(codeword: &[u8], nsym: usize) -> bool {
    if nsym == 0 || nsym > MAX_NSYM || codeword.len() <= nsym {
        return false;
    }
    for i in 0..nsym {
        let root = GF_EXP[(RS_FCR + i) % 255];
        if gf_poly_eval(codeword, root) != 0 {
            return false;
        }
    }
    true
}

// Polynomial helpers use ascending-power coefficient order below this point.

fn poly_mul_asc(p: &[u8], q: &[u8], out: &mut [u8]) -> usize {
    let n = p.len() + q.len() - 1;
    out[..n].fill(0);
    for (i, &a) in p.iter().enumerate() {
        if a != 0 {
            for (j, &b) in q.iter().enumerate() {
                out[i + j] ^= gf_mul(a, b);
            }
        }
    }
    n
}

fn poly_eval_asc(p: &[u8], x: u8) -> u8 {
    let mut y = 0u8;
    for &c in p.iter().rev() {
        y = gf_mul(y, x) ^ c;
    }
    y
}

/// Correct `codeword` (message + `nsym` non-inverted parity) in place.
///
/// Returns the number of corrected byte positions on success.
pub fn rs_correct(codeword: &mut [u8], nsym: usize) -> Result<usize> {
    let nmess = codeword.len();
    if nsym == 0 || nsym > MAX_NSYM || nmess <= nsym || nmess > 255 {
        return Err(Error::InvalidSize("bad codeword length"));
    }
    let mut synd = [0u8; MAX_NSYM];
    if rs_syndromes(codeword, nsym, &mut synd)? {
        return Ok(0);
    }
    let synd = &synd[..nsym];

    // Berlekamp-Massey: error locator in ascending-power coefficients.
    let mut err_loc = [0u8; MAX_NSYM + 1];
    let mut old_loc = [0u8; MAX_NSYM + 1];
    err_loc[0] = 1;
    old_loc[0] = 1;
    let mut err_len = 1usize;
    let mut old_len = 1usize;
    for i in 0..nsym {
        let mut delta = synd[i];
        for j in 1..err_len {
            delta ^= gf_mul(err_loc[j], synd[i - j]);
        }
        old_loc.copy_within(0..old_len, 1);
        old_loc[0] = 0;
        old_len += 1;
        if delta != 0 {
            if old_len > err_len {
                let mut new_loc = [0u8; MAX_NSYM + 1];
                for k in 0..old_len {
                    new_loc[k] = gf_mul(old_loc[k], delta);
                }
                let inv = gf_inverse(delta);
                for k in 0..err_len {
                    old_loc[k] = gf_mul(err_loc[k], inv);
                }
                old_len = err_len;
                err_loc = new_loc;
            }
            for k in 0..old_len {
                err_loc[k] ^= gf_mul(old_loc[k], delta);
            }
            err_len = poly_trim(&err_loc);
        }
    }
    let nerrors = err_len - 1;
    if nerrors * 2 > nsym {
        return Err(Error::RsUncorrectable);
    }

    // Chien search: position p (from start) is in error when Lambda(a^-(nmess-1-p)) == 0.
    let mut err_pos = [0usize; MAX_NSYM];
    let mut nfound = 0usize;
    for p in 0..nmess {
        let expo = (nmess - 1 - p) as i32;
        if poly_eval_asc(&err_loc[..err_len], gf_pow(2, -expo)) == 0 && nfound < MAX_NSYM {
            err_pos[nfound] = p;
            nfound += 1;
        }
    }
    if nfound != nerrors {
        return Err(Error::RsUncorrectable);
    }

    // Error evaluator: Omega(z) = (S(z) * Lambda(z)) mod z^nsym.
    let mut omega_full = [0u8; 2 * MAX_NSYM + 2];
    let olen = poly_mul_asc(synd, &err_loc[..err_len], &mut omega_full);
    let omega = &omega_full[..olen.min(nsym)];

    // Forney's formula with FCR = RS_FCR (120):
    // e_i = Omega(Xi^-1) / (Xi^RS_FCR * prod_{j != i}(1 + Xj/Xi)).
    for idx in 0..nfound {
        let p = err_pos[idx];
        let coef = (nmess - 1 - p) as i32;
        let xi = gf_pow(2, coef);
        let xi_inv = gf_inverse(xi);
        let mut denom = 1u8;
        for (jdx, &q) in err_pos[..nfound].iter().enumerate() {
            if jdx != idx {
                let xj = gf_pow(2, (nmess - 1 - q) as i32);
                denom = gf_mul(denom, 1 ^ gf_div(xj, xi));
            }
        }
        if denom == 0 {
            return Err(Error::RsUncorrectable);
        }
        let y = poly_eval_asc(omega, xi_inv);
        let scale = gf_pow(xi, RS_FCR as i32);
        let magnitude = gf_div(y, gf_mul(scale, denom));
        codeword[p] ^= magnitude;
    }

    let mut check = [0u8; MAX_NSYM];
    if !rs_syndromes(codeword, nsym, &mut check)? {
        return Err(Error::RsUncorrectable);
    }
    Ok(nfound)
}

/// Correct `codeword` whose trailing `nsym` parity bytes are bitwise-inverted
/// (`^ 0xFF`), as stored in e-Reader `.raw` strips.
///
/// The parity bytes remain bitwise-inverted on return.
pub fn rs_correct_inverted(codeword: &mut [u8], nsym: usize) -> Result<usize> {
    let nmess = codeword.len();
    if nsym == 0 || nsym > MAX_NSYM || nmess <= nsym || nmess > 255 {
        return Err(Error::InvalidSize("bad codeword length"));
    }
    let data_len = nmess - nsym;
    for p in codeword[data_len..].iter_mut() {
        *p ^= 0xFF;
    }
    let res = rs_correct(codeword, nsym);
    for p in codeword[data_len..].iter_mut() {
        *p ^= 0xFF;
    }
    res
}

fn poly_trim(p: &[u8]) -> usize {
    let mut n = p.len();
    while n > 1 && p[n - 1] == 0 {
        n -= 1;
    }
    n
}

/// Encode `msg` plus non-inverted parity into a newly allocated codeword.
#[cfg(feature = "alloc")]
pub fn rs_encode(msg: &[u8], nsym: usize) -> Result<alloc::vec::Vec<u8>> {
    let mut out = alloc::vec![0u8; msg.len() + nsym];
    out[..msg.len()].copy_from_slice(msg);
    rs_encode_parity(msg, nsym, &mut out[msg.len()..])?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn field_basics() {
        assert_eq!(GF_EXP[0], 1);
        assert_eq!(GF_EXP[1], 2);
        assert_eq!(GF_EXP[8], 0x87); // x^8 mod 0x187 = 0x87
        assert_eq!(GF_LOG[0], 0xFF);
        assert_eq!(gf_mul(0, 5), 0);
        assert_eq!(gf_mul(1, 5), 5);
        for a in 1u8..=255 {
            assert_eq!(gf_mul(a, gf_inverse(a)), 1);
        }
    }

    #[test]
    fn generator_degree_and_symmetry() {
        let mut g = [0u8; 17];
        let n = rs_generator(16, &mut g).unwrap();
        assert_eq!(n, 17);
        assert_eq!(g[0], 1);
        // FCR=120 with nsym=16 gives a self-reciprocal generator polynomial!
        for i in 0..=16 {
            assert_eq!(g[i], g[16 - i]);
        }
    }

    #[test]
    fn encode_then_check_rs_64_48() {
        let mut msg = [0u8; 48];
        for (i, b) in msg.iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(37).wrapping_add(13);
        }
        let mut cw = [0u8; 64];
        cw[..48].copy_from_slice(&msg);
        rs_encode_parity(&msg, 16, &mut cw[48..]).unwrap();
        assert!(rs_check(&cw, 16));
    }

    #[test]
    fn correct_rs_64_48_up_to_8_errors() {
        let mut msg = [0u8; 48];
        for (i, b) in msg.iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(29).wrapping_add(5);
        }
        let mut cw = [0u8; 64];
        cw[..48].copy_from_slice(&msg);
        rs_encode_parity_inverted(&msg, 16, &mut cw[48..]).unwrap();
        let orig = cw;
        // Corrupt 8 positions (max t = 8), including both data and parity.
        for i in 0..8 {
            cw[i * 7 + 2] ^= 0xA5u8.wrapping_add(i as u8);
        }
        assert_eq!(rs_correct_inverted(&mut cw, 16).unwrap(), 8);
        assert_eq!(cw, orig);
    }

    #[test]
    fn correct_rs_24_8_header_up_to_8_errors() {
        let hdr = [0x00, 0x01, 0x00, 0x01, 0x40, 0x10, 0x00, 0x2C];
        let mut cw = [0u8; 24];
        cw[..8].copy_from_slice(&hdr);
        rs_encode_parity_inverted(&hdr, 16, &mut cw[8..]).unwrap();
        let orig = cw;
        for i in 0..8 {
            cw[i * 3] ^= 0x5A;
        }
        assert_eq!(rs_correct_inverted(&mut cw, 16).unwrap(), 8);
        assert_eq!(cw, orig);
    }
}
