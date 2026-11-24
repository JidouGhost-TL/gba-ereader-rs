//! Reed-Solomon ECC over GF(256) with primitive polynomial `0x11D`.
//!
//! The field is GF(2^8) defined by `x^8 + x^4 + x^3 + x^2 + 1` (`0x11D`) with
//! primitive element `2`. Codewords are systematic: message bytes first,
//! followed by `nsym` parity bytes. The generator polynomial is
//! `g(x) = (x - a^0)(x - a^1)...(x - a^(nsym-1))`. A codeword with `nsym`
//! parity bytes corrects up to `nsym / 2` unknown errors, or `nsym` erasures
//! (positions known), matching the e-Reader dot-code ECC bounds.

use crate::error::{Error, Result};

/// Primitive polynomial for GF(256): `0x11D`.
pub const GF_PRIMITIVE_POLY: u16 = 0x11D;
/// Primitive element of the field.
pub const GF_PRIMITIVE_ELEMENT: u8 = 2;
/// Maximum supported parity length.
pub const MAX_NSYM: usize = 64;

/// Exponent (antilog) table, duplicated to 512 entries for wrap-free multiply.
pub const GF_EXP: [u8; 512] = build_exp();
/// Logarithm table.
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
    let mut log = [0u8; 256];
    let mut i = 0;
    while i < 255 {
        log[GF_EXP[i] as usize] = i as u8;
        i += 1;
    }
    log
}

/// Multiply two field elements.
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

/// Compute the generator polynomial of degree `nsym` into `out`.
///
/// `out` must hold at least `nsym + 1` bytes. Returns the length written.
pub fn rs_generator(nsym: usize, out: &mut [u8]) -> Result<usize> {
    if nsym == 0 || nsym > MAX_NSYM || out.len() < nsym + 1 {
        return Err(Error::InvalidSize("generator output too small"));
    }
    // g(x) built in highest-degree-first form, starting from [1].
    let mut g = [0u8; MAX_NSYM + 1];
    let mut glen = 1usize;
    g[0] = 1;
    for &a in GF_EXP.iter().take(nsym) {
        // Multiply g by (x - a^i) == (x + a^i) in characteristic 2.
        // Multiply by (x + a^i) with highest-degree-first coefficients.
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
        // Shift parity left by one.
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

/// Compute syndromes of `codeword` (message + parity) into `out`.
///
/// Returns `true` when all syndromes are zero (valid codeword).
pub fn rs_syndromes(codeword: &[u8], nsym: usize, out: &mut [u8]) -> Result<bool> {
    if nsym == 0 || nsym > MAX_NSYM || out.len() < nsym {
        return Err(Error::InvalidSize("syndrome output too small"));
    }
    let mut all_zero = true;
    for i in 0..nsym {
        let s = gf_poly_eval(codeword, GF_EXP[i]);
        out[i] = s;
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
    for &root in GF_EXP.iter().take(nsym) {
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

/// Correct `codeword` (message + `nsym` parity) in place.
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
        // old_loc *= x (shift ascending coefficients up by one power).
        old_loc.copy_within(0..old_len, 1);
        old_loc[0] = 0;
        old_len += 1;
        if delta != 0 {
            if old_len > err_len {
                // Swap roles with scaling.
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
            // err_loc += old_loc * delta
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

    // Chien search: position p (from start) is in error when the locator
    // vanishes at a^-(nmess-1-p).
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

    // Error evaluator: Omega(x) = (Syndrome(x) * Lambda(x)) mod x^nsym,
    // with ascending-power syndrome polynomial.
    let mut omega_full = [0u8; 2 * MAX_NSYM + 2];
    let olen = poly_mul_asc(synd, &err_loc[..err_len], &mut omega_full);
    let omega = &omega_full[..olen.min(nsym)];

    // Forney's formula per error position.
    for idx in 0..nfound {
        let p = err_pos[idx];
        let coef = nmess - 1 - p;
        let xi = gf_pow(2, coef as i32);
        let xi_inv = gf_inverse(xi);
        // Derivative-like denominator: product over other positions.
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
        // Lambda'(Xi^-1) = Xi * prod(1 + Xj/Xi), and the Forney numerator
        // Xi * Omega(Xi^-1) shares that Xi factor, leaving y / denom.
        let _ = xi;
        let magnitude = gf_div(y, denom);
        codeword[p] ^= magnitude;
    }

    // Verify correction.
    let mut check = [0u8; MAX_NSYM];
    if !rs_syndromes(codeword, nsym, &mut check)? {
        return Err(Error::RsUncorrectable);
    }
    Ok(nfound)
}

fn poly_trim(p: &[u8]) -> usize {
    let mut n = p.len();
    while n > 1 && p[n - 1] == 0 {
        n -= 1;
    }
    n
}

/// Encode `msg` plus parity into `out` (message first, parity appended).
///
/// Returns the total codeword length.
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
        assert_eq!(gf_mul(0, 5), 0);
        assert_eq!(gf_mul(1, 5), 5);
        for a in 1u8..=255 {
            assert_eq!(gf_mul(a, gf_inverse(a)), 1);
        }
    }

    #[test]
    fn generator_degree() {
        let mut g = [0u8; 33];
        let n = rs_generator(32, &mut g).unwrap();
        assert_eq!(n, 33);
        assert_eq!(g[0], 1);
    }

    #[test]
    fn encode_then_check() {
        let msg = b"hello e-reader world";
        let mut cw = [0u8; 64];
        cw[..msg.len()].copy_from_slice(msg);
        rs_encode_parity(msg, 16, &mut cw[msg.len()..msg.len() + 16]).unwrap();
        assert!(rs_check(&cw[..msg.len() + 16], 16));
    }

    #[test]
    fn correct_single_and_multi_errors() {
        let msg = b"deterministic payload 0123456789";
        let nsym = 16;
        let mut cw = [0u8; 80];
        let n = msg.len() + nsym;
        cw[..msg.len()].copy_from_slice(msg);
        rs_encode_parity(msg, nsym, &mut cw[msg.len()..n]).unwrap();
        let orig = cw;
        // 1 error
        cw[3] ^= 0x55;
        assert_eq!(rs_correct(&mut cw[..n], nsym).unwrap(), 1);
        assert_eq!(&cw[..n], &orig[..n]);
        // 8 errors (t = nsym/2)
        cw = orig;
        for i in 0..8 {
            cw[i * 5] ^= 0xA5u8.wrapping_add(i as u8);
        }
        assert_eq!(rs_correct(&mut cw[..n], nsym).unwrap(), 8);
        assert_eq!(&cw[..n], &orig[..n]);
    }

    #[test]
    fn long_block_roundtrip() {
        let mut msg = [0u8; 223];
        for (i, b) in msg.iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(31).wrapping_add(7);
        }
        let mut cw = [0u8; 255];
        cw[..223].copy_from_slice(&msg);
        rs_encode_parity(&msg, 32, &mut cw[223..]).unwrap();
        assert!(rs_check(&cw, 32));
        for i in 0..16 {
            cw[i * 13] ^= 0xFF;
        }
        assert_eq!(rs_correct(&mut cw, 32).unwrap(), 16);
        assert_eq!(&cw[..223], &msg[..]);
    }
}
