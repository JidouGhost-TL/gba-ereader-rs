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

const fn gf_mul_const(x: u8, y: u8) -> u8 {
    if x == 0 || y == 0 {
        0
    } else {
        GF_EXP[GF_LOG[x as usize] as usize + GF_LOG[y as usize] as usize]
    }
}

/// Precomputed `nsym = 16` generator polynomial (`g[0..=16]`).
const RS_GEN_16: [u8; 17] = build_gen_16();

const fn build_gen_16() -> [u8; 17] {
    let mut g = [0u8; 17];
    g[0] = 1;
    let mut i = 0;
    while i < 16 {
        let a = GF_EXP[(RS_FCR + i) % 255];
        let mut ng = [0u8; 17];
        let mut j = 0;
        while j <= i {
            ng[j] ^= g[j];
            ng[j + 1] ^= gf_mul_const(g[j], a);
            j += 1;
        }
        g = ng;
        i += 1;
    }
    g
}

/// Precomputed LFSR feedback table for `nsym = 16`:
/// `RS_GEN16_MUL[coef][j] = gf_mul(coef, RS_GEN_16[j + 1])`.
const RS_GEN16_MUL: [[u8; 16]; 256] = build_gen16_mul();

const fn build_gen16_mul() -> [[u8; 16]; 256] {
    let mut tbl = [[0u8; 16]; 256];
    let mut c = 1usize;
    while c < 256 {
        let mut j = 0usize;
        while j < 16 {
            tbl[c][j] = gf_mul_const(c as u8, RS_GEN_16[j + 1]);
            j += 1;
        }
        c += 1;
    }
    tbl
}

/// Precomputed Horner multiplication table for the 16 syndrome roots:
/// `RS_ROOT16_MUL[i][y] = gf_mul(y, alpha^(RS_FCR + i))`.
const RS_ROOT16_MUL: [[u8; 256]; 16] = build_root16_mul();

const fn build_root16_mul() -> [[u8; 256]; 16] {
    let mut tbl = [[0u8; 256]; 16];
    let mut i = 0usize;
    while i < 16 {
        let root = GF_EXP[(RS_FCR + i) % 255];
        let mut y = 1usize;
        while y < 256 {
            tbl[i][y] = gf_mul_const(y as u8, root);
            y += 1;
        }
        i += 1;
    }
    tbl
}

/// Multiply two GF(256) field elements.
#[must_use]
#[inline(always)]
pub fn gf_mul(x: u8, y: u8) -> u8 {
    if x == 0 || y == 0 {
        0
    } else {
        GF_EXP[GF_LOG[x as usize] as usize + GF_LOG[y as usize] as usize]
    }
}

/// Divide `x` by `y` (`y` must be nonzero; returns 0 when `x` is 0).
#[must_use]
#[inline(always)]
pub fn gf_div(x: u8, y: u8) -> u8 {
    if x == 0 {
        0
    } else {
        GF_EXP[GF_LOG[x as usize] as usize + 255 - GF_LOG[y as usize] as usize]
    }
}

/// Raise `x` to the power `n`.
#[must_use]
#[inline]
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
#[inline(always)]
pub fn gf_inverse(x: u8) -> u8 {
    if x == 0 {
        0
    } else {
        GF_EXP[255 - GF_LOG[x as usize] as usize]
    }
}

/// Evaluate a polynomial (coefficients highest-degree first) at `x`.
#[must_use]
#[inline]
pub fn gf_poly_eval(poly: &[u8], x: u8) -> u8 {
    if x == 0 {
        return poly.last().copied().unwrap_or(0);
    }
    let log_x = GF_LOG[x as usize] as usize;
    let mut y = 0u8;
    for &c in poly {
        y = if y == 0 {
            c
        } else {
            GF_EXP[GF_LOG[y as usize] as usize + log_x] ^ c
        };
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
    if nsym == 16 {
        out[..17].copy_from_slice(&RS_GEN_16);
        return Ok(17);
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
    if nsym == 16 {
        let mut p = [0u8; 16];
        for &b in msg {
            let coef = b ^ p[0];
            let row = &RS_GEN16_MUL[coef as usize];
            p[0] = p[1] ^ row[0];
            p[1] = p[2] ^ row[1];
            p[2] = p[3] ^ row[2];
            p[3] = p[4] ^ row[3];
            p[4] = p[5] ^ row[4];
            p[5] = p[6] ^ row[5];
            p[6] = p[7] ^ row[6];
            p[7] = p[8] ^ row[7];
            p[8] = p[9] ^ row[8];
            p[9] = p[10] ^ row[9];
            p[10] = p[11] ^ row[10];
            p[11] = p[12] ^ row[11];
            p[12] = p[13] ^ row[12];
            p[13] = p[14] ^ row[13];
            p[14] = p[15] ^ row[14];
            p[15] = row[15];
        }
        parity[..16].copy_from_slice(&p);
        return Ok(());
    }
    let mut gen = [0u8; MAX_NSYM + 1];
    let glen = rs_generator(nsym, &mut gen)?;
    let mut gen_log = [0u8; MAX_NSYM];
    for j in 0..nsym {
        gen_log[j] = GF_LOG[gen[j + 1] as usize];
    }
    let gen = &gen[..glen];
    let p = &mut parity[..nsym];
    p.fill(0);
    for &b in msg {
        let coef = b ^ p[0];
        if coef != 0 {
            let log_c = GF_LOG[coef as usize] as usize;
            for j in 0..nsym - 1 {
                let g = gen[j + 1];
                p[j] = p[j + 1] ^ if g == 0 { 0 } else { GF_EXP[gen_log[j] as usize + log_c] };
            }
            let g_last = gen[nsym];
            p[nsym - 1] = if g_last == 0 {
                0
            } else {
                GF_EXP[gen_log[nsym - 1] as usize + log_c]
            };
        } else {
            p.copy_within(1..nsym, 0);
            p[nsym - 1] = 0;
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

/// Fast 16-syndrome evaluation using compile-time Horner root tables.
#[inline]
fn rs_syndromes_16(codeword: &[u8], out: &mut [u8]) -> bool {
    let mut s = [0u8; 16];
    let t = &RS_ROOT16_MUL;
    for &c in codeword {
        s[0] = t[0][s[0] as usize] ^ c;
        s[1] = t[1][s[1] as usize] ^ c;
        s[2] = t[2][s[2] as usize] ^ c;
        s[3] = t[3][s[3] as usize] ^ c;
        s[4] = t[4][s[4] as usize] ^ c;
        s[5] = t[5][s[5] as usize] ^ c;
        s[6] = t[6][s[6] as usize] ^ c;
        s[7] = t[7][s[7] as usize] ^ c;
        s[8] = t[8][s[8] as usize] ^ c;
        s[9] = t[9][s[9] as usize] ^ c;
        s[10] = t[10][s[10] as usize] ^ c;
        s[11] = t[11][s[11] as usize] ^ c;
        s[12] = t[12][s[12] as usize] ^ c;
        s[13] = t[13][s[13] as usize] ^ c;
        s[14] = t[14][s[14] as usize] ^ c;
        s[15] = t[15][s[15] as usize] ^ c;
    }
    out[..16].copy_from_slice(&s);
    u128::from_ne_bytes(s) == 0
}

/// Compute syndromes of `codeword` (message + non-inverted parity) into `out`.
///
/// Returns `true` when all syndromes are zero (valid codeword).
pub fn rs_syndromes(codeword: &[u8], nsym: usize, out: &mut [u8]) -> Result<bool> {
    if nsym == 0 || nsym > MAX_NSYM || out.len() < nsym {
        return Err(Error::InvalidSize("syndrome output too small"));
    }
    if nsym == 16 {
        return Ok(rs_syndromes_16(codeword, out));
    }
    let mut acc_or = 0u8;
    for (i, slot) in out[..nsym].iter_mut().enumerate() {
        let root = GF_EXP[(RS_FCR + i) % 255];
        let s = gf_poly_eval(codeword, root);
        *slot = s;
        acc_or |= s;
    }
    Ok(acc_or == 0)
}

/// Check whether `codeword` is a valid RS codeword with `nsym` parity bytes.
#[must_use]
pub fn rs_check(codeword: &[u8], nsym: usize) -> bool {
    if nsym == 0 || nsym > MAX_NSYM || codeword.len() <= nsym {
        return false;
    }
    if nsym == 16 {
        let mut s = [0u8; 16];
        return rs_syndromes_16(codeword, &mut s);
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

fn poly_mul_asc_trunc(p: &[u8], q: &[u8], max_len: usize, out: &mut [u8]) -> usize {
    let n = (p.len() + q.len() - 1).min(max_len);
    out[..n].fill(0);
    for (i, &a) in p.iter().enumerate() {
        if i >= n {
            break;
        }
        if a != 0 {
            let log_a = GF_LOG[a as usize] as usize;
            let j_end = q.len().min(n - i);
            for (j, &b) in q[..j_end].iter().enumerate() {
                if b != 0 {
                    out[i + j] ^= GF_EXP[log_a + GF_LOG[b as usize] as usize];
                }
            }
        }
    }
    n
}

#[inline]
fn poly_eval_asc(p: &[u8], x: u8) -> u8 {
    if x == 0 {
        return p.first().copied().unwrap_or(0);
    }
    let log_x = GF_LOG[x as usize] as usize;
    let mut y = 0u8;
    for &c in p.iter().rev() {
        y = if y == 0 {
            c
        } else {
            GF_EXP[GF_LOG[y as usize] as usize + log_x] ^ c
        };
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
            let e = err_loc[j];
            let s = synd[i - j];
            if e != 0 && s != 0 {
                delta ^= GF_EXP[GF_LOG[e as usize] as usize + GF_LOG[s as usize] as usize];
            }
        }
        old_loc.copy_within(0..old_len, 1);
        old_loc[0] = 0;
        old_len += 1;
        if delta != 0 {
            let log_delta = GF_LOG[delta as usize] as usize;
            if old_len > err_len {
                let mut new_loc = [0u8; MAX_NSYM + 1];
                for k in 0..old_len {
                    let o = old_loc[k];
                    if o != 0 {
                        new_loc[k] = GF_EXP[GF_LOG[o as usize] as usize + log_delta];
                    }
                }
                let log_inv = 255 - log_delta;
                for k in 0..err_len {
                    let e = err_loc[k];
                    old_loc[k] = if e == 0 {
                        0
                    } else {
                        GF_EXP[GF_LOG[e as usize] as usize + log_inv]
                    };
                }
                core::mem::swap(&mut err_len, &mut old_len);
                err_loc = new_loc;
            }
            for k in 0..old_len {
                let o = old_loc[k];
                if o != 0 {
                    err_loc[k] ^= GF_EXP[GF_LOG[o as usize] as usize + log_delta];
                }
            }
            err_len = poly_trim(&err_loc[..err_len.max(old_len)]);
        }
    }
    let nerrors = err_len - 1;
    if nerrors * 2 > nsym {
        return Err(Error::RsUncorrectable);
    }

    // Log-domain Chien search:
    // Position p has exponent expo = nmess - 1 - p, so X_p^-1 = alpha^(255 - expo).
    // Stepping p from 0..nmess increments the exponent of each nonzero term j by j mod 255.
    let mut terms_j = [0usize; MAX_NSYM];
    let mut terms_exp = [0usize; MAX_NSYM];
    let mut nterms = 0usize;
    let base_inv = 255 - (nmess - 1);
    for (j, &c) in err_loc[..err_len].iter().enumerate().skip(1) {
        if c != 0 {
            terms_j[nterms] = j;
            terms_exp[nterms] = (GF_LOG[c as usize] as usize + j * base_inv) % 255;
            nterms += 1;
        }
    }

    let mut err_pos = [0usize; MAX_NSYM];
    let mut nfound = 0usize;
    let c0 = err_loc[0];
    for p in 0..nmess {
        let mut sum = c0;
        for t in 0..nterms {
            let e = terms_exp[t];
            sum ^= GF_EXP[e];
            let next = e + terms_j[t];
            terms_exp[t] = if next >= 255 { next - 255 } else { next };
        }
        if sum == 0 {
            err_pos[nfound] = p;
            nfound += 1;
            if nfound == nerrors {
                break;
            }
        }
    }
    if nfound != nerrors {
        return Err(Error::RsUncorrectable);
    }

    // Error evaluator: Omega(z) = (S(z) * Lambda(z)) mod z^nsym.
    let mut omega_buf = [0u8; MAX_NSYM];
    let olen = poly_mul_asc_trunc(synd, &err_loc[..err_len], nsym, &mut omega_buf);
    let omega = &omega_buf[..olen];

    // Forney's formula using formal derivative Lambda'(z) = sum_{k odd} Lambda[k] * z^(k-1):
    // e_i = Omega(Xi^-1) / (Xi^(RS_FCR - 1) * Lambda'(Xi^-1)).
    for &p in &err_pos[..nfound] {
        let coef = nmess - 1 - p;
        let inv_exp = 255 - coef;
        let xi_inv = GF_EXP[inv_exp];
        let y = poly_eval_asc(omega, xi_inv);
        if y == 0 {
            continue;
        }
        let log_z2 = (inv_exp * 2) % 255;
        let mut deriv = 0u8;
        let mut k = if (err_len - 1) & 1 == 1 { err_len - 1 } else { err_len.saturating_sub(2) };
        while k >= 1 {
            deriv = if deriv == 0 {
                err_loc[k]
            } else {
                GF_EXP[GF_LOG[deriv as usize] as usize + log_z2] ^ err_loc[k]
            };
            if k < 2 {
                break;
            }
            k -= 2;
        }
        if deriv == 0 {
            return Err(Error::RsUncorrectable);
        }
        let scale_exp = (coef * (RS_FCR - 1)) % 255;
        let denom_log = (scale_exp + GF_LOG[deriv as usize] as usize) % 255;
        let magnitude = GF_EXP[GF_LOG[y as usize] as usize + 255 - denom_log];
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
