//! # gba-ereader
//!
//! Pure Rust, zero-dependency toolkit for the Nintendo GBA **e-Reader**
//! dot-code system: the `vpk0` codec, GF(256) Reed-Solomon ECC, 8-to-10-bit
//! optical modulation, `.raw` strip ↔ `.bin` card payload formats, and a
//! printable dot-code strip renderer.
//!
//! ## Features
//! - **Zero external dependencies** (`[dependencies]` is empty).
//! - **`#![no_std]` compatible** with `default = ["std"]`, `std = ["alloc"]`, `alloc = []`.
//! - **`#![deny(unsafe_code)]`**: 100% safe Rust.
//! - [`vpk0`] decode + encode (methods 0 and 1), [`rs_ecc`] encode +
//!   decode/correct, [`dotcode`] strip formats with multi-strip reassembly,
//!   and [`render`] 300/600/1200 DPI printable bitmap output (PBM/PGM/BMP).
//!
//! ## Pipeline
//! `payload -> vpk0 compress -> .bin strip -> .raw optical strip ->
//! (print/scan) -> ECC repair -> .bin -> vpk0 decompress == payload`.
//!
//! ## Quickstart
//! ```rust
//! use gba_ereader::dotcode::{bin_to_raw, build_bin, parse_bin, raw_to_bin};
//! use gba_ereader::vpk0::{vpk0_compress, vpk0_decompress};
//!
//! let compressed = vpk0_compress(b"hello e-reader").unwrap();
//! let bin = build_bin(1, 0, 0, 1, &compressed).unwrap();
//! let raw = bin_to_raw(&bin).unwrap();
//! let repaired = raw_to_bin(&raw).unwrap();
//! let (_, payload) = parse_bin(&repaired).unwrap();
//! assert_eq!(vpk0_decompress(payload).unwrap(), b"hello e-reader".to_vec());
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
#![deny(unsafe_code)]

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod dotcode;
pub mod error;
pub mod render;
pub mod rs_ecc;
pub mod version;
pub mod vpk0;

#[cfg(feature = "alloc")]
pub use dotcode::{assemble_strips, bin_to_raw, build_bin, raw_to_bin, split_into_strips};
pub use dotcode::{
    demodulate, is_short_strip, modulate, parse_bin, CardHeader, CardType, CARD_HEADER_LEN,
    LONG_STRIP_RAW_LEN, SHORT_STRIP_RAW_LEN,
};
pub use error::{Error, Result};
#[cfg(feature = "alloc")]
pub use render::{render_strip, StripBitmap};
pub use render::{module_px, SUPPORTED_DPI};
#[cfg(feature = "alloc")]
pub use rs_ecc::rs_encode;
pub use rs_ecc::{
    rs_check, rs_correct, rs_correct_inverted, rs_encode_parity, rs_encode_parity_inverted,
};
pub use version::{EreaderRegion, EreaderRevision};
#[cfg(feature = "alloc")]
pub use vpk0::{vpk0_compress, vpk0_compress_with_method, vpk0_decompress};
pub use vpk0::{vpk0_decompressed_size, vpk0_decompress_into};
