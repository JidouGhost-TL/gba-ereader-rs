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

pub use dotcode::{CardHeader, CardType, CARD_HEADER_LEN};
pub use error::{Error, Result};
pub use version::{EreaderRegion, EreaderRevision};
#[cfg(feature = "alloc")]
pub use vpk0::{vpk0_compress, vpk0_compress_with_method, vpk0_decompress};
pub use vpk0::{vpk0_decompressed_size, vpk0_decompress_into};
