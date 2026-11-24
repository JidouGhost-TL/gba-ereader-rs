//! Error types for e-Reader parsing, codec, ECC, and rendering operations.

use core::fmt;

/// Errors returned by `gba-ereader` operations.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error {
    /// Input buffer ended before the expected structure was complete.
    UnexpectedEof,
    /// A magic signature did not match the expected value.
    InvalidMagic,
    /// Unsupported `vpk0` compression method byte.
    InvalidMethod(u8),
    /// Header or table field contained an invalid or out-of-range value.
    InvalidHeader(&'static str),
    /// Checksum mismatch: stored value versus computed value.
    InvalidChecksum {
        /// Checksum stored in the data.
        expected: u32,
        /// Checksum computed over the data.
        actual: u32,
    },
    /// Reed-Solomon decoding could not correct the received word.
    RsUncorrectable,
    /// A 10-bit optical symbol is not present in the modulation table.
    InvalidSymbol(u16),
    /// Strip structure was malformed.
    InvalidStrip(&'static str),
    /// Caller-provided output buffer is too small.
    OutputTooSmall,
    /// A size or length field is invalid.
    InvalidSize(&'static str),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnexpectedEof => write!(f, "Unexpected end of data"),
            Self::InvalidMagic => write!(f, "Invalid magic signature"),
            Self::InvalidMethod(m) => write!(f, "Unsupported vpk0 method: {m}"),
            Self::InvalidHeader(msg) => write!(f, "Invalid header: {msg}"),
            Self::InvalidChecksum { expected, actual } => {
                write!(f, "Checksum mismatch: stored 0x{expected:08X}, computed 0x{actual:08X}")
            }
            Self::RsUncorrectable => write!(f, "Reed-Solomon word is uncorrectable"),
            Self::InvalidSymbol(s) => write!(f, "Invalid 10-bit symbol: 0x{s:03X}"),
            Self::InvalidStrip(msg) => write!(f, "Invalid strip: {msg}"),
            Self::OutputTooSmall => write!(f, "Output buffer too small"),
            Self::InvalidSize(msg) => write!(f, "Invalid size: {msg}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}

/// Result type alias using [`Error`].
pub type Result<T> = core::result::Result<T, Error>;
