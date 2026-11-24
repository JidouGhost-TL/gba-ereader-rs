//! e-Reader hardware/software revision detection from GBA ROM headers.
//!
//! Three retail e-Reader ROM revisions are known:
//!
//! | Revision | Title | Game code | ROM size |
//! |---|---|---|---|
//! | [`EreaderRevision::CardEReaderJp`] | Card e-Reader (Japan) | `PEAJ` | 4 MiB |
//! | [`EreaderRevision::EReaderUsa`] | e-Reader (USA) | `PSAE` | 8 MiB |
//! | [`EreaderRevision::CardEReaderPlusJp`] | Card e-Reader+ (Japan) | `PSAJ` | 8 MiB |

/// Market region of an e-Reader revision.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum EreaderRegion {
    /// Japan.
    Japan,
    /// United States of America.
    Usa,
}

/// A distinct retail e-Reader ROM revision.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum EreaderRevision {
    /// Card e-Reader (Japan), game code `PEAJ`, 4 MiB ROM.
    CardEReaderJp,
    /// e-Reader (USA), game code `PSAE`, 8 MiB ROM.
    EReaderUsa,
    /// Card e-Reader+ (Japan), game code `PSAJ`, 8 MiB ROM.
    CardEReaderPlusJp,
}

impl EreaderRevision {
    /// All known revisions, in chronological order.
    pub const ALL: [Self; 3] = [Self::CardEReaderJp, Self::EReaderUsa, Self::CardEReaderPlusJp];

    /// Four-character GBA game code for this revision.
    #[must_use]
    pub const fn game_code(self) -> &'static [u8; 4] {
        match self {
            Self::CardEReaderJp => b"PEAJ",
            Self::EReaderUsa => b"PSAE",
            Self::CardEReaderPlusJp => b"PSAJ",
        }
    }

    /// Market region of this revision.
    #[must_use]
    pub const fn region(self) -> EreaderRegion {
        match self {
            Self::CardEReaderJp | Self::CardEReaderPlusJp => EreaderRegion::Japan,
            Self::EReaderUsa => EreaderRegion::Usa,
        }
    }

    /// Twelve-character GBA header title for this revision.
    #[must_use]
    pub const fn header_title(self) -> &'static [u8; 12] {
        match self {
            Self::CardEReaderJp | Self::EReaderUsa => b"CARDE READER",
            Self::CardEReaderPlusJp => b"CARDEREADER+",
        }
    }

    /// Detect the revision from raw ROM bytes.
    ///
    /// Reads the game code at offset `0xAC` and confirms the header title at
    /// `0xA0`. Returns `None` for non-e-Reader ROMs (zero false positives on
    /// unrelated titles, whose game codes differ).
    #[must_use]
    pub fn detect_from_bytes(rom: &[u8]) -> Option<Self> {
        if rom.len() < 0xC0 {
            return None;
        }
        let code = &rom[0xAC..0xB0];
        let title = &rom[0xA0..0xAC];
        Self::ALL
            .into_iter()
            .find(|&rev| code == rev.game_code() && title == rev.header_title())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fake_rom(rev: EreaderRevision) -> [u8; 256] {
        let mut rom = [0u8; 256];
        rom[0xA0..0xAC].copy_from_slice(rev.header_title());
        rom[0xAC..0xB0].copy_from_slice(rev.game_code());
        rom
    }

    #[test]
    fn detect_all_revisions() {
        for rev in EreaderRevision::ALL {
            assert_eq!(EreaderRevision::detect_from_bytes(&fake_rom(rev)), Some(rev));
        }
    }

    #[test]
    fn reject_unrelated_and_short() {
        assert_eq!(EreaderRevision::detect_from_bytes(&[0u8; 256]), None);
        assert_eq!(EreaderRevision::detect_from_bytes(&[0u8; 16]), None);
    }
}
