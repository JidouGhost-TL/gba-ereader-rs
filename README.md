# gba-ereader

Pure Rust, zero-dependency toolkit for the Nintendo GBA **e-Reader**
dot-code system: the `vpk0` codec, GF(256) Reed-Solomon ECC, 8-to-10-bit
optical modulation, `.raw` strip ↔ `.bin` card payload formats, and a
printable dot-code strip renderer.

> **See also:** the [`gbakit`](https://github.com/JidouGhost-TL/gbakit) CLI
> (`gbakit audio scan`, `gbakit audio export`, `gbakit audio soundfont`)
> for corpus-wide GBA analysis across the whole gbakit ecosystem.

## Features

- **Zero external dependencies** (`[dependencies]` is empty, `[dev-dependencies]` is empty).
- **`#![no_std]` first**: `#![cfg_attr(not(feature = "std"), no_std)]` with
  `default = ["std"]`, `std = ["alloc"]`, `alloc = []`.
- **`#![deny(unsafe_code)]`**: 100% safe Rust.
- **Dual license**: `MIT OR Apache-2.0`.

## Modules

| Module | Contents |
|---|---|
| `error` | `#[non_exhaustive]` error enum + `Result` alias |
| `version` | `EreaderRegion` / `EreaderRevision` + `detect_from_bytes` |
| `vpk0` | `vpk0` decode + encode (method 0 fixed-width, method 1 bit-width trees) |
| `rs_ecc` | GF(256) Reed-Solomon encode + decode/correct (`0x11D`) |
| `dotcode` | 8-to-10 modulation, `.raw` ↔ `.bin`, 48-byte card header, multi-strip assembly |
| `render` | Printable strip bitmaps (PBM/PGM/BMP) at 300/600/1200 DPI |

## ROM revisions

| Revision | Title | Game code | Region |
|---|---|---|---|
| `CardEReaderJp` | Card e-Reader (Japan) | `PEAJ` | Japan |
| `EReaderUsa` | e-Reader (USA) | `PSAE` | USA |
| `CardEReaderPlusJp` | Card e-Reader+ (Japan) | `PSAJ` | Japan |

Detection reads the game code at `0xAC` and confirms the header title at
`0xA0`, with zero false positives on unrelated titles.

## Pipeline

`payload -> vpk0 compress -> .bin strip -> .raw optical strip -> print/scan
-> ECC repair -> .bin -> vpk0 decompress == payload`

The `.bin` format is a 48-byte header (magic `ERB1`, region, card type,
strip index/count, payload/total lengths, payload CRC-32, header CRC-32)
followed by the payload. The `.raw` format packs 10-bit modulated symbols
MSB-first after a 2-byte sync header, with Reed-Solomon blocks
(223 data + 32 parity, correcting up to 16 errors per block) interleaved
column-wise. Short strips use 2 blocks; long strips use 8 blocks.

## Golden tests

`tests/golden_tests.rs` asserts byte-exact (`assert_eq!`, zero tolerance)
results on clean-room synthetic deterministic payloads: generator
polynomials, modulation-table prefix, the CRC-32 check value, `vpk0`
header bytes, revision detection, the full pipeline round-trip with
bit-flip corruption and ECC repair, and DPI scaling. No copyrighted card
data is used anywhere.

## Usage

```rust
use gba_ereader::vpk0::{vpk0_compress, vpk0_decompress};

let compressed = vpk0_compress(b"hello e-reader").unwrap();
assert_eq!(vpk0_decompress(&compressed).unwrap(), b"hello e-reader".to_vec());
```

## Benchmarks

```sh
cargo bench --bench bench_ereader
```

Benchmarks cover `vpk0` decode/encode, Reed-Solomon encode and
decode/correct (clean and 16-error), and `.raw` ↔ `.bin` strip round-trips.

## License

Licensed under either of MIT or Apache-2.0 at your option.
