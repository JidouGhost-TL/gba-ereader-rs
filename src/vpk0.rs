//! `vpk0` codec, byte-exact with the e-Reader ROM decoder
//! (`FUN_03004d0c` / `FUN_03004ad4` in IWRAM, verified in `libmgba`).
//!
//! Layout: `magic[4] | size_be[4] | method[1] | trees | tokens`.
//! The bitstream is MSB-first. After the 9-byte header come two serialized
//! Huffman-style *bit-width trees* (offset tree, then length tree). A tree
//! is serialized on a stack discipline: a `0` bit plus an 8-bit leaf value
//! pushes a leaf; a `1` bit combines the top two stack nodes (second-from-top
//! becomes child 0, top becomes child 1); a final `1` bit with one node on
//! the stack terminates the tree. Decoding a value walks a tree to a leaf,
//! whose value is a bit width `w`, then reads `w` bits as the value (a
//! single-leaf tree therefore encodes fixed-width values).
//!
//! Tokens: a `0` bit introduces an 8-bit literal; a `1` bit introduces a
//! back-reference: an offset value then a length value (copy count), each
//! decoded through its tree. Method 0 uses the offset value directly as the
//! distance. Method 1 splits the distance: a first value `v < 3` selects
//! the low bits (`v + 1`) and a second value gives `distance = (v + 1) +
//! (v2 - 2) * 4`; a first value `v >= 3` gives `distance = (v - 2) * 4`.

use crate::error::{Error, Result};

/// Magic signature of a `vpk0` stream.
pub const VPK0_MAGIC: &[u8; 4] = b"vpk0";
/// Header length in bytes (magic + size + method).
pub const VPK0_HEADER_LEN: usize = 9;
/// Maximum back-reference distance encodable by the encoder.
pub const VPK0_MAX_OFFSET: usize = 1 << 20;
/// Minimum match length used by the encoder.
pub const VPK0_MIN_MATCH: usize = 3;
/// Maximum match length used by the encoder.
pub const VPK0_MAX_MATCH: usize = 4096;

// ---------------------------------------------------------------------------
// Bit I/O (MSB-first, 64-bit buffered)
// ---------------------------------------------------------------------------

pub(crate) struct BitReader<'a> {
    data: &'a [u8],
    pos: usize,
    buf: u64,
    nbits: u32,
}

impl<'a> BitReader<'a> {
    #[inline]
    pub(crate) fn new(data: &'a [u8]) -> Self {
        let mut r = Self { data, pos: 0, buf: 0, nbits: 0 };
        r.refill();
        r
    }

    #[inline(always)]
    fn refill(&mut self) {
        if self.nbits <= 32 && self.pos + 4 <= self.data.len() {
            let word = u32::from_be_bytes([
                self.data[self.pos],
                self.data[self.pos + 1],
                self.data[self.pos + 2],
                self.data[self.pos + 3],
            ]);
            self.buf |= (word as u64) << (32 - self.nbits);
            self.nbits += 32;
            self.pos += 4;
            return;
        }
        while self.nbits <= 56 && self.pos < self.data.len() {
            self.buf |= (self.data[self.pos] as u64) << (56 - self.nbits);
            self.nbits += 8;
            self.pos += 1;
        }
    }

    #[inline(always)]
    pub(crate) fn read_bit(&mut self) -> Result<u32> {
        if self.nbits == 0 {
            self.refill();
            if self.nbits == 0 {
                return Err(Error::UnexpectedEof);
            }
        }
        let bit = (self.buf >> 63) as u32;
        self.buf <<= 1;
        self.nbits -= 1;
        Ok(bit)
    }

    #[inline(always)]
    pub(crate) fn bits(&mut self, count: u32) -> Result<u32> {
        if count == 0 {
            return Ok(0);
        }
        if self.nbits < count {
            self.refill();
            if self.nbits < count {
                return Err(Error::UnexpectedEof);
            }
        }
        let v = (self.buf >> (64 - count)) as u32;
        self.buf <<= count;
        self.nbits -= count;
        Ok(v)
    }
}

#[cfg(feature = "alloc")]
struct BitWriter {
    buf: alloc::vec::Vec<u8>,
    cur: u64,
    nbits: u32,
}

#[cfg(feature = "alloc")]
impl BitWriter {
    fn with_header(uncompressed_len: u32, method: u8, capacity: usize) -> Self {
        let mut buf = alloc::vec::Vec::with_capacity(VPK0_HEADER_LEN + capacity);
        buf.extend_from_slice(VPK0_MAGIC);
        buf.extend_from_slice(&uncompressed_len.to_be_bytes());
        buf.push(method);
        Self { buf, cur: 0, nbits: 0 }
    }

    #[inline(always)]
    fn bits(&mut self, value: u32, count: u32) {
        if count == 0 {
            return;
        }
        let mask = if count == 32 { u32::MAX } else { (1u32 << count) - 1 };
        self.cur = (self.cur << count) | (value & mask) as u64;
        self.nbits += count;
        if self.nbits >= 32 {
            self.nbits -= 32;
            let word = (self.cur >> self.nbits) as u32;
            self.buf.extend_from_slice(&word.to_be_bytes());
        }
    }

    fn finish(mut self) -> alloc::vec::Vec<u8> {
        while self.nbits >= 8 {
            self.nbits -= 8;
            self.buf.push((self.cur >> self.nbits) as u8);
        }
        if self.nbits > 0 {
            self.buf.push((self.cur << (8 - self.nbits)) as u8);
        }
        self.buf
    }
}

// ---------------------------------------------------------------------------
// Bit-width trees
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct Node {
    child: [u16; 2],
    sym: u8,
    leaf: bool,
}

impl Node {
    const EMPTY: Self = Self { child: [0, 0], sym: 0, leaf: false };
}

const MAX_TREE_NODES: usize = 1023;

struct Tree {
    nodes: [Node; MAX_TREE_NODES],
    count: usize,
    root: usize,
}

impl Tree {
    fn new() -> Self {
        Self { nodes: [Node::EMPTY; MAX_TREE_NODES], count: 0, root: 0 }
    }

    #[inline(always)]
    fn is_leaf_root(&self) -> bool {
        self.count == 1
    }
}

fn parse_tree(reader: &mut BitReader<'_>) -> Result<Tree> {
    let mut tree = Tree::new();
    let mut stack = [0u16; 512];
    let mut sp = 0usize;
    loop {
        let bit = reader.read_bit()?;
        if bit == 0 {
            let sym = reader.bits(8)? as u8;
            if tree.count >= MAX_TREE_NODES || sp >= stack.len() {
                return Err(Error::InvalidHeader("vpk0 tree too large"));
            }
            let idx = tree.count;
            tree.count += 1;
            tree.nodes[idx] = Node { child: [0, 0], sym, leaf: true };
            stack[sp] = idx as u16;
            sp += 1;
        } else {
            if sp <= 1 {
                break;
            }
            if tree.count >= MAX_TREE_NODES {
                return Err(Error::InvalidHeader("vpk0 tree too large"));
            }
            let child1 = stack[sp - 1];
            let child0 = stack[sp - 2];
            sp -= 1;
            let idx = tree.count;
            tree.count += 1;
            tree.nodes[idx] = Node { child: [child0, child1], sym: 0, leaf: false };
            stack[sp - 1] = idx as u16;
        }
    }
    if sp != 1 {
        return Err(Error::InvalidHeader("vpk0 tree malformed"));
    }
    tree.root = stack[0] as usize;
    Ok(tree)
}

#[inline(always)]
fn decode_value(tree: &Tree, reader: &mut BitReader<'_>) -> Result<u32> {
    let width = if tree.is_leaf_root() {
        tree.nodes[tree.root].sym as u32
    } else {
        let mut idx = tree.root;
        loop {
            let node = &tree.nodes[idx];
            if node.leaf {
                break node.sym as u32;
            }
            let bit = reader.read_bit()? as usize;
            idx = node.child[bit] as usize;
        }
    };
    if width > 24 {
        return Err(Error::InvalidHeader("vpk0 bit width too large"));
    }
    reader.bits(width)
}

// ---------------------------------------------------------------------------
// Header / decompression
// ---------------------------------------------------------------------------

fn parse_header(data: &[u8]) -> Result<(usize, u8)> {
    if data.len() < VPK0_HEADER_LEN {
        return Err(Error::UnexpectedEof);
    }
    if &data[0..4] != VPK0_MAGIC {
        return Err(Error::InvalidMagic);
    }
    let size = u32::from_be_bytes([data[4], data[5], data[6], data[7]]) as usize;
    Ok((size, data[8]))
}

/// Decompressed size declared by a `vpk0` header.
pub fn vpk0_decompressed_size(data: &[u8]) -> Result<usize> {
    Ok(parse_header(data)?.0)
}

/// Decompress a `vpk0` stream into a newly allocated vector.
#[cfg(feature = "alloc")]
pub fn vpk0_decompress(data: &[u8]) -> Result<alloc::vec::Vec<u8>> {
    let (size, _) = parse_header(data)?;
    let mut out = alloc::vec![0u8; size];
    let n = vpk0_decompress_into(data, &mut out)?;
    out.truncate(n);
    Ok(out)
}

/// Decompress a `vpk0` stream into a caller buffer; returns bytes written.
pub fn vpk0_decompress_into(data: &[u8], out: &mut [u8]) -> Result<usize> {
    let (size, method) = parse_header(data)?;
    if method > 1 {
        return Err(Error::InvalidMethod(method));
    }
    if out.len() < size {
        return Err(Error::OutputTooSmall);
    }
    let mut reader = BitReader::new(&data[VPK0_HEADER_LEN..]);
    let offset_tree = parse_tree(&mut reader)?;
    let length_tree = parse_tree(&mut reader)?;
    let mut produced = 0usize;
    while produced < size {
        if reader.read_bit()? == 0 {
            out[produced] = reader.bits(8)? as u8;
            produced += 1;
        } else {
            let v = decode_value(&offset_tree, &mut reader)?;
            let dist = if method == 0 {
                v as usize
            } else if v < 3 {
                let v2 = decode_value(&offset_tree, &mut reader)?;
                (v as usize + 1) + (v2 as usize).saturating_sub(2) * 4
            } else {
                (v as usize - 2) * 4
            };
            if dist == 0 || dist > produced {
                return Err(Error::InvalidHeader("vpk0 offset out of range"));
            }
            let len = decode_value(&length_tree, &mut reader)? as usize;
            if len > size - produced {
                return Err(Error::InvalidSize("vpk0 match overruns output"));
            }
            if len > 0 {
                if dist == 1 {
                    let b = out[produced - 1];
                    out[produced..produced + len].fill(b);
                } else if dist >= len {
                    let src = produced - dist;
                    out.copy_within(src..src + len, produced);
                } else {
                    // Overlapping match: copy in doubling non-overlapping spans.
                    let src = produced - dist;
                    let mut copied = 0usize;
                    while copied < len {
                        let chunk = core::cmp::min(dist + copied, len - copied);
                        out.copy_within(src..src + chunk, produced + copied);
                        copied += chunk;
                    }
                }
                produced += len;
            }
        }
    }
    Ok(produced)
}

// ---------------------------------------------------------------------------
// Compression
// ---------------------------------------------------------------------------

#[cfg(feature = "alloc")]
enum Token {
    Literal(u8),
    Match { dist: u32, len: u16 },
}

#[cfg(feature = "alloc")]
#[inline(always)]
fn offset_values(method: u8, dist: usize) -> (u32, Option<u32>) {
    if method == 0 {
        (dist as u32, None)
    } else if dist.is_multiple_of(4) {
        ((dist / 4 + 2) as u32, None)
    } else {
        ((dist % 4 - 1) as u32, Some((dist / 4 + 2) as u32))
    }
}

#[cfg(feature = "alloc")]
#[inline(always)]
fn bit_width(v: u32) -> u32 {
    32 - v.leading_zeros()
}

#[cfg(feature = "alloc")]
struct EncNode {
    child: [i32; 2],
    sym: u8,
    leaf: bool,
    freq: u64,
}

/// Build a Huffman tree over width symbols; returns nodes with root last.
#[cfg(feature = "alloc")]
fn build_tree(widths: &[(u32, u64)]) -> alloc::vec::Vec<EncNode> {
    let mut nodes: alloc::vec::Vec<EncNode> = widths
        .iter()
        .map(|&(w, f)| EncNode { child: [-1, -1], sym: w as u8, leaf: true, freq: f })
        .collect();
    if nodes.len() == 1 {
        return nodes;
    }
    let mut alive: alloc::vec::Vec<usize> = (0..nodes.len()).collect();
    while alive.len() > 1 {
        alive.sort_by_key(|&i| (nodes[i].freq, i));
        let a = alive.remove(0);
        let b = alive.remove(0);
        let freq = nodes[a].freq + nodes[b].freq;
        nodes.push(EncNode { child: [a as i32, b as i32], sym: 0, leaf: false, freq });
        alive.push(nodes.len() - 1);
    }
    nodes
}

#[cfg(feature = "alloc")]
fn write_tree(nodes: &[EncNode], w: &mut BitWriter) {
    fn emit(nodes: &[EncNode], idx: usize, w: &mut BitWriter) {
        let n = &nodes[idx];
        if n.leaf {
            w.bits(0, 1);
            w.bits(n.sym as u32, 8);
        } else {
            emit(nodes, n.child[0] as usize, w);
            emit(nodes, n.child[1] as usize, w);
            w.bits(1, 1);
        }
    }
    emit(nodes, nodes.len() - 1, w);
    w.bits(1, 1); // terminator
}

#[cfg(feature = "alloc")]
fn build_code_table(nodes: &[EncNode]) -> [(u32, u32); 32] {
    fn fill(nodes: &[EncNode], idx: usize, path: u32, depth: u32, table: &mut [(u32, u32); 32]) {
        let n = &nodes[idx];
        if n.leaf {
            let sym = n.sym as usize;
            if sym < table.len() {
                table[sym] = (path, depth);
            }
        } else {
            fill(nodes, n.child[0] as usize, path << 1, depth + 1, table);
            fill(nodes, n.child[1] as usize, (path << 1) | 1, depth + 1, table);
        }
    }
    let mut table = [(0u32, 0u32); 32];
    if nodes.len() > 1 {
        fill(nodes, nodes.len() - 1, 0, 0, &mut table);
    }
    table
}

#[cfg(feature = "alloc")]
#[inline(always)]
fn write_value(codes: &[(u32, u32); 32], value: u32, w: &mut BitWriter) {
    let width = bit_width(value);
    let (path, depth) = codes[width as usize];
    if depth > 0 {
        w.bits(path, depth);
    }
    w.bits(value, width);
}

/// Compress `data` with the given method (0 or 1) into a `vpk0` stream.
#[cfg(feature = "alloc")]
pub fn vpk0_compress_with_method(data: &[u8], method: u8) -> Result<alloc::vec::Vec<u8>> {
    if method > 1 {
        return Err(Error::InvalidMethod(method));
    }
    if data.len() > u32::MAX as usize {
        return Err(Error::InvalidSize("vpk0 input exceeds 4 GiB"));
    }

    // Tokenize with a 3-byte hash table (1-based positions, 0 = empty).
    let mut tokens: alloc::vec::Vec<Token> = alloc::vec::Vec::with_capacity(data.len() / 2 + 16);
    let mut last = alloc::vec![0u32; 8192];
    let mut off_counts = [0u64; 32];
    let mut len_counts = [0u64; 32];

    let mut i = 0usize;
    let data_len = data.len();
    while i < data_len {
        let mut best_len = 0usize;
        let mut best_off = 0usize;
        if i + VPK0_MIN_MATCH <= data_len {
            let v = ((data[i] as usize) << 16)
                | ((data[i + 1] as usize) << 8)
                | data[i + 2] as usize;
            let h = (v.wrapping_mul(0x1E35_A7BD) >> 19) & 8191;
            let cand = last[h];
            last[h] = (i + 1) as u32;
            if cand != 0 {
                let start = (cand - 1) as usize;
                let dist = i - start;
                if dist <= VPK0_MAX_OFFSET
                    && data[start] == data[i]
                    && data[start + 1] == data[i + 1]
                    && data[start + 2] == data[i + 2]
                {
                    let max_possible = core::cmp::min(VPK0_MAX_MATCH, data_len - i);
                    let mut l = VPK0_MIN_MATCH;
                    let s_slice = &data[start..start + max_possible];
                    let i_slice = &data[i..i + max_possible];
                    while l < max_possible && s_slice[l] == i_slice[l] {
                        l += 1;
                    }
                    best_len = l;
                    best_off = dist;
                }
            }
        }
        if best_len >= VPK0_MIN_MATCH {
            let (v1, v2) = offset_values(method, best_off);
            off_counts[bit_width(v1) as usize] += 1;
            if let Some(v2) = v2 {
                off_counts[bit_width(v2) as usize] += 1;
            }
            len_counts[bit_width(best_len as u32) as usize] += 1;
            tokens.push(Token::Match { dist: best_off as u32, len: best_len as u16 });
            i += best_len;
        } else {
            tokens.push(Token::Literal(data[i]));
            i += 1;
        }
    }

    let mut off_freq: alloc::vec::Vec<(u32, u64)> = off_counts
        .iter()
        .enumerate()
        .filter_map(|(w, &c)| if c > 0 { Some((w as u32, c)) } else { None })
        .collect();
    let mut len_freq: alloc::vec::Vec<(u32, u64)> = len_counts
        .iter()
        .enumerate()
        .filter_map(|(w, &c)| if c > 0 { Some((w as u32, c)) } else { None })
        .collect();
    if off_freq.is_empty() {
        off_freq.push((0, 1));
    }
    if len_freq.is_empty() {
        len_freq.push((0, 1));
    }
    let off_tree = build_tree(&off_freq);
    let len_tree = build_tree(&len_freq);
    let off_codes = build_code_table(&off_tree);
    let len_codes = build_code_table(&len_tree);

    let mut w = BitWriter::with_header(data_len as u32, method, data_len / 2 + 32);
    write_tree(&off_tree, &mut w);
    write_tree(&len_tree, &mut w);
    for t in &tokens {
        match *t {
            Token::Literal(b) => {
                w.bits(b as u32, 9);
            }
            Token::Match { dist, len } => {
                w.bits(1, 1);
                let (v1, v2) = offset_values(method, dist as usize);
                write_value(&off_codes, v1, &mut w);
                if let Some(v2) = v2 {
                    write_value(&off_codes, v2, &mut w);
                }
                write_value(&len_codes, len as u32, &mut w);
            }
        }
    }
    Ok(w.finish())
}

/// Compress `data` into a `vpk0` stream (method 0).
#[cfg(feature = "alloc")]
pub fn vpk0_compress(data: &[u8]) -> Result<alloc::vec::Vec<u8>> {
    vpk0_compress_with_method(data, 0)
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;

    #[test]
    fn header_layout() {
        let c = vpk0_compress(b"AB").unwrap();
        assert_eq!(&c[0..4], b"vpk0");
        assert_eq!(u32::from_be_bytes([c[4], c[5], c[6], c[7]]), 2);
        assert_eq!(c[8], 0);
        assert_eq!(vpk0_decompressed_size(&c).unwrap(), 2);
    }

    #[test]
    fn roundtrip_both_methods() {
        let mut data = alloc::vec::Vec::new();
        for i in 0..2000 {
            data.push((i % 7) as u8);
            if i % 97 == 0 {
                data.extend_from_slice(b" ereader ereader ");
            }
        }
        for m in [0u8, 1] {
            let c = vpk0_compress_with_method(&data, m).unwrap();
            assert_eq!(vpk0_decompress(&c).unwrap(), data);
        }
        let empty = vpk0_compress(&[]).unwrap();
        assert_eq!(vpk0_decompress(&empty).unwrap(), alloc::vec::Vec::<u8>::new());
        let one = vpk0_compress(&[42]).unwrap();
        assert_eq!(vpk0_decompress(&one).unwrap(), alloc::vec![42]);
        let runs = vpk0_compress_with_method(&[7u8; 500], 1).unwrap();
        assert_eq!(vpk0_decompress(&runs).unwrap(), alloc::vec![7u8; 500]);
    }

    #[test]
    fn reject_bad_streams() {
        assert_eq!(vpk0_decompress(b"xxxx\0\0\0\0\0"), Err(Error::InvalidMagic));
        assert_eq!(vpk0_decompress(b"vpk0\0\0\0\0\x09"), Err(Error::InvalidMethod(9)));
        assert_eq!(vpk0_decompress(b"vpk"), Err(Error::UnexpectedEof));
    }
}
