//! Guesses at the argument of `P_VirtualObject::setBoneMatrices`.
//!
//! The import's mangled name says the argument is a
//! `const std::rv<const M_Matrix4x3Template<float>*>&`: Remedy's own
//! container of pointers to 4x3 float matrices. Neither layout is known, so
//! the probe logs the container's first words and tries the two usual shapes
//! of an array type:
//!
//! * A: words 0 and 1 are `[begin, end)` of the pointer array, like `std::vector`;
//! * B: word 0 points to the pointer array and word 1 is the element count.
//!
//! Memory is read through a caller-supplied function so the guesses can be
//! tested without the game: `read(addr, buf)` fills `buf` with the bytes at
//! `addr`, or returns false if they are not readable. Pointers inside game
//! memory are 32-bit because Max Payne 1 is an x86 program.

use std::fmt::Write;

/// A longer array than any skeleton needs means the guess is wrong.
pub const MAX_COUNT: u32 = 256;
/// `M_Matrix4x3Template<float>` holds 12 floats; their order is unknown.
pub const MATRIX_FLOATS: usize = 12;
/// Matrices dumped per plausible interpretation.
const DUMPED: u32 = 2;

/// The first four 32-bit words of the container.
pub type Words = [u32; 4];

/// An array of matrix pointers as one interpretation sees it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Array {
    pub data: u32,
    pub count: u32,
}

/// What one array slot leads to, as far as memory is readable.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Element {
    /// The matrix pointer stored in the slot.
    pub ptr: Option<u32>,
    pub matrix: Option<[f32; MATRIX_FLOATS]>,
}

pub fn read_words(arg: usize, read: &impl Fn(usize, &mut [u8]) -> bool) -> Option<Words> {
    let mut bytes = [0u8; 16];
    if !read(arg, &mut bytes) {
        return None;
    }
    let mut words = [0u32; 4];
    for (word, chunk) in words.iter_mut().zip(bytes.chunks_exact(4)) {
        *word = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    Some(words)
}

/// Interpretation A, if the words make a non-empty, aligned, not too long array.
pub fn begin_end(words: &Words) -> Option<Array> {
    let [begin, end, ..] = *words;
    if begin == 0 || !begin.is_multiple_of(4) || !end.is_multiple_of(4) || end <= begin {
        return None;
    }
    let count = (end - begin) / 4;
    (count <= MAX_COUNT).then_some(Array { data: begin, count })
}

/// Interpretation B, if the words make a non-empty, aligned, not too long array.
pub fn data_count(words: &Words) -> Option<Array> {
    let [data, count, ..] = *words;
    (data != 0 && data.is_multiple_of(4) && (1..=MAX_COUNT).contains(&count)).then_some(Array { data, count })
}

impl Array {
    pub fn element(&self, index: u32, read: &impl Fn(usize, &mut [u8]) -> bool) -> Element {
        let ptr = (index < self.count)
            .then(|| self.data.checked_add(4 * index))
            .flatten()
            .and_then(|slot| read_u32(slot, read));
        let matrix = ptr.and_then(|ptr| read_matrix(ptr, read));
        Element { ptr, matrix }
    }
}

fn read_u32(addr: u32, read: &impl Fn(usize, &mut [u8]) -> bool) -> Option<u32> {
    let mut bytes = [0u8; 4];
    read(addr as usize, &mut bytes).then(|| u32::from_le_bytes(bytes))
}

fn read_matrix(addr: u32, read: &impl Fn(usize, &mut [u8]) -> bool) -> Option<[f32; MATRIX_FLOATS]> {
    let mut bytes = [0u8; 4 * MATRIX_FLOATS];
    if addr == 0 || !read(addr as usize, &mut bytes) {
        return None;
    }
    let mut matrix = [0f32; MATRIX_FLOATS];
    for (value, chunk) in matrix.iter_mut().zip(bytes.chunks_exact(4)) {
        *value = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    Some(matrix)
}

/// Log lines for one argument: the raw words, then each interpretation with
/// its first matrices. Every line starts with two spaces.
pub fn describe(words: &Words, read: &impl Fn(usize, &mut [u8]) -> bool) -> String {
    let mut out = format!(
        "  words {:#010x} {:#010x} {:#010x} {:#010x}",
        words[0], words[1], words[2], words[3]
    );
    let interpretations = [
        ("A [begin, end)", begin_end(words)),
        ("B (data, count)", data_count(words)),
    ];
    for (label, array) in interpretations {
        let Some(array) = array else {
            let _ = write!(out, "\n  {label}: implausible");
            continue;
        };
        let _ = write!(out, "\n  {label}: array {:#010x} count {}", array.data, array.count);
        for index in 0..DUMPED.min(array.count) {
            let element = array.element(index, read);
            let _ = match (element.ptr, element.matrix) {
                (None, _) => write!(out, "\n    [{index}] slot unreadable"),
                (Some(ptr), None) => write!(out, "\n    [{index}] {ptr:#010x}: unreadable"),
                (Some(ptr), Some(matrix)) => write!(out, "\n    [{index}] {ptr:#010x}: {matrix:?}"),
            };
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Readable regions of a fake address space.
    struct Memory(Vec<(u32, Vec<u8>)>);

    impl Memory {
        fn read(&self, addr: usize, buf: &mut [u8]) -> bool {
            for (base, bytes) in &self.0 {
                let start = addr.wrapping_sub(*base as usize);
                if addr >= *base as usize && start + buf.len() <= bytes.len() {
                    buf.copy_from_slice(&bytes[start..start + buf.len()]);
                    return true;
                }
            }
            false
        }
    }

    fn words_bytes(words: &[u32]) -> Vec<u8> {
        words.iter().flat_map(|w| w.to_le_bytes()).collect()
    }

    fn matrix_bytes(first: f32) -> Vec<u8> {
        (0..MATRIX_FLOATS)
            .flat_map(|i| (first + i as f32).to_le_bytes())
            .collect()
    }

    fn matrix(first: f32) -> [f32; MATRIX_FLOATS] {
        std::array::from_fn(|i| first + i as f32)
    }

    /// A container at 0x1000 laid out as `[begin, end)` over three matrix
    /// pointers at 0x2000; the third pointer leads to unmapped memory. A
    /// readable fourth pointer lies past `end`, where no element may be read.
    fn begin_end_memory() -> Memory {
        Memory(vec![
            (0x1000, words_bytes(&[0x2000, 0x200C, 0x200C, 0])),
            (0x2000, words_bytes(&[0x3000, 0x3100, 0x9000, 0x3000])),
            (0x3000, matrix_bytes(1.0)),
            (0x3100, matrix_bytes(100.0)),
        ])
    }

    #[test]
    fn reads_words_only_when_readable() {
        let memory = begin_end_memory();
        let read = |addr, buf: &mut [u8]| memory.read(addr, buf);
        assert_eq!(read_words(0x1000, &read), Some([0x2000, 0x200C, 0x200C, 0]));
        // The last word would lie past the end of the region.
        assert_eq!(read_words(0x1004, &read), None);
        assert_eq!(read_words(0, &read), None);
    }

    #[test]
    fn decodes_begin_end_with_matrices() {
        let memory = begin_end_memory();
        let read = |addr, buf: &mut [u8]| memory.read(addr, buf);
        let words = read_words(0x1000, &read).unwrap();
        let array = begin_end(&words).unwrap();
        assert_eq!(array, Array { data: 0x2000, count: 3 });
        assert_eq!(
            array.element(0, &read),
            Element {
                ptr: Some(0x3000),
                matrix: Some(matrix(1.0))
            }
        );
        assert_eq!(array.element(1, &read).matrix, Some(matrix(100.0)));
        assert_eq!(
            array.element(2, &read),
            Element {
                ptr: Some(0x9000),
                matrix: None
            }
        );
        assert_eq!(
            array.element(3, &read),
            Element {
                ptr: None,
                matrix: None
            }
        );
        // Word 1 is an address, far too large to be a count.
        assert_eq!(data_count(&words), None);
    }

    #[test]
    fn decodes_data_count() {
        let words = [0x2000, 2, 0, 0];
        assert_eq!(data_count(&words), Some(Array { data: 0x2000, count: 2 }));
        // As [begin, end) the same words run backwards.
        assert_eq!(begin_end(&words), None);
    }

    #[test]
    fn rejects_implausible_arrays() {
        for words in [
            [0, 0x10, 0, 0],        // null begin
            [0x2000, 0x2000, 0, 0], // empty
            [0x2002, 0x2010, 0, 0], // misaligned begin
            [0x2000, 0x2006, 0, 0], // misaligned end
            [0x2000, 0x1000, 0, 0], // end before begin
            [0x2000, 0x2404, 0, 0], // 257 elements
        ] {
            assert_eq!(begin_end(&words), None, "{words:x?}");
        }
        assert_eq!(begin_end(&[0x2000, 0x2400, 0, 0]).map(|a| a.count), Some(MAX_COUNT));

        for words in [
            [0, 4, 0, 0],      // null data
            [0x2001, 4, 0, 0], // misaligned data
            [0x2000, 0, 0, 0], // empty
            [0x2000, 257, 0, 0],
        ] {
            assert_eq!(data_count(&words), None, "{words:x?}");
        }
        assert_eq!(data_count(&[0x2000, MAX_COUNT, 0, 0]).map(|a| a.count), Some(MAX_COUNT));
    }

    #[test]
    fn array_slots_past_the_address_space_are_unreadable() {
        // Address 0 is readable here, so a slot address that wrapped around
        // to it would be read instead of rejected.
        let memory = Memory(vec![
            (0xFFFF_FFF0, words_bytes(&[0x3000; 4])),
            (0, words_bytes(&[0x3100])),
        ]);
        let read = |addr, buf: &mut [u8]| memory.read(addr, buf);
        let array = Array {
            data: 0xFFFF_FFFC,
            count: 2,
        };
        assert_eq!(array.element(0, &read).ptr, Some(0x3000));
        assert_eq!(
            array.element(1, &read),
            Element {
                ptr: None,
                matrix: None
            }
        );
    }

    #[test]
    fn describes_both_interpretations() {
        let memory = begin_end_memory();
        let read = |addr, buf: &mut [u8]| memory.read(addr, buf);
        let text = describe(&read_words(0x1000, &read).unwrap(), &read);
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines[0], "  words 0x00002000 0x0000200c 0x0000200c 0x00000000");
        assert_eq!(lines[1], "  A [begin, end): array 0x00002000 count 3");
        assert_eq!(
            lines[2],
            "    [0] 0x00003000: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]"
        );
        assert!(lines[3].starts_with("    [1] 0x00003100: [100.0, "), "{}", lines[3]);
        assert_eq!(lines[4], "  B (data, count): implausible");
        assert_eq!(lines.len(), 5);

        // B over an array whose first slot points nowhere and whose second
        // slot is past the readable region.
        let memory = Memory(vec![(0x2000, words_bytes(&[0x9000]))]);
        let read = |addr, buf: &mut [u8]| memory.read(addr, buf);
        let text = describe(&[0x2000, 5, 0, 0], &read);
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines[1], "  A [begin, end): implausible");
        assert_eq!(lines[2], "  B (data, count): array 0x00002000 count 5");
        assert_eq!(lines[3], "    [0] 0x00009000: unreadable");
        assert_eq!(lines[4], "    [1] slot unreadable");
        assert_eq!(lines.len(), 5);
    }
}
