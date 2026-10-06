//! Statistics of the `setBoneMatrices` hook. Game threads record every call;
//! the plugin thread reads and resets them every few seconds. Recording only
//! touches atomics: it never blocks or allocates.

use std::fmt;
use std::sync::atomic::Ordering::Relaxed;
use std::sync::atomic::{AtomicU32, AtomicUsize};

use crate::rv::{self, Words};

/// Distinct `this` pointers remembered; calls on objects beyond these are
/// only counted as untracked.
const TRACKED: usize = 256;

pub struct HookStats {
    total_calls: AtomicU32,
    calls: AtomicU32,
    unreadable: AtomicU32,
    untracked: AtomicU32,
    /// Starts at 1 so that a zero in `last_window` means "never seen".
    window: AtomicU32,
    window_objects: AtomicU32,
    /// Objects ever seen, in open addressing by `slot_of`; 0 is a free slot.
    objects: [AtomicUsize; TRACKED],
    last_window: [AtomicU32; TRACKED],
    sighted: AtomicU32,
    begin_end: CountRange,
    data_count: CountRange,
}

/// What the hook needs to know about the call it just recorded.
pub struct Call {
    /// 0 for the first call since the hook was installed.
    pub number: u32,
    /// Index of `this` among the objects seen so far, when this call is its
    /// first sighting.
    pub first_sighting: Option<u32>,
}

/// Statistics of one window.
#[derive(Debug, PartialEq)]
pub struct Window {
    pub calls: u32,
    /// Distinct objects that called in this window.
    pub objects: u32,
    /// Distinct objects since the hook was installed.
    pub total_objects: u32,
    /// Calls on objects that did not fit in the table (or null `this`).
    pub untracked: u32,
    pub unreadable: u32,
    pub begin_end: Option<Range>,
    pub data_count: Option<Range>,
}

/// Decoded element counts of the calls where an interpretation was plausible.
#[derive(Debug, PartialEq)]
pub struct Range {
    pub min: u32,
    pub max: u32,
    pub calls: u32,
}

struct CountRange {
    min: AtomicU32,
    max: AtomicU32,
    calls: AtomicU32,
}

impl CountRange {
    const fn new() -> Self {
        Self {
            min: AtomicU32::new(u32::MAX),
            max: AtomicU32::new(0),
            calls: AtomicU32::new(0),
        }
    }

    fn add(&self, array: Option<rv::Array>) {
        if let Some(array) = array {
            self.min.fetch_min(array.count, Relaxed);
            self.max.fetch_max(array.count, Relaxed);
            self.calls.fetch_add(1, Relaxed);
        }
    }

    fn take(&self) -> Option<Range> {
        let range = Range {
            min: self.min.swap(u32::MAX, Relaxed),
            max: self.max.swap(0, Relaxed),
            calls: self.calls.swap(0, Relaxed),
        };
        (range.calls > 0).then_some(range)
    }
}

impl HookStats {
    pub const fn new() -> Self {
        Self {
            total_calls: AtomicU32::new(0),
            calls: AtomicU32::new(0),
            unreadable: AtomicU32::new(0),
            untracked: AtomicU32::new(0),
            window: AtomicU32::new(1),
            window_objects: AtomicU32::new(0),
            objects: [const { AtomicUsize::new(0) }; TRACKED],
            last_window: [const { AtomicU32::new(0) }; TRACKED],
            sighted: AtomicU32::new(0),
            begin_end: CountRange::new(),
            data_count: CountRange::new(),
        }
    }

    /// Records one call on `this`; `words` is `None` when the argument was
    /// not readable.
    pub fn record(&self, this: usize, words: Option<&Words>) -> Call {
        let number = self.total_calls.fetch_add(1, Relaxed);
        self.calls.fetch_add(1, Relaxed);
        match words {
            Some(words) => {
                self.begin_end.add(rv::begin_end(words));
                self.data_count.add(rv::data_count(words));
            }
            None => {
                self.unreadable.fetch_add(1, Relaxed);
            }
        }
        let Some((slot, first_sighting)) = self.slot_of(this) else {
            self.untracked.fetch_add(1, Relaxed);
            return Call {
                number,
                first_sighting: None,
            };
        };
        let window = self.window.load(Relaxed);
        if self.last_window[slot].swap(window, Relaxed) != window {
            self.window_objects.fetch_add(1, Relaxed);
        }
        Call { number, first_sighting }
    }

    /// Finds the table slot of `this` or claims a free one; a claim also
    /// returns the object's sighting index.
    fn slot_of(&self, this: usize) -> Option<(usize, Option<u32>)> {
        if this == 0 {
            return None;
        }
        // Fibonacci hashing of the 32-bit pointer: objects are aligned, so the
        // low bits alone would crowd into a few slots.
        let start = ((this as u32).wrapping_mul(0x9E37_79B9) >> 24) as usize;
        for probe in 0..TRACKED {
            let slot = (start + probe) % TRACKED;
            let mut current = self.objects[slot].load(Relaxed);
            if current == 0 {
                match self.objects[slot].compare_exchange(0, this, Relaxed, Relaxed) {
                    Ok(_) => return Some((slot, Some(self.sighted.fetch_add(1, Relaxed)))),
                    // Another thread claimed it meanwhile, maybe for this object.
                    Err(winner) => current = winner,
                }
            }
            if current == this {
                return Some((slot, None));
            }
        }
        None
    }

    /// Statistics since the previous call; starts the next window. Calls
    /// racing with this may be counted in either window.
    pub fn take_window(&self) -> Window {
        self.window.fetch_add(1, Relaxed);
        Window {
            calls: self.calls.swap(0, Relaxed),
            objects: self.window_objects.swap(0, Relaxed),
            total_objects: self.sighted.load(Relaxed),
            untracked: self.untracked.swap(0, Relaxed),
            unreadable: self.unreadable.swap(0, Relaxed),
            begin_end: self.begin_end.take(),
            data_count: self.data_count.take(),
        }
    }
}

impl fmt::Display for Range {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "count {}..={} in {} calls", self.min, self.max, self.calls)
    }
}

/// Everything but the call rate, which needs the window's duration.
impl fmt::Display for Window {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{} calls, {} objects ({} ever), {} untracked calls, {} unreadable args",
            self.calls, self.objects, self.total_objects, self.untracked, self.unreadable
        )?;
        for (label, range) in [("A", &self.begin_end), ("B", &self.data_count)] {
            match range {
                Some(range) => write!(f, "; {label} {range}")?,
                None => write!(f, "; {label} never plausible")?,
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BEGIN_END_3: Words = [0x2000, 0x200C, 0, 0];
    const DATA_COUNT_7: Words = [0x2000, 7, 0, 0];

    #[test]
    fn counts_calls_objects_and_ranges_per_window() {
        let stats = HookStats::new();
        let first = stats.record(0x1000, Some(&BEGIN_END_3));
        assert_eq!((first.number, first.first_sighting), (0, Some(0)));
        let again = stats.record(0x1000, Some(&DATA_COUNT_7));
        assert_eq!((again.number, again.first_sighting), (1, None));
        let other = stats.record(0x5000, None);
        assert_eq!((other.number, other.first_sighting), (2, Some(1)));
        stats.record(0x5000, Some(&[0x2000, 0x2040, 0, 0]));

        assert_eq!(
            stats.take_window(),
            Window {
                calls: 4,
                objects: 2,
                total_objects: 2,
                untracked: 0,
                unreadable: 1,
                begin_end: Some(Range {
                    min: 3,
                    max: 16,
                    calls: 2
                }),
                data_count: Some(Range {
                    min: 7,
                    max: 7,
                    calls: 1
                }),
            }
        );

        // A known object counts once in the new window and is no new sighting.
        let call = stats.record(0x5000, Some(&BEGIN_END_3));
        assert_eq!((call.number, call.first_sighting), (4, None));
        stats.record(0x5000, Some(&BEGIN_END_3));
        let window = stats.take_window();
        assert_eq!((window.calls, window.objects, window.total_objects), (2, 1, 2));
        assert_eq!(window.data_count, None);

        assert_eq!(
            stats.take_window(),
            Window {
                calls: 0,
                objects: 0,
                total_objects: 2,
                untracked: 0,
                unreadable: 0,
                begin_end: None,
                data_count: None,
            }
        );
    }

    #[test]
    fn objects_beyond_the_table_are_untracked() {
        let stats = HookStats::new();
        // Page-aligned pointers, like objects of a common size, collide in the
        // low bits; every one of them must still find a slot.
        for i in 0..TRACKED {
            let call = stats.record(0x1000_0000 + i * 0x1000, None);
            assert_eq!(call.first_sighting, Some(i as u32));
        }
        assert_eq!(stats.record(0x0FFF_0000, None).first_sighting, None);
        assert_eq!(stats.record(0, None).first_sighting, None);
        // Known objects are still found in a full table.
        assert_eq!(stats.record(0x1000_0000 + 0x1000 * 100, None).first_sighting, None);
        let window = stats.take_window();
        assert_eq!(
            (window.objects, window.total_objects, window.untracked),
            (TRACKED as u32, TRACKED as u32, 2)
        );
    }

    #[test]
    fn formats_a_window() {
        let window = Window {
            calls: 10,
            objects: 2,
            total_objects: 5,
            untracked: 0,
            unreadable: 1,
            begin_end: Some(Range {
                min: 20,
                max: 31,
                calls: 9,
            }),
            data_count: None,
        };
        assert_eq!(
            window.to_string(),
            "10 calls, 2 objects (5 ever), 0 untracked calls, 1 unreadable args; \
             A count 20..=31 in 9 calls; B never plausible"
        );
    }
}
