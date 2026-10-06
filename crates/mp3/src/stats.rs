//! Script statistics. The script records every frame; the plugin thread logs
//! and resets them every few seconds. Recording only touches atomics.

use std::fmt;
use std::sync::atomic::AtomicU32;
use std::sync::atomic::Ordering::Relaxed;

pub struct ScriptStats {
    frames: AtomicU32,
    native_calls: AtomicU32,
    max_native_calls: AtomicU32,
    published: AtomicU32,
}

/// Statistics of one window.
#[derive(Debug, PartialEq)]
pub struct Window {
    pub frames: u32,
    pub native_calls: u32,
    /// Most native calls in one frame.
    pub max_native_calls: u32,
    /// Pose frames published.
    pub published: u32,
}

impl ScriptStats {
    pub const fn new() -> Self {
        Self {
            frames: AtomicU32::new(0),
            native_calls: AtomicU32::new(0),
            max_native_calls: AtomicU32::new(0),
            published: AtomicU32::new(0),
        }
    }

    pub fn record_frame(&self, native_calls: u32, published: bool) {
        self.frames.fetch_add(1, Relaxed);
        self.native_calls.fetch_add(native_calls, Relaxed);
        self.max_native_calls.fetch_max(native_calls, Relaxed);
        self.published.fetch_add(published.into(), Relaxed);
    }

    /// Statistics since the previous call; frames racing with it may count
    /// in either window.
    pub fn take_window(&self) -> Window {
        Window {
            frames: self.frames.swap(0, Relaxed),
            native_calls: self.native_calls.swap(0, Relaxed),
            max_native_calls: self.max_native_calls.swap(0, Relaxed),
            published: self.published.swap(0, Relaxed),
        }
    }
}

impl fmt::Display for Window {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        if self.frames == 0 {
            return write!(f, "no script frames (game paused or script not running)");
        }
        write!(
            f,
            "{} frames, {:.1} native calls/frame (max {}), {} pose frames published",
            self.frames,
            f64::from(self.native_calls) / f64::from(self.frames),
            self.max_native_calls,
            self.published
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn windows_count_frames_calls_and_publications() {
        let stats = ScriptStats::new();
        stats.record_frame(1, false);
        stats.record_frame(76, true);
        stats.record_frame(150, true);
        let window = stats.take_window();
        assert_eq!(
            window,
            Window {
                frames: 3,
                native_calls: 227,
                max_native_calls: 150,
                published: 2
            }
        );
        assert_eq!(
            window.to_string(),
            "3 frames, 75.7 native calls/frame (max 150), 2 pose frames published"
        );
        let empty = stats.take_window();
        assert_eq!(empty.max_native_calls, 0);
        assert_eq!(
            empty.to_string(),
            "no script frames (game paused or script not running)"
        );
    }
}
