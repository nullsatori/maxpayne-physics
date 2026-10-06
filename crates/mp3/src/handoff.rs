//! Who beats Max Payne 3's heartbeat.
//!
//! The plugin thread beats from startup, so Max Payne 1 sees the bridge at
//! once. When the script runs, the heartbeat moves to it, once per game
//! frame, so a paused or hung game stops beating. Exactly one side beats at a
//! time: the script only asks, and the plugin thread hands over between two of
//! its own beats.

use std::sync::atomic::AtomicU8;
use std::sync::atomic::Ordering::{AcqRel, Acquire, Release};

const PLUGIN: u8 = 0;
const REQUESTED: u8 = 1;
const SCRIPT: u8 = 2;

pub struct Handoff(AtomicU8);

/// What the plugin thread does on a tick.
#[derive(Debug, PartialEq)]
pub enum PluginTick {
    Beat,
    /// The script asked; it beats from its next frame on.
    HandOver,
    /// The script beats.
    Idle,
}

impl Handoff {
    pub const fn new() -> Self {
        Self(AtomicU8::new(PLUGIN))
    }

    /// Called by the plugin thread every tick.
    pub fn plugin_tick(&self) -> PluginTick {
        match self.0.load(Acquire) {
            PLUGIN => PluginTick::Beat,
            REQUESTED => {
                // Released after this thread's last beat, which the script's
                // first beat therefore follows.
                self.0.store(SCRIPT, Release);
                PluginTick::HandOver
            }
            _ => PluginTick::Idle,
        }
    }

    /// Called by the script every frame; returns whether it beats now. The
    /// first call asks for the heartbeat, which arrives on a later frame.
    pub fn script_frame(&self) -> bool {
        match self.0.compare_exchange(PLUGIN, REQUESTED, AcqRel, Acquire) {
            Ok(_) => false,
            Err(state) => state == SCRIPT,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
    use std::time::{Duration, Instant};

    use super::*;

    #[test]
    fn plugin_beats_until_it_hands_over() {
        let handoff = Handoff::new();
        assert_eq!(handoff.plugin_tick(), PluginTick::Beat);
        assert_eq!(handoff.plugin_tick(), PluginTick::Beat);
        assert!(!handoff.script_frame(), "the first frame only asks");
        assert!(!handoff.script_frame(), "not handed over yet");
        assert_eq!(handoff.plugin_tick(), PluginTick::HandOver);
        assert!(handoff.script_frame());
        assert_eq!(handoff.plugin_tick(), PluginTick::Idle);
        assert!(handoff.script_frame());
        assert_eq!(handoff.plugin_tick(), PluginTick::Idle);
    }

    #[test]
    fn never_two_beaters_at_once() {
        let handoff = Handoff::new();
        let beating = AtomicBool::new(false);
        let overlaps = AtomicU32::new(0);
        let script_beats = AtomicU32::new(0);
        // A broken handoff fails the test rather than hanging it.
        let deadline = Instant::now() + Duration::from_secs(10);
        let running = || script_beats.load(Ordering::SeqCst) < 1000 && Instant::now() < deadline;
        let beat = || {
            if beating.swap(true, Ordering::SeqCst) {
                overlaps.fetch_add(1, Ordering::SeqCst);
            }
            for _ in 0..50 {
                std::hint::spin_loop();
            }
            beating.store(false, Ordering::SeqCst);
        };
        std::thread::scope(|scope| {
            scope.spawn(|| {
                while running() {
                    if handoff.plugin_tick() == PluginTick::Beat {
                        beat();
                    }
                }
            });
            scope.spawn(|| {
                while running() {
                    if handoff.script_frame() {
                        beat();
                        script_beats.fetch_add(1, Ordering::SeqCst);
                    }
                }
            });
        });
        assert_eq!(overlaps.load(Ordering::SeqCst), 0, "two beaters at once");
        assert_eq!(
            script_beats.load(Ordering::SeqCst),
            1000,
            "the script never got the heartbeat"
        );
        assert_eq!(handoff.plugin_tick(), PluginTick::Idle);
    }
}
