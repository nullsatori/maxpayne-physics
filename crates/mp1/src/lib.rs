//! Injected into Max Payne 1 as `mp1phys.asi`. The game is 32-bit, so the
//! plugin only exists for x86 Windows.
//!
//! Milestone 1: connect to the bridge and report when Max Payne 3 appears or
//! leaves. The heartbeat runs on its own thread for now; it moves into the
//! game frame hook once that is found, so a paused game stops beating.
//!
//! Milestone 2 probes: list the executable's imports and e2mfc's exports next
//! to the DLL, and hook `P_VirtualObject::setBoneMatrices` to learn what the
//! engine passes for a character's bones.

// The plugin uses these on x86 Windows; elsewhere they build for host tests.
#[cfg_attr(not(all(windows, target_arch = "x86")), allow(dead_code))]
mod rv;
#[cfg_attr(not(all(windows, target_arch = "x86")), allow(dead_code))]
mod stats;

#[cfg(all(windows, target_arch = "x86"))]
mod hook;

#[cfg(all(windows, target_arch = "x86"))]
mod plugin {
    use std::time::{Duration, Instant};

    use bridge::{Liveness, Mp1Side};
    use winmod::{Module, log};

    use crate::hook::{self, E2MFC};

    const TICK: Duration = Duration::from_millis(50);
    /// Two seconds without a peer heartbeat means the peer is gone.
    const PEER_TIMEOUT_TICKS: u32 = 40;
    const MODULE_WAIT: Duration = Duration::from_secs(60);
    const STATS_PERIOD: Duration = Duration::from_secs(5);

    winmod::asi_entry!(run);

    fn run(module: Module) {
        winmod::init_log(&winmod::log_path(module));
        log!(
            "{} {} in {}",
            env!("CARGO_PKG_NAME"),
            env!("CARGO_PKG_VERSION"),
            winmod::module_path(Module::PROCESS).display()
        );
        for (module, path) in winmod::loaded_modules() {
            log!("module {:#010x} {}", module.base(), path.display());
        }

        let hooked = probe(module);

        let link = match Mp1Side::open(winmod::current_pid()) {
            Ok(link) => {
                log!("bridge open, waiting for Max Payne 3");
                Some(link)
            }
            Err(err) => {
                log!("bridge unavailable: {err}");
                None
            }
        };
        if link.is_none() && !hooked {
            return;
        }
        let mut liveness = Liveness::new(PEER_TIMEOUT_TICKS);
        let mut connected = false;
        let mut window_start = Instant::now();
        loop {
            if let Some(link) = &link {
                link.beat();
                let alive = liveness.update(link.peer_heartbeat());
                if alive != connected {
                    connected = alive;
                    if alive {
                        log!("Max Payne 3 connected (pid {})", link.peer_pid());
                    } else {
                        log!("Max Payne 3 disconnected");
                    }
                }
            }
            if hooked && window_start.elapsed() >= STATS_PERIOD {
                let seconds = window_start.elapsed().as_secs_f64();
                window_start = Instant::now();
                let window = hook::STATS.take_window();
                log!(
                    "setBoneMatrices: {:.1} calls/s, {window}",
                    f64::from(window.calls) / seconds
                );
            }
            std::thread::sleep(TICK);
        }
    }

    /// Writes the listings and installs the hook; returns whether it is in place.
    fn probe(module: Module) -> bool {
        let Some(exe) = winmod::module_handle(None) else {
            log!("no handle for the executable; running without probes");
            return false;
        };
        // SAFETY: the executable stays mapped for the life of the process.
        let exe_imports = match unsafe { peimage::Image::new(exe.base()) } {
            Ok(image) => image.imports(),
            Err(err) => {
                log!("cannot read the executable's PE headers: {err:?}; running without probes");
                return false;
            }
        };
        winmod::write_listing(module, "exe-imports.txt", &exe_imports);

        let Some(e2mfc) = winmod::wait_for_module(E2MFC, MODULE_WAIT) else {
            log!(
                "{E2MFC} not loaded after {} s; running without probes",
                MODULE_WAIT.as_secs()
            );
            return false;
        };
        // SAFETY: the executable imports from e2mfc, so it stays loaded.
        match unsafe { peimage::Image::new(e2mfc.base()) } {
            Ok(image) => {
                let mut exports = image.exports();
                exports.sort_by(|a, b| a.name.cmp(&b.name));
                winmod::write_listing(module, "e2mfc-exports.txt", &exports);
            }
            Err(err) => log!("cannot read {E2MFC}'s PE headers: {err:?}"),
        }

        let switch = winmod::side_path(module, "nohook");
        if switch.exists() {
            log!("{} exists; not hooking", switch.display());
            return false;
        }
        hook::install(&exe_imports)
    }
}
