//! Injected into Max Payne 3 as `mp3bridge.asi`. The game is 32-bit, so the
//! plugin only exists for x86 Windows.
//!
//! Milestone 1: connect to the bridge and report when Max Payne 1 appears or
//! leaves. The heartbeat runs on its own thread for now; it moves into the
//! game frame hook once that is found, so a paused game stops beating.
//!
//! Milestone 2 probes: list the executable's imports and Script Hook's
//! exports next to the DLL. Nothing is hooked.

#[cfg(all(windows, target_arch = "x86"))]
mod plugin {
    use std::time::Duration;

    use bridge::{Liveness, Mp3Side};
    use winmod::{Module, log};

    const TICK: Duration = Duration::from_millis(50);
    /// Two seconds without a peer heartbeat means the peer is gone.
    const PEER_TIMEOUT_TICKS: u32 = 40;
    const SCRIPT_HOOK: &str = "ScriptHook.dll";
    const MODULE_WAIT: Duration = Duration::from_secs(60);

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

        probe(module);

        let link = match Mp3Side::open(winmod::current_pid()) {
            Ok(link) => link,
            Err(err) => {
                log!("bridge unavailable: {err}");
                return;
            }
        };
        log!("bridge open, waiting for Max Payne 1");
        let mut liveness = Liveness::new(PEER_TIMEOUT_TICKS);
        let mut connected = false;
        loop {
            link.beat();
            let alive = liveness.update(link.peer_heartbeat());
            if alive != connected {
                connected = alive;
                if alive {
                    log!("Max Payne 1 connected (pid {})", link.peer_pid());
                } else {
                    log!("Max Payne 1 disconnected");
                }
            }
            std::thread::sleep(TICK);
        }
    }

    fn probe(module: Module) {
        match winmod::module_handle(None) {
            // SAFETY: the executable stays mapped for the life of the process.
            Some(exe) => match unsafe { peimage::Image::new(exe.base()) } {
                Ok(image) => winmod::write_listing(module, "exe-imports.txt", &image.imports()),
                Err(err) => log!("cannot read the executable's PE headers: {err:?}"),
            },
            None => log!("no handle for the executable"),
        }

        let Some(script_hook) = winmod::wait_for_module(SCRIPT_HOOK, MODULE_WAIT) else {
            log!("{SCRIPT_HOOK} not loaded after {} s", MODULE_WAIT.as_secs());
            return;
        };
        // SAFETY: read right after the module was found; Script Hook stays
        // loaded while the game runs.
        match unsafe { peimage::Image::new(script_hook.base()) } {
            Ok(image) => {
                let mut exports = image.exports();
                exports.sort_by(|a, b| a.name.cmp(&b.name));
                winmod::write_listing(module, "scripthook-exports.txt", &exports);
            }
            Err(err) => log!("cannot read {SCRIPT_HOOK}'s PE headers: {err:?}"),
        }
    }
}
