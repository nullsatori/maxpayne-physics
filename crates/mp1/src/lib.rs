//! Injected into Max Payne 1 as `mp1phys.asi`.
//!
//! Milestone 1: connect to the bridge and report when Max Payne 3 appears or
//! leaves. The heartbeat runs on its own thread for now; it moves into the
//! game frame hook once that is found, so a paused game stops beating.

#[cfg(windows)]
mod plugin {
    use std::time::Duration;

    use bridge::{Liveness, Mp1Side};
    use winmod::{Log, Module, log};

    const TICK: Duration = Duration::from_millis(50);
    /// Two seconds without a peer heartbeat means the peer is gone.
    const PEER_TIMEOUT_TICKS: u32 = 40;

    winmod::asi_entry!(run);

    fn run(module: Module) {
        let mut log = Log::create(&winmod::log_path(module));
        log!(
            log,
            "{} {} in {}",
            env!("CARGO_PKG_NAME"),
            env!("CARGO_PKG_VERSION"),
            winmod::module_path(Module::PROCESS).display()
        );
        for (module, path) in winmod::loaded_modules() {
            log!(log, "module {:#010x} {}", module.base(), path.display());
        }

        let link = match Mp1Side::open(winmod::current_pid()) {
            Ok(link) => link,
            Err(err) => {
                log!(log, "bridge unavailable: {err}");
                return;
            }
        };
        log!(log, "bridge open, waiting for Max Payne 3");
        let mut liveness = Liveness::new(PEER_TIMEOUT_TICKS);
        let mut connected = false;
        loop {
            link.beat();
            let alive = liveness.update(link.peer_heartbeat());
            if alive != connected {
                connected = alive;
                if alive {
                    log!(log, "Max Payne 3 connected (pid {})", link.peer_pid());
                } else {
                    log!(log, "Max Payne 3 disconnected");
                }
            }
            std::thread::sleep(TICK);
        }
    }
}
