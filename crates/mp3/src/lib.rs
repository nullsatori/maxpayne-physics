//! Injected into Max Payne 3 as `mp3bridge.asi`. The game is 32-bit, so the
//! plugin only exists for x86 Windows.
//!
//! The plugin thread opens the bridge and beats the heartbeat at once, then
//! waits for Script Hook, binds it and registers a script. Every game frame
//! the script samples the body bones of the player's ped (actor 0) and of the
//! puppets, publishes their poses and carries out Max Payne 1's commands.
//! Once it runs, it beats the heartbeat itself ([`handoff`]).

// The plugin uses these on x86 Windows; elsewhere they build for host tests.
#[cfg_attr(not(all(windows, target_arch = "x86")), allow(dead_code))]
mod frames;
#[cfg_attr(not(all(windows, target_arch = "x86")), allow(dead_code))]
mod handoff;
#[cfg_attr(not(all(windows, target_arch = "x86")), allow(dead_code))]
mod puppets;
#[cfg_attr(not(all(windows, target_arch = "x86")), allow(dead_code))]
mod stats;

#[cfg(all(windows, target_arch = "x86"))]
mod script;

#[cfg(all(windows, target_arch = "x86"))]
mod plugin {
    use std::time::{Duration, Instant};

    use bridge::{Liveness, Mp3Side};
    use scripthook::Api;
    use winmod::{Module, log};

    use crate::handoff::PluginTick;
    use crate::script::{self, HANDOFF, SETUP, STATS, Setup};

    const TICK: Duration = Duration::from_millis(50);
    /// Two seconds without a peer heartbeat means the peer is gone.
    const PEER_TIMEOUT_TICKS: u32 = 40;
    const SCRIPT_HOOK: &str = "ScriptHook.dll";
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

        let link = match Mp3Side::open(winmod::current_pid()) {
            Ok(link) => link,
            Err(err) => {
                log!("bridge unavailable: {err}");
                return;
            }
        };
        log!("bridge open (protocol v{}), waiting for Max Payne 1", bridge::VERSION);
        let started = Instant::now();
        let mut waiting_for_script_hook = true;
        let mut next_stats: Option<Instant> = None;
        let mut liveness = Liveness::new(PEER_TIMEOUT_TICKS);
        let mut connected = false;
        loop {
            match HANDOFF.plugin_tick() {
                PluginTick::Beat => link.beat(),
                PluginTick::HandOver => log!("the script beats the heartbeat from now on, once per game frame"),
                PluginTick::Idle => {}
            }
            let alive = liveness.update(link.peer_heartbeat());
            if alive != connected {
                connected = alive;
                if alive {
                    log!("Max Payne 1 connected (pid {})", link.peer_pid());
                } else {
                    log!("Max Payne 1 disconnected");
                }
            }
            if waiting_for_script_hook {
                if let Some(script_hook) = winmod::module_handle(Some(SCRIPT_HOOK)) {
                    waiting_for_script_hook = false;
                    if register(module, script_hook) {
                        next_stats = Some(Instant::now() + STATS_PERIOD);
                    }
                } else if started.elapsed() >= MODULE_WAIT {
                    waiting_for_script_hook = false;
                    log!("{SCRIPT_HOOK} not loaded after {} s; no script", MODULE_WAIT.as_secs());
                }
            }
            if let Some(due) = next_stats
                && Instant::now() >= due
            {
                next_stats = Some(due + STATS_PERIOD);
                log!("script: {}", STATS.take_window());
            }
            std::thread::sleep(TICK);
        }
    }

    /// Lists Script Hook's exports, binds the functions the script needs and
    /// registers it; returns whether the script is registered.
    fn register(module: Module, script_hook: Module) -> bool {
        // SAFETY: Script Hook loads the ASI plugins and stays loaded while the
        // game runs.
        let image = match unsafe { peimage::Image::new(script_hook.base()) } {
            Ok(image) => image,
            Err(err) => {
                log!("cannot read {SCRIPT_HOOK}'s PE headers: {err:?}; no script");
                return false;
            }
        };
        let mut exports = image.exports();
        exports.sort_by(|a, b| a.name.cmp(&b.name));
        winmod::write_listing(module, "scripthook-exports.txt", &exports);
        // SAFETY: as above; the exports are this image's.
        let api = match unsafe { Api::bind(&exports, script_hook.base()) } {
            Ok(api) => api,
            Err(err) => {
                log!("{SCRIPT_HOOK}: {err}; no script");
                return false;
            }
        };
        log!("bound scriptRegister, scriptWait, nativeInit, nativePush32 and nativeCall of {SCRIPT_HOOK}");
        let setup = Setup {
            api,
            pid: winmod::current_pid(),
            skeleton_path: winmod::side_path(module, "player-skeleton.txt"),
            registrar: std::thread::current().id(),
        };
        if SETUP.set(setup).is_err() {
            log!("script set up twice; not registering again");
            return false;
        }
        log!(
            "calling scriptRegister from the plugin thread ({:?}); the SDK sample registers in DllMain, so \
             registering later from another thread is unverified: without a 'script running' line below, it did \
             not work",
            std::thread::current().id()
        );
        // SAFETY: the module is this DLL, loaded for the life of the game;
        // script::entry catches its panics.
        unsafe { api.register(module.base(), script::entry) };
        log!("scriptRegister returned");
        true
    }
}
