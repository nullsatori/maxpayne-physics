//! The script Script Hook runs on the game's script fiber, the only place
//! natives may be called. It never waits for anything: each frame does a
//! bounded amount of work and yields with `scriptWait(0)`.

use std::panic::AssertUnwindSafe;
use std::path::PathBuf;
use std::sync::OnceLock;

use bridge::{ActorPose, Command, Mp3Side};
use scripthook::{Api, BODY_BONES, Natives, Ped, Script, body_bone_index};
use winmod::log;

use crate::frames::{self, BoneSample, Classifier, Semantics};
use crate::handoff::Handoff;
use crate::puppets::{self, Action, MAX_PUPPETS, MODEL_TIMEOUT_FRAMES, PLAYER_ACTOR, Puppet, Puppets, Slot};
use crate::stats::ScriptStats;

pub static HANDOFF: Handoff = Handoff::new();
pub static STATS: ScriptStats = ScriptStats::new();
/// Set by the plugin thread before it registers the script.
pub static SETUP: OnceLock<Setup> = OnceLock::new();

pub struct Setup {
    pub api: Api,
    pub pid: u32,
    pub skeleton_path: PathBuf,
}

/// `scriptWait(INFINITE)` is how the SDK's `TERMINATE()` ends a script.
const INFINITE: u32 = u32::MAX;
/// Commands carried out per frame, which bounds their native calls.
const COMMANDS_PER_FRAME: usize = 16;

// Neither the SDK nor anything else documents what these parameters mean in
// Max Payne 3. The values are guesses that do the least, after what the same
// natives take in other RAGE games (GTA IV and V); every call logs them so
// the first run in the game can confirm or correct them.

/// `CREATE_PED` pedType: a civilian male in GTA IV and V.
const PED_TYPE: i32 = 4;
/// `SET_PED_TO_RAGDOLL` time1 and time2, milliseconds: one second.
const RAGDOLL_MS: i32 = 1000;
/// `SET_PED_TO_RAGDOLL` p3, the ragdoll type in GTA V, where 0 relaxes the
/// ped. p4 to p6 are passed false and p7 zero.
const RAGDOLL_TYPE: i32 = 0;
/// `APPLY_FORCE_TO_PED` forceType: a one-off impulse in GTA V's numbering,
/// not a force that keeps acting. The force goes in world axes
/// (isDirectionRel false), whole (ignoreUpVec false), not scaled by mass
/// (isForceRel false), at the bone itself: whether the offset is in world or
/// bone axes is unknown, so the hit point is only logged.
const FORCE_TYPE: i32 = 1;

/// Script Hook's entry point; once set up, it never returns.
pub extern "C" fn entry() {
    let Some(setup) = SETUP.get() else {
        log!("script started before the plugin set it up; ending it");
        return;
    };
    log!("script running on thread {:?}", std::thread::current().id());
    // SAFETY: Script Hook calls this on the fiber of the script registered
    // with setup.api.
    let script = unsafe { setup.api.script() };
    // A panic must not unwind into Script Hook; winmod's panic hook logs it.
    let _ = std::panic::catch_unwind(AssertUnwindSafe(|| run(setup, script)));
    log!("script stopped");
    // SAFETY: as above.
    let script = unsafe { setup.api.script() };
    loop {
        script.wait(INFINITE);
    }
}

fn run(setup: &Setup, script: Script) {
    // This link is the only one that publishes poses and takes commands; the
    // plugin thread keeps its own for the heartbeat.
    let link = match Mp3Side::open(setup.pid) {
        Ok(link) => link,
        Err(err) => {
            log!("script: bridge unavailable: {err}");
            return;
        }
    };
    let mut natives = Natives::new(script);
    let mut state = State::new(setup.skeleton_path.clone());
    loop {
        let published = state.frame(&mut natives, &link);
        STATS.record_frame(natives.take_calls(), published);
        natives.backend().wait(0);
    }
}

struct State {
    puppets: Puppets,
    classifier: Classifier,
    /// The semantics last logged.
    semantics: Semantics,
    /// The player ped last logged.
    player: Option<Ped>,
    root: usize,
    samples: Vec<BoneSample>,
    /// The player, then live puppets.
    poses: Vec<ActorPose>,
    game_time: f64,
    /// Taken when the skeleton is written.
    skeleton_path: Option<PathBuf>,
}

impl State {
    fn new(skeleton_path: PathBuf) -> Self {
        let parents: Vec<Option<usize>> = BODY_BONES.iter().map(|bone| bone.parent).collect();
        Self {
            puppets: Puppets::default(),
            classifier: Classifier::new(&parents),
            semantics: Semantics::Unknown,
            player: None,
            root: body_bone_index("ROOT").expect("ROOT is a body bone"),
            samples: Vec::with_capacity(BODY_BONES.len()),
            poses: vec![ActorPose::new(0, 0, &[]); 1 + MAX_PUPPETS],
            game_time: 0.0,
            skeleton_path: Some(skeleton_path),
        }
    }

    /// One game frame; returns whether poses were published.
    fn frame(&mut self, natives: &mut Natives<Script>, link: &Mp3Side) -> bool {
        if HANDOFF.script_frame() {
            link.beat();
        }
        let player = if natives.does_main_player_exist() {
            let id = natives.get_player_id();
            Some(natives.get_player_ped(id)).filter(|&ped| natives.does_ped_exist(ped))
        } else {
            None
        };
        if player != self.player {
            self.player = player;
            match player {
                Some(ped) => log!("player ped {ped}: streaming its body bones as actor {PLAYER_ACTOR}"),
                None => log!("no main player; waiting"),
            }
        }
        let Some(player) = player else {
            return false;
        };
        self.game_time += f64::from(natives.timestep());

        frames::sample_ped(natives, player, &BODY_BONES, &mut self.samples);
        let semantics = self.classifier.observe(&self.samples);
        frames::fill_actor_pose(&mut self.poses[0], PLAYER_ACTOR, &self.samples, semantics);
        if semantics != self.semantics {
            self.semantics = semantics;
            match semantics {
                Semantics::BoneLocal => log!("GET_PED_BONE_COORDS offsets are bone-local: publishing bone rotations"),
                Semantics::World => {
                    log!("GET_PED_BONE_COORDS offsets are in world axes: no bone rotations, publishing positions only")
                }
                Semantics::Unknown => {}
            }
            self.write_skeleton(semantics);
        }
        let mut count = 1;
        for slot in &mut self.puppets.slots {
            let Some(Slot {
                actor,
                puppet: Puppet::Live { ped },
            }) = *slot
            else {
                continue;
            };
            if !natives.does_ped_exist(ped) {
                log!("actor {actor}: ped {ped} no longer exists; dropped");
                *slot = None;
                continue;
            }
            frames::sample_ped(natives, ped, &BODY_BONES, &mut self.samples);
            frames::fill_actor_pose(&mut self.poses[count], actor, &self.samples, semantics);
            count += 1;
        }
        link.publish_poses(self.game_time, &self.poses[..count]);

        for _ in 0..COMMANDS_PER_FRAME {
            let Some(command) = link.receive() else {
                break;
            };
            self.command(natives, command, player);
        }
        self.load_puppets(natives, self.poses[0].bones[self.root].translation);
        true
    }

    fn write_skeleton(&mut self, semantics: Semantics) {
        let Some(path) = self.skeleton_path.take() else {
            return;
        };
        match frames::skeleton_text(&BODY_BONES, &self.poses[0], semantics) {
            Ok(text) => match std::fs::write(&path, text) {
                Ok(()) => log!("wrote the player skeleton to {}", path.display()),
                Err(err) => log!("cannot write {}: {err}", path.display()),
            },
            Err(err) => log!("player skeleton not written: {err}"),
        }
    }

    fn command(&mut self, natives: &mut Natives<Script>, command: Command, player: Ped) {
        let action = match self.puppets.validate(command, player) {
            Ok(action) => action,
            Err(rejection) => {
                log!("{command:?} refused: {rejection}");
                return;
            }
        };
        match action {
            Action::SetTimeScale(scale) => {
                natives.set_time_scale(scale);
                log!("{command:?}: SET_TIME_SCALE({scale})");
            }
            Action::Spawn { slot, actor, model } => {
                natives.request_model(model);
                self.puppets.slots[slot] = Some(Slot {
                    actor,
                    puppet: Puppet::Loading { model, frames: 0 },
                });
                log!("{command:?}: REQUEST_MODEL({model:#010x}) for slot {slot}");
            }
            Action::CancelSpawn { slot, model } => {
                natives.set_model_as_no_longer_needed(model);
                self.puppets.slots[slot] = None;
                log!("{command:?}: model not loaded yet; SET_MODEL_AS_NO_LONGER_NEEDED({model:#010x})");
            }
            Action::Delete { slot, ped } => {
                let mut handle = ped;
                natives.delete_ped(&mut handle);
                self.puppets.slots[slot] = None;
                log!("{command:?}: DELETE_PED({ped}), handle afterwards {handle}");
            }
            Action::Hit {
                ped,
                bone,
                point,
                impulse,
            } => {
                let bone = BODY_BONES[bone];
                let ragdolled =
                    natives.set_ped_to_ragdoll(ped, RAGDOLL_MS, RAGDOLL_MS, RAGDOLL_TYPE, false, false, false, 0.0);
                let tag = i32::from(bone.tag);
                natives.apply_force_to_ped(ped, FORCE_TYPE, impulse, [0.0; 3], tag, false, false, false);
                log!(
                    "{command:?}: SET_PED_TO_RAGDOLL({ped}, {RAGDOLL_MS}, {RAGDOLL_MS}, {RAGDOLL_TYPE}, false, false, \
                     false, 0) = {ragdolled}; APPLY_FORCE_TO_PED({ped}, {FORCE_TYPE}, {impulse:?}, [0, 0, 0], \
                     {tag:#06x} ({}), false, false, false); hit point {point:?} only logged",
                    bone.name
                );
            }
        }
    }

    /// Creates the puppets whose models have loaded, next to the player.
    fn load_puppets(&mut self, natives: &mut Natives<Script>, player_root: [f32; 3]) {
        for (index, slot) in self.puppets.slots.iter_mut().enumerate() {
            let Some(Slot {
                actor,
                puppet: Puppet::Loading { model, frames },
            }) = *slot
            else {
                continue;
            };
            if natives.has_model_loaded(model) {
                // At the height of the player's ROOT, assumed to be the ped's
                // origin as in other RAGE games.
                let Some(position) = puppets::spawn_position(player_root, index) else {
                    natives.set_model_as_no_longer_needed(model);
                    *slot = None;
                    log!("actor {actor}: the player's ROOT {player_root:?} is not finite; spawn given up");
                    continue;
                };
                let ped = natives.create_ped(PED_TYPE, model, position, 0.0, false, false);
                natives.set_model_as_no_longer_needed(model);
                let call = format!("CREATE_PED({PED_TYPE}, {model:#010x}, {position:?}, 0, false, false) = {ped}");
                if natives.does_ped_exist(ped) {
                    natives.set_ped_invincible(ped, true);
                    *slot = Some(Slot {
                        actor,
                        puppet: Puppet::Live { ped },
                    });
                    log!("actor {actor}: {call} after {frames} frames; invincible, streaming its pose");
                } else {
                    *slot = None;
                    log!("actor {actor}: {call}, which does not exist; spawn given up");
                }
            } else if frames >= MODEL_TIMEOUT_FRAMES {
                natives.set_model_as_no_longer_needed(model);
                *slot = None;
                log!("actor {actor}: model {model:#010x} not loaded after {frames} frames; spawn given up");
            } else {
                *slot = Some(Slot {
                    actor,
                    puppet: Puppet::Loading {
                        model,
                        frames: frames + 1,
                    },
                });
            }
        }
    }
}
