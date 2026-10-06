//! Puppets: peds created for Max Payne 1's characters, and which of MP1's
//! commands are valid for them. Calling the natives is left to the script.

use std::fmt;

use bridge::Command;
use scripthook::{BODY_BONES, Hash, Ped};

/// Actor 0 is the player's ped; MP1 picks the ids of puppets.
pub const PLAYER_ACTOR: u32 = 0;
/// Most puppets at once. Sampling a ped takes 3 `GET_PED_BONE_COORDS` per
/// body bone plus a `DOES_PED_EXIST` every frame, so with the player the cap
/// keeps sampling to (1 + 4) * 73 = 365 native calls per frame.
pub const MAX_PUPPETS: usize = 4;
/// Frames to wait for a requested model before giving the spawn up.
pub const MODEL_TIMEOUT_FRAMES: u32 = 600;
/// Distance between the player and the first puppet, and between puppets.
const SPAWN_SPACING: f32 = 2.0;

/// `ePedHash::amb_man_avg` in `enums.h` of the plugin SDK
/// (github.com/alloc8or/mp3-plugin-sdk): an ambient man.
const AMB_MAN_AVG: Hash = 0x647F_FDD9;

/// The MP3 model of an MP1 character kind. A placeholder until MP1's
/// characters get MP3 models: the only kind, 0, is an ambient man.
pub fn model_for_kind(kind: u32) -> Option<Hash> {
    match kind {
        0 => Some(AMB_MAN_AVG),
        _ => None,
    }
}

/// Where the puppet in `slot` is created: along world x from the player's
/// root, which is horizontal whether the game's up axis is y or z.
pub fn spawn_position(player_root: [f32; 3], slot: usize) -> [f32; 3] {
    let [x, y, z] = player_root;
    [x + SPAWN_SPACING * (slot + 1) as f32, y, z]
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Puppet {
    /// The model is requested; `frames` counts the frames waited for it.
    Loading {
        model: Hash,
        frames: u32,
    },
    Live {
        ped: Ped,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Slot {
    pub actor: u32,
    pub puppet: Puppet,
}

/// Puppets in fixed slots; a puppet's slot also decides where it is created.
#[derive(Default)]
pub struct Puppets {
    pub slots: [Option<Slot>; MAX_PUPPETS],
}

/// What a valid command asks of the game.
#[derive(Debug, PartialEq)]
pub enum Action {
    SetTimeScale(f32),
    /// Request the model of a new puppet that goes into `slot`.
    Spawn {
        slot: usize,
        actor: u32,
        model: Hash,
    },
    /// Despawn of a puppet whose model has not loaded: release the model.
    CancelSpawn {
        slot: usize,
        model: Hash,
    },
    Delete {
        slot: usize,
        ped: Ped,
    },
    /// `bone` indexes [`BODY_BONES`]. The player takes hits too: MP1's Max
    /// is to be mirrored by MP3's.
    Hit {
        ped: Ped,
        bone: usize,
        point: [f32; 3],
        impulse: [f32; 3],
    },
}

#[derive(Debug, PartialEq)]
pub enum Rejection {
    PlayerActor,
    ActorInUse,
    NoFreeSlot,
    UnknownKind,
    UnknownActor,
    StillLoading,
    UnknownBone,
    NotFinite,
    TimeScaleRange,
}

impl fmt::Display for Rejection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Rejection::PlayerActor => write!(f, "actor {PLAYER_ACTOR} is the player"),
            Rejection::ActorInUse => write!(f, "the actor already has a puppet"),
            Rejection::NoFreeSlot => write!(f, "all {MAX_PUPPETS} puppets exist"),
            Rejection::UnknownKind => write!(f, "no model for that kind"),
            Rejection::UnknownActor => write!(f, "no such actor"),
            Rejection::StillLoading => write!(f, "the puppet's model is still loading"),
            Rejection::UnknownBone => write!(f, "the body bone table has {} bones", BODY_BONES.len()),
            Rejection::NotFinite => write!(f, "a value is not finite"),
            Rejection::TimeScaleRange => write!(f, "time scale must be in (0, 1]"),
        }
    }
}

impl Puppets {
    pub fn find(&self, actor: u32) -> Option<(usize, Puppet)> {
        self.slots
            .iter()
            .enumerate()
            .find_map(|(index, slot)| slot.filter(|slot| slot.actor == actor).map(|slot| (index, slot.puppet)))
    }

    /// Checks `command` against the puppets; `player` is the player's ped.
    pub fn validate(&self, command: Command, player: Ped) -> Result<Action, Rejection> {
        match command {
            Command::TimeScale { scale } => {
                if !scale.is_finite() {
                    Err(Rejection::NotFinite)
                } else if scale > 0.0 && scale <= 1.0 {
                    Ok(Action::SetTimeScale(scale))
                } else {
                    // Bullet time only slows down; what SET_TIME_SCALE does above 1 is unknown.
                    Err(Rejection::TimeScaleRange)
                }
            }
            Command::Spawn { actor, kind } => {
                if actor == PLAYER_ACTOR {
                    return Err(Rejection::PlayerActor);
                }
                if self.find(actor).is_some() {
                    return Err(Rejection::ActorInUse);
                }
                let slot = self
                    .slots
                    .iter()
                    .position(Option::is_none)
                    .ok_or(Rejection::NoFreeSlot)?;
                let model = model_for_kind(kind).ok_or(Rejection::UnknownKind)?;
                Ok(Action::Spawn { slot, actor, model })
            }
            Command::Despawn { actor } => {
                if actor == PLAYER_ACTOR {
                    return Err(Rejection::PlayerActor);
                }
                match self.find(actor).ok_or(Rejection::UnknownActor)? {
                    (slot, Puppet::Loading { model, .. }) => Ok(Action::CancelSpawn { slot, model }),
                    (slot, Puppet::Live { ped }) => Ok(Action::Delete { slot, ped }),
                }
            }
            Command::Hit {
                actor,
                bone,
                point,
                impulse,
            } => {
                let ped = if actor == PLAYER_ACTOR {
                    player
                } else {
                    match self.find(actor).ok_or(Rejection::UnknownActor)? {
                        (_, Puppet::Live { ped }) => ped,
                        (_, Puppet::Loading { .. }) => return Err(Rejection::StillLoading),
                    }
                };
                let bone = bone as usize;
                if bone >= BODY_BONES.len() {
                    return Err(Rejection::UnknownBone);
                }
                if !point.iter().chain(&impulse).all(|v| v.is_finite()) {
                    return Err(Rejection::NotFinite);
                }
                Ok(Action::Hit {
                    ped,
                    bone,
                    point,
                    impulse,
                })
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PLAYER: Ped = 0x101;

    fn puppets() -> Puppets {
        let mut puppets = Puppets::default();
        puppets.slots[0] = Some(Slot {
            actor: 5,
            puppet: Puppet::Live { ped: 0x202 },
        });
        puppets.slots[2] = Some(Slot {
            actor: 6,
            puppet: Puppet::Loading {
                model: AMB_MAN_AVG,
                frames: 3,
            },
        });
        puppets
    }

    #[test]
    fn spawns_take_a_free_slot_for_a_known_kind() {
        let mut puppets = puppets();
        let spawn = |actor, kind| Command::Spawn { actor, kind };
        assert_eq!(
            puppets.validate(spawn(9, 0), PLAYER),
            Ok(Action::Spawn {
                slot: 1,
                actor: 9,
                model: AMB_MAN_AVG
            })
        );
        assert_eq!(puppets.validate(spawn(0, 0), PLAYER), Err(Rejection::PlayerActor));
        assert_eq!(puppets.validate(spawn(5, 0), PLAYER), Err(Rejection::ActorInUse));
        assert_eq!(puppets.validate(spawn(6, 0), PLAYER), Err(Rejection::ActorInUse));
        assert_eq!(puppets.validate(spawn(9, 1), PLAYER), Err(Rejection::UnknownKind));
        puppets.slots[1] = puppets.slots[0].map(|slot| Slot { actor: 7, ..slot });
        puppets.slots[3] = puppets.slots[0].map(|slot| Slot { actor: 8, ..slot });
        assert_eq!(puppets.validate(spawn(9, 0), PLAYER), Err(Rejection::NoFreeSlot));
    }

    #[test]
    fn despawn_deletes_or_cancels() {
        let puppets = puppets();
        let despawn = |actor| Command::Despawn { actor };
        assert_eq!(
            puppets.validate(despawn(5), PLAYER),
            Ok(Action::Delete { slot: 0, ped: 0x202 })
        );
        assert_eq!(
            puppets.validate(despawn(6), PLAYER),
            Ok(Action::CancelSpawn {
                slot: 2,
                model: AMB_MAN_AVG
            })
        );
        assert_eq!(puppets.validate(despawn(0), PLAYER), Err(Rejection::PlayerActor));
        assert_eq!(puppets.validate(despawn(9), PLAYER), Err(Rejection::UnknownActor));
    }

    #[test]
    fn hits_need_a_live_actor_a_body_bone_and_finite_vectors() {
        let puppets = puppets();
        let hit = |actor, bone, point: [f32; 3], impulse: [f32; 3]| Command::Hit {
            actor,
            bone,
            point,
            impulse,
        };
        let (point, impulse) = ([1.0, 2.0, 3.0], [0.0, -40.0, 5.0]);
        assert_eq!(
            puppets.validate(hit(5, 7, point, impulse), PLAYER),
            Ok(Action::Hit {
                ped: 0x202,
                bone: 7,
                point,
                impulse
            })
        );
        assert_eq!(
            puppets.validate(hit(0, 23, point, impulse), PLAYER),
            Ok(Action::Hit {
                ped: PLAYER,
                bone: 23,
                point,
                impulse
            })
        );
        assert_eq!(
            puppets.validate(hit(6, 7, point, impulse), PLAYER),
            Err(Rejection::StillLoading)
        );
        assert_eq!(
            puppets.validate(hit(9, 7, point, impulse), PLAYER),
            Err(Rejection::UnknownActor)
        );
        assert_eq!(
            puppets.validate(hit(5, 24, point, impulse), PLAYER),
            Err(Rejection::UnknownBone)
        );
        let bad = [f32::NAN, f32::INFINITY, f32::NEG_INFINITY];
        for value in bad {
            assert_eq!(
                puppets.validate(hit(5, 7, [1.0, value, 3.0], impulse), PLAYER),
                Err(Rejection::NotFinite)
            );
            assert_eq!(
                puppets.validate(hit(5, 7, point, [0.0, 0.0, value]), PLAYER),
                Err(Rejection::NotFinite)
            );
        }
    }

    #[test]
    fn time_scale_must_slow_down_or_be_normal() {
        let puppets = Puppets::default();
        let scale = |scale| puppets.validate(Command::TimeScale { scale }, PLAYER);
        assert_eq!(scale(0.2), Ok(Action::SetTimeScale(0.2)));
        assert_eq!(scale(1.0), Ok(Action::SetTimeScale(1.0)));
        for out_of_range in [0.0, -0.5, 1.5] {
            assert_eq!(scale(out_of_range), Err(Rejection::TimeScaleRange));
        }
        for not_finite in [f32::NAN, f32::INFINITY] {
            assert_eq!(scale(not_finite), Err(Rejection::NotFinite));
        }
    }

    #[test]
    fn models_and_positions() {
        assert_eq!(model_for_kind(0), Some(0x647F_FDD9));
        assert_eq!(model_for_kind(1), None);
        assert_eq!(spawn_position([10.0, 20.0, 1.0], 0), [12.0, 20.0, 1.0]);
        assert_eq!(spawn_position([10.0, 20.0, 1.0], 3), [18.0, 20.0, 1.0]);
    }
}
