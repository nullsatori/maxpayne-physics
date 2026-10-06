//! One-line summaries of the pose stream from Max Payne 3, for the log.

use std::fmt;

use bridge::{ActorPose, MAX_ACTORS, MAX_BONES, PoseFrame};
use scripthook::body_bone_index;

/// Frame number, game time, actors, and actor 0's bone count, flags and
/// ROOT and PELVIS positions; non-finite values and out-of-range counts are
/// flagged rather than trusted.
pub struct Summary<'a>(pub &'a PoseFrame);

impl fmt::Display for Summary<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let frame = self.0;
        if frame.frame == 0 {
            return write!(f, "nothing published yet");
        }
        write!(
            f,
            "frame {}, game time {:.3} s, {} actors",
            frame.frame, frame.game_time, frame.actor_count
        )?;
        if !frame.game_time.is_finite() {
            write!(f, "; NON-FINITE game time")?;
        }
        // The counts come from another process; slicing by them must not panic.
        if frame.actor_count as usize > MAX_ACTORS {
            return write!(f, "; BAD actor count, over {MAX_ACTORS}");
        }
        let actors = frame.actors();
        if let Some(actor) = actors.first() {
            write!(
                f,
                "; actor {}: {} bones, flags {:#x}",
                actor.actor, actor.bone_count, actor.flags
            )?;
            if let Some(bones) = actor.bones.get(..actor.bone_count as usize) {
                for name in ["ROOT", "PELVIS"] {
                    if let Some(bone) = body_bone_index(name).and_then(|index| bones.get(index)) {
                        let [x, y, z] = bone.translation;
                        write!(f, ", {name} ({x:.3}, {y:.3}, {z:.3})")?;
                    }
                }
            }
        }
        let mut bad = actors
            .iter()
            .enumerate()
            .flat_map(|(index, actor)| bad_bones(index, actor));
        if let Some(first) = bad.next() {
            write!(f, "; NON-FINITE or BAD {first}")?;
            let more = bad.count();
            if more > 0 {
                write!(f, " and {more} more")?;
            }
        }
        Ok(())
    }
}

/// Bones of the actor at `index` with non-finite values, or the actor itself
/// when its bone count is out of range.
fn bad_bones(index: usize, actor: &ActorPose) -> Vec<String> {
    let Some(bones) = actor.bones.get(..actor.bone_count as usize) else {
        return vec![format!("actor {index}: bone count over {MAX_BONES}")];
    };
    bones
        .iter()
        .enumerate()
        .filter(|(_, bone)| !bone.translation.iter().chain(&bone.rotation).all(|v| v.is_finite()))
        .map(|(bone, _)| format!("actor {index} bone {bone}"))
        .collect()
}

#[cfg(test)]
mod tests {
    use bridge::BoneTransform;

    use super::*;

    fn bone(x: f32) -> BoneTransform {
        BoneTransform {
            translation: [x, x + 0.5, -x],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }

    fn frame(actors: &[ActorPose]) -> Box<PoseFrame> {
        let mut frame = PoseFrame::new_boxed();
        frame.frame = 42;
        frame.game_time = 12.25;
        frame.actor_count = actors.len() as u32;
        frame.actors[..actors.len()].copy_from_slice(actors);
        frame
    }

    #[test]
    fn summarizes_actor_zero() {
        let actors = [
            ActorPose::new(0, ActorPose::ROTATIONS_VALID, &[bone(1.0), bone(2.0), bone(3.0)]),
            ActorPose::new(7, 0, &[bone(4.0)]),
        ];
        assert_eq!(
            Summary(&frame(&actors)).to_string(),
            "frame 42, game time 12.250 s, 2 actors; actor 0: 3 bones, flags 0x1, \
             ROOT (1.000, 1.500, -1.000), PELVIS (2.000, 2.500, -2.000)"
        );
        assert_eq!(Summary(&PoseFrame::new_boxed()).to_string(), "nothing published yet");
        assert_eq!(
            Summary(&frame(&[])).to_string(),
            "frame 42, game time 12.250 s, 0 actors"
        );
    }

    #[test]
    fn flags_non_finite_values_and_bad_counts() {
        let mut broken = ActorPose::new(9, 0, &[bone(1.0), bone(2.0), bone(3.0)]);
        broken.bones[1].rotation[2] = f32::NAN;
        broken.bones[2].translation[0] = f32::INFINITY;
        let actors = [ActorPose::new(0, 0, &[bone(1.0)]), broken];
        let mut frame = frame(&actors);
        assert_eq!(
            Summary(&frame).to_string(),
            "frame 42, game time 12.250 s, 2 actors; actor 0: 1 bones, flags 0x0, ROOT (1.000, 1.500, -1.000); \
             NON-FINITE or BAD actor 1 bone 1 and 1 more"
        );

        frame.actors[0].bone_count = MAX_BONES as u32 + 1;
        frame.game_time = f64::NAN;
        assert_eq!(
            Summary(&frame).to_string(),
            "frame 42, game time NaN s, 2 actors; NON-FINITE game time; actor 0: 129 bones, flags 0x0; \
             NON-FINITE or BAD actor 0: bone count over 128 and 2 more"
        );

        frame.actor_count = MAX_ACTORS as u32 + 1;
        assert_eq!(
            Summary(&frame).to_string(),
            "frame 42, game time NaN s, 33 actors; NON-FINITE game time; BAD actor count, over 32"
        );
    }
}
