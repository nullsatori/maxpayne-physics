//! Bone frames from `GET_PED_BONE_COORDS`.
//!
//! The native returns a bone's position moved by an offset. Sampled at the
//! offsets 0, (d,0,0) and (0,d,0), it gives the bone origin `o` and two more
//! points `px`, `py`. If the offset is in the bone's own axes, `(px-o)/d` and
//! `(py-o)/d` are the bone's x and y axes in world space, and with
//! `z = x × y` they make its rotation. If it is in world axes, the
//! differences are always (d,0,0) and (0,d,0) whatever the bone's
//! orientation, and the native yields positions only. The SDK does not say
//! which; [`Classifier`] decides from what the game returns.

use bridge::{ActorPose, BoneTransform};
use glamx::{Mat3, Pose3, Quat, Vec3};
use pose::pose_from_bone;
use pose::skeleton::{Bone, Skeleton, SkeletonError};
use scripthook::{Backend, BodyBone, Natives, Ped};

/// The sampling offset d, in world units (metres in RAGE games).
pub const OFFSET: f32 = 0.1;
/// Offsets each bone is sampled at, in this order.
pub const SAMPLE_OFFSETS: [[f32; 3]; 3] = [[0.0; 3], [OFFSET, 0.0, 0.0], [0.0, OFFSET, 0.0]];

/// Largest deviation of a sampled axis length from 1.
const AXIS_LENGTH_TOLERANCE: f32 = 0.1;
/// Largest deviation of the angle between the sampled axes from 90 degrees, radians.
const ORTHOGONALITY_TOLERANCE: f32 = 0.1;

/// The positions of one bone sampled at [`SAMPLE_OFFSETS`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BoneSample {
    pub origin: Vec3,
    pub at_x: Vec3,
    pub at_y: Vec3,
}

impl BoneSample {
    /// The sampled offsets `px - o` and `py - o`.
    fn offsets(&self) -> (Vec3, Vec3) {
        (self.at_x - self.origin, self.at_y - self.origin)
    }

    /// The bone's world rotation if offsets are bone-local; `None` when the
    /// sampled axes are not finite or too far from orthonormal for a frame.
    pub fn rotation(&self) -> Option<Quat> {
        let (x, y) = self.offsets();
        let (x, y) = (x / OFFSET, y / OFFSET);
        if !(x.is_finite() && y.is_finite()) {
            return None;
        }
        let (x_length, y_length) = (x.length(), y.length());
        if (x_length - 1.0).abs() > AXIS_LENGTH_TOLERANCE || (y_length - 1.0).abs() > AXIS_LENGTH_TOLERANCE {
            return None;
        }
        let (x, y) = (x / x_length, y / y_length);
        // The cosine of the angle between them is the sine of its deviation from 90 degrees.
        if x.dot(y).abs() > ORTHOGONALITY_TOLERANCE.sin() {
            return None;
        }
        let y = (y - x * x.dot(y)).normalize();
        Some(Quat::from_mat3(&Mat3::from_cols(x, y, x.cross(y))).normalize())
    }
}

/// Samples every bone of `ped`: 3 native calls per bone, in bone order.
pub fn sample_ped<B: Backend>(natives: &mut Natives<B>, ped: Ped, bones: &[BodyBone], out: &mut Vec<BoneSample>) {
    out.clear();
    for bone in bones {
        let [origin, at_x, at_y] = SAMPLE_OFFSETS
            .map(|offset| Vec3::from_array(natives.get_ped_bone_coords(ped, i32::from(bone.tag), offset)));
        out.push(BoneSample { origin, at_x, at_y });
    }
}

/// How `GET_PED_BONE_COORDS` applies its offset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Semantics {
    /// Not decided yet.
    Unknown,
    /// In world axes: bone rotations are not available from this native.
    World,
    /// In the bone's axes: samples give bone rotations.
    BoneLocal,
}

/// Frames in one decision window.
pub const WINDOW_FRAMES: u32 = 30;
/// Largest difference, per component, between a sampled offset and the
/// requested one for the offset to count as world axes. A bone turned by more
/// than 0.01 rad (this / d) about a world axis exceeds it, by more than
/// 0.02 rad about any axis; f32 rounding of positions within about 4 km of
/// the origin stays below it.
const WORLD_TOLERANCE: f32 = 1.0e-3;
/// A bone further than this from identity, radians, counts as rotated.
const ROTATED_ANGLE: f32 = 0.05;
/// Rotated bones a window needs for [`Semantics::BoneLocal`].
const MIN_ROTATED_BONES: usize = 4;
/// A turn of a bone's direction from its parent, radians, that shows the pose
/// changed during a window.
const POSE_CHANGE_ANGLE: f32 = 0.1;
/// Shorter bone-to-parent distances have no reliable direction.
const MIN_LINK_LENGTH: f32 = 0.05;

/// Decides [`Semantics`] from one ped's bones, in windows of [`WINDOW_FRAMES`]
/// frames:
///
/// * [`Semantics::World`] if in every frame every bone's offsets are the
///   requested ones (within [`WORLD_TOLERANCE`]) and the pose changed: some
///   bone's direction from its parent turned by more than
///   [`POSE_CHANGE_ANGLE`]. With bone-local offsets that turn would have
///   shown in the parent's offsets, so a still pose with axis-aligned bones
///   (a T-pose facing a world axis), which looks the same either way, never
///   reads as World. Bones hanging from a root are left out of this test,
///   since a pelvis may move against the root without turning anything.
/// * [`Semantics::BoneLocal`] if every bone gives a valid frame in every
///   frame, and at least [`MIN_ROTATED_BONES`] bones were turned from
///   identity by more than [`ROTATED_ANGLE`] in some frame. World offsets
///   never show a turn; several bones keep one odd bone from deciding.
/// * Otherwise the window decides nothing ([`Classifier::take_undecided`]
///   says what it saw) and the next one starts.
///
/// A decision is final.
pub struct Classifier {
    parents: Vec<Option<usize>>,
    semantics: Semantics,
    /// Windows completed.
    windows: u32,
    frames: u32,
    all_world: bool,
    /// Bones that gave no valid frame in some frame of the window.
    invalid: Vec<bool>,
    rotated: Vec<bool>,
    pose_changed: bool,
    /// Direction from its parent of each tested bone, from the first frame of
    /// the window where it was long enough.
    first_links: Vec<Option<Vec3>>,
    undecided: Option<Undecided>,
}

/// What a window that decided nothing saw: the first run in the game needs
/// to tell a still player from a bone the native gives no frame for.
#[derive(Debug, PartialEq)]
pub struct Undecided {
    /// The window's number, from 1.
    pub window: u32,
    /// Every offset was the requested one.
    pub world_axes: bool,
    pub pose_changed: bool,
    /// Bones turned from identity in some frame.
    pub rotated: usize,
    /// Bones that gave no valid frame in some frame.
    pub invalid: Vec<usize>,
}

impl Undecided {
    /// One line for the log, with bone names from `bones`.
    pub fn describe(&self, bones: &[BodyBone]) -> String {
        let invalid = if self.invalid.is_empty() {
            "every bone frame valid".to_owned()
        } else {
            let names: Vec<&str> = self.invalid.iter().map(|&bone| bones[bone].name).collect();
            format!("no valid frame from {}", names.join(" "))
        };
        format!(
            "window {} of {WINDOW_FRAMES} frames: offsets {} the requested ones, pose {}, {} bones turned \
             ({MIN_ROTATED_BONES} needed), {invalid}",
            self.window,
            if self.world_axes { "always" } else { "not always" },
            if self.pose_changed { "changed" } else { "still" },
            self.rotated,
        )
    }
}

impl Classifier {
    pub fn new(parents: &[Option<usize>]) -> Self {
        let mut classifier = Self {
            parents: parents.to_vec(),
            semantics: Semantics::Unknown,
            windows: 0,
            frames: 0,
            all_world: true,
            invalid: Vec::new(),
            rotated: Vec::new(),
            pose_changed: false,
            first_links: Vec::new(),
            undecided: None,
        };
        classifier.start_window();
        classifier
    }

    fn start_window(&mut self) {
        let bones = self.parents.len();
        self.frames = 0;
        self.all_world = true;
        self.invalid = vec![false; bones];
        self.rotated = vec![false; bones];
        self.pose_changed = false;
        self.first_links = vec![None; bones];
    }

    /// Feeds one frame, a sample per bone; returns the semantics so far.
    pub fn observe(&mut self, samples: &[BoneSample]) -> Semantics {
        assert_eq!(samples.len(), self.parents.len(), "one sample per bone");
        if self.semantics != Semantics::Unknown {
            return self.semantics;
        }
        let requested = (Vec3::X * OFFSET, Vec3::Y * OFFSET);
        for (bone, sample) in samples.iter().enumerate() {
            let (x, y) = sample.offsets();
            if !(x.abs_diff_eq(requested.0, WORLD_TOLERANCE) && y.abs_diff_eq(requested.1, WORLD_TOLERANCE)) {
                self.all_world = false;
            }
            match sample.rotation() {
                Some(rotation) => self.rotated[bone] |= rotation.angle_between(Quat::IDENTITY) > ROTATED_ANGLE,
                None => self.invalid[bone] = true,
            }
            let Some(parent) = self.parents[bone].filter(|&parent| self.parents[parent].is_some()) else {
                continue;
            };
            let link = sample.origin - samples[parent].origin;
            if !link.is_finite() || link.length() < MIN_LINK_LENGTH {
                continue;
            }
            let direction = link.normalize();
            match self.first_links[bone] {
                Some(first) => self.pose_changed |= first.angle_between(direction) > POSE_CHANGE_ANGLE,
                None => self.first_links[bone] = Some(direction),
            }
        }
        self.frames += 1;
        if self.frames == WINDOW_FRAMES {
            self.windows += 1;
            let rotated = self.rotated.iter().filter(|&&rotated| rotated).count();
            let invalid: Vec<usize> = (0..self.invalid.len()).filter(|&bone| self.invalid[bone]).collect();
            if self.all_world && self.pose_changed {
                self.semantics = Semantics::World;
            } else if invalid.is_empty() && rotated >= MIN_ROTATED_BONES {
                self.semantics = Semantics::BoneLocal;
            } else {
                self.undecided = Some(Undecided {
                    window: self.windows,
                    world_axes: self.all_world,
                    pose_changed: self.pose_changed,
                    rotated,
                    invalid,
                });
            }
            self.start_window();
        }
        self.semantics
    }

    /// The last window that decided nothing, once.
    pub fn take_undecided(&mut self) -> Option<Undecided> {
        self.undecided.take()
    }
}

/// Fills `pose` with one ped's bones: world positions always; rotations, and
/// [`ActorPose::ROTATIONS_VALID`], only when they are measured: offsets are
/// bone-local and every bone gives a valid frame. Otherwise rotations are
/// identity.
pub fn fill_actor_pose(pose: &mut ActorPose, actor: u32, samples: &[BoneSample], semantics: Semantics) {
    pose.actor = actor;
    pose.bone_count = samples.len() as u32;
    let mut measured = semantics == Semantics::BoneLocal;
    for (slot, sample) in pose.bones.iter_mut().zip(samples) {
        let rotation = if measured { sample.rotation() } else { None };
        measured &= rotation.is_some();
        *slot = BoneTransform {
            translation: sample.origin.to_array(),
            rotation: rotation.unwrap_or(Quat::IDENTITY).to_array(),
        };
    }
    if !measured {
        for slot in &mut pose.bones[..samples.len()] {
            slot.rotation = Quat::IDENTITY.to_array();
        }
    }
    pose.flags = if measured { ActorPose::ROTATIONS_VALID } else { 0 };
}

/// `pose` in the pose crate's skeleton text format: names and parents from
/// `bones`, local transforms from the pose's world transforms.
pub fn skeleton_text(bones: &[BodyBone], pose: &ActorPose, semantics: Semantics) -> Result<String, SkeletonError> {
    assert_eq!(pose.bones().len(), bones.len(), "one transform per bone");
    let world: Vec<Pose3> = pose.bones().iter().map(pose_from_bone).collect();
    let skeleton = Skeleton::new(
        bones
            .iter()
            .zip(&world)
            .map(|(bone, bone_world)| Bone {
                name: bone.name.to_owned(),
                parent: bone.parent,
                rest: bone
                    .parent
                    .map_or(*bone_world, |parent| world[parent].inv_mul(bone_world)),
            })
            .collect(),
    )?;
    let rotations = if pose.flags & ActorPose::ROTATIONS_VALID != 0 {
        "rotations measured"
    } else {
        "rotations unavailable (identity)"
    };
    Ok(format!(
        "# Max Payne 3 player body bones from GET_PED_BONE_COORDS, one frame; offsets {semantics:?}, {rotations}\n{}",
        skeleton.to_text()
    ))
}

#[cfg(test)]
mod tests {
    use scripthook::{BODY_BONES, Recorder, body_bone_index};

    use super::*;

    fn parents() -> Vec<Option<usize>> {
        BODY_BONES.iter().map(|bone| bone.parent).collect()
    }

    /// World poses of an animated ped on BODY_BONES at frame `t`: every bone
    /// 0.2 m from its parent, local rotations swinging with time, the whole
    /// body turning by `yaw_rate` per frame, far from the world origin.
    fn animated(t: u32, yaw_rate: f32) -> Vec<Pose3> {
        let t = t as f32;
        let mut world: Vec<Pose3> = Vec::new();
        for (index, bone) in BODY_BONES.iter().enumerate() {
            let phase = index as f32;
            let swing = Quat::from_rotation_x(0.3 * (0.07 * t + phase).sin())
                * Quat::from_rotation_z(0.4 * (0.05 * t + 2.0 * phase).cos());
            world.push(match bone.parent {
                Some(parent) => world[parent] * Pose3::from_parts(Vec3::new(0.0, 0.2, 0.05), swing),
                None => Pose3::from_parts(Vec3::new(1500.0, -800.0, 30.0), Quat::from_rotation_z(yaw_rate * t)),
            });
        }
        world
    }

    /// The same skeleton standing still with every bone along the world axes.
    fn axis_aligned() -> Vec<Pose3> {
        axis_aligned_with_links(Vec3::new(0.0, 0.2, 0.05))
    }

    fn axis_aligned_with_links(link: Vec3) -> Vec<Pose3> {
        let mut world: Vec<Pose3> = Vec::new();
        for bone in &BODY_BONES {
            world.push(match bone.parent {
                Some(parent) => world[parent] * Pose3::from_translation(link),
                None => Pose3::from_translation(Vec3::new(1500.0, -800.0, 30.0)),
            });
        }
        world
    }

    /// `world` turned as a whole about its root by `angle` around `axis`.
    fn turned(world: &[Pose3], axis: Vec3, angle: f32) -> Vec<Pose3> {
        let turn = Quat::from_axis_angle(axis, angle);
        let root = world[0].translation;
        world
            .iter()
            .map(|pose| Pose3::from_parts(root + turn * (pose.translation - root), turn * pose.rotation))
            .collect()
    }

    /// Moves every sampled point by up to `amplitude` per axis, differently in
    /// each frame `t`, like rounding of positions far from the origin.
    fn jittered(mut samples: Vec<BoneSample>, t: u32, amplitude: f32) -> Vec<BoneSample> {
        for (bone, sample) in samples.iter_mut().enumerate() {
            for (k, point) in [&mut sample.origin, &mut sample.at_x, &mut sample.at_y]
                .into_iter()
                .enumerate()
            {
                let seed = (t * 97 + bone as u32 * 13 + k as u32 * 5) as f32;
                *point += amplitude * Vec3::new((seed * 1.7).sin(), (seed * 2.3).sin(), (seed * 3.1).sin());
            }
        }
        samples
    }

    fn bone_local(world: &[Pose3]) -> Vec<BoneSample> {
        let at = |pose: &Pose3, offset: [f32; 3]| pose.transform_point(Vec3::from_array(offset));
        world
            .iter()
            .map(|pose| BoneSample {
                origin: at(pose, SAMPLE_OFFSETS[0]),
                at_x: at(pose, SAMPLE_OFFSETS[1]),
                at_y: at(pose, SAMPLE_OFFSETS[2]),
            })
            .collect()
    }

    fn world_axes(world: &[Pose3]) -> Vec<BoneSample> {
        let at = |pose: &Pose3, offset: [f32; 3]| pose.translation + Vec3::from_array(offset);
        world
            .iter()
            .map(|pose| BoneSample {
                origin: at(pose, SAMPLE_OFFSETS[0]),
                at_x: at(pose, SAMPLE_OFFSETS[1]),
                at_y: at(pose, SAMPLE_OFFSETS[2]),
            })
            .collect()
    }

    fn same_rotation(a: Quat, b: Quat, tolerance: f32) -> bool {
        a.abs_diff_eq(b, tolerance) || a.abs_diff_eq(-b, tolerance)
    }

    #[test]
    fn rotation_from_bone_local_samples() {
        for t in [0, 17, 40] {
            let world = animated(t, 0.03);
            for (pose, sample) in world.iter().zip(bone_local(&world)) {
                let rotation = sample.rotation().unwrap();
                assert!(
                    same_rotation(rotation, pose.rotation, 2.0e-3),
                    "{rotation:?} != {:?}",
                    pose.rotation
                );
            }
        }
    }

    #[test]
    fn degenerate_frames_are_rejected() {
        let origin = Vec3::new(10.0, 20.0, 30.0);
        let sample = |x: Vec3, y: Vec3| BoneSample {
            origin,
            at_x: origin + x * OFFSET,
            at_y: origin + y * OFFSET,
        };
        assert!(
            sample(Vec3::ZERO, Vec3::ZERO).rotation().is_none(),
            "bone not found: all three equal"
        );
        assert!(sample(Vec3::X * 1.15, Vec3::Y).rotation().is_none(), "axis 15% long");
        assert!(sample(Vec3::X, Vec3::Y * 0.85).rotation().is_none(), "axis 15% short");
        let skewed = |angle: f32| Vec3::new(angle.sin(), angle.cos(), 0.0);
        for skew in [0.15, -0.15] {
            assert!(
                sample(Vec3::X, skewed(skew)).rotation().is_none(),
                "axes {skew} rad off square"
            );
        }
        assert!(sample(Vec3::X, Vec3::X).rotation().is_none(), "parallel axes");
        assert!(sample(Vec3::new(f32::NAN, 0.0, 0.0), Vec3::Y).rotation().is_none());
        // Small scale and skew are tolerated; the frame is squared up.
        let rotation = sample(Vec3::X * 1.05, skewed(0.05) * 0.95).rotation().unwrap();
        assert!(rotation.is_normalized());
        assert!(rotation.angle_between(Quat::IDENTITY) < 1.0e-5);
    }

    fn classify(classifier: &mut Classifier, frames: impl Iterator<Item = Vec<BoneSample>>) -> Vec<Semantics> {
        frames.map(|samples| classifier.observe(&samples)).collect()
    }

    /// Rounding of positions around 2 km from the origin, in metres.
    const NOISE: f32 = 1.0e-4;

    fn all_unknown(verdicts: &[Semantics]) -> bool {
        verdicts.iter().all(|&v| v == Semantics::Unknown)
    }

    #[test]
    fn bone_local_offsets_are_recognized_after_a_window() {
        let mut classifier = Classifier::new(&parents());
        let verdicts = classify(
            &mut classifier,
            (0..WINDOW_FRAMES).map(|t| jittered(bone_local(&animated(t, 0.01)), t, NOISE)),
        );
        assert!(all_unknown(&verdicts[..WINDOW_FRAMES as usize - 1]));
        assert_eq!(verdicts.last(), Some(&Semantics::BoneLocal));
        assert_eq!(classifier.take_undecided(), None, "the window decided");
        // Final: whole windows of world-like frames later do not change it.
        let verdicts = classify(
            &mut classifier,
            (0..2 * WINDOW_FRAMES).map(|t| world_axes(&animated(t, 0.01))),
        );
        assert!(verdicts.iter().all(|&v| v == Semantics::BoneLocal));
    }

    #[test]
    fn world_offsets_are_recognized_once_the_pose_moves() {
        let mut classifier = Classifier::new(&parents());
        let verdicts = classify(
            &mut classifier,
            (0..WINDOW_FRAMES).map(|t| jittered(world_axes(&animated(t, 0.01)), t, NOISE)),
        );
        assert!(all_unknown(&verdicts[..WINDOW_FRAMES as usize - 1]));
        assert_eq!(verdicts.last(), Some(&Semantics::World));
        let verdicts = classify(
            &mut classifier,
            (0..2 * WINDOW_FRAMES).map(|t| bone_local(&animated(t, 0.01))),
        );
        assert!(verdicts.iter().all(|&v| v == Semantics::World));
    }

    #[test]
    fn a_still_axis_aligned_pose_decides_nothing() {
        // Bone-local and world offsets give identical samples here.
        let still = bone_local(&axis_aligned());
        assert_eq!(still, world_axes(&axis_aligned()));
        let mut classifier = Classifier::new(&parents());
        let verdicts = classify(&mut classifier, (0..10 * WINDOW_FRAMES).map(|_| still.clone()));
        assert!(all_unknown(&verdicts));
        let undecided = classifier.take_undecided().unwrap();
        assert_eq!(
            undecided.describe(&BODY_BONES),
            "window 10 of 30 frames: offsets always the requested ones, pose still, 0 bones turned (4 needed), \
             every bone frame valid"
        );
        assert_eq!(classifier.take_undecided(), None, "taken once");

        // Once the ped turns, bone-local offsets show it...
        let turning = |t: u32| turned(&axis_aligned(), Vec3::Z, 0.02 * t as f32);
        let mut local = Classifier::new(&parents());
        let verdicts = classify(&mut local, (0..WINDOW_FRAMES).map(|t| bone_local(&turning(t))));
        assert_eq!(verdicts.last(), Some(&Semantics::BoneLocal));
        // ...and world offsets stay put while the body turns.
        let mut world = Classifier::new(&parents());
        let verdicts = classify(&mut world, (0..WINDOW_FRAMES).map(|t| world_axes(&turning(t))));
        assert_eq!(verdicts.last(), Some(&Semantics::World));
    }

    #[test]
    fn noise_a_short_link_or_a_moving_pelvis_do_not_decide_a_still_pose() {
        // HEAD 1 mm from NECK: noise swings that link's direction widely.
        let mut world = axis_aligned();
        let (neck, head) = (body_bone_index("NECK").unwrap(), body_bone_index("HEAD").unwrap());
        world[head].translation = world[neck].translation + Vec3::new(0.0, 1.0e-3, 0.0);
        let mut classifier = Classifier::new(&parents());
        let verdicts = classify(
            &mut classifier,
            (0..5 * WINDOW_FRAMES).map(|t| jittered(bone_local(&world), t, NOISE)),
        );
        assert!(all_unknown(&verdicts), "noise is neither a turn nor a change of pose");

        // The pelvis and everything on it bob against the root without turning.
        let bobbing = |t: u32| {
            let mut world = axis_aligned();
            let lift = Vec3::new(0.0, 0.0, 0.1 * (0.3 * t as f32).sin());
            for pose in &mut world[1..] {
                pose.translation += lift;
            }
            bone_local(&world)
        };
        let mut classifier = Classifier::new(&parents());
        let verdicts = classify(&mut classifier, (0..5 * WINDOW_FRAMES).map(bobbing));
        assert!(all_unknown(&verdicts), "a pelvis moving against the root shows no turn");
    }

    #[test]
    fn small_turns_are_not_read_as_world_offsets() {
        // The whole body rocks by 0.08 rad either way: a turn of every bone
        // above ROTATED_ANGLE that moves the links by more than
        // POSE_CHANGE_ANGLE, yet shifts the sampled offsets by under 0.01.
        // About x the x offsets stay put, about y the y offsets do.
        let still = axis_aligned_with_links(Vec3::splat(0.1));
        for axis in [Vec3::X, Vec3::Y] {
            let mut classifier = Classifier::new(&parents());
            let verdicts = classify(
                &mut classifier,
                (0..WINDOW_FRAMES).map(|t| bone_local(&turned(&still, axis, 0.08 * (0.2 * t as f32).cos()))),
            );
            assert_eq!(verdicts.last(), Some(&Semantics::BoneLocal), "rocking about {axis}");
        }
    }

    #[test]
    fn a_few_turned_bones_are_not_enough() {
        for (turned_bones, expected) in [
            (1, Semantics::Unknown),
            (MIN_ROTATED_BONES - 1, Semantics::Unknown),
            (MIN_ROTATED_BONES, Semantics::BoneLocal),
        ] {
            let mut world = axis_aligned();
            let bones = world.len();
            for pose in &mut world[bones - turned_bones..] {
                pose.rotation = Quat::from_rotation_z(0.3);
            }
            let mut classifier = Classifier::new(&parents());
            let verdicts = classify(&mut classifier, (0..2 * WINDOW_FRAMES).map(|_| bone_local(&world)));
            assert_eq!(verdicts.last(), Some(&expected), "{turned_bones} bones turned");
        }
    }

    #[test]
    fn each_window_starts_afresh() {
        let head = body_bone_index("HEAD").unwrap();
        // Bone-local frames of a moving body with one bad frame: the window
        // saw turns, a changing pose and offsets that are not world-like, yet
        // decides nothing.
        let spoilt = |t: u32| {
            let mut samples = bone_local(&animated(t, 0.01));
            if t == 3 {
                samples[head].at_y.z = f32::NAN;
            }
            samples
        };
        // None of it carries over into a still window...
        let still = bone_local(&axis_aligned());
        let mut classifier = Classifier::new(&parents());
        let verdicts = classify(
            &mut classifier,
            (0..WINDOW_FRAMES)
                .map(spoilt)
                .chain((0..WINDOW_FRAMES).map(|_| still.clone())),
        );
        assert!(all_unknown(&verdicts));
        // ...or into a window of world offsets.
        let mut classifier = Classifier::new(&parents());
        let verdicts = classify(
            &mut classifier,
            (0..WINDOW_FRAMES)
                .map(spoilt)
                .chain((0..WINDOW_FRAMES).map(|t| world_axes(&animated(t, 0.01)))),
        );
        assert_eq!(verdicts[WINDOW_FRAMES as usize - 1], Semantics::Unknown);
        assert_eq!(verdicts.last(), Some(&Semantics::World));
    }

    #[test]
    fn degenerate_bones_keep_the_verdict_unknown() {
        let head = body_bone_index("HEAD").unwrap();
        let mut classifier = Classifier::new(&parents());
        let frames = (0..5 * WINDOW_FRAMES).map(|t| {
            let mut samples = bone_local(&animated(t, 0.01));
            let origin = samples[head].origin;
            samples[head].at_x = origin;
            samples[head].at_y = origin;
            samples
        });
        assert!(all_unknown(&classify(&mut classifier, frames)));
        let undecided = classifier.take_undecided().unwrap();
        assert_eq!(
            (
                undecided.window,
                undecided.world_axes,
                undecided.pose_changed,
                &undecided.invalid
            ),
            (5, false, true, &vec![head])
        );
        assert!(undecided.rotated >= MIN_ROTATED_BONES);
        assert!(
            undecided
                .describe(&BODY_BONES)
                .ends_with(" bones turned (4 needed), no valid frame from HEAD")
        );

        // A window with a non-finite frame decides nothing either; the next one does.
        let mut classifier = Classifier::new(&parents());
        let frames = (0..2 * WINDOW_FRAMES).map(|t| {
            let mut samples = bone_local(&animated(t, 0.01));
            if t == 3 {
                samples[head].at_y.z = f32::NAN;
            }
            samples
        });
        let verdicts = classify(&mut classifier, frames);
        assert_eq!(verdicts[WINDOW_FRAMES as usize - 1], Semantics::Unknown);
        assert_eq!(verdicts.last(), Some(&Semantics::BoneLocal));
    }

    #[test]
    fn actor_poses_carry_rotations_only_when_measured() {
        let world = animated(5, 0.01);
        let samples = bone_local(&world);
        let mut pose = ActorPose::new(9, 0, &[]);
        fill_actor_pose(&mut pose, 3, &samples, Semantics::BoneLocal);
        assert_eq!(
            (pose.actor, pose.bone_count, pose.flags),
            (3, 24, ActorPose::ROTATIONS_VALID)
        );
        for (bone, expected) in pose.bones().iter().zip(&world) {
            assert!(Vec3::from_array(bone.translation).abs_diff_eq(expected.translation, 1.0e-6));
            assert!(same_rotation(
                Quat::from_array(bone.rotation),
                expected.rotation,
                2.0e-3
            ));
        }

        for semantics in [Semantics::World, Semantics::Unknown] {
            fill_actor_pose(&mut pose, 4, &samples, semantics);
            assert_eq!((pose.actor, pose.flags), (4, 0));
            assert!(pose.bones().iter().all(|bone| bone.rotation == [0.0, 0.0, 0.0, 1.0]));
            assert_eq!(pose.bones()[7].translation, world[7].translation.to_array());
        }

        // One bad bone withholds every rotation of that frame.
        let mut broken = samples.clone();
        broken[2].at_x = broken[2].origin;
        fill_actor_pose(&mut pose, 3, &broken, Semantics::BoneLocal);
        assert_eq!(pose.flags, 0);
        assert!(pose.bones().iter().all(|bone| bone.rotation == [0.0, 0.0, 0.0, 1.0]));
    }

    #[test]
    fn skeleton_text_reproduces_the_frame() {
        let world = animated(12, 0.01);
        let mut pose = ActorPose::new(0, 0, &[]);
        fill_actor_pose(&mut pose, 0, &bone_local(&world), Semantics::BoneLocal);
        let text = skeleton_text(&BODY_BONES, &pose, Semantics::BoneLocal).unwrap();
        assert!(text.starts_with("# Max Payne 3 player body bones"));
        let skeleton = Skeleton::parse(&text).unwrap();
        let names: Vec<&str> = skeleton.bones().iter().map(|bone| bone.name.as_str()).collect();
        let expected: Vec<&str> = BODY_BONES.iter().map(|bone| bone.name).collect();
        assert_eq!(names, expected);
        assert_eq!(skeleton.bones()[8].parent, Some(5));
        for (got, expected) in skeleton.world_rest().iter().zip(&world) {
            assert!(got.translation.abs_diff_eq(expected.translation, 1.0e-3));
            assert!(same_rotation(got.rotation, expected.rotation, 5.0e-3));
        }

        fill_actor_pose(&mut pose, 0, &world_axes(&world), Semantics::World);
        let positions_only = skeleton_text(&BODY_BONES, &pose, Semantics::World).unwrap();
        assert!(
            positions_only
                .lines()
                .next()
                .unwrap()
                .ends_with("rotations unavailable (identity)")
        );
        let skeleton = Skeleton::parse(&positions_only).unwrap();
        assert!(skeleton.bones().iter().all(|bone| bone.rest.rotation == Quat::IDENTITY));
        assert!(
            skeleton.world_rest()[14]
                .translation
                .abs_diff_eq(world[14].translation, 1.0e-3)
        );
    }

    #[test]
    fn samples_three_offsets_per_bone_in_table_order() {
        // Answers encode the request so the samples show where each came from.
        let mut natives = Natives::new(Recorder::new(|_, args| [args[1], args[2], args[3]]));
        let mut samples = Vec::new();
        sample_ped(&mut natives, 77, &BODY_BONES[..2], &mut samples);
        let calls = &natives.backend().calls;
        assert_eq!(calls.len(), 6);
        let d = OFFSET.to_bits();
        let expected_args = [
            [77, 0x0000, 0, 0, 0],
            [77, 0x0000, d, 0, 0],
            [77, 0x0000, 0, d, 0],
            [77, 0xC018, 0, 0, 0],
            [77, 0xC018, d, 0, 0],
            [77, 0xC018, 0, d, 0],
        ];
        for (call, args) in calls.iter().zip(expected_args) {
            assert_eq!((call.0, call.1.as_slice()), (0x4579_CAB1, args.as_slice()));
        }
        let word = |bits: u32| f32::from_bits(bits);
        assert_eq!(
            samples[1],
            BoneSample {
                origin: Vec3::new(word(0xC018), 0.0, 0.0),
                at_x: Vec3::new(word(0xC018), OFFSET, 0.0),
                at_y: Vec3::new(word(0xC018), 0.0, OFFSET),
            }
        );
        assert_eq!(natives.take_calls(), 6);
    }
}
