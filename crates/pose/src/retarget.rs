//! Maps poses of a source skeleton onto a target skeleton.
//!
//! A mapped target bone turns away from its rest pose by the same world-space
//! rotation as its source bone:
//!
//! ```text
//! T = C(S) * (C(S0)^-1 * T0)
//! ```
//!
//! where `C` converts rotations into the target world ([`CoordinateConversion`]),
//! `S` and `S0` are the source bone's current and rest world rotations and
//! `T0` is the target bone's rest world rotation. The bracket is fixed per
//! bone, so it is computed once. Rigs whose bones have different local axes
//! thereby agree as long as their rest poses look alike.
//!
//! Positions come from the target skeleton: each bone sits at its rest local
//! translation from its parent (forward kinematics), so the target keeps its
//! own bone lengths. Only roots, the mapped bones without a mapped ancestor,
//! take their position from the source: the converted source position times
//! the root scale ([`RootScale`]). Unmapped bones keep their rest rotation
//! relative to their parent; an unmapped bone without a parent stays at rest.
//!
//! Mapping text format: one `source_bone target_bone` pair per line, `#`
//! starts a comment, blank lines are ignored. A source bone may drive several
//! target bones; a target bone takes at most one source bone.

use std::fmt;

use glamx::{Pose3, Quat};

use crate::ParseError;
use crate::coords::CoordinateConversion;
use crate::skeleton::Skeleton;

/// Pairs of `(source bone name, target bone name)`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mapping {
    pub pairs: Vec<(String, String)>,
}

impl Mapping {
    /// Reads the text format described in the module docs. Names are checked
    /// against the skeletons by [`Retargeter::new`].
    pub fn parse(text: &str) -> Result<Self, ParseError> {
        let mut pairs = Vec::new();
        for (line, fields) in crate::records(text) {
            let [source, target] = fields[..] else {
                return Err(ParseError {
                    line,
                    message: format!("expected 2 fields (source_bone target_bone), found {}", fields.len()),
                });
            };
            pairs.push((source.to_owned(), target.to_owned()));
        }
        Ok(Self { pairs })
    }
}

/// Factor between converted source root positions and target root positions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RootScale {
    /// The rest distance from the origin of the first target root (in bone
    /// order), divided by that of its source bone after conversion. With
    /// both skeletons in model space and the origin on the ground below the
    /// root, these distances are the root heights, so a shorter target
    /// crouches and moves proportionally less. Positions are scaled about
    /// the origin, which is why poses should be relative to the character
    /// rather than to the level.
    FromRest,
    /// A given factor, e.g. when a rest root lies at the origin.
    Fixed(f32),
}

#[derive(Debug, PartialEq)]
pub enum RetargetError {
    NoPairs,
    UnknownSource(String),
    UnknownTarget(String),
    /// A target bone mapped from two source bones.
    DuplicateTarget(String),
    /// The root scale is not a positive finite number. With
    /// [`RootScale::FromRest`] this means a rest root lies at the origin.
    BadRootScale(f32),
}

impl fmt::Display for RetargetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RetargetError::NoPairs => write!(f, "the mapping has no bone pairs"),
            RetargetError::UnknownSource(name) => write!(f, "source skeleton has no bone {name}"),
            RetargetError::UnknownTarget(name) => write!(f, "target skeleton has no bone {name}"),
            RetargetError::DuplicateTarget(name) => write!(f, "target bone {name} is mapped twice"),
            RetargetError::BadRootScale(scale) => {
                write!(f, "root scale {scale} is not a positive finite number")
            }
        }
    }
}

/// How one target bone is posed.
#[derive(Clone, Copy, Debug)]
struct Drive {
    /// For mapped bones: the source bone and the rest offset `C(S0)^-1 * T0`.
    source: Option<(usize, Quat)>,
    /// Positioned from the source instead of from the parent.
    root: bool,
}

pub struct Retargeter {
    target: Skeleton,
    source_bones: usize,
    conversion: CoordinateConversion,
    root_scale: f32,
    drives: Vec<Drive>,
}

impl Retargeter {
    pub fn new(
        source: &Skeleton,
        target: &Skeleton,
        mapping: &Mapping,
        conversion: CoordinateConversion,
        root_scale: RootScale,
    ) -> Result<Self, RetargetError> {
        if mapping.pairs.is_empty() {
            return Err(RetargetError::NoPairs);
        }
        let source_rest = source.world_rest();
        let target_rest = target.world_rest();
        let mut drives = vec![
            Drive {
                source: None,
                root: false,
            };
            target.bones().len()
        ];
        for (source_name, target_name) in &mapping.pairs {
            let s = source
                .index_of(source_name)
                .ok_or_else(|| RetargetError::UnknownSource(source_name.clone()))?;
            let t = target
                .index_of(target_name)
                .ok_or_else(|| RetargetError::UnknownTarget(target_name.clone()))?;
            if drives[t].source.is_some() {
                return Err(RetargetError::DuplicateTarget(target_name.clone()));
            }
            let offset = conversion.rotation(source_rest[s].rotation).inverse() * target_rest[t].rotation;
            drives[t].source = Some((s, offset.normalize()));
        }

        // Parents precede children, so a bone's ancestors are settled before it.
        let mut covered = vec![false; drives.len()];
        let mut first_root = None;
        for (index, bone) in target.bones().iter().enumerate() {
            let ancestor_mapped = bone.parent.is_some_and(|parent| covered[parent]);
            covered[index] = ancestor_mapped || drives[index].source.is_some();
            if let (Some((s, _)), false) = (drives[index].source, ancestor_mapped) {
                drives[index].root = true;
                first_root.get_or_insert((index, s));
            }
        }
        let root_scale = match root_scale {
            RootScale::Fixed(scale) => scale,
            RootScale::FromRest => {
                // The lowest mapped index has no mapped ancestor, so a root exists.
                let (t, s) = first_root.expect("a non-empty mapping has a root");
                target_rest[t].translation.length() / conversion.point(source_rest[s].translation).length()
            }
        };
        if !(root_scale.is_finite() && root_scale > 0.0) {
            return Err(RetargetError::BadRootScale(root_scale));
        }

        Ok(Self {
            target: target.clone(),
            source_bones: source.bones().len(),
            conversion,
            root_scale,
            drives,
        })
    }

    pub fn target(&self) -> &Skeleton {
        &self.target
    }

    pub fn root_scale(&self) -> f32 {
        self.root_scale
    }

    /// Target world poses for the given source world poses.
    ///
    /// # Panics
    /// If `source` does not have one pose per source bone.
    pub fn apply(&self, source: &[Pose3]) -> Vec<Pose3> {
        assert_eq!(source.len(), self.source_bones, "one source pose per source bone");
        let mut world: Vec<Pose3> = Vec::with_capacity(self.drives.len());
        for (bone, drive) in self.target.bones().iter().zip(&self.drives) {
            let parent = bone.parent.map(|parent| world[parent]);
            let rotation = match (drive.source, parent) {
                (Some((s, offset)), _) => self.conversion.rotation(source[s].rotation) * offset,
                (None, Some(parent)) => parent.rotation * bone.rest.rotation,
                (None, None) => bone.rest.rotation,
            };
            let translation = match (drive.source, parent) {
                (Some((s, _)), _) if drive.root => self.root_scale * self.conversion.point(source[s].translation),
                (_, Some(parent)) => parent.transform_point(bone.rest.translation),
                (_, None) => bone.rest.translation,
            };
            world.push(Pose3::from_parts(translation, rotation));
        }
        world
    }
}

#[cfg(test)]
mod tests {
    use glamx::Vec3;

    use super::*;
    use crate::skeleton::Bone;
    use crate::testutil::{Rng, assert_poses_eq, humanoid};

    /// Every bone mapped to the bone of the same name.
    fn same_names(skeleton: &Skeleton) -> Mapping {
        Mapping {
            pairs: skeleton
                .bones()
                .iter()
                .map(|bone| (bone.name.clone(), bone.name.clone()))
                .collect(),
        }
    }

    /// `skeleton` with its rest world poses replaced.
    fn with_world_rest(skeleton: &Skeleton, world: &[Pose3]) -> Skeleton {
        let local = skeleton.to_local(world);
        let bones = skeleton
            .bones()
            .iter()
            .zip(local)
            .map(|(bone, rest)| Bone { rest, ..bone.clone() })
            .collect();
        Skeleton::new(bones).unwrap()
    }

    fn positions(poses: &[Pose3]) -> Vec<Vec3> {
        poses.iter().map(|pose| pose.translation).collect()
    }

    fn assert_points_eq(a: &[Vec3], b: &[Vec3], tolerance: f32) {
        assert_eq!(a.len(), b.len());
        for (i, (a, b)) in a.iter().zip(b).enumerate() {
            assert!(a.abs_diff_eq(*b, tolerance), "bone {i}: {a} != {b}");
        }
    }

    #[test]
    fn identical_skeletons_reproduce_the_source() {
        let skeleton = humanoid();
        let retargeter = Retargeter::new(
            &skeleton,
            &skeleton,
            &same_names(&skeleton),
            CoordinateConversion::IDENTITY,
            RootScale::FromRest,
        )
        .unwrap();
        assert!((retargeter.root_scale() - 1.0).abs() < 1.0e-6);
        let mut rng = Rng::new(10);
        for _ in 0..20 {
            let source = rng.skeleton_pose(&skeleton);
            assert_poses_eq(&retargeter.apply(&source), &source, 1.0e-5);
        }
        assert_poses_eq(
            &retargeter.apply(&skeleton.world_rest()),
            &skeleton.world_rest(),
            1.0e-6,
        );
    }

    #[test]
    fn different_rest_rotations_keep_joint_positions() {
        let source = humanoid();
        let mut rng = Rng::new(11);
        let rest: Vec<Pose3> = source
            .world_rest()
            .iter()
            .map(|pose| Pose3::from_parts(pose.translation, rng.rotation()))
            .collect();
        let target = with_world_rest(&source, &rest);
        let retargeter = Retargeter::new(
            &source,
            &target,
            &same_names(&source),
            CoordinateConversion::IDENTITY,
            RootScale::FromRest,
        )
        .unwrap();
        for _ in 0..20 {
            let pose = rng.skeleton_pose(&source);
            let result = retargeter.apply(&pose);
            assert_points_eq(&positions(&result), &positions(&pose), 1.0e-5);
        }
        // At rest the target is exactly in its own rest pose.
        assert_poses_eq(&retargeter.apply(&source.world_rest()), &rest, 1.0e-5);
    }

    #[test]
    fn scaled_target_keeps_its_bone_lengths() {
        let source = humanoid();
        let bones = source
            .bones()
            .iter()
            .map(|bone| Bone {
                rest: Pose3::from_parts(bone.rest.translation * 0.5, bone.rest.rotation),
                ..bone.clone()
            })
            .collect();
        let target = Skeleton::new(bones).unwrap();
        let retargeter = Retargeter::new(
            &source,
            &target,
            &same_names(&source),
            CoordinateConversion::IDENTITY,
            RootScale::FromRest,
        )
        .unwrap();
        assert!((retargeter.root_scale() - 0.5).abs() < 1.0e-6);
        let mut rng = Rng::new(12);
        for _ in 0..20 {
            let pose = rng.skeleton_pose(&source);
            let result = retargeter.apply(&pose);
            for (i, bone) in target.bones().iter().enumerate() {
                if let Some(parent) = bone.parent {
                    let length = result[i].translation.distance(result[parent].translation);
                    assert!((length - bone.rest.translation.length()).abs() < 1.0e-5, "bone {i}");
                }
            }
            // Same shape at half size, root included.
            let half: Vec<Vec3> = positions(&pose).iter().map(|p| *p * 0.5).collect();
            assert_points_eq(&positions(&result), &half, 1.0e-5);
        }
    }

    #[test]
    fn reflecting_conversion_mirrors_positions() {
        let source = humanoid();
        for spec in ["-x,y,z", "x,z,y", "-x,-y,-z"] {
            let conversion = CoordinateConversion::from_axes(spec, 100.0).unwrap();
            assert!(conversion.is_reflection());
            // The target is the source as it appears in target space.
            let rest: Vec<Pose3> = source.world_rest().iter().map(|pose| conversion.pose(pose)).collect();
            let target = with_world_rest(&source, &rest);
            let retargeter =
                Retargeter::new(&source, &target, &same_names(&source), conversion, RootScale::FromRest).unwrap();
            let mut rng = Rng::new(13);
            for _ in 0..20 {
                let pose = rng.skeleton_pose(&source);
                let result = retargeter.apply(&pose);
                let mirrored: Vec<Vec3> = positions(&pose).iter().map(|p| conversion.point(*p)).collect();
                assert_points_eq(&positions(&result), &mirrored, 1.0e-3);
                for pose in &result {
                    assert!(pose.rotation.is_normalized());
                }
            }
        }
    }

    #[test]
    fn unmapped_bones_follow_their_parents() {
        let source = humanoid();
        // The target has an extra unmapped root above the pelvis.
        let mut bones = vec![Bone {
            name: "root".to_owned(),
            parent: None,
            rest: Pose3::from_translation(Vec3::new(0.0, 0.0, 0.3)),
        }];
        bones.extend(source.bones().iter().map(|bone| Bone {
            parent: Some(bone.parent.map_or(0, |parent| parent + 1)),
            ..bone.clone()
        }));
        let target = Skeleton::new(bones).unwrap();
        let unmapped = ["head", "l_forearm", "r_shin"];
        let mut mapping = same_names(&source);
        mapping.pairs.retain(|(name, _)| !unmapped.contains(&name.as_str()));
        let retargeter = Retargeter::new(
            &source,
            &target,
            &mapping,
            CoordinateConversion::IDENTITY,
            RootScale::Fixed(1.0),
        )
        .unwrap();
        let mut rng = Rng::new(14);
        let pose = rng.skeleton_pose(&source);
        let result = retargeter.apply(&pose);
        assert_eq!(result[0], target.bones()[0].rest, "unmapped root stays at rest");
        // The pelvis is a root: placed from the source, not from "root".
        assert!(result[1].translation.abs_diff_eq(pose[0].translation, 1.0e-6));
        for name in unmapped {
            let index = target.index_of(name).unwrap();
            let bone = &target.bones()[index];
            let expected = result[bone.parent.unwrap()] * bone.rest;
            assert_poses_eq(&result[index..=index], &[expected], 1.0e-6);
        }
        // Mapped bones still match the source.
        for (s, bone) in source.bones().iter().enumerate() {
            if !unmapped.contains(&bone.name.as_str()) {
                assert_poses_eq(&result[s + 1..=s + 1], &pose[s..=s], 1.0e-5);
            }
        }
    }

    #[test]
    fn bad_mappings_are_errors() {
        let skeleton = humanoid();
        let new = |text: &str, root_scale| {
            let mapping = Mapping::parse(text).unwrap();
            let conversion = CoordinateConversion::IDENTITY;
            Retargeter::new(&skeleton, &skeleton, &mapping, conversion, root_scale).err()
        };
        assert_eq!(new("# empty", RootScale::FromRest), Some(RetargetError::NoPairs));
        assert_eq!(
            new("pelvis pelvis\ntail spine", RootScale::FromRest),
            Some(RetargetError::UnknownSource("tail".to_owned()))
        );
        assert_eq!(
            new("pelvis hips", RootScale::FromRest),
            Some(RetargetError::UnknownTarget("hips".to_owned()))
        );
        assert_eq!(
            new("pelvis pelvis\nspine pelvis", RootScale::FromRest),
            Some(RetargetError::DuplicateTarget("pelvis".to_owned()))
        );
        assert_eq!(new("spine spine\nspine head", RootScale::FromRest), None);
        for scale in [0.0, -1.0] {
            assert_eq!(
                new("pelvis pelvis", RootScale::Fixed(scale)),
                Some(RetargetError::BadRootScale(scale))
            );
        }
        assert!(matches!(
            new("pelvis pelvis", RootScale::Fixed(f32::NAN)),
            Some(RetargetError::BadRootScale(_))
        ));

        // FromRest cannot scale a root that rests at the origin.
        let mut bones = skeleton.bones().to_vec();
        bones[0].rest = Pose3::IDENTITY;
        let at_origin = Skeleton::new(bones).unwrap();
        let mapping = same_names(&skeleton);
        let result = Retargeter::new(
            &skeleton,
            &at_origin,
            &mapping,
            CoordinateConversion::IDENTITY,
            RootScale::FromRest,
        );
        assert!(matches!(result.err(), Some(RetargetError::BadRootScale(_))));
    }

    #[test]
    fn mapping_parse() {
        let text = "# source target\nhips pelvis\n\n  spine_1 spine # lower spine\n";
        let mapping = Mapping::parse(text).unwrap();
        assert_eq!(
            mapping.pairs,
            vec![
                ("hips".to_owned(), "pelvis".to_owned()),
                ("spine_1".to_owned(), "spine".to_owned()),
            ]
        );
        assert_eq!(Mapping::parse("a b\nc\n").unwrap_err().line, 2);
        assert_eq!(Mapping::parse("a b c").unwrap_err().line, 1);
    }
}
