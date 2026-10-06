//! Bone hierarchies with rest poses.
//!
//! Parents precede their children, so one pass in index order meets every
//! parent before its children.
//!
//! Text format, for exchanging skeletons found by reverse engineering:
//!
//! ```text
//! # name parent tx ty tz qx qy qz qw
//! pelvis -      0 0.95 0  0 0 0 1
//! spine  pelvis 0 0.2  0  0 0 0 1
//! ```
//!
//! One bone per line: its name, the parent's name (`-` for a root; the parent
//! must be on an earlier line), then the rest pose relative to the parent as
//! a translation and a rotation quaternion `x y z w`. `#` starts a comment;
//! blank lines are ignored.

use std::collections::HashSet;
use std::fmt::{self, Write as _};

use glamx::{Pose3, Quat, Vec3};

use crate::ParseError;

/// Largest accepted `| |q|^2 - 1 |` of a rest rotation. Within it the
/// rotation is renormalized (text holds few digits); beyond it the four
/// numbers are not a rotation at all.
const UNIT_TOLERANCE: f32 = 1.0e-3;

#[derive(Clone, Debug, PartialEq)]
pub struct Bone {
    /// Unique; one token without `#` and not `-`, so the text format can carry it.
    pub name: String,
    /// Index of the parent bone, which precedes this one; `None` for a root.
    pub parent: Option<usize>,
    /// Rest pose relative to the parent, or to skeleton space for a root.
    pub rest: Pose3,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Skeleton {
    bones: Vec<Bone>,
}

#[derive(Debug, PartialEq)]
pub enum SkeletonError {
    /// Empty, `-`, or containing whitespace or `#`.
    BadName {
        bone: usize,
        name: String,
    },
    DuplicateName {
        bone: usize,
        name: String,
    },
    /// The parent index does not precede the bone.
    ParentOrder {
        bone: usize,
        parent: usize,
    },
    /// Non-finite values, or a rotation that is not a unit quaternion.
    BadRest {
        bone: usize,
    },
}

impl SkeletonError {
    fn bone(&self) -> usize {
        match *self {
            SkeletonError::BadName { bone, .. }
            | SkeletonError::DuplicateName { bone, .. }
            | SkeletonError::ParentOrder { bone, .. }
            | SkeletonError::BadRest { bone } => bone,
        }
    }
}

impl fmt::Display for SkeletonError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SkeletonError::BadName { bone, name } => {
                write!(
                    f,
                    "bone {bone}: name {name:?} must be one token without '#' and not '-'"
                )
            }
            SkeletonError::DuplicateName { bone, name } => write!(f, "bone {bone}: name {name} is used twice"),
            SkeletonError::ParentOrder { bone, parent } => {
                write!(f, "bone {bone}: parent {parent} does not precede it")
            }
            SkeletonError::BadRest { bone } => {
                write!(
                    f,
                    "bone {bone}: rest pose is not finite or its rotation is not a unit quaternion"
                )
            }
        }
    }
}

impl Skeleton {
    /// Validates the bones and renormalizes their rest rotations.
    pub fn new(mut bones: Vec<Bone>) -> Result<Self, SkeletonError> {
        let mut names = HashSet::new();
        for (index, bone) in bones.iter().enumerate() {
            let name = &bone.name;
            if name.is_empty() || name == "-" || name.contains(|c: char| c.is_whitespace() || c == '#') {
                return Err(SkeletonError::BadName {
                    bone: index,
                    name: name.clone(),
                });
            }
            if !names.insert(name.as_str()) {
                return Err(SkeletonError::DuplicateName {
                    bone: index,
                    name: name.clone(),
                });
            }
            if let Some(parent) = bone.parent.filter(|&parent| parent >= index) {
                return Err(SkeletonError::ParentOrder { bone: index, parent });
            }
            if !bone.rest.is_finite() || (bone.rest.rotation.length_squared() - 1.0).abs() > UNIT_TOLERANCE {
                return Err(SkeletonError::BadRest { bone: index });
            }
        }
        for bone in &mut bones {
            bone.rest.rotation = bone.rest.rotation.normalize();
        }
        Ok(Self { bones })
    }

    pub fn bones(&self) -> &[Bone] {
        &self.bones
    }

    pub fn index_of(&self, name: &str) -> Option<usize> {
        self.bones.iter().position(|bone| bone.name == name)
    }

    pub fn world_rest(&self) -> Vec<Pose3> {
        let rest: Vec<Pose3> = self.bones.iter().map(|bone| bone.rest).collect();
        self.to_world(&rest)
    }

    /// World poses from local ones.
    ///
    /// # Panics
    /// If `local` does not have one pose per bone.
    pub fn to_world(&self, local: &[Pose3]) -> Vec<Pose3> {
        assert_eq!(local.len(), self.bones.len(), "one local pose per bone");
        let mut world: Vec<Pose3> = Vec::with_capacity(local.len());
        for (bone, pose) in self.bones.iter().zip(local) {
            world.push(match bone.parent {
                Some(parent) => world[parent] * *pose,
                None => *pose,
            });
        }
        world
    }

    /// Local poses from world ones.
    ///
    /// # Panics
    /// If `world` does not have one pose per bone.
    pub fn to_local(&self, world: &[Pose3]) -> Vec<Pose3> {
        assert_eq!(world.len(), self.bones.len(), "one world pose per bone");
        self.bones
            .iter()
            .zip(world)
            .map(|(bone, pose)| match bone.parent {
                Some(parent) => world[parent].inv_mul(pose),
                None => *pose,
            })
            .collect()
    }

    /// Reads the text format described in the module docs.
    pub fn parse(text: &str) -> Result<Self, ParseError> {
        let mut bones: Vec<Bone> = Vec::new();
        let mut lines = Vec::new();
        for (line, fields) in crate::records(text) {
            let error = |message: String| ParseError { line, message };
            if fields.len() != 9 {
                return Err(error(format!(
                    "expected 9 fields (name parent tx ty tz qx qy qz qw), found {}",
                    fields.len()
                )));
            }
            let mut values = [0.0f32; 7];
            for (value, field) in values.iter_mut().zip(&fields[2..]) {
                *value = field.parse().map_err(|_| error(format!("{field:?} is not a number")))?;
            }
            let parent = match fields[1] {
                "-" => None,
                name => Some(
                    bones
                        .iter()
                        .position(|bone| bone.name == name)
                        .ok_or_else(|| error(format!("parent {name} is not defined on an earlier line")))?,
                ),
            };
            let [tx, ty, tz, qx, qy, qz, qw] = values;
            bones.push(Bone {
                name: fields[0].to_owned(),
                parent,
                rest: Pose3::from_parts(Vec3::new(tx, ty, tz), Quat::from_xyzw(qx, qy, qz, qw)),
            });
            lines.push(line);
        }
        Self::new(bones).map_err(|err| ParseError {
            line: lines[err.bone()],
            message: err.to_string(),
        })
    }

    /// Writes the text format; [`Skeleton::parse`] reads it back.
    pub fn to_text(&self) -> String {
        let mut text = String::from("# name parent tx ty tz qx qy qz qw (rest pose relative to the parent)\n");
        for bone in &self.bones {
            let parent = bone.parent.map_or("-", |parent| self.bones[parent].name.as_str());
            let (t, q) = (bone.rest.translation, bone.rest.rotation);
            // Display of f32 prints the shortest digits that parse back to the same value.
            let _ = writeln!(
                text,
                "{} {parent} {} {} {} {} {} {} {}",
                bone.name, t.x, t.y, t.z, q.x, q.y, q.z, q.w
            );
        }
        text
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::{HUMANOID, Rng, assert_poses_eq, humanoid};

    fn bone(name: &str, parent: Option<usize>) -> Bone {
        Bone {
            name: name.to_owned(),
            parent,
            rest: Pose3::IDENTITY,
        }
    }

    #[test]
    fn parses_names_parents_and_rest_poses() {
        let skeleton = humanoid();
        let bones = skeleton.bones();
        assert_eq!(bones.len(), 11);
        assert_eq!(bones[0].name, "pelvis");
        assert_eq!(bones[0].parent, None);
        assert_eq!(bones[4].name, "l_forearm");
        assert_eq!(bones[4].parent, Some(3));
        assert_eq!(bones[1].rest, Pose3::from_translation(Vec3::new(0.0, 0.2, 0.0)));
        assert_eq!(skeleton.index_of("r_shin"), Some(10));
        assert_eq!(skeleton.index_of("tail"), None);
        for bone in bones {
            assert!(bone.rest.rotation.is_normalized());
        }
    }

    #[test]
    fn text_round_trip() {
        let mut rng = Rng::new(1);
        let mut bones = humanoid().bones().to_vec();
        for bone in &mut bones {
            bone.rest = rng.pose(1.0);
        }
        let skeleton = Skeleton::new(bones).unwrap();
        let text = skeleton.to_text();
        let parsed = Skeleton::parse(&text).unwrap();
        for (a, b) in skeleton.bones().iter().zip(parsed.bones()) {
            assert_eq!((&a.name, a.parent), (&b.name, b.parent));
        }
        let rest = |s: &Skeleton| s.bones().iter().map(|b| b.rest).collect::<Vec<_>>();
        assert_poses_eq(&rest(&skeleton), &rest(&parsed), 1.0e-6);
    }

    #[test]
    fn world_rest_composes_parents() {
        let world = humanoid().world_rest();
        let at = |i: usize| world[i].translation;
        assert!(at(2).abs_diff_eq(Vec3::new(0.0, 1.55, 0.0), 1.0e-6));
        // The left arm is turned so that its +Y points along world +X.
        assert!(at(4).abs_diff_eq(Vec3::new(0.48, 1.45, 0.0), 1.0e-6));
        assert!(at(6).abs_diff_eq(Vec3::new(-0.48, 1.45, 0.0), 1.0e-6));
        // Thighs point down.
        assert!(at(8).abs_diff_eq(Vec3::new(0.1, 0.48, 0.0), 1.0e-6));
    }

    #[test]
    fn local_and_world_are_inverse() {
        let skeleton = humanoid();
        let mut rng = Rng::new(2);
        let local: Vec<Pose3> = skeleton.bones().iter().map(|_| rng.pose(1.0)).collect();
        let world = skeleton.to_world(&local);
        assert_poses_eq(&skeleton.to_local(&world), &local, 1.0e-5);
        assert_poses_eq(&skeleton.to_world(&skeleton.to_local(&world)), &world, 1.0e-5);
    }

    #[test]
    fn invalid_bones_are_errors() {
        let check = |bones: Vec<Bone>, expected: SkeletonError| {
            assert_eq!(Skeleton::new(bones), Err(expected));
        };
        for name in ["", "-", "left arm", "a#b"] {
            check(
                vec![bone("root", None), bone(name, Some(0))],
                SkeletonError::BadName {
                    bone: 1,
                    name: name.to_owned(),
                },
            );
        }
        check(
            vec![bone("root", None), bone("root", Some(0))],
            SkeletonError::DuplicateName {
                bone: 1,
                name: "root".to_owned(),
            },
        );
        check(
            vec![bone("a", Some(1)), bone("b", None)],
            SkeletonError::ParentOrder { bone: 0, parent: 1 },
        );
        check(
            vec![bone("a", Some(0))],
            SkeletonError::ParentOrder { bone: 0, parent: 0 },
        );
        let mut bad = bone("a", None);
        bad.rest.rotation = Quat::from_xyzw(0.0, 0.0, 0.0, 0.0);
        check(vec![bad.clone()], SkeletonError::BadRest { bone: 0 });
        bad.rest = Pose3::from_translation(Vec3::new(f32::NAN, 0.0, 0.0));
        check(vec![bad], SkeletonError::BadRest { bone: 0 });
        assert_eq!(Skeleton::new(Vec::new()).map(|s| s.bones().len()), Ok(0));
    }

    #[test]
    fn parse_errors_name_the_line() {
        let line_of = |text: &str| Skeleton::parse(text).unwrap_err().line;
        let first = HUMANOID.lines().take(3).collect::<Vec<_>>().join("\n");
        // Lines 1-3 are the header, pelvis and spine.
        assert_eq!(line_of(&format!("{first}\nhead spine 0 0.4 0 0 0 0")), 4);
        assert_eq!(line_of(&format!("{first}\nhead spine 0 0.4 zero 0 0 0 1")), 4);
        assert_eq!(line_of(&format!("{first}\nhead neck 0 0.4 0 0 0 0 1")), 4);
        assert_eq!(line_of(&format!("{first}\n\nspine pelvis 0 0.4 0 0 0 0 1")), 5);
        assert_eq!(line_of(&format!("{first}\nhead spine 0 0.4 0 0 0 0 2")), 4);
        assert_eq!(line_of(&format!("{first}\nhead spine 0 0.4 NaN 0 0 0 1")), 4);
        // A child before its parent is an unknown parent.
        assert_eq!(line_of("spine pelvis 0 0 0 0 0 0 1\npelvis - 0 0 0 0 0 0 1"), 1);
        assert!(Skeleton::parse("# nothing\n\n").unwrap().bones().is_empty());
    }
}
