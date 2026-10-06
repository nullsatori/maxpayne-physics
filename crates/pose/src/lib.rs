//! Skeletons and poses, independent of either game.
//!
//! Max Payne 3 streams world poses of its puppets' bones; Max Payne 1 needs
//! poses of its own skeletons, in its own coordinates and matrix format. The
//! steps in between are plain math and live here, testable on the host:
//!
//! * [`skeleton`]: bone hierarchies with rest poses, and a text format for them;
//! * [`coords`]: axis and unit conversion between the games' world spaces;
//! * [`retarget`]: maps poses of one skeleton onto another;
//! * [`matrix`]: candidate layouts of MP1's 4x3 bone matrices.
//!
//! A bone's [`Pose3`] maps points from the bone's frame to its parent's frame
//! (a local pose) or to the skeleton's space (a world pose). Pose arrays are
//! indexed like the bones of their skeleton.

pub mod coords;
pub mod matrix;
pub mod retarget;
pub mod skeleton;

use std::fmt;

use bridge::BoneTransform;
use glamx::{Pose3, Quat, Vec3};

/// Both store quaternions as `[x, y, z, w]`; values are copied, not renormalized.
pub fn pose_from_bone(bone: &BoneTransform) -> Pose3 {
    Pose3::from_parts(Vec3::from_array(bone.translation), Quat::from_array(bone.rotation))
}

pub fn bone_from_pose(pose: &Pose3) -> BoneTransform {
    BoneTransform {
        translation: pose.translation.to_array(),
        rotation: pose.rotation.to_array(),
    }
}

/// A line of a text file that could not be read.
#[derive(Debug, PartialEq)]
pub struct ParseError {
    /// 1-based.
    pub line: usize,
    pub message: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "line {}: {}", self.line, self.message)
    }
}

/// Whitespace-separated fields of each line that has any once `#` comments
/// are removed, with 1-based line numbers.
fn records(text: &str) -> impl Iterator<Item = (usize, Vec<&str>)> {
    text.lines().enumerate().filter_map(|(index, line)| {
        let content = line.split('#').next().unwrap_or_default();
        let fields: Vec<&str> = content.split_whitespace().collect();
        (!fields.is_empty()).then_some((index + 1, fields))
    })
}

#[cfg(test)]
mod testutil {
    use glamx::{Pose3, Quat, Vec3};

    use crate::skeleton::Skeleton;

    /// Bones point along their local +Y; arms and legs are rotated at rest so
    /// that rest rotations matter. Y up, meters, feet near y = 0.
    pub const HUMANOID: &str = "\
# name     parent  tx    ty    tz   qx qy qz         qw
pelvis     -       0     0.95  0    0  0  0          1
spine      pelvis  0     0.2   0    0  0  0          1
head       spine   0     0.4   0    0  0  0          1
l_arm      spine   0.2   0.3   0    0  0  -0.7071068 0.7071068
l_forearm  l_arm   0     0.28  0    0  0  0          1
r_arm      spine   -0.2  0.3   0    0  0  0.7071068  0.7071068
r_forearm  r_arm   0     0.28  0    0  0  0          1
l_thigh    pelvis  0.1   -0.05 0    0  0  1          0
l_shin     l_thigh 0     0.42  0    0  0  0          1
r_thigh    pelvis  -0.1  -0.05 0    0  0  1          0
r_shin     r_thigh 0     0.42  0    0  0  0          1
";

    pub fn humanoid() -> Skeleton {
        Skeleton::parse(HUMANOID).unwrap()
    }

    /// Deterministic xorshift generator, so failures reproduce.
    pub struct Rng(u64);

    impl Rng {
        pub fn new(seed: u64) -> Self {
            Self(seed | 1)
        }

        /// Uniform in [-1, 1).
        pub fn signed(&mut self) -> f32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 40) as f32 / (1u64 << 23) as f32 - 1.0
        }

        pub fn vec3(&mut self, range: f32) -> Vec3 {
            Vec3::new(self.signed(), self.signed(), self.signed()) * range
        }

        /// Uniformly distributed: rejection-sampled from the unit 4-ball.
        pub fn rotation(&mut self) -> Quat {
            loop {
                let q = Quat::from_xyzw(self.signed(), self.signed(), self.signed(), self.signed());
                if (0.01..=1.0).contains(&q.length_squared()) {
                    return q.normalize();
                }
            }
        }

        pub fn pose(&mut self, range: f32) -> Pose3 {
            Pose3::from_parts(self.vec3(range), self.rotation())
        }

        /// World pose of `skeleton` with random joint rotations, rest bone
        /// lengths and a random root position.
        pub fn skeleton_pose(&mut self, skeleton: &Skeleton) -> Vec<Pose3> {
            let local: Vec<Pose3> = skeleton
                .bones()
                .iter()
                .map(|bone| match bone.parent {
                    Some(_) => Pose3::from_parts(bone.rest.translation, self.rotation()),
                    None => self.pose(2.0),
                })
                .collect();
            skeleton.to_world(&local)
        }
    }

    pub fn same_rotation(a: Quat, b: Quat, tolerance: f32) -> bool {
        a.abs_diff_eq(b, tolerance) || a.abs_diff_eq(-b, tolerance)
    }

    pub fn assert_poses_eq(a: &[Pose3], b: &[Pose3], tolerance: f32) {
        assert_eq!(a.len(), b.len());
        for (i, (a, b)) in a.iter().zip(b).enumerate() {
            assert!(
                a.translation.abs_diff_eq(b.translation, tolerance) && same_rotation(a.rotation, b.rotation, tolerance),
                "bone {i}: {a:?} != {b:?}"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bone_transform_round_trip() {
        let bone = BoneTransform {
            translation: [1.0, -2.5, 3.25],
            rotation: [0.5, -0.5, 0.5, 0.5],
        };
        let pose = pose_from_bone(&bone);
        assert_eq!(pose.translation, Vec3::new(1.0, -2.5, 3.25));
        assert_eq!(pose.rotation, Quat::from_xyzw(0.5, -0.5, 0.5, 0.5));
        assert_eq!(bone_from_pose(&pose), bone);
    }

    #[test]
    fn records_skip_comments_and_blank_lines() {
        let text = "# header\n\n a b # trailing\n\t\r\nc\r\n#\n";
        let records: Vec<_> = records(text).collect();
        assert_eq!(records, vec![(3, vec!["a", "b"]), (5, vec!["c"])]);
    }
}
