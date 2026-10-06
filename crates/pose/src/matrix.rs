//! Candidate layouts of Max Payne 1 bone matrices.
//!
//! MP1 passes bone transforms as `M_Matrix4x3Template<float>`: twelve floats
//! in an order that is not known yet. Written for column vectors, a rigid
//! transform is the 3x4 matrix `[R | t]` with `p' = R p + t`. The candidates:
//!
//! * [`Shape::Rows4x3`]: a 3x3 block in `m[0..9]`, then the translation in `m[9..12]`;
//! * [`Shape::Rows3x4`]: three rows of four, with the translation in `m[3]`, `m[7]`, `m[11]`.
//!
//! The 3x3 block is stored row by row, as `R` or, with [`Layout::transposed`],
//! as `R^T`. `Rows3x4` untransposed is `[R | t]` stored row by row, which is
//! also the row-vector matrix (`p' = p M`, Direct3D style) stored column by
//! column. `Rows4x3` transposed is that row-vector 4x3 matrix stored row by
//! row, which is also `[R | t]` stored column by column. The other two
//! combinations cover a separate 3x3 block in the other convention.
//!
//! [`decode`] accepts a matrix whose block is a rotation times a uniform
//! scale. Reading the wrong shape mixes translation into the block, which
//! fails that test unless the translation happens to fit, so [`classify`]
//! tells the shapes apart over many samples. The transpose flag cannot be
//! found this way: `R^T` is a rotation whenever `R` is, so every rigid matrix
//! decodes under both flags (to inverse rotations). It has to come from an
//! experiment in the game, e.g. writing a known rotation such as a quarter
//! turn of the head and seeing which way it turns, or, if the matrices turn
//! out to be bone world transforms, checking that a child's offset in its
//! parent's frame, `R_parent^T (t_child - t_parent)`, stays constant over
//! animation frames, which only holds with the right flag.

use glamx::{Mat3, Pose3, Quat, Vec3};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Shape {
    Rows4x3,
    Rows3x4,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Layout {
    pub shape: Shape,
    /// The 3x3 block holds `R^T` rather than `R`.
    pub transposed: bool,
}

impl Layout {
    pub const ALL: [Layout; 4] = [
        Layout {
            shape: Shape::Rows4x3,
            transposed: false,
        },
        Layout {
            shape: Shape::Rows4x3,
            transposed: true,
        },
        Layout {
            shape: Shape::Rows3x4,
            transposed: false,
        },
        Layout {
            shape: Shape::Rows3x4,
            transposed: true,
        },
    ];

    /// Where entry `(row, col)` of `[R | t]` is stored; column 3 is `t`.
    fn index(self, row: usize, col: usize) -> usize {
        let (r, c) = if col < 3 && self.transposed {
            (col, row)
        } else {
            (row, col)
        };
        match (self.shape, c) {
            (Shape::Rows4x3, 3) => 9 + r,
            (Shape::Rows4x3, _) => 3 * r + c,
            (Shape::Rows3x4, _) => 4 * r + c,
        }
    }
}

/// Largest accepted deviation of `M^T M / s^2` from identity per element,
/// where `M` is the 3x3 block and `s` its uniform scale.
pub const TOLERANCE: f32 = 1.0e-3;

/// Smallest accepted uniform scale; below it the rotation is unrecoverable.
const MIN_SCALE: f32 = 1.0e-6;

pub fn encode(pose: &Pose3, layout: Layout) -> [f32; 12] {
    let rotation = Mat3::from_quat(pose.rotation);
    let mut m = [0.0; 12];
    for row in 0..3 {
        for col in 0..3 {
            m[layout.index(row, col)] = rotation.col(col)[row];
        }
        m[layout.index(row, 3)] = pose.translation[row];
    }
    m
}

/// The pose and uniform scale stored in `m`, or `None` when the 3x3 block is
/// not a rotation times a positive uniform scale.
pub fn decode(m: &[f32; 12], layout: Layout) -> Option<(Pose3, f32)> {
    if !m.iter().all(|v| v.is_finite()) {
        return None;
    }
    let column = |col| {
        Vec3::new(
            m[layout.index(0, col)],
            m[layout.index(1, col)],
            m[layout.index(2, col)],
        )
    };
    let block = Mat3::from_cols(column(0), column(1), column(2));
    // A reflection has a negative determinant, hence a negative scale.
    let scale = block.determinant().cbrt();
    if scale.is_nan() || scale < MIN_SCALE {
        return None;
    }
    let rotation = block * scale.recip();
    if !(rotation.transpose() * rotation).abs_diff_eq(Mat3::IDENTITY, TOLERANCE) {
        return None;
    }
    let pose = Pose3::from_parts(column(3), Quat::from_mat3(&rotation).normalize());
    Some((pose, scale))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ShapeScore {
    pub shape: Shape,
    /// Samples that decode as a rigid transform with a uniform scale.
    pub rigid: usize,
}

/// Scores both shapes over `samples`, such as every bone matrix of a few
/// frames, best first. The transpose flag does not change the scores; see
/// the module docs.
pub fn classify(samples: &[[f32; 12]]) -> [ShapeScore; 2] {
    let score = |shape| {
        let layout = Layout {
            shape,
            transposed: false,
        };
        ShapeScore {
            shape,
            rigid: samples.iter().filter(|m| decode(m, layout).is_some()).count(),
        }
    };
    let mut scores = [score(Shape::Rows4x3), score(Shape::Rows3x4)];
    scores.sort_by_key(|score| std::cmp::Reverse(score.rigid));
    scores
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::{Rng, same_rotation};

    fn assert_close(a: &[f32; 12], b: &[f32; 12]) {
        assert!(a.iter().zip(b).all(|(a, b)| (a - b).abs() < 1.0e-6), "{a:?} != {b:?}");
    }

    #[test]
    fn layouts_store_documented_entries() {
        // R turns x into y; [R | t] = [[0 -1 0 1] [1 0 0 2] [0 0 1 3]].
        let pose = Pose3::from_parts(
            Vec3::new(1.0, 2.0, 3.0),
            Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
        );
        let [rows4x3, rows4x3_t, rows3x4, rows3x4_t] = Layout::ALL.map(|layout| encode(&pose, layout));
        assert_close(&rows4x3, &[0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 3.0]);
        assert_close(
            &rows4x3_t,
            &[0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 3.0],
        );
        assert_close(&rows3x4, &[0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0, 3.0]);
        assert_close(
            &rows3x4_t,
            &[0.0, 1.0, 0.0, 1.0, -1.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0, 3.0],
        );
    }

    #[test]
    fn every_layout_round_trips() {
        let mut rng = Rng::new(20);
        for layout in Layout::ALL {
            for _ in 0..200 {
                let pose = rng.pose(10.0);
                let (decoded, scale) = decode(&encode(&pose, layout), layout).unwrap();
                assert!(decoded.translation.abs_diff_eq(pose.translation, 1.0e-6), "{layout:?}");
                assert!(same_rotation(decoded.rotation, pose.rotation, 1.0e-5), "{layout:?}");
                assert!((scale - 1.0).abs() < 1.0e-5);
            }
        }
    }

    #[test]
    fn uniform_scale_is_reported() {
        let mut rng = Rng::new(21);
        for layout in Layout::ALL {
            let pose = rng.pose(10.0);
            let mut m = encode(&pose, layout);
            for row in 0..3 {
                for col in 0..3 {
                    m[layout.index(row, col)] *= 2.5;
                }
            }
            let (decoded, scale) = decode(&m, layout).unwrap();
            assert!((scale - 2.5).abs() < 1.0e-5);
            assert!(same_rotation(decoded.rotation, pose.rotation, 1.0e-5));
            assert_eq!(decoded.translation, pose.translation);
        }
    }

    #[test]
    fn float_noise_is_tolerated_but_stretch_is_not() {
        let mut rng = Rng::new(25);
        for layout in Layout::ALL {
            let pose = rng.pose(5.0);
            // Stretching one axis by 1 + e moves M^T M / s^2 about 4e/3 from identity.
            let stretched = |e: f32| {
                let mut m = encode(&pose, layout);
                for row in 0..3 {
                    m[layout.index(row, 1)] *= 1.0 + e;
                }
                m
            };
            assert!(decode(&stretched(1.0e-4), layout).is_some(), "{layout:?}");
            assert_eq!(decode(&stretched(3.0e-3), layout), None, "{layout:?}");
        }
    }

    #[test]
    fn transpose_flag_is_not_observable_in_one_matrix() {
        let mut rng = Rng::new(22);
        for layout in Layout::ALL {
            let flipped = Layout {
                transposed: !layout.transposed,
                ..layout
            };
            let pose = rng.pose(10.0);
            let (decoded, _) = decode(&encode(&pose, layout), flipped).unwrap();
            assert!(same_rotation(decoded.rotation, pose.rotation.inverse(), 1.0e-5));
        }
    }

    #[test]
    fn classify_finds_the_shape() {
        let mut rng = Rng::new(23);
        for layout in Layout::ALL {
            let mut samples: Vec<[f32; 12]> = (0..200).map(|_| encode(&rng.pose(5.0), layout)).collect();
            // Bones at the origin without rotation, as unused slots might be.
            samples.extend([encode(&Pose3::IDENTITY, layout); 10]);
            let [best, other] = classify(&samples);
            assert_eq!(best.shape, layout.shape, "{layout:?}");
            assert_eq!(best.rigid, samples.len());
            assert_eq!(other.rigid, 0, "{layout:?}");
        }
    }

    #[test]
    fn garbage_is_rejected() {
        let mut rng = Rng::new(24);
        let garbage: Vec<[f32; 12]> = (0..1000)
            .map(|_| std::array::from_fn(|_| rng.signed() * 10.0))
            .collect();
        for layout in Layout::ALL {
            assert_eq!(garbage.iter().filter(|m| decode(m, layout).is_some()).count(), 0);
        }
        assert!(classify(&garbage).iter().all(|score| score.rigid == 0));

        let pose = rng.pose(5.0);
        for layout in Layout::ALL {
            let good = encode(&pose, layout);
            let with = |row: usize, col: usize, f: &dyn Fn(f32) -> f32| {
                let mut m = good;
                m[layout.index(row, col)] = f(m[layout.index(row, col)]);
                m
            };
            let mut mirrored = good;
            for row in 0..3 {
                mirrored[layout.index(row, 0)] *= -1.0;
            }
            let mut stretched = good;
            for row in 0..3 {
                stretched[layout.index(row, 1)] *= 2.0;
            }
            let rejected = [
                [0.0; 12],
                mirrored,
                stretched,
                with(0, 1, &|v| v + 0.1),
                with(2, 3, &|_| f32::NAN),
                with(1, 1, &|_| f32::INFINITY),
            ];
            for m in rejected {
                assert_eq!(decode(&m, layout), None, "{layout:?} {m:?}");
            }
        }
    }
}
