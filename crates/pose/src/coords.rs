//! Conversion from one game's world space to the other's.
//!
//! A conversion is the linear map `p' = scale * B p` with an orthonormal
//! basis `B`; `B` is a reflection (determinant -1) when the two spaces differ
//! in handedness. Rotations convert by conjugation, `R' = B R B^T`. That is a
//! proper rotation even when `B` reflects, and it keeps the conversion
//! consistent: converting `R p` gives `R' p'`. There is no offset; both
//! spaces share their origin.

use std::fmt;

use glamx::{Mat3, Pose3, Quat, Vec3};

/// Largest accepted deviation of `B^T B` from identity, per element.
const ORTHONORMAL_TOLERANCE: f32 = 1.0e-4;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoordinateConversion {
    basis: Mat3,
    scale: f32,
    /// The rotation left when the reflection is taken out: `basis = sign * proper`.
    /// Conjugating by it equals conjugating by `basis`, since the signs cancel.
    proper: Quat,
    sign: f32,
}

#[derive(Debug, PartialEq)]
pub enum CoordsError {
    NotOrthonormal,
    /// The scale must be positive and finite.
    BadScale(f32),
    /// The axis spec is not three distinct signed axes such as `x,-z,y`.
    BadAxes(String),
}

impl fmt::Display for CoordsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CoordsError::NotOrthonormal => write!(f, "basis is not orthonormal"),
            CoordsError::BadScale(scale) => write!(f, "scale {scale} is not a positive finite number"),
            CoordsError::BadAxes(spec) => write!(f, "axis spec {spec:?} is not three distinct signed axes like x,-z,y"),
        }
    }
}

impl CoordinateConversion {
    pub const IDENTITY: Self = Self {
        basis: Mat3::IDENTITY,
        scale: 1.0,
        proper: Quat::IDENTITY,
        sign: 1.0,
    };

    pub fn new(basis: Mat3, scale: f32) -> Result<Self, CoordsError> {
        if !(scale.is_finite() && scale > 0.0) {
            return Err(CoordsError::BadScale(scale));
        }
        if !basis.is_finite() || !(basis.transpose() * basis).abs_diff_eq(Mat3::IDENTITY, ORTHONORMAL_TOLERANCE) {
            return Err(CoordsError::NotOrthonormal);
        }
        // Orthonormal, so the determinant is +1 or -1.
        let sign = basis.determinant().signum();
        Ok(Self {
            basis,
            scale,
            proper: Quat::from_mat3(&(basis * sign)).normalize(),
            sign,
        })
    }

    /// Builds the basis from an axis spec: three comma-separated items, one
    /// per target axis in the order x', y', z', each naming the source axis
    /// (`x`, `y` or `z`, optionally signed `-` or `+`) whose coordinate it
    /// takes. So `"x,-z,y"` means x' = x, y' = -z, z' = y, that is
    /// `p' = scale * (p.x, -p.z, p.y)`, and the items are the rows of `B`.
    /// Each source axis must appear once. An odd number of minus signs or an
    /// odd permutation (one swap, such as `y,x,z`), but not both, makes a
    /// reflection.
    pub fn from_axes(spec: &str, scale: f32) -> Result<Self, CoordsError> {
        let bad = || CoordsError::BadAxes(spec.to_owned());
        let mut items = spec.split(',');
        let mut rows = [Vec3::ZERO; 3];
        for row in &mut rows {
            let item = items.next().ok_or_else(bad)?.trim();
            let (sign, axis) = match item.strip_prefix('-') {
                Some(axis) => (-1.0, axis),
                None => (1.0, item.strip_prefix('+').unwrap_or(item)),
            };
            let axis = match axis {
                "x" => Vec3::X,
                "y" => Vec3::Y,
                "z" => Vec3::Z,
                _ => return Err(bad()),
            };
            *row = sign * axis;
        }
        if items.next().is_some() {
            return Err(bad());
        }
        let basis = Mat3::from_rows(rows[0], rows[1], rows[2]);
        // Signed unit axes give determinant +-1, or 0 when one repeats.
        if basis.determinant() == 0.0 {
            return Err(bad());
        }
        Self::new(basis, scale)
    }

    pub fn basis(&self) -> Mat3 {
        self.basis
    }

    pub fn scale(&self) -> f32 {
        self.scale
    }

    /// Whether the conversion flips handedness.
    pub fn is_reflection(&self) -> bool {
        self.sign < 0.0
    }

    pub fn inverse(&self) -> Self {
        Self {
            basis: self.basis.transpose(),
            scale: 1.0 / self.scale,
            proper: self.proper.inverse(),
            sign: self.sign,
        }
    }

    pub fn point(&self, p: Vec3) -> Vec3 {
        self.scale * (self.basis * p)
    }

    pub fn rotation(&self, rotation: Quat) -> Quat {
        self.proper * rotation * self.proper.inverse()
    }

    pub fn pose(&self, pose: &Pose3) -> Pose3 {
        Pose3::from_parts(self.point(pose.translation), self.rotation(pose.rotation))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::{Rng, same_rotation};

    /// Proper and improper bases, with and without unit changes.
    fn conversions() -> Vec<CoordinateConversion> {
        let specs = [
            ("x,y,z", 1.0),
            ("x,-z,y", 1.0),
            ("-x,y,z", 1.0),
            ("x,z,y", 100.0),
            ("-y,+z,-x", 0.01),
            ("z,x,y", 2.5),
        ];
        specs
            .iter()
            .map(|&(spec, scale)| CoordinateConversion::from_axes(spec, scale).unwrap())
            .collect()
    }

    fn rotation_matrix_is_proper(q: Quat) -> bool {
        let m = Mat3::from_quat(q);
        (m.transpose() * m).abs_diff_eq(Mat3::IDENTITY, 1.0e-5) && (m.determinant() - 1.0).abs() < 1.0e-5
    }

    #[test]
    fn axis_spec_lists_source_axes_per_target_axis() {
        let conversion = CoordinateConversion::from_axes("x,-z,y", 2.0).unwrap();
        assert_eq!(conversion.point(Vec3::new(1.0, 2.0, 3.0)), Vec3::new(2.0, -6.0, 4.0));
        assert!(!conversion.is_reflection());
        assert_eq!(conversion.scale(), 2.0);
        let mirror = CoordinateConversion::from_axes(" -x , y , z ", 1.0).unwrap();
        assert_eq!(mirror.point(Vec3::new(1.0, 2.0, 3.0)), Vec3::new(-1.0, 2.0, 3.0));
        assert!(mirror.is_reflection());
        assert!(CoordinateConversion::from_axes("y,x,z", 1.0).unwrap().is_reflection());
        assert!(!CoordinateConversion::from_axes("-y,x,z", 1.0).unwrap().is_reflection());
        assert_eq!(
            CoordinateConversion::from_axes("x,y,z", 1.0),
            Ok(CoordinateConversion::IDENTITY)
        );
    }

    #[test]
    fn invalid_specs_and_bases_are_errors() {
        for spec in ["", "x,y", "x,y,z,x", "x,x,z", "x,y,w", "x,--y,z", "X,Y,Z", "x y z"] {
            assert_eq!(
                CoordinateConversion::from_axes(spec, 1.0),
                Err(CoordsError::BadAxes(spec.to_owned())),
                "{spec:?}"
            );
        }
        for scale in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            assert!(matches!(
                CoordinateConversion::from_axes("x,y,z", scale),
                Err(CoordsError::BadScale(_))
            ));
        }
        let sheared = Mat3::from_cols(Vec3::X, Vec3::new(0.1, 1.0, 0.0), Vec3::Z);
        assert_eq!(
            CoordinateConversion::new(sheared, 1.0),
            Err(CoordsError::NotOrthonormal)
        );
        assert_eq!(
            CoordinateConversion::new(Mat3::IDENTITY * 2.0, 1.0),
            Err(CoordsError::NotOrthonormal)
        );
        let nan = Mat3::from_cols(Vec3::X, Vec3::Y, Vec3::new(0.0, 0.0, f32::NAN));
        assert_eq!(CoordinateConversion::new(nan, 1.0), Err(CoordsError::NotOrthonormal));
    }

    #[test]
    fn arbitrary_orthonormal_basis_is_accepted() {
        let mut rng = Rng::new(3);
        for _ in 0..50 {
            let rotation = Mat3::from_quat(rng.rotation());
            let reflected = rotation * Mat3::from_diagonal(Vec3::new(1.0, 1.0, -1.0));
            assert!(!CoordinateConversion::new(rotation, 1.0).unwrap().is_reflection());
            assert!(CoordinateConversion::new(reflected, 1.0).unwrap().is_reflection());
        }
    }

    #[test]
    fn rotations_stay_proper_and_match_conjugation() {
        let mut rng = Rng::new(4);
        for conversion in conversions() {
            let b = conversion.basis();
            for _ in 0..100 {
                let r = rng.rotation();
                let converted = conversion.rotation(r);
                assert!(rotation_matrix_is_proper(converted), "{conversion:?}");
                let expected = b * Mat3::from_quat(r) * b.transpose();
                assert!(
                    Mat3::from_quat(converted).abs_diff_eq(expected, 1.0e-5),
                    "{conversion:?}"
                );
            }
        }
    }

    #[test]
    fn inverse_undoes_conversion() {
        let mut rng = Rng::new(5);
        for conversion in conversions() {
            let inverse = conversion.inverse();
            for _ in 0..100 {
                let pose = rng.pose(10.0);
                let back = inverse.pose(&conversion.pose(&pose));
                assert!(back.translation.abs_diff_eq(pose.translation, 1.0e-4), "{conversion:?}");
                assert!(same_rotation(back.rotation, pose.rotation, 1.0e-5), "{conversion:?}");
            }
        }
    }

    #[test]
    fn converting_a_rotated_point_rotates_the_converted_point() {
        let mut rng = Rng::new(6);
        for conversion in conversions() {
            for _ in 0..100 {
                let (r, p) = (rng.rotation(), rng.vec3(10.0));
                let tolerance = 1.0e-4 * conversion.scale().max(1.0);
                let direct = conversion.point(r * p);
                let composed = conversion.rotation(r) * conversion.point(p);
                assert!(direct.abs_diff_eq(composed, tolerance), "{conversion:?}");

                let (a, b) = (rng.pose(10.0), rng.pose(10.0));
                let whole = conversion.pose(&(a * b));
                let parts = conversion.pose(&a) * conversion.pose(&b);
                assert!(whole.translation.abs_diff_eq(parts.translation, tolerance));
                assert!(same_rotation(whole.rotation, parts.rotation, 1.0e-5));
            }
        }
    }
}
