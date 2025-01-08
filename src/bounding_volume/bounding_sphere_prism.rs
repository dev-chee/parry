use crate::bounding_volume::BoundingSphere;
use crate::math::{Isometry, Point, Real};
use crate::shape::Prism;
use na::ComplexField;

impl Prism {
    /// Computes the world-space bounding sphere of this prism, transformed by `pos`.
    #[inline]
    pub fn bounding_sphere(&self, pos: &Isometry<Real>) -> BoundingSphere {
        self.local_bounding_sphere().transform_by(pos)
    }

    /// Computes the local-space bounding sphere of this prism.
    #[inline]
    pub fn local_bounding_sphere(&self) -> BoundingSphere {
        let (sin, cos) = ComplexField::sin_cos(self.angle_with_x - self.half_angle);
        let dir0 = na::Vector3::new(cos, na::zero(), sin) * self.radius;
        let (sin, cos) = ComplexField::sin_cos(self.angle_with_x + self.half_angle);
        let dir1 = na::Vector3::new(cos, na::zero(), sin) * self.radius;

        let dr = Real::max((dir1 - dir0).norm(), self.radius) * 0.5;
        let radius = ComplexField::sqrt(dr * dr + self.half_height * self.half_height);
        let (sin, cos) = ComplexField::sin_cos(self.angle_with_x);
        let origin = Point::new(cos * dr, na::zero(), sin * dr);
        BoundingSphere::new(origin, radius)
    }
}
