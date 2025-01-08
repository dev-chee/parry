use super::SupportMap;
use crate::math::{self, Point, Real, Vector};
#[cfg(feature = "std")]
use either::Either;
use na::{ComplexField, RealField};

/// Prism shape with its principal axis aligned with the `y` axis.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "bytemuck", derive(bytemuck::Pod, bytemuck::Zeroable))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize, CheckBytes),
    archive(as = "Self")
)]
#[derive(PartialEq, Debug, Copy, Clone)]
#[repr(C)]
pub struct Prism {
    /// The half-height of the prism.
    pub half_height: Real,
    /// The radius of the prism.
    pub radius: Real,
    /// The angle between the central axis and the x-axis.
    pub angle_with_x: Real,
    /// The half-angle of the prism.
    pub half_angle: Real,
}

impl Prism {
    /// Create a new prism.
    /// # Arguments
    /// * `half_height` - the half length of the prism along the `y` axis.
    /// * `radius` - the radius of the prism's cross-section.
    /// * `angle_with_x` - the angle between the central axis and the x-axis.
    /// * `half_angle` - the half angle of the prism's cross-section.
    pub fn new(half_height: Real, radius: Real, angle_with_x: Real, half_angle: Real) -> Self {
        assert!(
            half_height > na::zero()
                && radius > na::zero()
                && ComplexField::abs(angle_with_x) < RealField::two_pi()
                && half_angle > na::zero()
                && half_angle <= RealField::frac_pi_2()
        );
        Self {
            half_height,
            radius,
            angle_with_x,
            half_angle,
        }
    }

    /// Computes a scaled version of this prism.
    #[cfg(feature = "std")]
    #[inline]
    pub fn scaled(
        self,
        scale: &Vector<Real>,
        nsubdivs: u32,
    ) -> Option<Either<Self, super::ConvexPolyhedron>> {
        if scale.x != scale.z {
            let (mut vtx, idx) = self.to_trimesh(nsubdivs);
            vtx.iter_mut()
                .for_each(|pt| pt.coords = pt.coords.component_mul(scale));
            Some(Either::Right(super::ConvexPolyhedron::from_convex_mesh(
                vtx, &idx,
            )?))
        } else {
            Some(Either::Left(Self::new(
                self.half_height * scale.y,
                self.radius * scale.x,
                self.angle_with_x,
                self.half_angle,
            )))
        }
    }

    fn edge(&self, dir: &Vector<Real>, central: &Vector<Real>) -> Vector<Real> {
        // dir is in the same direction as the central axis
        let (sin, cos) = if dir.x - central.x > math::DEFAULT_EPSILON {
            ComplexField::sin_cos(self.angle_with_x - self.half_angle)
        } else {
            ComplexField::sin_cos(self.angle_with_x + self.half_angle)
        };

        dir * (self.radius * Vector::new(cos, na::zero(), sin)).dot(&dir)
    }
}

impl SupportMap for Prism {
    fn local_support_point(&self, dir: &Vector<Real>) -> Point<Real> {
        let mut vres = *dir;
        vres[1] = na::zero();

        if vres.normalize_mut() <= math::DEFAULT_EPSILON {
            vres = na::zero()
        } else {
            let (sin, cos) = ComplexField::sin_cos(self.angle_with_x);
            let central = Vector::new(cos, na::zero(), sin);

            // calculate the dot product of the direction vector and the central axis
            let dot = vres.dot(&central);
            const ANGLE_EPSILON: Real = 0.0000001;
            if ComplexField::abs(dot) < ANGLE_EPSILON {
                // perpendicular to the central axis
                vres = self.edge(&vres, &central)
            } else if dot < na::zero() {
                // dir is in the opposite direction of the central axis
                vres = na::zero();
            } else {
                if dot > ComplexField::cos(self.half_angle) {
                    // dir is in the angle range of the prism
                    vres *= self.radius;
                } else {
                    // dir is in the same direction as the central axis
                    vres = self.edge(&vres, &central);
                }
            }
        }

        vres[1] = self.half_height.copysign(dir[1]);
        Point::from(vres)
    }
}
