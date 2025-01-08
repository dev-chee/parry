use crate::math::{Point, Real, Vector};
use crate::query::{PointProjection, PointQuery};
use crate::shape::{FeatureId, Prism};
use na::{self, ComplexField};

impl PointQuery for Prism {
    #[inline]
    fn project_local_point(&self, pt: &Point<Real>, solid: bool) -> PointProjection {
        // Project on the basis.
        let mut dir_from_basis_center = pt.coords.xz();
        let planar_dist_from_basis_center = dir_from_basis_center.normalize_mut();

        let outside = {
            let (sin, cos) = ComplexField::sin_cos(self.angle_with_x);
            let central_dir = na::Vector2::new(cos, sin);

            (planar_dist_from_basis_center <= crate::math::DEFAULT_EPSILON)
                || dir_from_basis_center.dot(&central_dir) < ComplexField::cos(self.half_angle)
        };

        if outside {
            dir_from_basis_center = na::zero();
        }

        let proj2d = dir_from_basis_center * self.radius;

        if pt.y >= -self.half_height
            && pt.y <= self.half_height
            && planar_dist_from_basis_center <= self.radius
        {
            // The point is inside of the cylinder.
            if solid {
                if outside {
                    PointProjection::new(false, Point::new(0.0, pt.y, 0.0))
                } else {
                    PointProjection::new(true, *pt)
                }
            } else {
                let dist_to_top = self.half_height - pt.coords.y;
                let dist_to_bottom = pt.coords.y - (-self.half_height);
                let dist_to_side = self.radius - planar_dist_from_basis_center;

                if dist_to_top < dist_to_bottom && dist_to_top < dist_to_side {
                    if outside {
                        PointProjection::new(false, Point::new(0.0, self.half_height, 0.0))
                    } else {
                        PointProjection::new(
                            true,
                            Point::new(pt.coords.x, self.half_height, pt.coords.z),
                        )
                    }
                } else if dist_to_bottom < dist_to_top && dist_to_bottom < dist_to_side {
                    if outside {
                        PointProjection::new(false, Point::new(0.0, -self.half_height, 0.0))
                    } else {
                        PointProjection::new(
                            true,
                            Point::new(pt.coords.x, -self.half_height, pt.coords.z),
                        )
                    }
                } else {
                    PointProjection::new(true, Point::new(proj2d[0], pt.y, proj2d[1]))
                }
            }
        } else {
            // The point is outside of the cylinder.
            if pt.y > self.half_height {
                if planar_dist_from_basis_center <= self.radius {
                    if outside {
                        PointProjection::new(false, Point::new(0.0, self.half_height, 0.0))
                    } else {
                        PointProjection::new(
                            false,
                            Point::new(pt.coords.x, self.half_height, pt.coords.z),
                        )
                    }
                } else {
                    PointProjection::new(false, Point::new(proj2d[0], self.half_height, proj2d[1]))
                }
            } else if pt.y < -self.half_height {
                // Project on the bottom plane or the bottom circle.
                if planar_dist_from_basis_center <= self.radius {
                    if outside {
                        PointProjection::new(false, Point::new(0.0, -self.half_height, 0.0))
                    } else {
                        PointProjection::new(
                            false,
                            Point::new(pt.coords.x, -self.half_height, pt.coords.z),
                        )
                    }
                } else {
                    PointProjection::new(false, Point::new(proj2d[0], -self.half_height, proj2d[1]))
                }
            } else {
                // Project on the side.
                PointProjection::new(false, Point::new(proj2d[0], pt.y, proj2d[1]))
            }
        }
    }

    #[inline]
    fn project_local_point_and_get_feature(
        &self,
        pt: &Point<Real>,
    ) -> (PointProjection, FeatureId) {
        // TODO: get the actual feature.
        (self.project_local_point(pt, false), FeatureId::Unknown)
    }
}
