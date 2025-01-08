use crate::{
    math::{Point, Real, Vector, DIM},
    shape::Prism,
    transformation::utils,
};
use na::{self, ComplexField};

impl Prism {
    /// convert a prism to a polygon mesh
    pub fn to_trimesh(&self, nsubdiv: u32) -> (Vec<Point<Real>>, Vec<[u32; 3]>) {
        let scale = Vector::new(self.radius, self.half_height * 2.0, self.radius);
        let (vtx, idx) = unit_prism(nsubdiv, self.half_angle, self.angle_with_x);
        (utils::scaled(vtx, scale), idx)
    }
}

/// create a unit prism
fn unit_prism(
    nsubdiv: u32,
    half_angle: Real,
    angle_with_x: Real,
) -> (Vec<Point<Real>>, Vec<[u32; 3]>) {
    let dtheta = 2.0 * half_angle / (nsubdiv as Real);

    let (top0, len) = (0 as usize, (nsubdiv + 2) as usize);
    let btm0 = top0 + len;
    let mut vertices = {
        let mut v = Vec::with_capacity(len * 2);
        unsafe {
            v.set_len(len * 2);
        }
        v
    };
    let mut indices = Vec::with_capacity((len + nsubdiv as usize) * 2);

    all_vertices(
        1.0,
        angle_with_x - half_angle,
        dtheta,
        &mut vertices,
        top0,
        btm0,
    );

    lateral_surface_indices(top0 as u32, btm0 as u32, &mut indices);
    top_surface_indices(top0 as u32, len as u32, &mut indices);
    bottom_surface_indices(btm0 as u32, len as u32, &mut indices);

    (vertices, indices)
}

#[inline]
/// all vertices of a prism
fn all_vertices(
    radius: Real,
    theta0: Real,
    dtheta: Real,
    out_coords: &mut Vec<Point<Real>>,
    top0: usize,
    btm0: usize,
) {
    out_coords[top0] = Point::new(0.0, 0.5, 0.0);
    out_coords[btm0] = Point::new(0.0, -0.5, 0.0);

    let mut top = top0 + 1;
    let mut btm = btm0 + 1;
    (0..(btm0 - top)).for_each(|i| {
        let theta = theta0 + dtheta * (i as Real);
        let (sin, cos) = ComplexField::sin_cos(theta);
        let (x, z) = (cos * radius, sin * radius);
        out_coords[top] = Point::new(x, 0.5, z);
        out_coords[btm] = Point::new(x, -0.5, z);
        top += 1;
        btm += 1;
    });
}

#[inline]
/// indices of lateral surface
fn lateral_surface_indices(top0: u32, btm0: u32, indices: &mut Vec<[u32; DIM]>) {
    (0..btm0 - top0 - 1).for_each(|i| {
        utils::push_rectangle_indices(top0 + i + 1, top0 + i, btm0 + i + 1, btm0 + i, indices);
    });

    utils::push_rectangle_indices(
        top0,
        top0 + (btm0 - top0 - 1),
        btm0,
        btm0 + (btm0 - top0 - 1),
        indices,
    );
}

#[inline]
/// indices of top surface
fn top_surface_indices(top0: u32, len: u32, indices: &mut Vec<[u32; DIM]>) {
    (1..len - 1).for_each(|i| {
        indices.push([top0, top0 + i + 1, top0 + i]);
    });
}

#[inline]
/// indices of bottom surface
fn bottom_surface_indices(btm0: u32, len: u32, indices: &mut Vec<[u32; DIM]>) {
    (1..len - 1).for_each(|i| {
        indices.push([btm0, btm0 + i, btm0 + i + 1]);
    });
}
