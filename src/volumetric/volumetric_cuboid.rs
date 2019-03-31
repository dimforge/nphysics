use num::Zero;

use na::{self, RealField};
use ncollide::shape::Cuboid;
use crate::volumetric::Volumetric;
use crate::math::{AngularInertia, Point, Vector, DIM};

/// The volume of a cuboid.
#[inline]
pub fn cuboid_volume<N: RealField>(half_extents: &Vector<N>) -> N {
    let mut res = N::one();

    for half_extent in half_extents.iter() {
        res = res * *half_extent * na::convert(2.0f64)
    }

    res
}

/// The area of a cuboid.
#[inline]
pub fn cuboid_area<N: RealField>(half_extents: &Vector<N>) -> N {
    if DIM == 2 {
        (half_extents[0] + half_extents[1]) * na::convert(4.0f64)
    } else {
        let he = half_extents;
        let xx = he[0] + he[0];
        let yy = he[1] + he[1];
        let zz = he[2] + he[2];

        let side_xy = xx * yy;
        let side_xz = xx * zz;
        let side_yz = yy * zz;

        (side_xy + side_xz + side_yz) * na::convert(2.0f64)
    }
}

/// The center of mass of a cuboid.
#[inline]
pub fn cuboid_center_of_mass<N: RealField>() -> Point<N> {
    Point::origin()
}

/// The unit angular inertia of a cuboid.
#[inline]
pub fn cuboid_unit_angular_inertia<N: RealField>(half_extents: &Vector<N>) -> AngularInertia<N> {
    if DIM == 2 {
        let _2: N = na::convert(2.0f64);
        let _i12: N = na::convert(1.0f64 / 12.0);
        let w = _i12 * _2 * _2;
        let ix = w * half_extents[0] * half_extents[0];
        let iy = w * half_extents[1] * half_extents[1];

        let mut res = AngularInertia::zero();

        res[(0, 0)] = ix + iy;

        res
    } else {
        let _0: N = na::zero();
        let _2: N = na::convert(2.0f64);
        let _i12: N = na::convert(1.0f64 / 12.0);
        let w = _i12 * _2 * _2;
        let ix = w * half_extents[0] * half_extents[0];
        let iy = w * half_extents[1] * half_extents[1];
        let iz = w * half_extents[2] * half_extents[2];

        let mut res = AngularInertia::zero();

        res[(0, 0)] = iy + iz;
        res[(1, 1)] = ix + iz;
        res[(2, 2)] = ix + iy;

        res
    }
}

impl<N: RealField> Volumetric<N> for Cuboid<N> {
    fn area(&self) -> N {
        cuboid_area(self.half_extents())
    }

    fn volume(&self) -> N {
        cuboid_volume(self.half_extents())
    }

    fn center_of_mass(&self) -> Point<N> {
        cuboid_center_of_mass()
    }

    fn unit_angular_inertia(&self) -> AngularInertia<N> {
        cuboid_unit_angular_inertia(self.half_extents())
    }
}
