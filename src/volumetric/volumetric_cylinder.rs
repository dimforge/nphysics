use num::Zero;

use na::{self, RealField};
use crate::math::{AngularInertia, Point, DIM};

/// The volume of a cylinder.
#[inline]
pub fn cylinder_volume<N: RealField>(half_height: N, radius: N) -> N {
    if DIM == 2 {
        half_height * radius * na::convert(4.0f64)
    } else {
        half_height * radius * radius * N::pi() * na::convert(2.0f64)
    }
}

/// The area of a cylinder.
#[inline]
pub fn cylinder_area<N: RealField>(half_height: N, radius: N) -> N {
    if DIM == 2 {
        (half_height + radius) * na::convert(2.0f64)
    } else {
        let _pi = N::pi();
        let basis = radius * radius * _pi;
        let side = _pi * radius * (half_height + half_height) * na::convert(2.0f64);

        side + basis + basis
    }
}

/// The center of mass of a cylinder.
#[inline]
pub fn cylinder_center_of_mass<N: RealField>() -> Point<N> {
    Point::origin()
}

/// The unit angular inertia of a cylinder.
#[inline]
pub fn cylinder_unit_angular_inertia<N: RealField>(half_height: N, radius: N) -> AngularInertia<N> {
    if DIM == 2 {
        // Same a the rectangle.
        let _2: N = na::convert(2.0f64);
        let _i12: N = na::convert(1.0f64 / 12.0);
        let w = _i12 * _2 * _2;
        let ix = w * half_height * half_height;
        let iy = w * radius * radius;

        let mut res = AngularInertia::zero();

        res[(0, 0)] = ix + iy;

        res
    } else {
        let sq_radius = radius * radius;
        let sq_height = half_height * half_height * na::convert(4.0f64);
        let off_principal = (sq_radius * na::convert(3.0f64) + sq_height) / na::convert(12.0f64);

        let mut res = AngularInertia::zero();

        res[(0, 0)] = off_principal.clone();
        res[(1, 1)] = sq_radius / na::convert(2.0f64);
        res[(2, 2)] = off_principal;

        res
    }
}

//impl<N: RealField> Volumetric<N> for Cylinder<N> {
//    fn area(&self) -> N {
//        cylinder_area(self.half_height(), self.radius())
//    }
//
//    fn volume(&self) -> N {
//        cylinder_volume(self.half_height(), self.radius())
//    }
//
//    fn center_of_mass(&self) -> Point<N> {
//        cylinder_center_of_mass()
//    }
//
//    fn unit_angular_inertia(&self) -> AngularInertia<N> {
//        cylinder_unit_angular_inertia(self.half_height(), self.radius())
//    }
//}
