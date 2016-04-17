use std::ops::IndexMut;
use num::Zero;
use na::{BaseFloat, Origin, Point2, Point3, Matrix1, Matrix3};
use na;
use ncollide::shape::{Cylinder2, Cylinder3};
use ncollide::math::Scalar;
use volumetric::Volumetric;


/// The volume of a cylinder.
#[inline]
pub fn cylinder_volume<N: Scalar>(dimension: usize, half_height: N, radius: N) -> N {
    assert!(dimension == 2 || dimension == 3);

    match dimension {
        2 => {
            half_height * radius * na::cast(4.0f64)
        }
        3 => {
            half_height * radius * radius * BaseFloat::pi() * na::cast(2.0f64)
        }
        _ => unreachable!()
    }
}

/// The area of a cylinder.
#[inline]
pub fn cylinder_area<N: Scalar>(dimension: usize, half_height: N, radius: N) -> N {
    assert!(dimension == 2 || dimension == 3);

    match dimension {
        2 => {
            (half_height + radius) * na::cast(2.0f64)
        }
        3 => {
            let _pi: N = BaseFloat::pi();
            let basis = radius * radius * _pi;
            let side  = _pi * radius * (half_height + half_height) * na::cast(2.0f64);

            side + basis + basis
        }
        _ => unreachable!()
    }
}

/// The center of mass of a cylinder.
#[inline]
pub fn cylinder_center_of_mass<P: Origin>() -> P {
    na::origin()
}

/// The unit angular inertia of a cylinder.
#[inline]
pub fn cylinder_unit_angular_inertia<N, I>(dimension: usize, half_height: N, radius: N) -> I
    where N: Scalar,
          I: Zero + IndexMut<(usize, usize), Output = N> {
    assert!(dimension == 2 || dimension == 3);

    match dimension {
        2 => {
            // Same a the rectangle.
            let _2:   N = na::cast(2.0f64);
            let _i12: N = na::cast(1.0f64 / 12.0);
            let w       = _i12 * _2 * _2;
            let ix      = w * half_height * half_height;
            let iy      = w * radius * radius;

            let mut res = na::zero::<I>();

            res[(0, 0)] = ix + iy;

            res
        }
        3 => {
            let sq_radius = radius * radius;
            let sq_height = half_height * half_height * na::cast(4.0f64);
            let off_principal = (sq_radius * na::cast(3.0f64) + sq_height) / na::cast(12.0f64);

            let mut res = na::zero::<I>();

            res[(0, 0)] = off_principal.clone();
            res[(1, 1)] = sq_radius / na::cast(2.0f64);
            res[(2, 2)] = off_principal;

            res
        }
        _ => unreachable!()
    }
}

macro_rules! impl_volumetric_cylinder(
    ($t: ident, $dimension: expr, $p: ident, $i: ident) => (
        impl<N: Scalar> Volumetric<N, $p<N>, $i<N>> for $t<N> {
            fn area(&self) -> N {
                cylinder_area($dimension, self.half_height(), self.radius())
            }

            fn volume(&self) -> N {
                cylinder_volume($dimension, self.half_height(), self.radius())
            }

            fn center_of_mass(&self) -> $p<N> {
                cylinder_center_of_mass()
            }

            fn unit_angular_inertia(&self) -> $i<N> {
                cylinder_unit_angular_inertia($dimension, self.half_height(), self.radius())
            }
        }
    )
);

impl_volumetric_cylinder!(Cylinder2, 2, Point2, Matrix1);
impl_volumetric_cylinder!(Cylinder3, 3, Point3, Matrix3);
