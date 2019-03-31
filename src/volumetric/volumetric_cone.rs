use std::ops::IndexMut;
use num::Zero;

use na::{self, RealField};
use ncollide::math::Point;

/// The volume of a cone.
#[inline]
pub fn cone_volume<N: RealField>(dimension: usize, half_height: N, radius: N) -> N {
    assert!(dimension == 2 || dimension == 3);

    match dimension {
        2 => radius * half_height * na::convert(2.0f64),
        3 => radius * radius * N::pi() * half_height * na::convert(2.0f64 / 3.0),
        _ => unreachable!(),
    }
}

/// The area of a cone.
#[inline]
pub fn cone_area<N: RealField>(dimension: usize, half_height: N, radius: N) -> N {
    assert!(dimension == 2 || dimension == 3);

    match dimension {
        2 => {
            let height = half_height * na::convert(2.0f64);
            let side = (height * height + radius * radius).sqrt();

            radius * na::convert(2.0f64) + side
        }
        3 => {
            let _pi = N::pi();
            let height = half_height + half_height;
            let side = (height * height + radius * radius).sqrt();

            radius * radius * _pi + side * radius * _pi
        }
        _ => unreachable!(),
    }
}

/// The center of mass of a cone.
#[inline]
pub fn cone_center_of_mass<N: RealField>(half_height: N) -> Point<N> {
    let mut com = Point::origin();
    com[1] = -half_height / na::convert(2.0f64);

    com
}

/// The unit angular inertia of a cone.
#[inline]
pub fn cone_unit_angular_inertia<N, I>(dimension: usize, half_height: N, radius: N) -> I
where
    N: RealField,
    I: Zero + IndexMut<(usize, usize), Output = N>,
{
    assert!(dimension == 2 || dimension == 3);

    match dimension {
        2 => {
            // FIXME: not sure about that…
            let mut res = I::zero();

            res[(0, 0)] = radius * half_height * half_height * half_height / na::convert(3.0f64);

            res
        }
        3 => {
            let sq_radius = radius * radius;
            let sq_height = half_height * half_height * na::convert(4.0f64);
            let off_principal =
                sq_radius * na::convert(3.0f64 / 20.0) + sq_height * na::convert(3.0f64 / 5.0);

            let principal = sq_radius * na::convert(3.0f64 / 10.0);

            let mut res = I::zero();

            res[(0, 0)] = off_principal.clone();
            res[(1, 1)] = principal;
            res[(2, 2)] = off_principal;

            res
        }
        _ => unreachable!(),
    }
}

//macro_rules! impl_volumetric_cone(
//    ($t: ident, $dimension: expr, $p: ident, $i: ident) => (
//        impl<N: RealField> Volumetric<N, $p<N>, $i<N>> for $t<N> {
//            fn area(&self) -> N {
//                cone_area($dimension, self.half_height(), self.radius())
//            }
//
//            fn volume(&self) -> N {
//                cone_volume($dimension, self.half_height(), self.radius())
//            }
//
//            fn center_of_mass(&self) -> $p<N> {
//                cone_center_of_mass(self.half_height())
//            }
//
//            fn unit_angular_inertia(&self) -> $i<N> {
//                cone_unit_angular_inertia($dimension, self.half_height(), self.radius())
//            }
//        }
//    )
//);
//
//impl_volumetric_cone!(Cone2, 2, Point2, Matrix1);
//impl_volumetric_cone!(Cone3, 3, Point3, Matrix3);
