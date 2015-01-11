use std::ops::IndexMut;
use na::{BaseFloat, Orig, Zero, Pnt2, Pnt3, Mat1, Mat3};
use na;
use ncollide::shape::{Cylinder2, Cylinder3};
use ncollide::math::Scalar;
use volumetric::Volumetric;


/// The volume of a cylinder.
#[inline]
pub fn cylinder_volume<N: Scalar>(dim: usize, half_height: N, radius: N) -> N {
    assert!(dim == 2 || dim == 3);

    match dim {
        2 => {
            half_height * radius * na::cast(4.0f64)
        }
        3 => {
            half_height * radius * radius * BaseFloat::pi() * na::cast(2.0f64)
        }
        _ => unreachable!()
    }
}

/// The surface of a cylinder.
#[inline]
pub fn cylinder_surface<N: Scalar>(dim: usize, half_height: N, radius: N) -> N {
    assert!(dim == 2 || dim == 3);

    match dim {
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
pub fn cylinder_center_of_mass<P: Orig>() -> P {
    na::orig()
}

/// The unit angular inertia of a cylinder.
#[inline]
pub fn cylinder_unit_angular_inertia<N, I>(dim: usize, half_height: N, radius: N) -> I
    where N: Scalar,
          I: Zero + IndexMut<(usize, usize), Output = N> {
    assert!(dim == 2 || dim == 3);

    match dim {
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
    ($t: ident, $dim: expr, $p: ident, $i: ident) => (
        impl<N: Scalar> Volumetric<N, $p<N>, $i<N>> for $t<N> {
            fn surface(&self) -> N {
                cylinder_surface($dim, self.half_height(), self.radius())
            }

            fn volume(&self) -> N {
                cylinder_volume($dim, self.half_height(), self.radius())
            }

            fn center_of_mass(&self) -> $p<N> {
                cylinder_center_of_mass()
            }

            fn unit_angular_inertia(&self) -> $i<N> {
                cylinder_unit_angular_inertia($dim, self.half_height(), self.radius())
            }
        }
    )
);

impl_volumetric_cylinder!(Cylinder2, 2, Pnt2, Mat1);
impl_volumetric_cylinder!(Cylinder3, 3, Pnt3, Mat3);
