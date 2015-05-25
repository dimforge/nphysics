use std::ops::IndexMut;
use num::Zero;
use na::{BaseFloat, Orig, Pnt2, Pnt3, Mat1, Mat3};
use na;
use ncollide::shape::{Cone2, Cone3};
use ncollide::math::Scalar;
use volumetric::Volumetric;


/// The volume of a cone.
#[inline]
pub fn cone_volume<N: Scalar>(dim: usize, half_height: N, radius: N) -> N {
    assert!(dim == 2 || dim == 3);

    match dim {
        2 => {
            radius * half_height * na::cast(2.0f64)
        }
        3 => {
            radius * radius * BaseFloat::pi() * half_height * na::cast(2.0f64 / 3.0)
        }
        _ => unreachable!()
    }
}

/// The surface of a cone.
#[inline]
pub fn cone_surface<N: Scalar>(dim: usize, half_height: N, radius: N) -> N {
    assert!(dim == 2 || dim == 3);

    match dim {
        2 => {
            let height = half_height * na::cast(2.0f64);
            let side   = (height * height + radius * radius).sqrt();

            radius * na::cast(2.0f64) + side
        }
        3 => {
            let _pi    = BaseFloat::pi();
            let height = half_height + half_height;
            let side   = (height * height + radius * radius).sqrt();

            radius * radius *_pi + side * radius * _pi
        }
        _ => unreachable!()
    }
}

/// The center of mass of a cone.
#[inline]
pub fn cone_center_of_mass<N, P>(half_height: N) -> P
    where N: Scalar,
          P: Orig + IndexMut<usize, Output = N> {
    let mut com = na::orig::<P>();
    com[1] = -half_height / na::cast(2.0f64);

    com
}

/// The unit angular inertia of a cone.
#[inline]
pub fn cone_unit_angular_inertia<N, I>(dim: usize, half_height: N, radius: N) -> I
    where N: Scalar,
          I: Zero + IndexMut<(usize, usize), Output = N> {
    assert!(dim == 2 || dim == 3);

    match dim {
        2 => {
            // FIXME: not sure about that…
            let mut res = na::zero::<I>();

            res[(0, 0)] = radius * half_height * half_height * half_height / na::cast(3.0f64);

            res
        }
        3 => {
            let sq_radius = radius * radius;
            let sq_height = half_height * half_height *
                na::cast(4.0f64);
            let off_principal = sq_radius * na::cast(3.0f64 / 20.0) +
                sq_height * na::cast(3.0f64 / 5.0);

            let principal = sq_radius * na::cast(3.0f64 / 10.0);

            let mut res = na::zero::<I>();

            res[(0, 0)] = off_principal.clone();
            res[(1, 1)] = principal;
            res[(2, 2)] = off_principal;

            res
        }
        _ => unreachable!()
    }
}

macro_rules! impl_volumetric_cone(
    ($t: ident, $dim: expr, $p: ident, $i: ident) => (
        impl<N: Scalar> Volumetric<N, $p<N>, $i<N>> for $t<N> {
            fn surface(&self) -> N {
                cone_surface($dim, self.half_height(), self.radius())
            }

            fn volume(&self) -> N {
                cone_volume($dim, self.half_height(), self.radius())
            }

            fn center_of_mass(&self) -> $p<N> {
                cone_center_of_mass(self.half_height())
            }

            fn unit_angular_inertia(&self) -> $i<N> {
                cone_unit_angular_inertia($dim, self.half_height(), self.radius())
            }
        }
    )
);

impl_volumetric_cone!(Cone2, 2, Pnt2, Mat1);
impl_volumetric_cone!(Cone3, 3, Pnt3, Mat3);
