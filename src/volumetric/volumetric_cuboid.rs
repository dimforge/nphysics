use std::ops::{Index, IndexMut};
use na::{Pnt2, Pnt3, Mat1, Mat3, Orig, Iterable, Zero};
use na;
use ncollide::shape::{Cuboid2, Cuboid3};
use ncollide::math::Scalar;
use volumetric::Volumetric;


/// The volume of a cuboid.
#[inline]
pub fn cuboid_volume<N, V>(dim: usize, half_extents: &V) -> N
    where N: Scalar,
          V: Iterable<N> {
    assert!(dim == 2 || dim == 3);

    let mut res: N = na::one();

    for half_extent in half_extents.iter() {
        res = res * *half_extent * na::cast(2.0f64)
    }

    res
}

/// The surface of a cuboid.
#[inline]
pub fn cuboid_surface<N, V>(dim: usize, half_extents: &V) -> N
    where N: Scalar,
          V: Index<usize, Output = N> {
    assert!(dim == 2 || dim == 3);

    match dim {
        2 => {
            (half_extents[0] + half_extents[1]) * na::cast(4.0f64)
        }
        3 => {
            let he = half_extents;
            let xx = he[0] + he[0];
            let yy = he[1] + he[1];
            let zz = he[2] + he[2];

            let side_xy = xx * yy;
            let side_xz = xx * zz;
            let side_yz = yy * zz;

            (side_xy + side_xz + side_yz) * na::cast(2.0f64)
        }
        _ => unreachable!()
    }
}

/// The center of mass of a cuboid.
#[inline]
pub fn cuboid_center_of_mass<P: Orig>() -> P {
    na::orig()
}

/// The unit angular inertia of a cuboid.
#[inline]
pub fn cuboid_unit_angular_inertia<N, V, I>(dim: usize, half_extents: &V) -> I
    where N: Scalar,
          V: Index<usize, Output = N>,
          I: Zero + IndexMut<(usize, usize), Output = N> {
    assert!(dim == 2 || dim == 3);

    match dim {
        2 => {
            let _2: N   = na::cast(2.0f64);
            let _i12: N = na::cast(1.0f64 / 12.0);
            let w       = _i12 * _2 * _2;
            let ix      = w * half_extents[0] * half_extents[0];
            let iy      = w * half_extents[1] * half_extents[1];

            let mut res = na::zero::<I>();

            res[(0, 0)] = ix + iy;

            res
        }
        3 => {
            let _0: N   = na::zero();
            let _2: N   = na::cast(2.0f64);
            let _i12: N = na::cast(1.0f64 / 12.0);
            let w       = _i12 * _2 * _2;
            let ix      = w * half_extents[0] * half_extents[0];
            let iy      = w * half_extents[1] * half_extents[1];
            let iz      = w * half_extents[2] * half_extents[2];

            let mut res = na::zero::<I>();

            res[(0, 0)] = iy + iz;
            res[(1, 1)] = ix + iz;
            res[(2, 2)] = ix + iy;

            res
        }
        _ => unreachable!()
    }
}

macro_rules! impl_volumetric_cuboid(
    ($t: ident, $dim: expr, $p: ident, $i: ident) => (
        impl<N: Scalar> Volumetric<N, $p<N>, $i<N>> for $t<N> {
            fn surface(&self) -> N {
                cuboid_surface($dim, self.half_extents())
            }

            fn volume(&self) -> N {
                cuboid_volume($dim, self.half_extents())
            }

            fn center_of_mass(&self) -> $p<N> {
                cuboid_center_of_mass()
            }

            fn unit_angular_inertia(&self) -> $i<N> {
                cuboid_unit_angular_inertia($dim, self.half_extents())
            }
        }
    )
);

impl_volumetric_cuboid!(Cuboid2, 2, Pnt2, Mat1);
impl_volumetric_cuboid!(Cuboid3, 3, Pnt3, Mat3);
