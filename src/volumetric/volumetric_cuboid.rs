use std::ops::{Index, IndexMut};
use num::Zero;
use na::{Point2, Point3, Matrix1, Matrix3, Origin, Iterable};
use na;
use ncollide::shape::{Cuboid2, Cuboid3};
use ncollide::math::Scalar;
use volumetric::Volumetric;


/// The volume of a cuboid.
#[inline]
pub fn cuboid_volume<N, V>(dimension: usize, half_extents: &V) -> N
    where N: Scalar,
          V: Iterable<N> {
    assert!(dimension == 2 || dimension == 3);

    let mut res: N = na::one();

    for half_extent in half_extents.iter() {
        res = res * *half_extent * na::cast(2.0f64)
    }

    res
}

/// The area of a cuboid.
#[inline]
pub fn cuboid_area<N, V>(dimension: usize, half_extents: &V) -> N
    where N: Scalar,
          V: Index<usize, Output = N> {
    assert!(dimension == 2 || dimension == 3);

    match dimension {
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
pub fn cuboid_center_of_mass<P: Origin>() -> P {
    na::origin()
}

/// The unit angular inertia of a cuboid.
#[inline]
pub fn cuboid_unit_angular_inertia<N, V, I>(dimension: usize, half_extents: &V) -> I
    where N: Scalar,
          V: Index<usize, Output = N>,
          I: Zero + IndexMut<(usize, usize), Output = N> {
    assert!(dimension == 2 || dimension == 3);

    match dimension {
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
    ($t: ident, $dimension: expr, $p: ident, $i: ident) => (
        impl<N: Scalar> Volumetric<N, $p<N>, $i<N>> for $t<N> {
            fn area(&self) -> N {
                cuboid_area($dimension, self.half_extents())
            }

            fn volume(&self) -> N {
                cuboid_volume($dimension, self.half_extents())
            }

            fn center_of_mass(&self) -> $p<N> {
                cuboid_center_of_mass()
            }

            fn unit_angular_inertia(&self) -> $i<N> {
                cuboid_unit_angular_inertia($dimension, self.half_extents())
            }
        }
    )
);

impl_volumetric_cuboid!(Cuboid2, 2, Point2, Matrix1);
impl_volumetric_cuboid!(Cuboid3, 3, Point3, Matrix3);
