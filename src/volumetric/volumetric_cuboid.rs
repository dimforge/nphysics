use std::ops::IndexMut;
use num::Zero;

use alga::general::Real;
use na::{Point2, Point3, Matrix1, Matrix3};
use na;
use ncollide::shape::{Cuboid2, Cuboid3};
use ncollide::math::{Point, Vector};
use volumetric::Volumetric;


/// The volume of a cuboid.
#[inline]
pub fn cuboid_volume<V: Vector>(half_extents: &V) -> V::Real {
    assert!(na::dimension::<V>() == 2 || na::dimension::<V>() == 3);

    let mut res: V::Real = na::one();

    for i in 0 .. na::dimension::<V>() {
        res = res * half_extents[i] * na::convert(2.0f64)
    }

    res
}

/// The area of a cuboid.
#[inline]
pub fn cuboid_area<V: Vector>(half_extents: &V) -> V::Real {
    assert!(na::dimension::<V>() == 2 || na::dimension::<V>() == 3);

    match na::dimension::<V>() {
        2 => {
            (half_extents[0] + half_extents[1]) * na::convert(4.0f64)
        }
        3 => {
            let he = half_extents;
            let xx = he[0] + he[0];
            let yy = he[1] + he[1];
            let zz = he[2] + he[2];

            let side_xy = xx * yy;
            let side_xz = xx * zz;
            let side_yz = yy * zz;

            (side_xy + side_xz + side_yz) * na::convert(2.0f64)
        }
        _ => unreachable!()
    }
}

/// The center of mass of a cuboid.
#[inline]
pub fn cuboid_center_of_mass<P: Point>() -> P {
    P::origin()
}

/// The unit angular inertia of a cuboid.
#[inline]
pub fn cuboid_unit_angular_inertia<V, I>(half_extents: &V) -> I
    where V: Vector,
          I: Zero + IndexMut<(usize, usize), Output = V::Real> {
    assert!(na::dimension::<V>() == 2 || na::dimension::<V>() == 3);

    match na::dimension::<V>() {
        2 => {
            let _2: V::Real   = na::convert(2.0f64);
            let _i12: V::Real = na::convert(1.0f64 / 12.0);
            let w  = _i12 * _2 * _2;
            let ix = w * half_extents[0] * half_extents[0];
            let iy = w * half_extents[1] * half_extents[1];

            let mut res = I::zero();

            res[(0, 0)] = ix + iy;

            res
        }
        3 => {
            let _0: V::Real   = na::zero();
            let _2: V::Real   = na::convert(2.0f64);
            let _i12: V::Real = na::convert(1.0f64 / 12.0);
            let w  = _i12 * _2 * _2;
            let ix = w * half_extents[0] * half_extents[0];
            let iy = w * half_extents[1] * half_extents[1];
            let iz = w * half_extents[2] * half_extents[2];

            let mut res = I::zero();

            res[(0, 0)] = iy + iz;
            res[(1, 1)] = ix + iz;
            res[(2, 2)] = ix + iy;

            res
        }
        _ => unreachable!()
    }
}

macro_rules! impl_volumetric_cuboid(
    ($t: ident, $p: ident, $i: ident) => (
        impl<N: Real> Volumetric<N, $p<N>, $i<N>> for $t<N> {
            fn area(&self) -> N {
                cuboid_area(self.half_extents())
            }

            fn volume(&self) -> N {
                cuboid_volume(self.half_extents())
            }

            fn center_of_mass(&self) -> $p<N> {
                cuboid_center_of_mass()
            }

            fn unit_angular_inertia(&self) -> $i<N> {
                cuboid_unit_angular_inertia(self.half_extents())
            }
        }
    )
);

impl_volumetric_cuboid!(Cuboid2, Point2, Matrix1);
impl_volumetric_cuboid!(Cuboid3, Point3, Matrix3);
