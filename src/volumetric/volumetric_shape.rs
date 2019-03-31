use na::RealField;
use ncollide::shape::{Ball, Compound, Cuboid, Shape, Capsule};
#[cfg(feature = "dim3")]
use ncollide::shape::ConvexHull;
#[cfg(feature = "dim2")]
use ncollide::shape::ConvexPolygon;
use crate::volumetric::Volumetric;
use crate::math::{AngularInertia, Point};

macro_rules! dispatch(
    ($p: ty, $i: ty, $sself: ident.$name: ident($($argN: ident),*)) => {
        {
            if let Some(b) = $sself.as_shape::<Ball<N>>() {
                return b.$name($($argN,)*)
            }
            if let Some(c) = $sself.as_shape::<Compound<N>>() {
                return c.$name($($argN,)*)
            }
            // else if let Some(c) = $sself.as_shape::<Cone<N>>() {
            //     (c as &Volumetric<N, $p, $i>).$name($($argN,)*)
            // }
            #[cfg(feature = "dim3")]
            {
                if let Some(c) = $sself.as_shape::<ConvexHull<N>>() {
                    return c.$name($($argN,)*)
                }
            }
            #[cfg(feature = "dim2")]
            {
                if let Some(c) = $sself.as_shape::<ConvexPolygon<N>>() {
                    return c.$name($($argN,)*)
                }
            }
            if let Some(c) = $sself.as_shape::<Cuboid<N>>() {
                return c.$name($($argN,)*)
            }
            if let Some(c) = $sself.as_shape::<Capsule<N>>() {
                return c.$name($($argN,)*)
            }
            // if let Some(c) = $sself.as_shape::<Cylinder<N>>() {
            //     return c.$name($($argN,)*)
            // }

            /*
             * XXX: dispatch by custom type.
             */
            panic!("The `Volumetric` is not implemented by the given shape.")
        }
    }
);

impl<N: RealField> Volumetric<N> for Shape<N> {
    fn area(&self) -> N {
        dispatch!(Point<N>, AngularInertia<N>, self.area())
    }

    fn volume(&self) -> N {
        dispatch!(Point<N>, AngularInertia<N>, self.volume())
    }

    fn center_of_mass(&self) -> Point<N> {
        dispatch!(Point<N>, AngularInertia<N>, self.center_of_mass())
    }

    fn unit_angular_inertia(&self) -> AngularInertia<N> {
        dispatch!(Point<N>, AngularInertia<N>, self.unit_angular_inertia())
    }

    fn mass_properties(&self, density: N) -> (N, Point<N>, AngularInertia<N>) {
        dispatch!(Point<N>, AngularInertia<N>, self.mass_properties(density))
    }
}
