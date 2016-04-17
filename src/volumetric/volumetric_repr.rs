use na::{Point2, Isometry2, Point3, Matrix3, Isometry3, Matrix1};
use ncollide::shape::{Ball, Cone, Cylinder, ConvexHull2, ConvexHull3, Cuboid2, Cuboid3, Compound2, Compound3};
use ncollide::inspection::Repr;
use ncollide::math::Scalar;
use volumetric::Volumetric;

macro_rules! dispatch(
    ($p: ty, $i: ty, $compound: ty, $convex: ty, $cuboid: ty, $sself: ident.$name: ident($($argN: ident),*)) => {
        {
            let repr = $sself.repr();

            if let Some(b) = repr.downcast_ref::<Ball<N>>() {
                (b as &Volumetric<N, $p, $i>).$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<$compound>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<Cone<N>>() {
                (c as &Volumetric<N, $p, $i>).$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<$convex>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<$cuboid>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = repr.downcast_ref::<Cylinder<N>>() {
                (c as &Volumetric<N, $p, $i>).$name($($argN,)*)
            }
            else {
                /*
                 * XXX: dispatch by custom type.
                 */
                panic!("The `Volumetric` is not implemented by the given shape.")
            }
        }
    }
);

impl<N> Volumetric<N, Point2<N>, Matrix1<N>> for Repr<Point2<N>, Isometry2<N>>
    where N: Scalar {
    fn area(&self) -> N {
        dispatch!(Point2<N>, Matrix1<N>, Compound2<N>, ConvexHull2<N>, Cuboid2<N>, self.area())
    }

    fn volume(&self) -> N {
        dispatch!(Point2<N>, Matrix1<N>, Compound2<N>, ConvexHull2<N>, Cuboid2<N>, self.volume())
    }

    fn center_of_mass(&self) -> Point2<N> {
        dispatch!(Point2<N>, Matrix1<N>, Compound2<N>, ConvexHull2<N>, Cuboid2<N>, self.center_of_mass())
    }

    fn unit_angular_inertia(&self) -> Matrix1<N> {
        dispatch!(Point2<N>, Matrix1<N>, Compound2<N>, ConvexHull2<N>, Cuboid2<N>, self.unit_angular_inertia())
    }

    fn mass_properties(&self, density: N) -> (N, Point2<N>, Matrix1<N>) {
        dispatch!(Point2<N>, Matrix1<N>, Compound2<N>, ConvexHull2<N>, Cuboid2<N>, self.mass_properties(density))
    }
}

impl<N> Volumetric<N, Point3<N>, Matrix3<N>> for Repr<Point3<N>, Isometry3<N>>
    where N: Scalar {
    fn area(&self) -> N {
        dispatch!(Point3<N>, Matrix3<N>, Compound3<N>, ConvexHull3<N>, Cuboid3<N>, self.area())
    }

    fn volume(&self) -> N {
        dispatch!(Point3<N>, Matrix3<N>, Compound3<N>, ConvexHull3<N>, Cuboid3<N>, self.volume())
    }

    fn center_of_mass(&self) -> Point3<N> {
        dispatch!(Point3<N>, Matrix3<N>, Compound3<N>, ConvexHull3<N>, Cuboid3<N>, self.center_of_mass())
    }

    fn unit_angular_inertia(&self) -> Matrix3<N> {
        dispatch!(Point3<N>, Matrix3<N>, Compound3<N>, ConvexHull3<N>, Cuboid3<N>, self.unit_angular_inertia())
    }

    fn mass_properties(&self, density: N) -> (N, Point3<N>, Matrix3<N>) {
        dispatch!(Point3<N>, Matrix3<N>, Compound3<N>, ConvexHull3<N>, Cuboid3<N>, self.mass_properties(density))
    }
}
