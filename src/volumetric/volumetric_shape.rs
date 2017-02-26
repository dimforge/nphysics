use alga::general::Real;
use ncollide::shape::{Shape, Ball, Cone, Cylinder, ConvexHull, Compound, Cuboid};
use volumetric::Volumetric;
use math::{Point, Vector, Isometry, AngularInertia};

macro_rules! dispatch(
    ($p: ty, $i: ty, $compound: ty, $convex: ty, $cuboid: ty, $sself: ident.$name: ident($($argN: ident),*)) => {
        {
            if let Some(b) = $sself.as_shape::<Ball<N>>() {
                (b as &Volumetric<N, $p, $i>).$name($($argN,)*)
            }
            else if let Some(c) = $sself.as_shape::<$compound>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = $sself.as_shape::<Cone<N>>() {
                (c as &Volumetric<N, $p, $i>).$name($($argN,)*)
            }
            else if let Some(c) = $sself.as_shape::<$convex>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = $sself.as_shape::<$cuboid>() {
                c.$name($($argN,)*)
            }
            else if let Some(c) = $sself.as_shape::<Cylinder<N>>() {
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

type DCompound<N>   = Compound<Point<N>, Isometry<N>>;
type DConvexHull<N> = ConvexHull<Point<N>>;
type DCuboid<N>     = Cuboid<Vector<N>>;

impl<N: Real> Volumetric<N, Point<N>, AngularInertia<N>> for Shape<Point<N>, Isometry<N>> {
    fn area(&self) -> N {
        dispatch!(Point<N>, AngularInertia<N>, DCompound<N>, DConvexHull<N>, DCuboid<N>, self.area())
    }

    fn volume(&self) -> N {
        dispatch!(Point<N>, AngularInertia<N>, DCompound<N>, DConvexHull<N>, DCuboid<N>, self.volume())
    }

    fn center_of_mass(&self) -> Point<N> {
        dispatch!(Point<N>, AngularInertia<N>, DCompound<N>, DConvexHull<N>, DCuboid<N>, self.center_of_mass())
    }

    fn unit_angular_inertia(&self) -> AngularInertia<N> {
        dispatch!(Point<N>, AngularInertia<N>, DCompound<N>, DConvexHull<N>, DCuboid<N>, self.unit_angular_inertia())
    }

    fn mass_properties(&self, density: N) -> (N, Point<N>, AngularInertia<N>) {
        dispatch!(Point<N>, AngularInertia<N>, DCompound<N>, DConvexHull<N>, DCuboid<N>, self.mass_properties(density))
    }
}
