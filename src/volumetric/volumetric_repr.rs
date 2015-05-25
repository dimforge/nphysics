use na::{Pnt2, Iso2, Pnt3, Mat3, Iso3, Mat1};
use ncollide::shape::{Ball, Cone, Cylinder, Convex3, Cuboid2, Cuboid3, Compound2, Compound3};
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

impl<N> Volumetric<N, Pnt2<N>, Mat1<N>> for Repr<Pnt2<N>, Iso2<N>>
    where N: Scalar {
    fn surface(&self) -> N {
        dispatch!(Pnt2<N>, Mat1<N>, Compound2<N>, Cuboid2<N>/* Convex2<N> */, Cuboid2<N>, self.surface())
        //                                              XXX:   ^^^^^^^^^^
        //                                              Change this when Volumetric is implemented for 2D convex.
    }

    fn volume(&self) -> N {
        dispatch!(Pnt2<N>, Mat1<N>, Compound2<N>, Cuboid2<N>/* Convex2<N> */, Cuboid2<N>, self.volume())
        //                                              XXX:   ^^^^^^^^^^
        //                                              Change this when Volumetric is implemented for 2D convex.
    }

    fn center_of_mass(&self) -> Pnt2<N> {
        dispatch!(Pnt2<N>, Mat1<N>, Compound2<N>, Cuboid2<N>/* Convex2<N> */, Cuboid2<N>, self.center_of_mass())
        //                                              XXX:   ^^^^^^^^^^
        //                                              Change this when Volumetric is implemented for 2D convex.
    }

    fn unit_angular_inertia(&self) -> Mat1<N> {
        dispatch!(Pnt2<N>, Mat1<N>, Compound2<N>, Cuboid2<N>/* Convex2<N> */, Cuboid2<N>, self.unit_angular_inertia())
        //                                              XXX:   ^^^^^^^^^^
        //                                              Change this when Volumetric is implemented for 2D convex.
    }

    fn mass_properties(&self, density: N) -> (N, Pnt2<N>, Mat1<N>) {
        dispatch!(Pnt2<N>, Mat1<N>, Compound2<N>, Cuboid2<N>/* Convex2<N> */, Cuboid2<N>, self.mass_properties(density))
        //                                              XXX:   ^^^^^^^^^^
        //                                              Change this when Volumetric is implemented for 2D convex.
    }
}

impl<N> Volumetric<N, Pnt3<N>, Mat3<N>> for Repr<Pnt3<N>, Iso3<N>>
    where N: Scalar {
    fn surface(&self) -> N {
        dispatch!(Pnt3<N>, Mat3<N>, Compound3<N>, Convex3<N>, Cuboid3<N>, self.surface())
    }

    fn volume(&self) -> N {
        dispatch!(Pnt3<N>, Mat3<N>, Compound3<N>, Convex3<N>, Cuboid3<N>, self.volume())
    }

    fn center_of_mass(&self) -> Pnt3<N> {
        dispatch!(Pnt3<N>, Mat3<N>, Compound3<N>, Convex3<N>, Cuboid3<N>, self.center_of_mass())
    }

    fn unit_angular_inertia(&self) -> Mat3<N> {
        dispatch!(Pnt3<N>, Mat3<N>, Compound3<N>, Convex3<N>, Cuboid3<N>, self.unit_angular_inertia())
    }

    fn mass_properties(&self, density: N) -> (N, Pnt3<N>, Mat3<N>) {
        dispatch!(Pnt3<N>, Mat3<N>, Compound3<N>, Convex3<N>, Cuboid3<N>, self.mass_properties(density))
    }
}
