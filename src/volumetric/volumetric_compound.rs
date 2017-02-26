use num::Zero;

use alga::general::Real;
use alga::linear::EuclideanSpace;
use na;
use ncollide::shape::Compound;
use volumetric::{Volumetric, InertiaTensor};
use math::{Point, Isometry, AngularInertia};

impl<N: Real> Volumetric<N, Point<N>, AngularInertia<N>> for Compound<Point<N>, Isometry<N>> {
    fn area(&self) -> N {
        let mut stot: N = na::zero();

        for &(_, ref s) in self.shapes().iter() {
            stot = stot + s.area()
        }

        stot
    }

    fn volume(&self) -> N {
        let mut vtot: N = na::zero();

        for &(_, ref s) in self.shapes().iter() {
            vtot = vtot + s.volume()
        }

        vtot
    }

    fn center_of_mass(&self) -> Point<N> {
        let mut mtot = N::zero();
        let mut ctot: Point<N> = EuclideanSpace::origin();
        let mut gtot: Point<N> = EuclideanSpace::origin(); // geometric center.

        let shapes = self.shapes();

        for &(ref m, ref s) in shapes.iter() {
            let (mpart, cpart, _) = s.mass_properties(na::one());

            mtot = mtot + mpart;
            ctot = ctot + (*m * cpart * mpart).coords;
            gtot = gtot + (*m * cpart).coords;
        }

        if mtot.is_zero() {
            gtot
        }
        else {
            ctot / mtot
        }
    }

    fn unit_angular_inertia(&self) -> AngularInertia<N> {
        let mut itot: AngularInertia<N> = Zero::zero();

        let com    = self.center_of_mass();
        let shapes = self.shapes();

        for &(ref m, ref s) in shapes.iter() {
            let (mpart, cpart, ipart) = s.mass_properties(na::one());

            itot = itot + ipart.to_world_space(m)
                               .to_relative_wrt_point(mpart, &(*m * cpart + (-com.coords)));
        }

        itot
    }

    /// The mass properties of this `CompoundData`.
    ///
    /// If `density` is not zero, it will be multiplied with the density of every object of the
    /// compound shape.
    fn mass_properties(&self, density: N) -> (N, Point<N>, AngularInertia<N>) {
        let mut itot: AngularInertia<N> = Zero::zero();
        let mut ctot: Point<N> = EuclideanSpace::origin();
        let mut gtot: Point<N> = EuclideanSpace::origin(); // geometric center.
        let mut mtot = N::zero();

        let shapes = self.shapes();
        let props: Vec<_> = shapes.iter().map(|&(_, ref s)| s.mass_properties(na::one())).collect();

        for (&(ref m, _), &(ref mpart, ref cpart, _)) in shapes.iter().zip(props.iter()) {
            mtot = mtot + *mpart;
            ctot = ctot + (*m * *cpart * *mpart).coords;
            gtot = gtot + (*m * *cpart).coords;
        }

        if mtot.is_zero() {
            ctot = gtot;
        }
        else {
            ctot = ctot / mtot;
        }

        for (&(ref m, _), &(ref mpart, ref cpart, ref ipart)) in shapes.iter().zip(props.iter()) {
            itot = itot + ipart.to_world_space(m).to_relative_wrt_point(*mpart, &(*m * *cpart + (-ctot.coords)));
        }

        (mtot * density, ctot, itot * density)
    }
}
