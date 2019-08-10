use num::Zero;

use na::{self, RealField};
use ncollide::shape::{Ball, Multiball};
use crate::volumetric::{InertiaTensor, Volumetric};
use crate::math::{AngularInertia, Point};

impl<N: RealField> Volumetric<N> for Multiball<N> {
    fn area(&self) -> N {
        let num_balls: N = na::convert(self.centers().len() as f64);
        Ball::new(self.radius()).area() * num_balls
    }

    fn volume(&self) -> N {
        let num_balls: N = na::convert(self.centers().len() as f64);
        Ball::new(self.radius()).volume() * num_balls
    }

    fn center_of_mass(&self) -> Point<N> {
        // The center of mass is the geometrical center
        // because all the balls have the same radius.
        ncollide::utils::center(self.centers())
    }

    fn unit_angular_inertia(&self) -> AngularInertia<N> {
        let mut itot = AngularInertia::zero();
        let com = self.center_of_mass();
        let ball = Ball::new(self.radius());

        for c in self.centers() {
            // We ignore the returned center because for a ball
            // it's always the origin anyway.
            let (mpart, _, ipart) = ball.mass_properties(na::one());
            itot += ipart.to_relative_wrt_point(mpart, &(c - com.coords));
        }

        itot
    }

    fn mass_properties(&self, density: N) -> (N, Point<N>, AngularInertia<N>) {
        // NOTE: we don't keep the default implementation so we don't recompute
        // the center of mass twice.
        let mut itot = AngularInertia::zero();
        let com = self.center_of_mass();
        let ball = Ball::new(self.radius());

        for c in self.centers() {
            // We ignore the returned center because for a ball
            // it's always the origin anyway.
            let (mpart, _, ipart) = ball.mass_properties(na::one());
            itot += ipart.to_relative_wrt_point(mpart, &(c - com.coords));
        }

        (self.mass(density), com, itot * density)
    }
}
