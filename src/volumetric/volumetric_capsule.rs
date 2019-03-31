use na::RealField;
use ncollide::shape::Capsule;

use crate::volumetric::{self, Volumetric};
use crate::math::{Point, AngularInertia};

/// Computes the volume of a capsule.
pub fn capsule_volume<N: RealField>(half_height: N, radius: N) -> N {
    volumetric::cylinder_volume(half_height, radius) + volumetric::ball_volume(radius)
}

/// Computes the area of a capsule.
pub fn capsule_area<N: RealField>(half_height: N, radius: N) -> N {
    volumetric::cylinder_area(half_height, radius) + volumetric::ball_area(radius)
}

/// Computes the unit angular inertia of a capsule.
pub fn capsule_unit_angular_inertia<N: RealField>(half_height: N, radius: N) -> AngularInertia<N> {
    let mut res = volumetric::cylinder_unit_angular_inertia(half_height, radius);
    res += volumetric::ball_unit_angular_inertia(radius);

    // Parallel axis theorem for the hemispheres.
    if cfg!(feature = "dim3") {
        let h = half_height * na::convert(2.0);
        let extra = h * h * na::convert(0.5) + h * radius * na::convert(3.0 / 8.0);
        res[(0, 0)] += extra;
        res[(2, 2)] += extra;
    } else {
        let h = half_height * na::convert(2.0);
        let extra = h * h * na::convert(0.5) + h * radius * na::convert(3.0 / 8.0);
        res[(0, 0)] += extra;
    }

    res
}


impl<N: RealField> Volumetric<N> for Capsule<N> {
    fn area(&self) -> N {
        capsule_area(self.half_height(), self.radius())
    }

    fn volume(&self) -> N {
        capsule_volume(self.half_height(), self.radius())
    }

    fn center_of_mass(&self) -> Point<N> {
        Point::origin()
    }

    fn unit_angular_inertia(&self) -> AngularInertia<N> {
        capsule_unit_angular_inertia(self.half_height(), self.radius())
    }
}
