//! Traits to compute inertial properties.

use na::{self, RealField};
use na::{Isometry2, Isometry3, Matrix1, Matrix3, Point2, Point3, Vector1, Vector3};
use crate::math::{AngularInertia, Inertia, Point};

/// Trait implemented by inertia tensors.
pub trait InertiaTensor<N, P, AV, M> {
    /// Applies this inertia tensor to a vector.
    ///
    /// This is usually done by a matrix-vector multiplication.
    fn apply(&self, a: &AV) -> AV;

    /// Transforms this inertia tensor from local space to world space.
    fn to_world_space(&self, _: &M) -> Self;

    /// Computes this inertia tensor relative to a given point.
    fn to_relative_wrt_point(&self, _: N, _: &P) -> Self;
}

/// Trait implemented by objects which have a mass, a center of mass, and an inertia tensor.
pub trait Volumetric<N: RealField> {
    /// Computes the area of this object.
    fn area(&self) -> N;

    /// Computes the volume of this object.
    fn volume(&self) -> N;

    /// Computes the center of mass of this object.
    fn center_of_mass(&self) -> Point<N>;

    /// Computes the angular inertia tensor of this object.
    fn unit_angular_inertia(&self) -> AngularInertia<N>;

    /// Given its density, this computes the mass of this object.
    fn mass(&self, density: N) -> N {
        self.volume() * density
    }

    /// Given its mass, this computes the angular inertia of this object.
    fn angular_inertia(&self, mass: N) -> AngularInertia<N> {
        self.unit_angular_inertia() * mass
    }

    /// Given its density, this computes the mass, center of mass, and inertia tensor of this object.
    fn mass_properties(&self, density: N) -> (N, Point<N>, AngularInertia<N>) {
        let mass = self.mass(density);
        let com = self.center_of_mass();
        let ai = self.angular_inertia(mass);

        (mass, com, ai)
    }

    fn inertia(&self, density: N) -> Inertia<N> {
        let (mass, _, ai) = self.mass_properties(density);
        Inertia::new_with_angular_matrix(mass, ai)
    }
}

impl<N: RealField> InertiaTensor<N, Point2<N>, Vector1<N>, Isometry2<N>> for Matrix1<N> {
    #[inline]
    fn apply(&self, av: &Vector1<N>) -> Vector1<N> {
        *self * *av
    }

    #[inline]
    fn to_world_space(&self, _: &Isometry2<N>) -> Matrix1<N> {
        *self
    }

    #[inline]
    fn to_relative_wrt_point(&self, mass: N, pt: &Point2<N>) -> Matrix1<N> {
        *self + Matrix1::new(mass * pt.coords.norm_squared())
    }
}

impl<N: RealField> InertiaTensor<N, Point3<N>, Vector3<N>, Isometry3<N>> for Matrix3<N> {
    #[inline]
    fn apply(&self, av: &Vector3<N>) -> Vector3<N> {
        *self * *av
    }

    #[inline]
    fn to_world_space(&self, t: &Isometry3<N>) -> Matrix3<N> {
        let rot = t.rotation.to_rotation_matrix();
        let irot = rot.inverse();
        rot * *self * irot
    }

    #[inline]
    fn to_relative_wrt_point(&self, mass: N, pt: &Point3<N>) -> Matrix3<N> {
        let diag = pt.coords.norm_squared();
        let diagm = Matrix3::new(
            diag,
            na::zero(),
            na::zero(),
            na::zero(),
            diag,
            na::zero(),
            na::zero(),
            na::zero(),
            diag,
        );

        *self + (diagm - pt.coords * pt.coords.transpose()) * mass
    }
}
