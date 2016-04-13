//! Traits to compute inertial properties.

use std::ops::Mul;
use na;
use na::{Pnt2, Pnt3, Vec1, Vec3, Iso2, Iso3, Mat1, Mat3};
use ncollide::math::Scalar;

/// Trait implemented by inertia tensors.
pub trait InertiaTensor<N, P, AV, M> {
    /// Applies this inertia tensor to a vector.
    ///
    /// This is usually done by a matrix-vector multiplication.
    fn apply(&self, a: &AV) -> AV;

    /// Transforms this inertia tensor from local space to world space.
    fn to_world_space(&self, &M) -> Self;

    /// Computes this inertia tensor relative to a given point.
    fn to_relative_wrt_point(&self, N, &P) -> Self;
}

/// Trait implemented by objects which have a mass, a center of mass, and an inertia tensor.
pub trait Volumetric<N: Scalar, P, I: Mul<N, Output = I>> {
    /// Computes the area of this object.
    fn area(&self) -> N;

    /// Computes the volume of this object.
    fn volume(&self) -> N;

    /// Computes the center of mass of this object.
    fn center_of_mass(&self) -> P;

    /// Computes the angular inertia tensor of this object.
    fn unit_angular_inertia(&self) -> I;

    /// Given its density, this computes the mass of this object.
    fn mass(&self, density: N) -> N {
        self.volume()  * density
    }

    /// Given its mass, this computes the angular inertia of this object.
    fn angular_inertia(&self, mass: N) -> I {
        self.unit_angular_inertia() * mass
    }

    /// Given its density, this computes the mass, center of mass, and inertia tensor of this object.
    fn mass_properties(&self, density: N) -> (N, P, I) {
        let mass = self.mass(density);
        let com  = self.center_of_mass();
        let ai   = self.angular_inertia(mass);

        (mass, com, ai)
    }

}

impl<N: Scalar> InertiaTensor<N, Pnt2<N>, Vec1<N>, Iso2<N>> for Mat1<N> {
    #[inline]
    fn apply(&self, av: &Vec1<N>) -> Vec1<N> {
        *self * *av
    }

    #[inline]
    fn to_world_space(&self, _: &Iso2<N>) -> Mat1<N> {
        self.clone()
    }

    #[inline]
    fn to_relative_wrt_point(&self, mass: N, pt: &Pnt2<N>) -> Mat1<N> {
        *self + Mat1::new(mass * na::sqnorm(pt.as_vec()))
    }
}

impl<N: Scalar> InertiaTensor<N, Pnt3<N>, Vec3<N>, Iso3<N>> for Mat3<N> {
    #[inline]
    fn apply(&self, av: &Vec3<N>) -> Vec3<N> {
        *self * *av
    }

    #[inline]
    fn to_world_space(&self, t: &Iso3<N>) -> Mat3<N> {
        let inv = na::inv(&t.rotation).unwrap();
        *t.rotation.submat() * *self * *inv.submat()
    }

    #[inline]
    fn to_relative_wrt_point(&self, mass: N, pt: &Pnt3<N>) -> Mat3<N> {
        let diag  = na::sqnorm(pt.as_vec());
        let diagm = Mat3::new(
            diag.clone(), na::zero(),   na::zero(),
            na::zero(),   diag.clone(), na::zero(),
            na::zero(),   na::zero(),   diag
        );

        *self + (diagm - na::outer(pt.as_vec(), pt.as_vec())) * mass
    }
}
