//! Constant linear and angular force generator.

use ncollide::math::Scalar;
use math::{Vector, Orientation};
use object::RigidBody;
use integration::Integrator;

/// A constant linear and angular force generator.
pub struct BodyForceGenerator<N: Scalar> {
    lin_acc: Vector<N>,
    ang_acc: Orientation<N>
}

impl<N: Scalar> BodyForceGenerator<N> {
    /// Creates a new `BodyForceGenerator`.
    ///
    /// # Arguments:
    ///
    /// * `lin_acc` - the linear acceleration to apply to every body on the scene.
    /// * `ang_acc` - the angular acceleration to apply to every body on the scene.
    pub fn new(lin_acc: Vector<N>, ang_acc: Orientation<N>) -> BodyForceGenerator<N> {
        BodyForceGenerator {
            lin_acc: lin_acc,
            ang_acc: ang_acc
        }
    }
}

impl<N: Scalar> BodyForceGenerator<N> {
    /// The linear acceleration applied by this force generator.
    #[inline]
    pub fn lin_acc(&self) -> Vector<N> {
        self.lin_acc.clone()
    }

    /// Sets the linear acceleration applied by this force generator.
    #[inline]
    pub fn set_lin_acc(&mut self, lin_acc: Vector<N>) {
        self.lin_acc = lin_acc;
    }

    /// The angular acceleration applied by this force generator.
    #[inline]
    pub fn ang_acc(&self) -> Orientation<N> {
        self.ang_acc.clone()
    }

    /// Sets the angular acceleration applied by this force generator.
    #[inline]
    pub fn set_ang_acc(&mut self, ang_acc: Orientation<N>) {
        self.ang_acc = ang_acc;
    }
}

impl<N: Scalar> Integrator<N, RigidBody<N>> for BodyForceGenerator<N> {
    #[inline]
    fn update(&mut self, _: N, rb: &mut RigidBody<N>) {
        rb.set_gravity(self.lin_acc.clone());
        //rb.set_ang_acc(self.ang_acc.clone());
    }
}
