//! Constant linear and angular force generator.

use ncollide::math::{N, LV, AV};
use object::RigidBody;
use integration::Integrator;

/// A constant linear and angular force generator.
pub struct BodyForceGenerator {
    priv lin_acc: LV,
    priv ang_acc: AV
}

impl BodyForceGenerator {
    /// Creates a new `BodyForceGenerator`.
    ///
    /// # Arguments:
    ///
    /// * `lin_acc` - the linear acceleration to apply to every body on the scene.
    /// * `ang_acc` - the angular acceleration to apply to every body on the scene.
    pub fn new(lin_acc: LV, ang_acc: AV) -> BodyForceGenerator {
        BodyForceGenerator {
            lin_acc: lin_acc,
            ang_acc: ang_acc
        }
    }
}

impl BodyForceGenerator {
    /// The linear acceleration applied by this force generator.
    #[inline]
    pub fn lin_acc(&self) -> LV {
        self.lin_acc.clone()
    }

    /// Sets the linear acceleration applied by this force generator.
    #[inline]
    pub fn set_lin_acc(&mut self, lin_acc: LV) {
        self.lin_acc = lin_acc;
    }

    /// The angular acceleration applied by this force generator.
    #[inline]
    pub fn ang_acc(&self) -> AV {
        self.ang_acc.clone()
    }

    /// Sets the angular acceleration applied by this force generator.
    #[inline]
    pub fn set_ang_acc(&mut self, ang_acc: AV) {
        self.ang_acc = ang_acc;
    }
}

impl Integrator<RigidBody> for BodyForceGenerator {
    #[inline]
    fn update(&mut self, _: N, rb: &mut RigidBody) {
        rb.set_lin_acc(self.lin_acc.clone());
        rb.set_ang_acc(self.ang_acc.clone());
    }
}
