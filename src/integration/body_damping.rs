//! Linear and angular velocity damping.

use alga::general::Real;

use integration::Integrator;
use object::RigidBody;

/// A linear and angular velocity damper.
///
/// This will remove a part of the linear and angular velocity of every rigid body at each frame.
/// Do not use unless you are experiencing unrealistic vibrations or very unstable joints.
pub struct BodyDamping<N: Real> {
    linear_damping:  N,
    angular_damping: N
}

impl<N: Real> BodyDamping<N> {
    /// Creates a new `BodyDamping`.
    ///
    /// # Arguments:
    /// * `linear_damping` - coefficient in [0, 1] the linear velocity of each rigid body is
    ///                      multiplied at each update.
    /// * `linear_damping` - coefficient in [0, 1] the angular velocity of each rigid body is
    ///                      multiplied at each update.
    #[inline]
    pub fn new(linear_damping: N, angular_damping: N) -> BodyDamping<N> {
        BodyDamping {
            linear_damping:  linear_damping,
            angular_damping: angular_damping,
        }
    }
}

impl<N: Real> Integrator<N, RigidBody<N>> for BodyDamping<N> {
    fn update(&mut self, _: N, rb: &mut RigidBody<N>) {
        let new_lin = rb.lin_vel() * self.linear_damping;

        rb.set_lin_vel(new_lin);
        let new_ang = rb.ang_vel() * self.angular_damping;
        rb.set_ang_vel(new_ang);
    }
}
