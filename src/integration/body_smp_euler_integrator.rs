//! Semi-implicit Euler integrator.

use alga::general::Real;

use object::RigidBody;
use integration::Integrator;
use integration::euler;

/// A semi-implicit Euler integrator.
pub struct BodySmpEulerIntegrator;

impl BodySmpEulerIntegrator {
    /// Creates a new `BodySmpEulerIntegrator`.
    #[inline]
    pub fn new() -> BodySmpEulerIntegrator {
        BodySmpEulerIntegrator
    }
}

impl<N: Real> Integrator<N, RigidBody<N>> for BodySmpEulerIntegrator {
    #[inline]
    fn update(&mut self, dt: N, rb: &mut RigidBody<N>) {
        if rb.can_move() {
            let (t, lv, av) = euler::semi_implicit_integrate(
                dt.clone(),
                rb.position(),
                rb.center_of_mass(),
                &rb.lin_vel(),
                &rb.ang_vel(),
                &rb.lin_acc(),
                &rb.ang_acc());

            rb.append_transformation(&t);
            rb.set_lin_vel(lv);
            rb.set_ang_vel(av);
        }
    }
}
