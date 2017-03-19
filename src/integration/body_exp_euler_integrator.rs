//! Explicit Euler integrator.

use alga::general::Real;

use object::RigidBody;
use integration::Integrator;
use integration::euler;

/// An explicit Euler integrator.
///
/// Do not use this, prefer the `BodySmpEulerIntegrator` instead.
pub struct BodyExpEulerIntegrator;

impl BodyExpEulerIntegrator {
    /// Creates a new `BodyExpEulerIntegrator`.
    #[inline]
    pub fn new() -> BodyExpEulerIntegrator {
        BodyExpEulerIntegrator
    }
}

impl<N: Real> Integrator<N, RigidBody<N>> for BodyExpEulerIntegrator {
    #[inline]
    fn update(&mut self, dt: N, rb: &mut RigidBody<N>) {
        if rb.can_move() {
            let (t, lv, av) = euler::explicit_integrate(
                dt.clone(),
                rb.position(),
                rb.center_of_mass(),
                &rb.lin_vel(),
                &rb.ang_vel(),
                &rb.lin_acc(),
                &rb.ang_acc());

            rb.append_transformation(&t);
            rb.set_lin_vel_internal(lv);
            rb.set_ang_vel_internal(av);
        }
    }
}
