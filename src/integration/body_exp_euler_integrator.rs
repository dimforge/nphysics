use nalgebra::na::Transformation;
use ncollide::math::N;
use object::RigidBody;
use integration::Integrator;
use integration::euler;

pub struct BodyExpEulerIntegrator;

impl BodyExpEulerIntegrator {
    #[inline]
    pub fn new() -> BodyExpEulerIntegrator {
        BodyExpEulerIntegrator
    }
}

impl Integrator<RigidBody> for BodyExpEulerIntegrator {
    #[inline]
    fn update(&mut self, dt: N, rb: &mut RigidBody) {
        if rb.can_move() {
            let (t, lv, av) = euler::explicit_integrate(
                dt.clone(),
                rb.transform_ref(),
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
