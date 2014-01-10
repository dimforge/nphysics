use nalgebra::na::Transformation;
use object::RigidBody;
use integration::Integrator;
use integration::euler;
use ncollide::math::N;

pub struct BodySmpEulerIntegrator;

impl BodySmpEulerIntegrator {
    #[inline]
    pub fn new() -> BodySmpEulerIntegrator {
        BodySmpEulerIntegrator
    }
}

impl Integrator<RigidBody> for BodySmpEulerIntegrator {
    #[inline]
    fn update(&mut self, dt: N, rb: &mut RigidBody) {
        if rb.can_move() {
            let (t, lv, av) = euler::semi_implicit_integrate(
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
