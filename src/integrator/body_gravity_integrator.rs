use std::num::One;
use nalgebra::traits::workarounds::scalar_op::ScalarMul;
use nalgebra::traits::rotation::Rotation;
use nalgebra::traits::translation::Translation;
use body::dynamic::Dynamic;
use body::transformable::Transformable;
use integrator::integrator::Integrator;
use integrator::euler;

struct BodyGravityIntegrator<LV, AV, T, RB, II, M>
{
  linear_gravity:  LV,
  angular_gravity: AV
}

impl<LV: Copy, AV: Copy, T, RB, II, M>
BodyGravityIntegrator<LV, AV, T, RB, II, M>
{
  pub fn new(&lin_g: &LV, &ang_g: &AV) ->
         BodyGravityIntegrator<LV, AV, T, RB, II, M>
  { BodyGravityIntegrator { linear_gravity: lin_g, angular_gravity: ang_g } }
}

impl<RB: Dynamic<T, LV, AV, II> + Transformable<M>,
     M:  Translation<LV> + Rotation<AV> + One,
     LV: Add<LV, LV> + ScalarMul<T> + Neg<LV>,
     AV: Add<AV, AV> + ScalarMul<T>,
     T:  Copy,
     II>
    Integrator<T, RB> for BodyGravityIntegrator<LV, AV, T, RB, II, M>
{
  fn integrate(&self, dt: T, b: &mut RB)
  {
    b.set_ext_lin_force(&self.linear_gravity);
    b.set_ext_ang_force(&self.angular_gravity);
    euler::integrate_body_velocity(dt, b);
    euler::integrate_body_position(dt, b);
  }
}
