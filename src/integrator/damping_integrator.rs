use nalgebra::traits::scalar_op::ScalarMul;
use body::dynamic::Dynamic;
use integrator::integrator::Integrator;

#[deriving(ToStr, Clone, Eq)]
struct DampingIntegrator<T>
{
  damping: T,
}

impl<T> DampingIntegrator<T>
{
  fn new(damping_factor: T) -> DampingIntegrator<T>
  { DampingIntegrator { damping: damping_factor } }
}

impl<T,
     RB: Dynamic<T, LV, AV, II>,
     LV: ScalarMul<T>,
     AV: ScalarMul<T>,
     II>
     Integrator<T, RB> for DampingIntegrator<T>
{
  fn integrate(&self, _: T, b: &mut RB)
  {
    let new_lin_vel = b.lin_vel().scalar_mul(&self.damping);
    let new_ang_vel = b.ang_vel().scalar_mul(&self.damping);
    b.set_lin_vel(&new_lin_vel);
    b.set_ang_vel(&new_ang_vel);
  }
}
