pub trait Integrator<T, RB>
{
  fn integrate(&self, T, &mut RB);
}
