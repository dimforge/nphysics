pub trait Integrator<N, O>
{
  fn add(&mut self, @mut O);
  fn remove(&mut self, @mut O);
  fn update(&mut self, N);
}
