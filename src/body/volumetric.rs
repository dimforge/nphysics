pub trait Volumetric<N, II>
{
  fn volume(&self)  -> N;
  fn inertia(&self) -> II;
}
