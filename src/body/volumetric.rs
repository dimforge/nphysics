use ncollide::geom::transformed::Transformed;

pub trait Volumetric<N, II>
{
  fn volume(&self)      -> N;
  fn inertia(&self, &N) -> II;
}

impl<G: Volumetric<N, II>, M, N, II> Volumetric<N, II> for Transformed<G, M, N>
{
  fn volume(&self) -> N
  { self.sub_geom().volume() }

  fn inertia(&self, mass: &N) -> II
  {
    // FIXME: take in account the transform
    self.sub_geom().inertia(mass)
  }
}
