use std::num::{Zero, Real, NumCast};
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::ring::Ring;
use nalgebra::mat::Mat1;
use ncollide::geom::default_geom::{Plane, Ball, Implicit};
use body::volumetric::Volumetric;
use dim2::aliases;

impl<N: Real + DivisionRing + NumCast + Zero + Clone>
Volumetric<N, aliases::InertiaTensor2d<N>> for aliases::Geom2d<N>
{
  #[inline]
  fn volume(&self) -> N
  { 
    match *self
    {
      Plane(ref p)    => p.volume(),
      Ball(ref b)     => b.volume(),
      Implicit(ref i) => i.volume()
    }
  }

  #[inline]
  fn inertia(&self, mass: &N) -> aliases::InertiaTensor2d<N>
  {
    match *self
    {
      Plane(ref p)    => p.inertia(mass),
      Ball(ref b)     => b.inertia(mass),
      Implicit(ref i) => i.inertia(mass)
    }
  }
}

impl<N: Real + DivisionRing + NumCast + Clone>
Volumetric<N, aliases::InertiaTensor2d<N>> for aliases::Ball2d<N>
{
  #[inline]
  fn volume(&self)  -> N
  { Real::pi::<N>() * self.radius() * self.radius() }

  #[inline]
  fn inertia(&self, mass: &N) -> aliases::InertiaTensor2d<N>
    // FIXME: remove the inverse
  { Mat1::new( [ self.radius() * self.radius() * *mass / NumCast::from::<N, float>(2.0) ] ) }
}

impl<N: NumCast + Ring + Clone>
Volumetric<N, aliases::InertiaTensor2d<N>> for aliases::Box2d<N>
{
  #[inline]
  fn volume(&self)  -> N
  {
    NumCast::from::<N, float>(4.0) *
    self.half_extents().at[0] *
    self.half_extents().at[1]
  }

  #[inline]
  fn inertia(&self, mass: &N) -> aliases::InertiaTensor2d<N>
    // FIXME: remove the inverse
  {
    let _2   = NumCast::from::<N, float>(2.0);
    let _i12 = NumCast::from::<N, float>(1.0 / 12.0);
    let w    = _i12 * *mass * _2 * _2;
    let ix   = w * self.half_extents().at[0] * self.half_extents().at[0];
    let iy   = w * self.half_extents().at[1] * self.half_extents().at[1];

    Mat1::new([ix + iy])
  }
}

impl<N: Zero + Clone>
Volumetric<N, aliases::InertiaTensor2d<N>> for aliases::Plane2d<N>
{
  #[inline]
  fn volume(&self) -> N
  { Zero::zero() }

  #[inline]
  fn inertia(&self, _: &N) -> aliases::InertiaTensor2d<N>
  { Zero::zero() }
}
