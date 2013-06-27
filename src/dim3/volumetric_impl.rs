use std::num::{Zero, Real, NumCast};
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::dim3::mat3::Mat3;
use ncollide::geom::default_geom::{Plane, Ball, Implicit};
use body::volumetric::Volumetric;
use dim3::aliases;

impl<N: Real + DivisionRing + NumCast + Zero + Copy>
Volumetric<N, aliases::InertiaTensor3d<N>> for aliases::Geom3d<N>
{
  #[inline]
  fn volume(&self)  -> N
  { 
    match *self
    {
      Plane(ref p)    => p.volume(),
      Ball(ref b)     => b.volume(),
      Implicit(_) => fail!("") // i.volume()
    }
  }

  #[inline]
  fn inertia(&self) -> aliases::InertiaTensor3d<N>
  {
    match *self
    {
      Plane(ref p)    => p.inertia(),
      Ball(ref b)     => b.inertia(),
      Implicit(_) => fail!("") // i.inertia()
    }
  }
}

impl<N: Real + DivisionRing + NumCast + Copy>
Volumetric<N, aliases::InertiaTensor3d<N>> for aliases::Ball3d<N>
{
  #[inline]
  fn volume(&self)  -> N
  { Real::pi::<N>() * self.radius() * self.radius() * self.radius() }

  #[inline]
  fn inertia(&self) -> aliases::InertiaTensor3d<N>
  {
    let _0   = Zero::zero();
    let diag = NumCast::from::<N, float>(3.0 / 5.0) * self.radius() *
                                                      self.radius();

    Mat3::new(copy diag, copy _0  , copy _0,
              copy _0  , copy diag, copy _0,
              copy _0  , copy _0  , copy diag)
  }
}

impl<N: Zero + Copy>
Volumetric<N, aliases::InertiaTensor3d<N>> for aliases::Plane3d<N>
{
  #[inline]
  fn volume(&self)  -> N
  { Zero::zero() }

  #[inline]
  fn inertia(&self) -> aliases::InertiaTensor3d<N>
  { Zero::zero() }
}
