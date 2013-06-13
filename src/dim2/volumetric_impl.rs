use std::num::{One, Zero, Real, NumCast};
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::dim1::mat1::Mat1;
use ncollide::geom::default_geom::{Plane, Ball};
use body::volumetric::Volumetric;
use dim2::aliases;

impl<N: Real + DivisionRing + NumCast + Zero + Copy>
Volumetric<N, aliases::InertiaTensor2d<N>> for aliases::Geom2d<N>
{
  fn volume(&self)  -> N
  { 
    match *self
    {
      Plane(p) => p.volume(),
      Ball(b)  => b.volume()
    }
  }

  fn inertia(&self) -> aliases::InertiaTensor2d<N>
  {
    match *self
    {
      Plane(p) => p.inertia(),
      Ball(b)  => b.inertia()
    }
  }
}

impl<N: Real + DivisionRing + NumCast + Copy>
Volumetric<N, aliases::InertiaTensor2d<N>> for aliases::Ball2d<N>
{
  fn volume(&self)  -> N
  { Real::pi::<N>() * self.radius() * self.radius() }

  fn inertia(&self) -> aliases::InertiaTensor2d<N>
    // FIXME: remove the inverse
  { Mat1::new(One::one::<N>() / (NumCast::from::<N, float>(2.0 / 5.0) *
                                 self.radius() * self.radius())) }
}

impl<N: Zero + Copy>
Volumetric<N, aliases::InertiaTensor2d<N>> for aliases::Plane2d<N>
{
  fn volume(&self)  -> N
  { Zero::zero() }

  fn inertia(&self) -> aliases::InertiaTensor2d<N>
  { Zero::zero() }
}
