use std::num::{Zero, Real, NumCast};
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::mat::Mat3;
use ncollide::geom::default_geom::{Plane, Ball, Implicit};
use body::volumetric::Volumetric;
use dim3::aliases;

impl<N: Real + DivisionRing + NumCast + Zero + Clone>
Volumetric<N, aliases::InertiaTensor3d<N>> for aliases::Geom3d<N>
{
  #[inline]
  fn volume(&self)  -> N
  { 
    match *self
    {
      Plane(ref p)    => p.volume(),
      Ball(ref b)     => b.volume(),
      Implicit(ref i) => i.volume()
    }
  }

  #[inline]
  fn inertia(&self, mass: &N) -> aliases::InertiaTensor3d<N>
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
Volumetric<N, aliases::InertiaTensor3d<N>> for aliases::Ball3d<N>
{
  #[inline]
  fn volume(&self)  -> N
  { Real::pi::<N>() * self.radius() * self.radius() * self.radius() }

  #[inline]
  fn inertia(&self, mass: &N) -> aliases::InertiaTensor3d<N>
  {
    let _0   = Zero::zero::<N>();
    let diag = NumCast::from::<N, float>(2.0 / 5.0) *
               *mass                                *
               self.radius()                        *
               self.radius();

    Mat3::new( [ diag.clone(), _0.clone()  , _0.clone(),
                 _0.clone()  , diag.clone(), _0.clone(),
                 _0.clone()  , _0          , diag ])
  }
}

impl<N: Zero + Clone>
Volumetric<N, aliases::InertiaTensor3d<N>> for aliases::Plane3d<N>
{
  #[inline]
  fn volume(&self) -> N
  { Zero::zero() }

  #[inline]
  fn inertia(&self, _: &N) -> aliases::InertiaTensor3d<N>
  { Zero::zero() }
}

impl<N: Zero + NumCast + DivisionRing + Clone>
Volumetric<N, aliases::InertiaTensor3d<N>> for aliases::Box3d<N>
{
  #[inline]
  fn volume(&self) -> N
  {
    NumCast::from::<N, float>(8.0) *
    self.half_extents().at[0]      *
    self.half_extents().at[1]      *
    self.half_extents().at[2]
  }

  #[inline]
  fn inertia(&self, mass: &N) -> aliases::InertiaTensor3d<N>
  {
    let _0   = Zero::zero::<N>();
    let _2   = NumCast::from::<N, float>(2.0);
    let _i12 = NumCast::from::<N, float>(1.0 / 12.0);
    let w    = _i12 * *mass * _2 * _2;
    let ix   = w * self.half_extents().at[0] * self.half_extents().at[0];
    let iy   = w * self.half_extents().at[1] * self.half_extents().at[1];
    let iz   = w * self.half_extents().at[2] * self.half_extents().at[2];

    Mat3::new([iy + iz,    _0.clone(), _0.clone(),
               _0.clone(), ix + iz   , _0.clone(),
               _0.clone(), _0        , ix + iy])
  }
}
