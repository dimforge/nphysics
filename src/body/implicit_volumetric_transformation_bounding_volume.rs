use std::num::One;
use nalgebra::traits::transformation::{Transformation, Transformable};
use ncollide::bounding_volume::has_bounding_volume::HasBoundingVolume;
use ncollide::geom::default_geom::{DefaultGeom, Implicit};
use ncollide::geom::implicit::Implicit;
use body::volumetric::Volumetric;

pub trait ImplicitVolumetricTransformationBoundingVolume<V, N, M, II, BV>
          : Implicit<V> + Volumetric<N, II> + Transformation<M> + HasBoundingVolume<BV>
{
  // FIXME: those methods are workarounds: why dont trait objects of this
  // traits dont inherit from all the parent traits?
  fn _support_point(&self, dir: &V) -> V;
  fn _volume(&self)                 -> N;
  fn _inertia(&self)                -> II;
  fn _transformation(&self)         -> M;
  fn _inv_transformation(&self)     -> M;
  fn _transform_by(&mut self, &M);
  fn _bounding_volume(&self)        -> BV;
}

impl<T: Implicit<V> + Volumetric<N, II> + Transformation<M> + HasBoundingVolume<BV>,
     V,
     N,
     M,
     II,
     BV>
ImplicitVolumetricTransformationBoundingVolume<V, N, M, II, BV> for T
{
  // FIXME: those methods are workarounds: why dont trait objects of this
  // traits dont inherit from all the parent traits?
  #[inline]
  fn _support_point(&self, dir: &V) -> V
  { self.support_point(dir) }

  #[inline]
  fn _volume(&self) -> N
  { self.volume() }

  #[inline]
  fn _inertia(&self) -> II
  { self.inertia() }

  #[inline]
  fn _transformation(&self) -> M
  { self.transformation() }

  #[inline]
  fn _inv_transformation(&self) -> M
  { self.inv_transformation() }

  #[inline]
  fn _transform_by(&mut self, m: &M)
  { self.transform_by(m) }

  #[inline]
  fn _bounding_volume(&self) -> BV
  { self.bounding_volume() }
}

pub fn new_implicit<
  G:  Transformable<M, G2>,
  G2: Owned + ImplicitVolumetricTransformationBoundingVolume<V, N, M, II, BV>,
  V, N, M: One, II, BV>
  (geom: &G) -> DefaultGeom<N, V, M, ~ImplicitVolumetricTransformationBoundingVolume<V, N, M, II, BV>>
{
  Implicit(
    ~geom.transformed(&One::one())
    as ~ImplicitVolumetricTransformationBoundingVolume<V, N, M, II, BV>
  )
}

// FIXME: all the following are workarounds to make
// ~ImplicitVolumetricTransformationBoundingVolume implement all the traits it
// inherits from.
// This surely is a limitation (bug?) of the current type system for trait
// objects with inheritance.
impl<V, N, M, II, BV> Implicit<V>
for ~ImplicitVolumetricTransformationBoundingVolume<V, N, M, II, BV>
{
  #[inline]
  fn support_point(&self, dir: &V) -> V
  { self._support_point(dir) }
}

impl<V, N, M, II, BV> Volumetric<N, II>
for ~ImplicitVolumetricTransformationBoundingVolume<V, N, M, II, BV>
{
  #[inline]
  fn volume(&self) -> N
  { self._volume() }

  #[inline]
  fn inertia(&self) -> II
  { self._inertia() }
}

impl<V, N, M, II, BV> Transformation<M>
for ~ImplicitVolumetricTransformationBoundingVolume<V, N, M, II, BV>
{
  #[inline]
  fn transformation(&self) -> M
  { self._transformation() }

  #[inline]
  fn inv_transformation(&self) -> M
  { self._inv_transformation() }


  #[inline]
  fn transform_by(&mut self, m: &M)
  { self._transform_by(m) }
}

impl<V, N, M, II, BV> HasBoundingVolume<BV>
for ~ImplicitVolumetricTransformationBoundingVolume<V, N, M, II, BV>
{
  #[inline]
  fn bounding_volume(&self) -> BV
  { self._bounding_volume() }
}
