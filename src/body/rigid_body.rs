use std::num::{Zero, One};
use nalgebra::traits::inv::Inv;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::rotation::Rotation;
// FIXME: use nalgebra::traits::delta_transform::DeltaTransform;
use ncollide::broad::brute_force_bounding_volume_broad_phase::BoundingVolumeProxy;
use ncollide::broad::brute_force_bounding_volume_broad_phase::HasBoundingVolumeProxy;
use ncollide::bounding_volume::has_bounding_volume::HasBoundingVolume;
use gt = ncollide::geom::transformable;
use ncollide::geom::has_geom::HasGeom;
use ncollide::utils::exact::Exact;
use ncollide::utils::default::Default;
use body::volumetric::Volumetric;
use body::dynamic::Dynamic;
use body::transformable::Transformable;
use body::can_move::CanMove;
use constraint::index_proxy::{HasIndexProxy, IndexProxy};

pub enum RigidBodyState {
  Static,
  Dynamic,
  Kinematic // FIXME: not yet supported
}


pub struct RigidBody<S, N, M, LV, AV, II, BPP>
{
  priv state:            RigidBodyState,
  priv geom:             S,
  priv transformed_geom: S, // FIXME: use another type for transformed geom?
  priv local_to_world:   M,
  priv world_to_local:   M,
  priv lin_vel:          LV,
  priv ang_vel:          AV,
  priv inv_mass:         N,
  priv ls_inv_inertia:   II,
  priv inv_inertia:      II,
  priv lin_force:        LV,
  priv ang_force:        AV,
  priv index:            IndexProxy,
  priv bp_proxy:         BPP
}

impl<S: gt::Transformable<M, S>, N, M/*FIXME: : DeltaTransform<II>*/, LV, AV,
     II: Mul<II, II> + Copy,
     BPP>
RigidBody<S, N, M, LV, AV, II, BPP>
{
  fn moved(&mut self)
  {
    self.geom.transform_to(&self.local_to_world, &mut self.transformed_geom);

    // FIXME: the inverse inertia should be computed lazily (use a @mut ?).
    self.inv_inertia = // FIXME: self.local_to_world.delta_transform() *
                       copy self.ls_inv_inertia // FIXME:                  *
                       // FIXME: self.world_to_local.delta_transform()
  }
}

impl<S:   gt::Transformable<M, S> + Volumetric<N, II>,
     N:   One + Zero + Div<N, N> + Mul<N, N>,
     M:   One, // FIXME: + DeltaTransform<II>,
     LV:  Zero,
     AV:  Zero,
     II:  One + Zero + Inv + Mul<II, II> + Copy,
     BPP: Default>
RigidBody<S, N, M, LV, AV, II, BPP>
{
  pub fn new(geom: S, density: N, state: RigidBodyState) -> RigidBody<S, N, M, LV, AV, II, BPP>
  {
    let (inv_mass, inv_inertia) =
      match state
      {
        Static    => (Zero::zero(), Zero::zero()),
        Dynamic   => {
          let volume = geom.volume();
          if volume.is_zero()
          { fail!("A dynamic body cannot have a zero volume.") }
          if density.is_zero()
          { fail!("A dynamic body cannot have a zero density.") }
          (One::one::<N>() / (density * volume), geom.inertia().inverse())
        },
        Kinematic => {
          println("Warning: for now, kinematic object behave like static one.");
          (Zero::zero(), Zero::zero())
        }
      };

    let mut res =
      RigidBody {
        state:            state,
        transformed_geom: geom.transformed(&One::one()),
        geom:             geom,
        local_to_world:   One::one(),
        world_to_local:   One::one(),
        lin_vel:          Zero::zero(),
        ang_vel:          Zero::zero(),
        inv_mass:         inv_mass,
        ls_inv_inertia:   copy inv_inertia,
        inv_inertia:      inv_inertia,
        lin_force:        Zero::zero(),
        ang_force:        Zero::zero(),
        index:            IndexProxy::new(),
        bp_proxy:         Default::default()
      };

    res.moved();

    res
  }
}

impl<S, N: Copy, M: Copy, LV: Copy, AV: Copy, II: Copy, BPP>
    Dynamic<N, LV, AV, II> for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline(always)]
  fn lin_vel(&self) -> LV
  { copy self.lin_vel }
  #[inline(always)]
  fn set_lin_vel(&mut self, lv: &LV)
  { self.lin_vel = copy *lv }

  #[inline(always)]
  fn ang_vel(&self) -> AV
  { copy self.ang_vel }
  #[inline(always)]
  fn set_ang_vel(&mut self, av: &AV)
  { self.ang_vel = copy *av }

  #[inline(always)]
  fn inv_mass(&self) -> N
  { copy self.inv_mass }
  #[inline(always)]
  fn set_inv_mass(&mut self, m: &N)
  { self.inv_mass = copy *m }

  #[inline(always)]
  fn inv_inertia(&self) -> II
  { copy self.inv_inertia }
  #[inline(always)]
  fn set_inv_inertia(&mut self, ii: &II)
  { self.inv_inertia = copy *ii }

  // FIXME: create another trait for forces?
  #[inline(always)]
  fn ext_lin_force(&self) -> LV
  { copy self.lin_force }
  #[inline(always)]
  fn set_ext_lin_force(&mut self, lf: &LV)
  { self.lin_force = copy *lf }

  #[inline(always)]
  fn ext_ang_force(&self) -> AV
  { copy self.ang_force }
  #[inline(always)]
  fn set_ext_ang_force(&mut self, af: &AV)
  { self.ang_force = copy *af }
}

impl<S: gt::Transformable<M, S>,
     N,
     M: Copy + Inv + Mul<M, M>, // FIXME: + DeltaTransform<II>,
     LV,
     AV,
     II: Mul<II, II> + Copy,
     BPP>
Transformable<M> for
    RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline(always)]
  fn local_to_world(&self) -> M
  { copy self.local_to_world }

  #[inline(always)]
  fn world_to_local(&self) -> M
  { copy self.world_to_local }

  #[inline(always)]
  fn append(&mut self, &to_append: &M)
  {
    self.local_to_world = self.local_to_world * to_append;
    self.world_to_local = self.local_to_world.inverse();
    self.moved();
  }
}

impl<S: Copy + gt::Transformable<M, S>,
     N: Copy,
     M: Copy + Translation<LV>, // FIXME: + DeltaTransform<II>,
     LV: Copy,
     AV: Copy,
     II: Copy + Mul<II, II>,
     BPP: Copy>
    Translation<LV> for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline(always)]
  fn translation(&self) -> LV
  { self.local_to_world.translation() }

  #[inline(always)]
  fn translated(&self, trans: &LV) -> RigidBody<S, N, M, LV, AV, II, BPP>
  {
    let mut &cpy = copy self;

    cpy.translate(trans);

    cpy
  }

  #[inline(always)]
  fn translate(&mut self, trans: &LV)
  {
    self.local_to_world.translate(trans);
    self.moved();
  }
}

impl<S:  Copy + gt::Transformable<M, S>,
     N:  Copy,
     M:  Copy + Rotation<AV>, // FIXME: + DeltaTransform<II>,
     LV: Copy,
     AV: Copy,
     II: Copy + Mul<II, II>,
     BPP: Copy>
    Rotation<AV> for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline(always)]
  fn rotation(&self) -> AV
  { self.local_to_world.rotation() }

  #[inline(always)]
  fn rotated(&self, rot: &AV) -> RigidBody<S, N, M, LV, AV, II, BPP>
  {
    let mut &cpy = copy self;

    cpy.rotate(rot);

    cpy
  }

  #[inline(always)]
  fn rotate(&mut self, rot: &AV)
  {
    self.local_to_world.rotate(rot);
    self.moved();
  }
}

impl<S, N, M, LV, AV, II, BPP> HasGeom<S> for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline(always)]
  fn geom<'r>(&'r self) -> &'r S
  { &'r self.transformed_geom }

  #[inline(always)]
  fn geom_mut<'r>(&'r mut self) -> &'r mut S
  { &'r mut self.transformed_geom }
}

impl<S, N, M, LV, AV, II, BPP> CanMove for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline(always)]
  fn can_move(&self) -> bool
  {
    match self.state
    {
      Dynamic => true,
      _       => false
    }
  }
}

impl<S: HasBoundingVolume<BV>, N, M, LV, AV, II, BPP, BV>
    HasBoundingVolume<BV> for RigidBody<S, N, M, LV, AV, II, BPP>
{
  fn bounding_volume(&self) -> BV
  { self.transformed_geom.bounding_volume() }
}

// Implementation of proxys
impl<S, N, M, LV, AV, II, BPP>
    HasIndexProxy for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline(always)]
  fn proxy<'l>(&'l self) -> &'l IndexProxy
  { &'l self.index }

  #[inline(always)]
  fn proxy_mut<'l>(&'l mut self) -> &'l mut IndexProxy
  { &mut self.index }
}

impl<S, N, M, LV, AV, II, BPP: Exact<BoundingVolumeProxy<BV>>, BV>
    HasBoundingVolumeProxy<BV> for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline(always)]
  fn proxy<'l>(&'l self) -> &'l BoundingVolumeProxy<BV>
  { self.bp_proxy.exact() }

  #[inline(always)]
  fn proxy_mut<'l>(&'l mut self) -> &'l mut BoundingVolumeProxy<BV>
  { self.bp_proxy.exact_mut() }
}
