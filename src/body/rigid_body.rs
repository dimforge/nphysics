use std::num::{Zero, One};
use nalgebra::traits::inv::Inv;
use nalgebra::traits::translation::{Translation, Translatable};
use nalgebra::traits::rotation::{Rotation, Rotatable};
use nalgebra::traits::transformation::Transformation;
// FIXME: use nalgebra::traits::delta_transform::DeltaTransform;
use ncollide::broad::brute_force_bounding_volume_broad_phase::BoundingVolumeProxy;
use ncollide::broad::brute_force_bounding_volume_broad_phase::HasBoundingVolumeProxy;
use ncollide::bounding_volume::has_bounding_volume::HasBoundingVolume;
use ncollide::geom::has_geom::HasGeom;
use ncollide::utils::exact::Exact;
use ncollide::utils::default::Default;
use body::volumetric::Volumetric;
use body::dynamic::Dynamic;
use body::can_move::CanMove;
use body::material::Material;
use constraint::index_proxy::{HasIndexProxy, IndexProxy};

#[deriving(ToStr, Eq, Clone)]
pub enum RigidBodyState {
  Static,
  Dynamic,
  Kinematic // FIXME: not yet supported
}


#[deriving(ToStr, Clone, Eq)]
pub struct RigidBody<S, N, M, LV, AV, II, BPP>
{
  priv state:            RigidBodyState,
  priv geom:             S,
  priv local_to_world:   M, // FIXME: useless in fact…
  priv world_to_local:   M,
  priv lin_vel:          LV,
  priv ang_vel:          AV,
  priv inv_mass:         N,
  priv ls_inv_inertia:   II, // NOTE: 'ls' means 'local space'…
  priv inv_inertia:      II,
  priv lin_force:        LV,
  priv ang_force:        AV,
  priv restitution:      N,
  priv friction:         N,
  priv index:            IndexProxy,
  priv bp_proxy:         BPP,
}

impl<S: Transformation<M>,
     N,
     M/*FIXME: : DeltaTransform<II>*/,
     LV,
     AV,
     II: Mul<II, II> + Clone,
     BPP>
RigidBody<S, N, M, LV, AV, II, BPP>
{
  fn moved(&mut self, delta: &M)
  {
    self.geom.transform_by(delta);

    // FIXME: the inverse inertia should be computed lazily (use a @mut ?).
    self.inv_inertia = // FIXME: self.local_to_world.delta_transform() *
                       self.ls_inv_inertia.clone() // FIXME:                  *
                       // FIXME: self.world_to_local.delta_transform()
  }
}

impl<S:   Transformation<M> + Volumetric<N, II>,
     N:   One + Zero + Div<N, N> + Mul<N, N>,
     M:   One, // FIXME: + DeltaTransform<II>,
     LV:  Zero,
     AV:  Zero,
     II:  One + Zero + Inv + Mul<II, II> + Clone,
     BPP: Default>
RigidBody<S, N, M, LV, AV, II, BPP>
{
  pub fn new(geom:        S,
             density:     N,
             state:       RigidBodyState,
             restitution: N,
             friction:    N) -> RigidBody<S, N, M, LV, AV, II, BPP>
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

          let mass = density * volume;

          match geom.inertia(&mass).inverse()
          {
            Some(ii) => (One::one::<N>() / mass, ii),
            None     => fail!("A dynamic body cannot have a singular inertia tensor.")
          }
        },
        Kinematic => {
          println("Warning: for now, kinematic object behave like static one.");
          (Zero::zero(), Zero::zero())
        }
      };

    let mut res =
      RigidBody {
        state:            state,
        geom:             geom,
        local_to_world:   One::one(),
        world_to_local:   One::one(),
        lin_vel:          Zero::zero(),
        ang_vel:          Zero::zero(),
        inv_mass:         inv_mass,
        ls_inv_inertia:   inv_inertia.clone(),
        inv_inertia:      inv_inertia,
        lin_force:        Zero::zero(),
        ang_force:        Zero::zero(),
        friction:         friction,
        restitution:      restitution,
        index:            IndexProxy::new(),
        bp_proxy:         Default::default()
      };

    res.moved(&One::one());

    res
  }
}

impl<S, N: Clone, M: Clone, LV: Clone, AV: Clone, II: Clone, BPP>
    Dynamic<N, LV, AV, II> for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline]
  fn lin_vel(&self) -> LV
  { self.lin_vel.clone() }
  #[inline]
  fn set_lin_vel(&mut self, lv: &LV)
  { self.lin_vel = lv.clone() }

  #[inline]
  fn ang_vel(&self) -> AV
  { self.ang_vel.clone() }
  #[inline]
  fn set_ang_vel(&mut self, av: &AV)
  { self.ang_vel = av.clone() }

  #[inline]
  fn inv_mass(&self) -> N
  { self.inv_mass.clone() }
  #[inline]
  fn set_inv_mass(&mut self, m: &N)
  { self.inv_mass = m.clone() }

  #[inline]
  fn inv_inertia(&self) -> II
  { self.inv_inertia.clone() }
  #[inline]
  fn set_inv_inertia(&mut self, ii: &II)
  { self.inv_inertia = ii.clone() }

  // FIXME: create another trait for forces?
  #[inline]
  fn ext_lin_force(&self) -> LV
  { self.lin_force.clone() }
  #[inline]
  fn set_ext_lin_force(&mut self, lf: &LV)
  { self.lin_force = lf.clone() }

  #[inline]
  fn ext_ang_force(&self) -> AV
  { self.ang_force.clone() }
  #[inline]
  fn set_ext_ang_force(&mut self, af: &AV)
  { self.ang_force = af.clone() }
}

impl<S: Transformation<M>,
     N,
     M: Clone + Inv + Mul<M, M>, // FIXME: + DeltaTransform<II>,
     LV,
     AV,
     II: Mul<II, II> + Clone,
     BPP>
Transformation<M> for
    RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline]
  fn transformation(&self) -> M
  { self.local_to_world.clone() }

  #[inline]
  fn inv_transformation(&self) -> M
  { self.world_to_local.clone() }


  #[inline]
  fn transform_by(&mut self, to_append: &M)
  {
    self.local_to_world = *to_append * self.local_to_world;

    match self.local_to_world.inverse()
    {
      Some(l2w) => self.world_to_local = l2w,
      None      => fail!("Internal error: rigid body has a singular local_to_world transform.")
    }
    self.moved(to_append);
  }
}

// FIXME: implement Transfomable too

impl<S: Transformation<M>,
     N,
     M: Translation<LV> + One, // FIXME: + DeltaTransform<II>,
     LV,
     AV,
     II: Mul<II, II> + Clone,
     BPP>
    Translation<LV> for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline]
  fn translation(&self) -> LV
  { self.local_to_world.translation() }

  #[inline]
  fn inv_translation(&self) -> LV
  { self.local_to_world.inv_translation() }


  #[inline]
  fn translate_by(&mut self, trans: &LV)
  {
    self.local_to_world.translate_by(trans);

    let mut delta = One::one::<M>();
    delta.translate_by(trans);
    self.moved(&delta);
  }
}

impl<S: Clone + Transformation<M>,
     N: Clone,
     M: Clone + Translation<LV> + One, // FIXME: + DeltaTransform<II>,
     LV: Clone,
     AV: Clone,
     II: Clone + Mul<II, II>,
     BPP: Clone>
    Translatable<LV, RigidBody<S, N, M, LV, AV, II, BPP>> for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline]
  fn translated(&self, trans: &LV) -> RigidBody<S, N, M, LV, AV, II, BPP>
  {
    let mut cpy = self.clone();

    cpy.translate_by(trans);

    cpy
  }
}

impl<S: Transformation<M>,
     N,
     M: Rotation<AV> + One,
     LV,
     AV,
     II: Mul<II, II> + Clone,
     BPP>
    Rotation<AV> for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline]
  fn rotation(&self) -> AV
  { self.local_to_world.rotation() }

  #[inline]
  fn inv_rotation(&self) -> AV
  { self.local_to_world.inv_rotation() }

  #[inline]
  fn rotate_by(&mut self, rot: &AV)
  {
    self.local_to_world.rotate_by(rot);

    let mut delta = One::one::<M>();
    delta.rotate_by(rot);
    self.moved(&delta);
  }
}

impl<S:   Clone + Transformation<M>,
     N:   Clone,
     M:   Clone + Rotation<AV> + One,
     LV:  Clone,
     AV:  Clone,
     II:  Clone + Mul<II, II>,
     BPP: Clone>
    Rotatable<AV, RigidBody<S, N, M, LV, AV, II, BPP>> for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline]
  fn rotated(&self, rot: &AV) -> RigidBody<S, N, M, LV, AV, II, BPP>
  {
    let mut cpy = self.clone();

    cpy.rotate_by(rot);

    cpy
  }
}

impl<S, N, M, LV, AV, II, BPP> HasGeom<S> for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline]
  fn geom<'r>(&'r self) -> &'r S
  { &'r self.geom }

  #[inline]
  fn geom_mut<'r>(&'r mut self) -> &'r mut S
  { &'r mut self.geom }
}

impl<S, N, M, LV, AV, II, BPP> CanMove for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline]
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
  { self.geom.bounding_volume() }
}

// Implementation of proxys
impl<S, N, M, LV, AV, II, BPP>
    HasIndexProxy for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline]
  fn proxy<'l>(&'l self) -> &'l IndexProxy
  { &'l self.index }

  #[inline]
  fn proxy_mut<'l>(&'l mut self) -> &'l mut IndexProxy
  { &mut self.index }
}

impl<S, N, M, LV, AV, II, BPP: Exact<BoundingVolumeProxy<BV>>, BV>
    HasBoundingVolumeProxy<BV> for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline]
  fn proxy<'l>(&'l self) -> &'l BoundingVolumeProxy<BV>
  { self.bp_proxy.exact() }

  #[inline]
  fn proxy_mut<'l>(&'l mut self) -> &'l mut BoundingVolumeProxy<BV>
  { self.bp_proxy.exact_mut() }
}

impl<S, N: Clone, M, LV, AV, II, BPP>
    Material<N> for RigidBody<S, N, M, LV, AV, II, BPP>
{
  #[inline]
  pub fn restitution_coefficient(&self) -> N
  { self.restitution.clone() }

  #[inline]
  pub fn friction_coefficient(&self) -> N
  { self.friction.clone() }
}
