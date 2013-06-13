use std::num::{Zero, One};
use nalgebra::traits::inv::Inv;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::rotation::Rotation;
// FIXME: use nalgebra::traits::delta_transform::DeltaTransform;
use gt = ncollide::geom::transformable;
use ncollide::geom::has_geom::HasGeom;
use body::volumetric::Volumetric;
use body::dynamic::Dynamic;
use body::transformable::Transformable;
use body::can_move::CanMove;
use proxy::index_proxy::IndexProxy;
use proxy::has_proxy::HasProxy;

pub enum RigidBodyState {
  Static,
  Dynamic,
  Kinematic // FIXME: not yet supported
}


pub struct RigidBody<S, N, M, LV, AV, II>
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
}

impl<S: gt::Transformable<M, S>, N, M/*FIXME: : DeltaTransform<II>*/, LV, AV,
     II: Mul<II, II> + Copy>
RigidBody<S, N, M, LV, AV, II>
{
  fn moved(&mut self)
  {
    self.geom.transform_to(&self.local_to_world, &mut self.transformed_geom);

    // FIXME: the inverse inertia should be computed lazily (use a @mut ?).
    self.inv_inertia = // FIXME: self.local_to_world.delta_transform() *
                       self.ls_inv_inertia // FIXME:                  *
                       // FIXME: self.world_to_local.delta_transform()
  }
}

impl<S:  gt::Transformable<M, S> + Volumetric<N, II>,
     N:  One + Zero + Div<N, N> + Mul<N, N>,
     M:  One, // FIXME: + DeltaTransform<II>,
     LV: Zero,
     AV: Zero,
     II: One + Zero + Inv + Mul<II, II> + Copy>
RigidBody<S, N, M, LV, AV, II>
{
  pub fn new(geom: S, density: N, state: RigidBodyState) -> RigidBody<S, N, M, LV, AV, II>
  {
    let (inv_mass, inv_inertia) =
      match state
      {
        Static    => (Zero::zero(), Zero::zero()),
        Dynamic   => {
          let volume = geom.volume();
          if (volume.is_zero())
          { fail!("A dynamic body cannot have a zero volume.") }
          if (density.is_zero())
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
        transformed_geom: geom.transform(&One::one()),
        geom:             geom,
        local_to_world:   One::one(),
        world_to_local:   One::one(),
        lin_vel:          Zero::zero(),
        ang_vel:          Zero::zero(),
        inv_mass:         inv_mass,
        ls_inv_inertia:   inv_inertia,
        inv_inertia:      inv_inertia,
        lin_force:        Zero::zero(),
        ang_force:        Zero::zero(),
        index:            IndexProxy::new()
      };

    res.moved();

    res
  }
}

impl<S, N: Copy, M: Copy, LV: Copy, AV: Copy, II: Copy>
    Dynamic<N, LV, AV, II> for RigidBody<S, N, M, LV, AV, II>
{
  fn lin_vel(&self) -> LV
  { self.lin_vel }
  fn set_lin_vel(&mut self, &lv: &LV)
  { self.lin_vel = lv }

  fn ang_vel(&self) -> AV
  { self.ang_vel }
  fn set_ang_vel(&mut self, &av: &AV)
  { self.ang_vel = av }

  fn inv_mass(&self) -> N
  { self.inv_mass }
  fn set_inv_mass(&mut self, &m: &N)
  { self.inv_mass = m }

  fn inv_inertia(&self) -> II
  { self.inv_inertia }
  fn set_inv_inertia(&mut self, &ii: &II)
  { self.inv_inertia = ii }

  // FIXME: create another trait for forces?
  fn ext_lin_force(&self) -> LV
  { self.lin_force }
  fn set_ext_lin_force(&mut self, &lf: &LV)
  { self.lin_force = lf }

  fn ext_ang_force(&self) -> AV
  { self.ang_force }
  fn set_ext_ang_force(&mut self, &af: &AV)
  { self.ang_force = af }
}

impl<S: gt::Transformable<M, S>,
     N,
     M: Copy + Inv + Mul<M, M>, // FIXME: + DeltaTransform<II>,
     LV,
     AV,
     II: Mul<II, II> + Copy>
Transformable<M> for
    RigidBody<S, N, M, LV, AV, II>
{
  fn local_to_world(&self) -> M
  { self.local_to_world }

  fn world_to_local(&self) -> M
  { self.world_to_local }

  fn append(&mut self, &to_append: &M)
  {
    self.local_to_world *= to_append;
    self.world_to_local = self.local_to_world.inverse();
    self.moved();
  }
}

impl<S: Copy + gt::Transformable<M, S>,
     N: Copy,
     M: Copy + Translation<LV>, // FIXME: + DeltaTransform<II>,
     LV: Copy,
     AV: Copy,
     II: Copy + Mul<II, II>>
    Translation<LV> for RigidBody<S, N, M, LV, AV, II>
{
  fn translation(&self) -> LV
  { self.local_to_world.translation() }

  fn translated(&self, trans: &LV) -> RigidBody<S, N, M, LV, AV, II>
  {
    let mut &cpy = copy self;

    cpy.translate(trans);

    cpy
  }

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
     II: Copy + Mul<II, II>>
    Rotation<AV> for RigidBody<S, N, M, LV, AV, II>
{
  fn rotation(&self) -> AV
  { self.local_to_world.rotation() }

  fn rotated(&self, rot: &AV) -> RigidBody<S, N, M, LV, AV, II>
  {
    let mut &cpy = copy self;

    cpy.rotate(rot);

    cpy
  }

  fn rotate(&mut self, rot: &AV)
  {
    self.local_to_world.rotate(rot);
    self.moved();
  }
}

impl<S, N, M, LV, AV, II>
    HasProxy<IndexProxy> for RigidBody<S, N, M, LV, AV, II>
{
  fn proxy<'l>(&'l self) -> &'l IndexProxy
  { &'l self.index }

  fn proxy_mut<'l>(&'l mut self) -> &'l mut IndexProxy
  { &mut self.index }

  fn set_proxy(&mut self, index: &IndexProxy)
  { self.index = *index }
}

impl<S, N, M, LV, AV, II> HasGeom<S> for RigidBody<S, N, M, LV, AV, II>
{
  fn geom<'r>(&'r self) -> &'r S
  { &'r self.transformed_geom }

  fn geom_mut<'r>(&'r mut self) -> &'r mut S
  { &'r mut self.transformed_geom }
}

impl<S, N, M, LV, AV, II> CanMove for RigidBody<S, N, M, LV, AV, II>
{
  fn can_move(&self) -> bool
  {
    match self.state
    {
      Dynamic => true,
      _       => false
    }
  }
}
