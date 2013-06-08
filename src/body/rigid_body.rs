use std::num::{Zero, One};
use nalgebra::traits::inv::Inv;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::rotation::Rotation;
use body::dynamic::Dynamic;
use body::transformable::Transformable;


pub struct RigidBody<S, T, M, LV, AV, II>
{
  shape:          ~S,
  local_to_world: M,
  world_to_local: M,
  lin_vel:        LV,
  ang_vel:        AV,
  inv_mass:       T,
  inv_inertia:    II,
  lin_force:      LV,
  ang_force:      AV
}

impl<S, T: One, M: One, LV: Zero, AV: Zero, II: One>
    RigidBody<S, T, M, LV, AV, II>
{
  pub fn new(shape: ~S) -> RigidBody<S, T, M, LV, AV, II>
  {
    RigidBody {
      shape:          shape,
      local_to_world: One::one(),
      world_to_local: One::one(),
      lin_vel:        Zero::zero(),
      ang_vel:        Zero::zero(),
      inv_mass:       One::one(),
      inv_inertia:    One::one(),
      lin_force:      Zero::zero(),
      ang_force:      Zero::zero(),
    }
  }
}

impl<S, T: Copy, M: Copy, LV: Copy, AV: Copy, II: Copy>
    Dynamic<T, LV, AV, II> for RigidBody<S, T, M, LV, AV, II>
{
  fn lin_vel(&self) -> LV
  { self.lin_vel }
  fn set_lin_vel(&mut self, &lv: &LV)
  { self.lin_vel = lv }

  fn ang_vel(&self) -> AV
  { self.ang_vel }
  fn set_ang_vel(&mut self, &av: &AV)
  { self.ang_vel = av }

  fn inv_mass(&self) -> T
  { self.inv_mass }
  fn set_inv_mass(&mut self, &m: &T)
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

impl<S, T, M: Copy + Inv + Mul<M, M>, LV, AV, II> Transformable<M> for
    RigidBody<S, T, M, LV, AV, II>
{
  fn local_to_world(&self) -> M
  { self.local_to_world }

  fn world_to_local(&self) -> M
  { self.world_to_local }

  fn append(&mut self, &to_append: &M)
  {
    self.local_to_world *= to_append;
    self.world_to_local = self.local_to_world.inverse();
  }
}

impl<S: Copy,
     T: Copy,
     M: Copy + Translation<LV>,
     LV: Copy,
     AV: Copy,
     II: Copy>
    Translation<LV> for RigidBody<S, T, M, LV, AV, II>
{
  fn translation(&self) -> LV
  { self.local_to_world.translation() }

  fn translated(&self, trans: &LV) -> RigidBody<S, T, M, LV, AV, II>
  {
    let mut &cpy = copy self;

    cpy.translate(trans);

    cpy
  }

  fn translate(&mut self, trans: &LV)
  { self.local_to_world.translate(trans); }
}

impl<S:  Copy,
     T:  Copy,
     M:  Copy + Rotation<AV>,
     LV: Copy,
     AV: Copy,
     II: Copy>
    Rotation<AV> for RigidBody<S, T, M, LV, AV, II>
{
  fn rotation(&self) -> AV
  { self.local_to_world.rotation() }

  fn rotated(&self, rot: &AV) -> RigidBody<S, T, M, LV, AV, II>
  {
    let mut &cpy = copy self;

    cpy.rotate(rot);

    cpy
  }

  fn rotate(&mut self, rot: &AV)
  { self.local_to_world.rotate(rot); }
}
