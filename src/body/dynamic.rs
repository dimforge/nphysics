pub trait Dynamic<N, LV, AV, II>
{
  fn lin_vel(&self) -> LV;
  fn set_lin_vel(&mut self, &LV);

  fn ang_vel(&self) -> AV;
  fn set_ang_vel(&mut self, &AV);

  fn inv_mass(&self) -> N;
  fn set_inv_mass(&mut self, &N);

  fn inv_inertia(&self) -> II;
  fn set_inv_inertia(&mut self, &II);

  // FIXME: create another trait for forces?
  fn ext_lin_force(&self) -> LV;
  fn set_ext_lin_force(&mut self, &LV);

  fn ext_ang_force(&self) -> AV;
  fn set_ext_ang_force(&mut self, &AV);
}
