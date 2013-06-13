// FIXME: define that on nalgebra?
pub trait Transformable<M>
{
  fn local_to_world(&self) -> M;
  fn world_to_local(&self) -> M;
  fn append(&mut self, &M);
}
