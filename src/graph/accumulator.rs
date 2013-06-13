pub trait Accumulator<E, N, R>
{
  fn reset(&mut self);
  fn accumulate_from_edge(&mut self, edge: &mut E) -> bool;
  fn accumulate_from_node(&mut self, node: &mut N) -> bool;
  fn result(&mut self) -> R;
}
