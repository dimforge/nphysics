pub trait ConstraintSolver<N, RB, C>
{
  // FIXME: use &mut [(@mut RB, @mut RB, @mut C)] to allow in-place shuffling
  fn solve(&mut self, N, &mut [(@mut RB, @mut RB, @mut C)], &[@mut RB]);
}
