pub trait Solver<N, I>
{
    fn solve(&mut self, N, &[I]);
}
