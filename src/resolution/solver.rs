use ncollide::math::N;

pub trait Solver<I> {
    fn solve(&mut self, N, &[I]);
    fn priority(&self) -> f64;
}
