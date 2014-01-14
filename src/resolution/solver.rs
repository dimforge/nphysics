use ncollide::math::N;

/// Trait implemented by constraint solvers.
pub trait Solver<I> {
    /// Solve the set of constraints of type `I`.
    fn solve(&mut self, N, &[I]);
}
