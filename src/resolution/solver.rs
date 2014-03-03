use ncollide::math::Scalar;

/// Trait implemented by constraint solvers.
pub trait Solver<I> {
    /// Solve the set of constraints of type `I`.
    fn solve(&mut self, Scalar, &[I]);
}
