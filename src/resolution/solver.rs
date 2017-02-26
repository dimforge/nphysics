use alga::general::Real;

/// Trait implemented by constraint solvers.
pub trait Solver<N: Real, I> {
    /// Solve the set of constraints of type `I`.
    fn solve(&mut self, N, &[I]);
}
