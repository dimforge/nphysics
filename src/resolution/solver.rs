use na::BaseFloat;

/// Trait implemented by constraint solvers.
pub trait Solver<N: BaseFloat, I> {
    /// Solve the set of constraints of type `I`.
    fn solve(&mut self, N, &[I]);
}
