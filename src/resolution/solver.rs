use alga::general::Real;
use world::RigidBodyStorage;

/// Trait implemented by constraint solvers.
pub trait Solver<N: Real, I> {
    /// Solve the set of constraints of type `I`.
    fn solve(&mut self, N, &[I], &mut RigidBodyStorage<N>);
}
