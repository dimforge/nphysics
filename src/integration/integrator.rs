//! Trait implemented by every integrators.

use alga::general::Real;

/// Trait implemented by every integrator.
///
/// An integrator is a structure capable of updating a dynamic body position and orientation after a given time-step.
pub trait Integrator<N: Real, O> {
    /// Updates the position and orientation of the object `o` after a time step of `dt`.
    fn update(&mut self, dt: N, o: &mut O);
}
