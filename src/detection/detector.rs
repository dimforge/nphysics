//! Collision detector.

use ncollide::math::Scalar;
use detection::activation_manager::ActivationManager;

/// Trait implemented by collision detectors.
pub trait Detector<N: Scalar, I, BF> {
    /// Updates the collision detector, given an (already updated) broad-phase, and an activation
    /// manager.
    fn update(&mut self, &mut BF, &mut ActivationManager<N>);
    /// Collects every interferences detected by this collision detector.
    fn interferences(&mut self, &mut Vec<I>, &mut BF);
}
