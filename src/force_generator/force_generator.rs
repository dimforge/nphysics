#![allow(missing_docs)] // for downcast.

use downcast::Any;
use na::Real;

use object::BodySet;
use solver::IntegrationParameters;

/// The handle of a force generator.
pub type ForceGeneratorHandle = usize;

/// A persistent force generator.
/// 
/// A force generator applies a force to one or several bodies at each step of the simulation.
pub trait ForceGenerator<N: Real>: Any + Send + Sync {
    /// Apply forces to some bodies.
    fn apply(&mut self, params: &IntegrationParameters<N>, bodies: &mut BodySet<N>) -> bool;
}

downcast!(<N> ForceGenerator<N> where N: Real);
