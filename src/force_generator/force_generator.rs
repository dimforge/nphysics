#![allow(missing_docs)] // for downcast.

use downcast_rs::Downcast;
use na::RealField;

use crate::object::BodySet;
use crate::solver::IntegrationParameters;

/// The handle of a force generator.
pub type ForceGeneratorHandle = usize;

/// A persistent force generator.
/// 
/// A force generator applies a force to one or several bodies at each step of the simulation.
pub trait ForceGenerator<N: RealField>: Downcast + Send + Sync {
    /// Apply forces to some bodies.
    fn apply(&mut self, params: &IntegrationParameters<N>, bodies: &mut BodySet<N>) -> bool;
}

impl_downcast!(ForceGenerator<N> where N: RealField);
