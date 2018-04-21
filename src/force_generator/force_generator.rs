use downcast::Any;
use na::Real;

use object::BodySet;
use solver::IntegrationParameters;

pub type ForceGeneratorHandle = usize;

pub trait ForceGenerator<N: Real>: Any {
    fn apply(&mut self, params: &IntegrationParameters<N>, bodies: &mut BodySet<N>) -> bool;
}

downcast!(<N> ForceGenerator<N> where N: Real);