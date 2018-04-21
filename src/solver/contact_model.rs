use na::{DVector, Real};

use detection::ColliderContactManifold;
use solver::{ConstraintSet, IntegrationParameters};
use object::BodySet;

pub trait ContactModel<N: Real>: 'static {
    fn num_velocity_constraints(&self, manifold: &ColliderContactManifold<N>) -> usize;
    fn constraints(
        &mut self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        manifolds: &[ColliderContactManifold<N>],
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    );

    fn cache_impulses(&mut self, constraints: &ConstraintSet<N>);
}