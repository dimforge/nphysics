use na::{DVector, Real};

use detection::BodyContactManifold;
use solver::{ConstraintSet, IntegrationParameters};
use object::BodySet;

pub trait ContactModel<N: Real>: 'static {
    fn nconstraints(&self, manifold: &BodyContactManifold<N>) -> usize;
    fn build_constraints(
        &mut self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        manifolds: &[BodyContactManifold<N>],
        ground_jacobian_id: &mut usize,
        jacobian_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    );

    fn cache_impulses(&mut self, constraints: &ConstraintSet<N>);
}
