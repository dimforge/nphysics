use na::{DVector, Real};

use detection::BodyContactManifold;
use solver::{BilateralConstraint, BilateralGroundConstraint, ConstraintSet, IntegrationParameters,
             UnilateralConstraint, UnilateralGroundConstraint};
use object::BodySet;

pub trait ContactModel<N: Real>: 'static {
    fn nconstraints(&self, manifold: &BodyContactManifold<N>) -> usize;
    fn build_constraints(
        &self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        manifolds: &[BodyContactManifold<N>],
        ground_jacobian_id: &mut usize,
        jacobian_id: &mut usize,
        jacobians: &mut [N],
        velocity_constraints: &mut ConstraintSet<N>,
    );

    fn cache_impulses(
        &mut self,
        bodies: &BodySet<N>,
        manifolds: &[BodyContactManifold<N>],
        ground_contacts: &[UnilateralGroundConstraint<N>],
        contacts: &[UnilateralConstraint<N>],
        ground_friction: &[BilateralGroundConstraint<N>],
        friction: &[BilateralConstraint<N>],
    );
}
