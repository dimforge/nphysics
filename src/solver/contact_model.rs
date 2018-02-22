use na::{DVector, Real};

use detection::BodyContactManifold;
use solver::{BilateralConstraint, BilateralGroundConstraint, IntegrationParameters,
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
        out_ground_contacts: &mut Vec<UnilateralGroundConstraint<N>>,
        out_contacts: &mut Vec<UnilateralConstraint<N>>,
        out_ground_friction: &mut Vec<BilateralGroundConstraint<N>>,
        out_friction: &mut Vec<BilateralConstraint<N>>,
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
