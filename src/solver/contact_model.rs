use na::{DVector, Real};

use detection::ContactConstraint;
use solver::{BilateralConstraint2, BilateralGroundConstraint, IntegrationParameters,
             UnilateralConstraint2, UnilateralGroundConstraint};
use object::BodySet;

pub trait ContactModel<N: Real>: 'static {
    fn nconstraints(&self) -> usize;
    fn build_constraints(
        &self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        ground_jacobian_id: &mut usize,
        jacobian_id: &mut usize,
        jacobians: &mut [N],
        c: &ContactConstraint<N>,
        out_ground_contacts: &mut Vec<UnilateralGroundConstraint<N>>,
        out_contacts: &mut Vec<UnilateralConstraint2<N>>,
        out_ground_friction: &mut Vec<BilateralGroundConstraint<N>>,
        out_friction: &mut Vec<BilateralConstraint2<N>>,
    ) -> bool;
}
