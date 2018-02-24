use downcast::Any;
use na::{DVector, Real};

use object::{BodyHandle, BodySet};
use solver::{BilateralConstraint, BilateralGroundConstraint, ConstraintSet, IntegrationParameters,
             UnilateralConstraint, UnilateralGroundConstraint};

pub type ConstraintHandle = usize;

// FIXME: keep this on this module?
pub trait ConstraintGenerator<N: Real>: Any {
    fn nconstraints(&self) -> usize;
    fn anchors(&self) -> (BodyHandle, BodyHandle);
    fn build_constraints(
        &self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        ground_jacobian_id: &mut usize,
        jacobian_id: &mut usize,
        jacobians: &mut [N],
        velocity_constraints: &mut ConstraintSet<N>,
    );
}

downcast!(<N> ConstraintGenerator<N> where N: Real);
