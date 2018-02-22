use downcast::Any;
use na::{DVector, Real};

use object::{BodyHandle, BodySet};
use solver::{BilateralConstraint, BilateralGroundConstraint, IntegrationParameters,
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
        out_unilateral_ground: &mut Vec<UnilateralGroundConstraint<N>>,
        out_unilateral: &mut Vec<UnilateralConstraint<N>>,
        out_bilateral_ground: &mut Vec<BilateralGroundConstraint<N>>,
        out_bilateral: &mut Vec<BilateralConstraint<N>>,
    );
}

downcast!(<N> ConstraintGenerator<N> where N: Real);
