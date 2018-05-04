use downcast::Any;
use na::{DVector, Real};

use object::{BodyHandle, BodySet};
use solver::{ConstraintSet, IntegrationParameters, NonlinearConstraintGenerator};

pub type ConstraintHandle = usize;

// FIXME: keep this on this module?
pub trait JointConstraint<N: Real>: NonlinearConstraintGenerator<N> + Any + Send + Sync {
    fn is_active(&self, bodies: &BodySet<N>) -> bool {
        let (b1, b2) = self.anchors();
        let body1 = bodies.body(b1);
        let body2 = bodies.body(b2);

        let ndofs1 = body1.status_dependent_ndofs();
        let ndofs2 = body2.status_dependent_ndofs();

        (ndofs1 != 0 && body1.is_active()) || (ndofs2 != 0 && body2.is_active())
    }

    fn num_velocity_constraints(&self) -> usize;
    fn anchors(&self) -> (BodyHandle, BodyHandle);
    fn velocity_constraints(
        &mut self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        velocity_constraints: &mut ConstraintSet<N>,
    );
    fn cache_impulses(&mut self, constraints: &ConstraintSet<N>);
}

downcast!(<N> JointConstraint<N> where N: Real);
