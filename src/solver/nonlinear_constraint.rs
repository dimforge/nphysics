use na::{Real, Unit};
use ncollide::bounding_volume::PolyhedralCone;
use ncollide::query::ContactKinematic;

use object::{BodyHandle, BodySet};
use solver::IntegrationParameters;
use math::{Point, Vector};

pub struct GenericNonlinearConstraint<N: Real> {
    pub body1: BodyHandle,
    pub body2: BodyHandle,
    pub dim1: usize,
    pub dim2: usize,
    pub wj_id1: usize,
    pub wj_id2: usize,
    pub rhs: N,
    pub r: N,
}

impl<N: Real> GenericNonlinearConstraint<N> {
    pub fn new(
        body1: BodyHandle,
        body2: BodyHandle,
        dim1: usize,
        dim2: usize,
        wj_id1: usize,
        wj_id2: usize,
        rhs: N,
        r: N,
    ) -> Self {
        GenericNonlinearConstraint {
            body1,
            body2,
            dim1,
            dim2,
            wj_id1,
            wj_id2,
            rhs,
            r,
        }
    }
}

pub trait NonlinearConstraintGenerator<N: Real> {
    fn num_position_constraints(&self, bodies: &BodySet<N>) -> usize;
    fn position_constraint(
        &self,
        params: &IntegrationParameters<N>,
        i: usize,
        bodies: &mut BodySet<N>,
        jacobians: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N>>;
}

pub struct NonlinearUnilateralConstraint<N: Real> {
    pub r: N,
    pub rhs: N,

    pub ndofs1: usize,
    pub body1: BodyHandle,

    pub ndofs2: usize,
    pub body2: BodyHandle,

    pub kinematic: ContactKinematic<Vector<N>>,

    pub local1: Point<N>,
    pub normal1: Unit<Vector<N>>,
    pub ncone1: PolyhedralCone<Vector<N>>,
    pub margin1: N,

    pub local2: Point<N>,
    pub normal2: Unit<Vector<N>>,
    pub ncone2: PolyhedralCone<Vector<N>>,
    pub margin2: N,
}

impl<N: Real> NonlinearUnilateralConstraint<N> {
    pub fn new(
        body1: BodyHandle,
        ndofs1: usize,
        body2: BodyHandle,
        ndofs2: usize,
        local1: Point<N>,
        normal1: Unit<Vector<N>>,
        ncone1: PolyhedralCone<Vector<N>>,
        margin1: N,
        local2: Point<N>,
        normal2: Unit<Vector<N>>,
        ncone2: PolyhedralCone<Vector<N>>,
        margin2: N,
        kinematic: ContactKinematic<Vector<N>>,
    ) -> Self {
        let r = N::zero();
        let rhs = N::zero();

        NonlinearUnilateralConstraint {
            r,
            rhs,
            ndofs1,
            body1,
            ndofs2,
            body2,
            kinematic,
            local1,
            normal1,
            ncone1,
            margin1,
            local2,
            normal2,
            ncone2,
            margin2,
        }
    }
}

pub struct MultibodyJointLimitsNonlinearConstraintGenerator {
    link: BodyHandle,
}

impl MultibodyJointLimitsNonlinearConstraintGenerator {
    pub fn new(link: BodyHandle) -> Self {
        MultibodyJointLimitsNonlinearConstraintGenerator { link }
    }
}

impl<N: Real> NonlinearConstraintGenerator<N> for MultibodyJointLimitsNonlinearConstraintGenerator {
    fn num_position_constraints(&self, bodies: &BodySet<N>) -> usize {
        if let Some(link) = bodies.multibody_link(self.link) {
            link.joint().num_position_constraints()
        } else {
            0
        }
    }

    fn position_constraint(
        &self,
        params: &IntegrationParameters<N>,
        i: usize,
        bodies: &mut BodySet<N>,
        jacobians: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N>> {
        if let Some(link) = bodies.multibody_link(self.link) {
            link.joint().position_constraint(i, &link, 0, jacobians)
        } else {
            None
        }
    }
}
