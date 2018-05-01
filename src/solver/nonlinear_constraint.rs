use na::{Real, Unit};
use ncollide::query::ContactKinematic;

use object::{BodyHandle, BodySet};
use solver::IntegrationParameters;
use math::Vector;

pub struct GenericNonlinearConstraint<N: Real> {
    pub body1: BodyHandle,
    pub body2: BodyHandle,
    pub is_angular: bool,
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
        is_angular: bool,
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
            is_angular,
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

    pub kinematic: ContactKinematic<N>,

    pub normal1: Unit<Vector<N>>,
    pub normal2: Unit<Vector<N>>,
}

impl<N: Real> NonlinearUnilateralConstraint<N> {
    pub fn new(
        body1: BodyHandle,
        ndofs1: usize,
        body2: BodyHandle,
        ndofs2: usize,
        normal1: Unit<Vector<N>>,
        normal2: Unit<Vector<N>>,
        kinematic: ContactKinematic<N>,
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
            normal1,
            normal2,
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
        _: &IntegrationParameters<N>,
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
