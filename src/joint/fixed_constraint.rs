use na::{DVector, Real};

use object::{BodyHandle, BodySet};
use solver::{ConstraintSet, IntegrationParameters, GenericNonlinearConstraint, NonlinearConstraintGenerator};
use solver::helper;
use joint::JointConstraint;
use math::{Isometry, Point, Vector, AngularVector, SPATIAL_DIM};

pub struct FixedConstraint<N: Real> {
    b1: BodyHandle,
    b2: BodyHandle,
    joint_to_b1: Isometry<N>,
    joint_to_b2: Isometry<N>,
    lin_impulses: Vector<N>,
    ang_impulses: AngularVector<N>
}

impl<N: Real> FixedConstraint<N> {
    pub fn new(
        b1: BodyHandle,
        b2: BodyHandle,
        joint_to_b1: Isometry<N>,
        joint_to_b2: Isometry<N>,
    ) -> Self {
        FixedConstraint {
            b1,
            b2,
            joint_to_b1,
            joint_to_b2,
            lin_impulses: Vector::zeros(),
            ang_impulses: AngularVector::zeros()
        }
    }

    pub fn set_anchor_1(&mut self, local1: Isometry<N>) {
        self.joint_to_b1 = local1
    }

    pub fn set_anchor_2(&mut self, local2: Isometry<N>) {
        self.joint_to_b2 = local2
    }
}

impl<N: Real> JointConstraint<N> for FixedConstraint<N> {
    fn num_velocity_constraints(&self) -> usize {
        SPATIAL_DIM
    }

    fn anchors(&self) -> (BodyHandle, BodyHandle) {
        (self.b1, self.b2)
    }

    fn velocity_constraints(
        &mut self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    ) {
        let b1 = bodies.body_part(self.b1);
        let b2 = bodies.body_part(self.b2);

        let pos1 = b1.position() * self.joint_to_b1;
        let pos2 = b2.position() * self.joint_to_b2;

        let anchor1 = Point::from_coordinates(pos1.translation.vector);
        let anchor2 = Point::from_coordinates(pos2.translation.vector);

        let rotation1 = pos1.rotation;
        let rotation2 = pos2.rotation;

        let assembly_id1 = b1.parent_companion_id();
        let assembly_id2 = b2.parent_companion_id();

        helper::cancel_relative_linear_velocity(
            params,
            &b1,
            &b2,
            assembly_id1,
            assembly_id2,
            &anchor1,
            &anchor2,
            ext_vels,
            &self.lin_impulses,
            0,
            ground_j_id,
            j_id,
            jacobians,
            constraints,
        );
        helper::cancel_relative_angular_velocity(
            params,
            &b1,
            &b2,
            assembly_id1,
            assembly_id2,
            &rotation1,
            &rotation2,
            &anchor1,
            &anchor2,
            ext_vels,
            &self.ang_impulses,
            3,
            ground_j_id,
            j_id,
            jacobians,
            constraints,
        );
    }

    fn cache_impulses(&mut self, constraints: &ConstraintSet<N>) {
    }
}


impl<N: Real> NonlinearConstraintGenerator<N> for FixedConstraint<N> {
    fn num_position_constraints(&self, _: &BodySet<N>) -> usize {
        0
    }

    fn position_constraint(
        &self,
        params: &IntegrationParameters<N>,
        i: usize,
        bodies: &mut BodySet<N>,
        jacobians: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N>> {
        None
    }
}
