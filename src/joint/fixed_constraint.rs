use na::{DVector, Real};

use object::{BodyHandle, BodySet};
use solver::{BilateralConstraint, BilateralGroundConstraint, IntegrationParameters,
             UnilateralConstraint, UnilateralGroundConstraint};
use solver::helper;
use joint::ConstraintGenerator;
use math::{Isometry, Point, SPATIAL_DIM};

pub struct FixedConstraint<N: Real> {
    b1: BodyHandle,
    b2: BodyHandle,
    joint_to_b1: Isometry<N>,
    joint_to_b2: Isometry<N>,
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
        }
    }

    pub fn set_anchor_1(&mut self, local1: Isometry<N>) {
        self.joint_to_b1 = local1
    }

    pub fn set_anchor_2(&mut self, local2: Isometry<N>) {
        self.joint_to_b2 = local2
    }
}

impl<N: Real> ConstraintGenerator<N> for FixedConstraint<N> {
    fn nconstraints(&self) -> usize {
        SPATIAL_DIM
    }

    fn anchors(&self) -> (BodyHandle, BodyHandle) {
        (self.b1, self.b2)
    }

    fn build_constraints(
        &self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        ground_jacobian_id: &mut usize,
        jacobian_id: &mut usize,
        jacobians: &mut [N],
        _: &mut Vec<UnilateralGroundConstraint<N>>,
        _: &mut Vec<UnilateralConstraint<N>>,
        out_ground: &mut Vec<BilateralGroundConstraint<N>>,
        out: &mut Vec<BilateralConstraint<N>>,
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

        helper::cancel_relative_linear_motion(
            params,
            &b1,
            &b2,
            assembly_id1,
            assembly_id2,
            &anchor1,
            &anchor2,
            ext_vels,
            ground_jacobian_id,
            jacobian_id,
            jacobians,
            out_ground,
            out,
        );
        helper::cancel_relative_angular_motion(
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
            ground_jacobian_id,
            jacobian_id,
            jacobians,
            out_ground,
            out,
        );
    }
}
