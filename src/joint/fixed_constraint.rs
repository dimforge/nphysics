use na::{DVector, Real};
use std::ops::Range;

use joint::JointConstraint;
use math::{AngularVector, Isometry, Point, Vector, DIM, SPATIAL_DIM};
use object::{BodyHandle, BodySet};
use solver::helper;
use solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters,
             NonlinearConstraintGenerator};

/// A constraint that removes all degrees of freedom between two body parts.
pub struct FixedConstraint<N: Real> {
    b1: BodyHandle,
    b2: BodyHandle,
    joint_to_b1: Isometry<N>,
    joint_to_b2: Isometry<N>,
    lin_impulses: Vector<N>,
    ang_impulses: AngularVector<N>,
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,
}

impl<N: Real> FixedConstraint<N> {
    /// Create a fixed constraint between two body parts.
    /// 
    /// This will ensure the frames `joint_to_b1` and `joint_to_b2` attached to the
    /// body parts `b1` adn `b2` respectively always coincide.
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
            ang_impulses: AngularVector::zeros(),
            bilateral_ground_rng: 0..0,
            bilateral_rng: 0..0,
        }
    }

    /// Changes the frame attached to the first body part.
    pub fn set_anchor_1(&mut self, local1: Isometry<N>) {
        self.joint_to_b1 = local1
    }

    /// Changes the frame attached to the second body part.
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
        _: &IntegrationParameters<N>,
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

        let assembly_id1 = b1.parent_companion_id();
        let assembly_id2 = b2.parent_companion_id();

        let first_bilateral_ground = constraints.velocity.bilateral_ground.len();
        let first_bilateral = constraints.velocity.bilateral.len();

        helper::cancel_relative_linear_velocity(
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
            &b1,
            &b2,
            assembly_id1,
            assembly_id2,
            &anchor1,
            &anchor2,
            ext_vels,
            &self.ang_impulses,
            DIM,
            ground_j_id,
            j_id,
            jacobians,
            constraints,
        );

        self.bilateral_ground_rng =
            first_bilateral_ground..constraints.velocity.bilateral_ground.len();
        self.bilateral_rng = first_bilateral..constraints.velocity.bilateral.len();
    }

    fn cache_impulses(&mut self, constraints: &ConstraintSet<N>) {
        for c in &constraints.velocity.bilateral_ground[self.bilateral_ground_rng.clone()] {
            if c.impulse_id < DIM {
                self.lin_impulses[c.impulse_id] = c.impulse;
            } else {
                self.ang_impulses[c.impulse_id - DIM] = c.impulse;
            }
        }

        for c in &constraints.velocity.bilateral[self.bilateral_rng.clone()] {
            if c.impulse_id < DIM {
                self.lin_impulses[c.impulse_id] = c.impulse;
            } else {
                self.ang_impulses[c.impulse_id - DIM] = c.impulse;
            }
        }
    }
}

impl<N: Real> NonlinearConstraintGenerator<N> for FixedConstraint<N> {
    fn num_position_constraints(&self, bodies: &BodySet<N>) -> usize {
        // FIXME: calling this at each iteration of the non-linear resolution is costly.
        if self.is_active(bodies) {
            2
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
        let body1 = bodies.body_part(self.b1);
        let body2 = bodies.body_part(self.b2);

        let pos1 = body1.position() * self.joint_to_b1;
        let pos2 = body2.position() * self.joint_to_b2;

        let anchor1 = Point::from_coordinates(pos1.translation.vector);
        let anchor2 = Point::from_coordinates(pos2.translation.vector);

        if i == 0 {
            let rotation1 = pos1.rotation;
            let rotation2 = pos2.rotation;

            helper::cancel_relative_rotation(
                params,
                &body1,
                &body2,
                &anchor1,
                &anchor2,
                &rotation1,
                &rotation2,
                jacobians,
            )
        } else if i == 1 {
            helper::cancel_relative_translation(
                params,
                &body1,
                &body2,
                &anchor1,
                &anchor2,
                jacobians,
            )
        } else {
            None
        }
    }
}
