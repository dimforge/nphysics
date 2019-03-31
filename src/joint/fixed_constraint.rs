use na::{DVector, RealField};
use std::ops::Range;

use crate::joint::JointConstraint;
use crate::math::{AngularVector, Rotation, Point, Vector, DIM, SPATIAL_DIM};
use crate::object::{BodyPartHandle, BodySet};
use crate::solver::helper;
use crate::solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters,
             NonlinearConstraintGenerator};

/// A constraint that removes all degrees of freedom between two body parts.
pub struct FixedConstraint<N: RealField> {
    b1: BodyPartHandle,
    b2: BodyPartHandle,
    anchor1: Point<N>,
    ref_frame1: Rotation<N>,
    anchor2: Point<N>,
    ref_frame2: Rotation<N>,
    lin_impulses: Vector<N>,
    ang_impulses: AngularVector<N>,
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,
}

impl<N: RealField> FixedConstraint<N> {
    /// Create a fixed constraint between two body parts.
    /// 
    /// This will ensure the frames `joint_to_b1` and `joint_to_b2` attached to the
    /// body parts `b1` adn `b2` respectively always coincide.
    pub fn new(
        b1: BodyPartHandle,
        b2: BodyPartHandle,
        anchor1: Point<N>,
        ref_frame1: Rotation<N>,
        anchor2: Point<N>,
        ref_frame2: Rotation<N>,
    ) -> Self {
        FixedConstraint {
            b1,
            b2,
            anchor1,
            ref_frame1,
            anchor2,
            ref_frame2,
            lin_impulses: Vector::zeros(),
            ang_impulses: AngularVector::zeros(),
            bilateral_ground_rng: 0..0,
            bilateral_rng: 0..0,
        }
    }

    /// Changes the reference frame for the first body part.
    pub fn set_reference_frame_1(&mut self, ref_frame1: Rotation<N>) {
        self.ref_frame1 = ref_frame1
    }

    /// Changes the reference frame for the second body part.
    pub fn set_reference_frame_2(&mut self, ref_frame2: Rotation<N>) {
        self.ref_frame2 = ref_frame2
    }

    /// Changes the attached material point from the first body part.
    pub fn set_anchor_1(&mut self, anchor1: Point<N>) {
        self.anchor1 = anchor1
    }

    /// Changes the attached material point from the second body part.
    pub fn set_anchor_2(&mut self, anchor2: Point<N>) {
        self.anchor2 = anchor2
    }
}

impl<N: RealField> JointConstraint<N> for FixedConstraint<N> {
    fn num_velocity_constraints(&self) -> usize {
        SPATIAL_DIM
    }

    fn anchors(&self) -> (BodyPartHandle, BodyPartHandle) {
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
        let body1 = try_ret!(bodies.body(self.b1.0));
        let body2 = try_ret!(bodies.body(self.b2.0));
        let part1 = try_ret!(body1.part(self.b1.1));
        let part2 = try_ret!(body2.part(self.b2.1));

        let pos1 = body1.position_at_material_point(part1, &self.anchor1) * self.ref_frame1;
        let pos2 = body2.position_at_material_point(part2, &self.anchor2) * self.ref_frame2;

        let anchor1 = Point::from(pos1.translation.vector);
        let anchor2 = Point::from(pos2.translation.vector);

        let assembly_id1 = body1.companion_id();
        let assembly_id2 = body2.companion_id();

        let first_bilateral_ground = constraints.velocity.bilateral_ground.len();
        let first_bilateral = constraints.velocity.bilateral.len();

        helper::cancel_relative_linear_velocity(
            body1,
            part1,
            body2,
            part2,
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
            body1,
            part1,
            body2,
            part2,
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

impl<N: RealField> NonlinearConstraintGenerator<N> for FixedConstraint<N> {
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
        let body1 = bodies.body(self.b1.0)?;
        let body2 = bodies.body(self.b2.0)?;
        let part1 = body1.part(self.b1.1)?;
        let part2 = body2.part(self.b2.1)?;

        let pos1 = body1.position_at_material_point(part1, &self.anchor1) * self.ref_frame1;
        let pos2 = body2.position_at_material_point(part2, &self.anchor2) * self.ref_frame2;

        let anchor1 = Point::from(pos1.translation.vector);
        let anchor2 = Point::from(pos2.translation.vector);

        if i == 0 {
            let rotation1 = pos1.rotation;
            let rotation2 = pos2.rotation;

            helper::cancel_relative_rotation(
                params,
                body1,
                part1,
                body2,
                part2,
                &anchor1,
                &anchor2,
                &rotation1,
                &rotation2,
                jacobians,
            )
        } else if i == 1 {
            helper::cancel_relative_translation(
                params,
                body1,
                part1,
                body2,
                part2,
                &anchor1,
                &anchor2,
                jacobians,
            )
        } else {
            None
        }
    }
}
