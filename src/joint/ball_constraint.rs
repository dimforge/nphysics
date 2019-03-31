use std::ops::Range;
use na::{DVector, RealField};

use crate::object::{BodyPartHandle, BodySet};
use crate::solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters,
             NonlinearConstraintGenerator};
use crate::solver::helper;
use crate::joint::JointConstraint;
use crate::math::{Point, Vector, DIM};

/// A constraint that removes all relative linear motion between two body parts.
pub struct BallConstraint<N: RealField> {
    b1: BodyPartHandle,
    b2: BodyPartHandle,
    anchor1: Point<N>,
    anchor2: Point<N>,
    impulses: Vector<N>,
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,
}

impl<N: RealField> BallConstraint<N> {
    /// Creates a ball constraint between two body parts.
    /// 
    /// This will ensure the two points identified by `anchor1` and `anchor2` will coincide.
    /// Both are given in the local-space of their corresponding body part.
    pub fn new(b1: BodyPartHandle, b2: BodyPartHandle, anchor1: Point<N>, anchor2: Point<N>) -> Self {
        BallConstraint {
            b1,
            b2,
            anchor1,
            anchor2,
            impulses: Vector::zeros(),
            bilateral_ground_rng: 0..0,
            bilateral_rng: 0..0,
        }
    }

    /// Change the first anchor, expressed in the local space of the first body part.
    pub fn set_anchor_1(&mut self, anchor1: Point<N>) {
        self.anchor1 = anchor1;
    }

    /// Change the second anchor, expressed in the local space of the second body part.
    pub fn set_anchor_2(&mut self, anchor2: Point<N>) {
        self.anchor2 = anchor2;
    }
}

impl<N: RealField> JointConstraint<N> for BallConstraint<N> {
    fn num_velocity_constraints(&self) -> usize {
        DIM
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

        /*
         *
         * Joint constraints.
         *
         */
        let anchor1 = body1.world_point_at_material_point(part1, &self.anchor1);
        let anchor2 = body2.world_point_at_material_point(part2, &self.anchor2);

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
            &self.impulses,
            0,
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
            self.impulses[c.impulse_id] = c.impulse;
        }

        for c in &constraints.velocity.bilateral[self.bilateral_rng.clone()] {
            self.impulses[c.impulse_id] = c.impulse;
        }
    }
}

impl<N: RealField> NonlinearConstraintGenerator<N> for BallConstraint<N> {
    fn num_position_constraints(&self, bodies: &BodySet<N>) -> usize {
        // FIXME: calling this at each iteration of the non-linear resolution is costly.
        if self.is_active(bodies) {
            1
        } else {
            0
        }
    }

    fn position_constraint(
        &self,
        params: &IntegrationParameters<N>,
        _: usize,
        bodies: &mut BodySet<N>,
        jacobians: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N>> {
        let body1 = bodies.body(self.b1.0)?;
        let body2 = bodies.body(self.b2.0)?;
        let part1 = body1.part(self.b1.1)?;
        let part2 = body2.part(self.b2.1)?;

        let anchor1 = body1.world_point_at_material_point(part1, &self.anchor1);
        let anchor2 = body2.world_point_at_material_point(part2, &self.anchor2);

        helper::cancel_relative_translation(params, body1, part1, body2, part2, &anchor1, &anchor2, jacobians)
    }
}
