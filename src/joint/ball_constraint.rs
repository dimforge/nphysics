use std::ops::Range;
use na::{DVector, Real};

use object::{BodyHandle, BodySet};
use solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters,
             NonlinearConstraintGenerator};
use solver::helper;
use joint::JointConstraint;
use math::{Point, Vector, DIM};

pub struct BallConstraint<N: Real> {
    b1: BodyHandle,
    b2: BodyHandle,
    anchor1: Point<N>,
    anchor2: Point<N>,
    impulses: Vector<N>,
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,
}

impl<N: Real> BallConstraint<N> {
    pub fn new(b1: BodyHandle, b2: BodyHandle, anchor1: Point<N>, anchor2: Point<N>) -> Self {
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

    pub fn set_anchor_1(&mut self, anchor1: Point<N>) {
        self.anchor1 = anchor1;
    }

    pub fn set_anchor_2(&mut self, anchor2: Point<N>) {
        self.anchor2 = anchor2;
    }
}

impl<N: Real> JointConstraint<N> for BallConstraint<N> {
    fn num_velocity_constraints(&self) -> usize {
        DIM
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
        let body1 = bodies.body_part(self.b1);
        let body2 = bodies.body_part(self.b2);

        /*
         *
         * Joint constraints.
         *
         */
        let pos1 = body1.position();
        let pos2 = body2.position();

        let anchor1 = pos1 * self.anchor1;
        let anchor2 = pos2 * self.anchor2;

        let assembly_id1 = body1.parent_companion_id();
        let assembly_id2 = body2.parent_companion_id();

        let first_bilateral_ground = constraints.velocity.bilateral_ground.len();
        let first_bilateral = constraints.velocity.bilateral.len();

        helper::cancel_relative_linear_velocity(
            params,
            &body1,
            &body2,
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

impl<N: Real> NonlinearConstraintGenerator<N> for BallConstraint<N> {
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
        let body1 = bodies.body_part(self.b1);
        let body2 = bodies.body_part(self.b2);

        let pos1 = body1.position();
        let pos2 = body2.position();

        let anchor1 = pos1 * self.anchor1;
        let anchor2 = pos2 * self.anchor2;

        helper::cancel_relative_translation(params, &body1, &body2, &anchor1, &anchor2, jacobians)
    }
}
