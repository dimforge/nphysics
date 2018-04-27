use na::{DVector, Real, Unit};
use std::ops::Range;

use joint::JointConstraint;
use math::{AngularVector, Point, Vector, DIM};
use object::{BodyHandle, BodySet};
use solver::helper;
use solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters,
             NonlinearConstraintGenerator};

pub struct PlanarConstraint<N: Real> {
    b1: BodyHandle,
    b2: BodyHandle,
    anchor1: Point<N>,
    anchor2: Point<N>,
    axis1: Unit<AngularVector<N>>,
    axis2: Unit<AngularVector<N>>,
    lin_impulse: N,
    ang_impulses: [N; 2],
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,
}

impl<N: Real> PlanarConstraint<N> {
    pub fn new(
        b1: BodyHandle,
        b2: BodyHandle,
        anchor1: Point<N>,
        axis1: Unit<AngularVector<N>>,
        anchor2: Point<N>,
        axis2: Unit<AngularVector<N>>,
    ) -> Self {
        PlanarConstraint {
            b1,
            b2,
            anchor1,
            anchor2,
            axis1,
            axis2,
            lin_impulse: N::zero(),
            ang_impulses: [N::zero(), N::zero()],
            bilateral_ground_rng: 0..0,
            bilateral_rng: 0..0,
        }
    }
}

impl<N: Real> JointConstraint<N> for PlanarConstraint<N> {
    fn num_velocity_constraints(&self) -> usize {
        3
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

        /*
         *
         * Joint constraints.
         *
         */
        let pos1 = b1.position();
        let pos2 = b2.position();

        let anchor1 = pos1 * self.anchor1;
        let anchor2 = pos2 * self.anchor2;

        let assembly_id1 = b1.parent_companion_id();
        let assembly_id2 = b2.parent_companion_id();

        let first_bilateral_ground = constraints.velocity.bilateral_ground.len();
        let first_bilateral = constraints.velocity.bilateral.len();

        let axis1 = pos1 * self.axis1;

        helper::cancel_relative_linear_velocity_wrt_axis(
            params,
            &b1,
            &b2,
            assembly_id1,
            assembly_id2,
            &anchor1,
            &anchor2,
            &axis1,
            ext_vels,
            self.lin_impulse,
            0,
            ground_j_id,
            j_id,
            jacobians,
            constraints,
        );

        let axis2 = pos2 * self.axis2;

        helper::restrict_relative_angular_velocity_to_axis(
            params,
            &b1,
            &b2,
            assembly_id1,
            assembly_id2,
            &axis1,
            &axis2,
            &anchor1,
            &anchor2,
            ext_vels,
            &self.ang_impulses[..],
            1,
            ground_j_id,
            j_id,
            jacobians,
            constraints,
        );

        /*
         *
         * Limit constraints.
         *
         */

        self.bilateral_ground_rng =
            first_bilateral_ground..constraints.velocity.bilateral_ground.len();
        self.bilateral_rng = first_bilateral..constraints.velocity.bilateral.len();
    }

    fn cache_impulses(&mut self, constraints: &ConstraintSet<N>) {
        for c in &constraints.velocity.bilateral_ground[self.bilateral_ground_rng.clone()] {
            if c.impulse_id == 0 {
                self.lin_impulse = c.impulse
            } else {
                self.ang_impulses[c.impulse_id - 1] = c.impulse;
            }
        }

        for c in &constraints.velocity.bilateral[self.bilateral_rng.clone()] {
            if c.impulse_id == 0 {
                self.lin_impulse = c.impulse
            } else {
                self.ang_impulses[c.impulse_id - 1] = c.impulse;
            }
        }
    }
}

impl<N: Real> NonlinearConstraintGenerator<N> for PlanarConstraint<N> {
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

        let pos1 = body1.position();
        let pos2 = body2.position();

        let anchor1 = pos1 * self.anchor1;
        let anchor2 = pos2 * self.anchor2;

        let axis1 = pos1 * self.axis1;

        if i == 0 {
            return helper::cancel_relative_translation_wrt_axis(
                params,
                &body1,
                &body2,
                &anchor1,
                &anchor2,
                &axis1,
                jacobians,
            );
        }

        if i == 1 {
            let axis2 = pos2 * self.axis2;

            return helper::align_axis(
                params,
                &body1,
                &body2,
                &anchor1,
                &anchor2,
                &axis1,
                &axis2,
                jacobians,
            );
        }

        return None;
    }
}
