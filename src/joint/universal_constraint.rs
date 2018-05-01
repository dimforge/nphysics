use na::{DVector, Real, Unit};
use std::ops::Range;

use joint::JointConstraint;
use math::{AngularVector, Point, Vector, DIM};
use object::{BodyHandle, BodySet};
use solver::helper;
use solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters,
             NonlinearConstraintGenerator};

pub struct UniversalConstraint<N: Real> {
    b1: BodyHandle,
    b2: BodyHandle,
    anchor1: Point<N>,
    anchor2: Point<N>,
    axis1: Unit<AngularVector<N>>,
    axis2: Unit<AngularVector<N>>,
    angle: N,
    lin_impulses: Vector<N>,
    ang_impulse: N,
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,
}

impl<N: Real> UniversalConstraint<N> {
    pub fn new(
        b1: BodyHandle,
        b2: BodyHandle,
        anchor1: Point<N>,
        axis1: Unit<AngularVector<N>>,
        anchor2: Point<N>,
        axis2: Unit<AngularVector<N>>,
        angle: N,
    ) -> Self {
        UniversalConstraint {
            b1,
            b2,
            anchor1,
            anchor2,
            axis1,
            axis2,
            angle,
            lin_impulses: Vector::zeros(),
            ang_impulse: N::zero(),
            bilateral_ground_rng: 0..0,
            bilateral_rng: 0..0,
        }
    }
}

impl<N: Real> JointConstraint<N> for UniversalConstraint<N> {
    fn num_velocity_constraints(&self) -> usize {
        4
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

        let axis1 = pos1 * self.axis1;
        let axis2 = pos2 * self.axis2;
        if let Some(orth) = Unit::try_new(axis1.cross(&*axis2), N::default_epsilon()) {
            helper::cancel_relative_angular_velocity_wrt_axis(
                &b1,
                &b2,
                assembly_id1,
                assembly_id2,
                &anchor1,
                &anchor2,
                &orth,
                ext_vels,
                self.ang_impulse,
                DIM,
                ground_j_id,
                j_id,
                jacobians,
                constraints,
            );
        }

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
            if c.impulse_id < DIM {
                self.lin_impulses[c.impulse_id] = c.impulse;
            } else {
                self.ang_impulse = c.impulse;
            }
        }

        for c in &constraints.velocity.bilateral[self.bilateral_rng.clone()] {
            if c.impulse_id < DIM {
                self.lin_impulses[c.impulse_id] = c.impulse;
            } else {
                self.ang_impulse = c.impulse;
            }
        }
    }
}

impl<N: Real> NonlinearConstraintGenerator<N> for UniversalConstraint<N> {
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

        if i == 0 {
            return helper::cancel_relative_translation(
                params,
                &body1,
                &body2,
                &anchor1,
                &anchor2,
                jacobians,
            );
        }

        if i == 1 {
            let axis1 = pos1 * self.axis1;
            let axis2 = pos2 * self.axis2;

            return helper::restore_angle_between_axis(
                params,
                &body1,
                &body2,
                &anchor1,
                &anchor2,
                &axis1,
                &axis2,
                self.angle,
                jacobians,
            );
        }

        return None;
    }
}
