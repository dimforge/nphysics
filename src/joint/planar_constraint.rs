use na::{DVector, RealField, Unit};
use std::ops::Range;

use crate::joint::JointConstraint;
use crate::math::{AngularVector, Point};
use crate::object::{BodyPartHandle, BodySet};
use crate::solver::helper;
use crate::solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters,
             NonlinearConstraintGenerator};

/// A constraint that removes one relative translational degree of freedom, and all but one rotational degrees of freedom.
///
/// This ensures a body moves only on a plane wrt. its parent.
pub struct PlanarConstraint<N: RealField> {
    b1: BodyPartHandle,
    b2: BodyPartHandle,
    anchor1: Point<N>,
    anchor2: Point<N>,
    axis1: Unit<AngularVector<N>>,
    axis2: Unit<AngularVector<N>>,
    lin_impulse: N,
    ang_impulses: [N; 2],
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,
}

impl<N: RealField> PlanarConstraint<N> {
    /// Create a new planar constraint which ensures the two provided axii always coincide.
    ///
    /// All anchros and axii are expressed in their corresponding body part local coordinate frame.
    pub fn new(
        b1: BodyPartHandle,
        b2: BodyPartHandle,
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

impl<N: RealField> JointConstraint<N> for PlanarConstraint<N> {
    fn num_velocity_constraints(&self) -> usize {
        3
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
        let pos1 = body1.position_at_material_point(part1, &self.anchor1);
        let pos2 = body2.position_at_material_point(part2, &self.anchor2);

        let anchor1 = Point::from(pos1.translation.vector);
        let anchor2 = Point::from(pos2.translation.vector);

        let assembly_id1 = body1.companion_id();
        let assembly_id2 = body2.companion_id();

        let first_bilateral_ground = constraints.velocity.bilateral_ground.len();
        let first_bilateral = constraints.velocity.bilateral.len();

        let axis1 = pos1 * self.axis1;

        helper::cancel_relative_linear_velocity_wrt_axis(
            body1,
            part1,
            body2,
            part2,
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

        helper::restrict_relative_angular_velocity_to_axis(
            body1,
            part1,
            body2,
            part2,
            assembly_id1,
            assembly_id2,
            &axis1,
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

impl<N: RealField> NonlinearConstraintGenerator<N> for PlanarConstraint<N> {
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

        let pos1 = body1.position_at_material_point(part1, &self.anchor1);
        let pos2 = body2.position_at_material_point(part2, &self.anchor2);

        let anchor1 = Point::from(pos1.translation.vector);
        let anchor2 = Point::from(pos2.translation.vector);

        let axis1 = pos1 * self.axis1;

        if i == 0 {
            return helper::cancel_relative_translation_wrt_axis(
                params,
                body1,
                part1,
                body2,
                part2,
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
                body1,
                part1,
                body2,
                part2,
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
