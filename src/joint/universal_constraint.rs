use na::{DVector, RealField, Unit};
use std::ops::Range;

use crate::joint::JointConstraint;
use crate::math::{AngularVector, Point, Vector, DIM};
use crate::object::{BodyPartHandle, BodySet};
use crate::solver::helper;
use crate::solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters,
             NonlinearConstraintGenerator};

/// A constraint that removes all but two relative rotations along distinct axii.
pub struct UniversalConstraint<N: RealField> {
    b1: BodyPartHandle,
    b2: BodyPartHandle,
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

impl<N: RealField> UniversalConstraint<N> {
    /// Create a new universal constraint that ensure the angle between `axis1` and `axis2` is always equal to `angle`.
    ///
    /// All anchors and axii are expressed in the local coordinate systems of the corresponding body parts.
    pub fn new(
        b1: BodyPartHandle,
        b2: BodyPartHandle,
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

impl<N: RealField> JointConstraint<N> for UniversalConstraint<N> {
    fn num_velocity_constraints(&self) -> usize {
        4
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

        let axis1 = pos1 * self.axis1;
        let axis2 = pos2 * self.axis2;
        if let Some(orth) = Unit::try_new(axis1.cross(&*axis2), N::default_epsilon()) {
            helper::cancel_relative_angular_velocity_wrt_axis(
                body1,
                part1,
                body2,
                part2,
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

impl<N: RealField> NonlinearConstraintGenerator<N> for UniversalConstraint<N> {
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

        if i == 0 {
            return helper::cancel_relative_translation(
                params,
                body1,
                part1,
                body2,
                part2,
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
                body1,
                part1,
                body2,
                part2,
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
