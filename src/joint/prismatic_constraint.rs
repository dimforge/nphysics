use na::{DVector, RealField, Unit};
use std::ops::Range;

use crate::joint::{unit_constraint, JointConstraint};
use crate::math::{AngularVector, Point, Vector, DIM, SPATIAL_DIM};
use crate::object::{BodyPartHandle, BodySet};
use crate::solver::helper;
use crate::solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters,
             NonlinearConstraintGenerator};

/// A constraint that remove all be one translational degrees of freedom.
pub struct PrismaticConstraint<N: RealField> {
    b1: BodyPartHandle,
    b2: BodyPartHandle,
    anchor1: Point<N>,
    anchor2: Point<N>,
    axis1: Unit<Vector<N>>,
    lin_impulses: Vector<N>,
    ang_impulses: AngularVector<N>,
    limit_impulse: N,
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,

    min_offset: Option<N>,
    max_offset: Option<N>,
}

impl<N: RealField> PrismaticConstraint<N> {
    /// Create a new prismatic constraint that ensures the relative motion between the two
    /// body parts are restricted to a single translation along the `axis1` axis (expressed in
    /// the local coordinates frame of `b1`).
    pub fn new(
        b1: BodyPartHandle,
        b2: BodyPartHandle,
        anchor1: Point<N>,
        axis1: Unit<Vector<N>>,
        anchor2: Point<N>,
    ) -> Self {
        let min_offset = None;
        let max_offset = None;

        PrismaticConstraint {
            b1,
            b2,
            anchor1,
            anchor2,
            axis1,
            lin_impulses: Vector::zeros(),
            ang_impulses: AngularVector::zeros(),
            limit_impulse: N::zero(),
            bilateral_ground_rng: 0..0,
            bilateral_rng: 0..0,
            min_offset,
            max_offset,
        }
    }

    /// The lower limit, if any, of the relative translation (along the joint axis) of the body parts attached to this joint.
    pub fn min_offset(&self) -> Option<N> {
        self.min_offset
    }

    /// The upper limit, if any, of the relative translation (along the joint axis) of the body parts attached to this joint.
    pub fn max_offset(&self) -> Option<N> {
        self.max_offset
    }

    /// Disable the lower limit of the relative translational motion along the joint axis.
    pub fn disable_min_offset(&mut self) {
        self.min_offset = None;
    }

    /// Disable the upper limit of the relative translational motion along the joint axis.
    pub fn disable_max_offset(&mut self) {
        self.max_offset = None;
    }

    /// Enables the lower limit of the relative translational motion along the joint axis.
    pub fn enable_min_offset(&mut self, limit: N) {
        self.min_offset = Some(limit);
        self.assert_limits();
    }

    /// Disable the lower limit of the relative translational motion along the joint axis.
    pub fn enable_max_offset(&mut self, limit: N) {
        self.max_offset = Some(limit);
        self.assert_limits();
    }

    fn assert_limits(&self) {
        if let (Some(min_offset), Some(max_offset)) = (self.min_offset, self.max_offset) {
            assert!(
                min_offset <= max_offset,
                "RevoluteJoint constraint limits: the min angle must be larger than (or equal to) the max angle.");
        }
    }
}

impl<N: RealField> JointConstraint<N> for PrismaticConstraint<N> {
    fn num_velocity_constraints(&self) -> usize {
        (SPATIAL_DIM - 1) + 2
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

        let axis = pos1 * self.axis1;

        helper::restrict_relative_linear_velocity_to_axis(
            body1,
            part1,
            body2,
            part2,
            assembly_id1,
            assembly_id2,
            &anchor1,
            &anchor2,
            &axis,
            ext_vels,
            self.lin_impulses.as_slice(),
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
            DIM - 1,
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
        unit_constraint::build_linear_limits_velocity_constraint(
            body1,
            part1,
            body2,
            part2,
            assembly_id1,
            assembly_id2,
            &anchor1,
            &anchor2,
            &axis,
            self.min_offset,
            self.max_offset,
            ext_vels,
            self.limit_impulse,
            SPATIAL_DIM - 1,
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
            if c.impulse_id < DIM - 1 {
                self.lin_impulses[c.impulse_id] = c.impulse;
            } else if c.impulse_id < SPATIAL_DIM - 1 {
                self.ang_impulses[c.impulse_id - DIM + 1] = c.impulse;
            } else {
                self.limit_impulse = c.impulse
            }
        }

        for c in &constraints.velocity.bilateral[self.bilateral_rng.clone()] {
            if c.impulse_id < DIM - 1 {
                self.lin_impulses[c.impulse_id] = c.impulse;
            } else if c.impulse_id < SPATIAL_DIM - 1 {
                self.ang_impulses[c.impulse_id - DIM + 1] = c.impulse;
            } else {
                self.limit_impulse = c.impulse
            }
        }
    }
}

impl<N: RealField> NonlinearConstraintGenerator<N> for PrismaticConstraint<N> {
    fn num_position_constraints(&self, bodies: &BodySet<N>) -> usize {
        // FIXME: calling this at each iteration of the non-linear resolution is costly.
        if self.is_active(bodies) {
            if self.min_offset.is_some() || self.max_offset.is_some() {
                3
            } else {
                2
            }
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
            return helper::cancel_relative_rotation(
                params,
                body1,
                part1,
                body2,
                part2,
                &anchor1,
                &anchor2,
                &pos1.rotation,
                &pos2.rotation,
                jacobians,
            );
        } else if i == 1 {
            let axis = pos1 * self.axis1;

            return helper::project_anchor_to_axis(
                params,
                body1,
                part1,
                body2,
                part2,
                &anchor1,
                &anchor2,
                &axis,
                jacobians,
            );
        } else if i == 2 {
            let axis = pos1 * self.axis1;

            return unit_constraint::build_linear_limits_position_constraint(
                params,
                body1,
                part1,
                body2,
                part2,
                &anchor1,
                &anchor2,
                &axis,
                self.min_offset,
                self.max_offset,
                jacobians,
            );
        }

        return None;
    }
}
