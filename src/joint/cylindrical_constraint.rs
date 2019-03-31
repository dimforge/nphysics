use std::ops::Range;
use na::{DVector, RealField, Unit};

use crate::object::{BodyPartHandle, BodySet};
use crate::solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters,
             NonlinearConstraintGenerator};
use crate::solver::helper;
use crate::joint::JointConstraint;
use crate::math::{AngularVector, Point, Vector, DIM, SPATIAL_DIM};

/// A constraint that removes all degrees of freedom (of one body part relative to a second one) except one translation along an axis and one rotation along the same axis.
pub struct CylindricalConstraint<N: RealField> {
    b1: BodyPartHandle,
    b2: BodyPartHandle,
    anchor1: Point<N>,
    anchor2: Point<N>,
    axis1: Unit<Vector<N>>,
    axis2: Unit<Vector<N>>,
    lin_impulses: Vector<N>,
    ang_impulses: AngularVector<N>,
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,

    // min_offset: Option<N>,
    // max_offset: Option<N>,
}

impl<N: RealField> CylindricalConstraint<N> {
    /// Creates a cartesian constraint between two body parts.
    /// 
    /// This will ensure `axis1` and `axis2` always coincide. All the axis and anchors
    /// are provided on the local space of the corresponding body parts.
    pub fn new(
        b1: BodyPartHandle,
        b2: BodyPartHandle,
        anchor1: Point<N>,
        axis1: Unit<Vector<N>>,
        anchor2: Point<N>,
        axis2: Unit<Vector<N>>,
    ) -> Self {
        // let min_offset = None;
        // let max_offset = None;

        CylindricalConstraint {
            b1,
            b2,
            anchor1,
            anchor2,
            axis1,
            axis2,
            lin_impulses: Vector::zeros(),
            ang_impulses: AngularVector::zeros(),
            bilateral_ground_rng: 0..0,
            bilateral_rng: 0..0,
            // min_offset,
            // max_offset,
        }
    }

    // pub fn min_offset(&self) -> Option<N> {
    //     self.min_offset
    // }

    // pub fn max_offset(&self) -> Option<N> {
    //     self.max_offset
    // }

    // pub fn disable_min_offset(&mut self) {
    //     self.min_offset = None;
    // }

    // pub fn disable_max_offset(&mut self) {
    //     self.max_offset = None;
    // }

    // pub fn enable_min_offset(&mut self, limit: N) {
    //     self.min_offset = Some(limit);
    //     self.assert_limits();
    // }

    // pub fn enable_max_offset(&mut self, limit: N) {
    //     self.max_offset = Some(limit);
    //     self.assert_limits();
    // }

    // fn assert_limits(&self) {
    //     if let (Some(min_offset), Some(max_offset)) = (self.min_offset, self.max_offset) {
    //         assert!(
    //             min_offset <= max_offset,
    //             "Cylindrical constraint limits: the min angle must be larger than (or equal to) the max angle.");
    //     }
    // }
}

impl<N: RealField> JointConstraint<N> for CylindricalConstraint<N> {
    fn num_velocity_constraints(&self) -> usize {
        SPATIAL_DIM - 2
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

        helper::restrict_relative_linear_velocity_to_axis(
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
            self.lin_impulses.as_slice(),
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
            self.ang_impulses.as_slice(),
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

impl<N: RealField> NonlinearConstraintGenerator<N> for CylindricalConstraint<N> {
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
        let axis2 = pos2 * self.axis2;

        if i == 0 {
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

        if i == 1 {
            return helper::project_anchor_to_axis(
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

        return None;
    }
}
