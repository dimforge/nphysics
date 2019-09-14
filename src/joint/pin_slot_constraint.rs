use na::{DVector, RealField, Unit};
use std::ops::Range;

use crate::joint::JointConstraint;
use crate::math::{AngularVector, Point, Vector, DIM, SPATIAL_DIM};
use crate::object::{Body, BodyHandle, BodyPartHandle, BodySet};
use crate::solver::helper;
use crate::solver::{
    GenericNonlinearConstraint, IntegrationParameters, LinearConstraints,
    NonlinearConstraintGenerator,
};

/// A constraint that removes two translational and two rotational degrees of freedoms.
///
/// This is different frmo the cylindrical constraint since the remaining rotation and translation
/// are not restricted to be done wrt. the same axis.
pub struct PinSlotConstraint<N: RealField, Handle: BodyHandle> {
    b1: BodyPartHandle<Handle>,
    b2: BodyPartHandle<Handle>,
    anchor1: Point<N>,
    anchor2: Point<N>,
    axis_v1: Unit<Vector<N>>,
    axis_w1: Unit<Vector<N>>,
    axis_w2: Unit<Vector<N>>,
    lin_impulses: Vector<N>,
    ang_impulses: AngularVector<N>,
    break_torque_squared: N,
    break_force_squared: N,
    broken: bool,
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,
    // min_offset: Option<N>,
    // max_offset: Option<N>,
}

impl<N: RealField, Handle: BodyHandle> PinSlotConstraint<N, Handle> {
    /// Creates a new pin-slot constraint.
    ///
    /// This will ensure the relative linear motions are always along `axis_v1` (here expressed
    /// in the local coordinate frame of `b1`), and that `axis_w1` and `axis_w2` always coincide.
    /// All axises and anchors are expressed in the local coordinate frame of their respective body
    /// part.
    pub fn new(
        b1: BodyPartHandle<Handle>,
        b2: BodyPartHandle<Handle>,
        anchor1: Point<N>,
        axis_v1: Unit<Vector<N>>,
        axis_w1: Unit<Vector<N>>,
        anchor2: Point<N>,
        axis_w2: Unit<Vector<N>>,
    ) -> Self
    {
        // let min_offset = None;
        // let max_offset = None;

        PinSlotConstraint {
            b1,
            b2,
            anchor1,
            anchor2,
            axis_v1,
            axis_w1,
            axis_w2,
            lin_impulses: Vector::zeros(),
            ang_impulses: AngularVector::zeros(),
            break_force_squared: N::max_value(),
            break_torque_squared: N::max_value(),
            broken: false,
            bilateral_ground_rng: 0..0,
            bilateral_rng: 0..0,
            // min_offset,
            // max_offset,
        }
    }

    /// The maximum force this joint can absorb before breaking.
    pub fn set_break_force(&mut self, break_force: N) {
        self.break_force_squared = break_force * break_force;
    }

    /// The maximum torque this joint can absorb before breaking.
    pub fn set_break_torque(&mut self, break_torque: N) {
        self.break_torque_squared = break_torque * break_torque;
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
    //             "RevoluteJoint constraint limits: the min angle must be larger than (or equal to) the max angle.");
    //     }
    // }
}

impl<N: RealField, Handle: BodyHandle, Bodies: BodySet<N, Handle = Handle>>
    JointConstraint<N, Bodies> for PinSlotConstraint<N, Handle>
{
    fn is_broken(&self) -> bool {
        self.broken
    }

    fn num_velocity_constraints(&self) -> usize {
        SPATIAL_DIM - 2
    }

    fn anchors(&self) -> (BodyPartHandle<Handle>, BodyPartHandle<Handle>) {
        (self.b1, self.b2)
    }

    fn velocity_constraints(
        &mut self,
        _: &IntegrationParameters<N>,
        bodies: &Bodies,
        ext_vels: &DVector<N>,
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut LinearConstraints<N, usize>,
    )
    {
        let body1 = try_ret!(bodies.get(self.b1.0));
        let part1 = try_ret!(body1.part(self.b1.1));
        let body2 = try_ret!(bodies.get(self.b2.0));
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

        let first_bilateral_ground = constraints.bilateral_ground.len();
        let first_bilateral = constraints.bilateral.len();

        let axis_v1 = pos1 * self.axis_v1;
        let axis_w1 = pos1 * self.axis_w1;

        helper::restrict_relative_linear_velocity_to_axis(
            body1,
            part1,
            self.b1,
            body2,
            part2,
            self.b2,
            assembly_id1,
            assembly_id2,
            &anchor1,
            &anchor2,
            &axis_v1,
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
            self.b1,
            body2,
            part2,
            self.b2,
            assembly_id1,
            assembly_id2,
            &axis_w1,
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

        self.bilateral_ground_rng = first_bilateral_ground..constraints.bilateral_ground.len();
        self.bilateral_rng = first_bilateral..constraints.bilateral.len();
    }

    fn cache_impulses(&mut self, constraints: &LinearConstraints<N, usize>, inv_dt: N) {
        for c in &constraints.bilateral_ground[self.bilateral_ground_rng.clone()] {
            if c.impulse_id < DIM {
                self.lin_impulses[c.impulse_id] = c.impulse;
            } else {
                self.ang_impulses[c.impulse_id - DIM] = c.impulse;
            }
        }

        for c in &constraints.bilateral[self.bilateral_rng.clone()] {
            if c.impulse_id < DIM {
                self.lin_impulses[c.impulse_id] = c.impulse;
            } else {
                self.ang_impulses[c.impulse_id - DIM] = c.impulse;
            }
        }

        let inv_dt2 = inv_dt * inv_dt;

        if self.lin_impulses.norm_squared() * inv_dt2 > self.break_force_squared
            || self.ang_impulses.norm_squared() * inv_dt2 > self.break_torque_squared
        {
            self.broken = true;
        }
    }
}

impl<N: RealField, Handle: BodyHandle, Bodies: BodySet<N, Handle = Handle>>
    NonlinearConstraintGenerator<N, Bodies> for PinSlotConstraint<N, Handle>
{
    fn num_position_constraints(&self, bodies: &Bodies) -> usize {
        // FIXME: calling this at each iteration of the non-linear resolution is costly.
        if self.is_active(bodies) {
            2
        } else {
            0
        }
    }

    fn position_constraint(
        &self,
        parameters: &IntegrationParameters<N>,
        i: usize,
        bodies: &mut Bodies,
        jacobians: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N, Handle>>
    {
        let body1 = bodies.get(self.b1.0)?;
        let body2 = bodies.get(self.b2.0)?;
        let part1 = body1.part(self.b1.1)?;
        let part2 = body2.part(self.b2.1)?;

        let pos1 = part1.position();
        let pos2 = part2.position();

        let anchor1 = pos1 * self.anchor1;
        let anchor2 = pos2 * self.anchor2;

        if i == 0 {
            let axis1 = pos1 * self.axis_w1;
            let axis2 = pos2 * self.axis_w2;

            return helper::align_axis(
                parameters, body1, part1, self.b1, body2, part2, self.b2, &anchor1, &anchor2,
                &axis1, &axis2, jacobians,
            );
        }

        if i == 1 {
            let axis = pos1 * self.axis_v1;

            return helper::project_anchor_to_axis(
                parameters, body1, part1, self.b1, body2, part2, self.b2, &anchor1, &anchor2,
                &axis, jacobians,
            );
        }

        return None;
    }
}
