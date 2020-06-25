use na::{DVector, RealField, Unit};
use std::ops::Range;

use crate::joint::JointConstraint;
use crate::math::{AngularVector, Point};
use crate::object::{BodyHandle, BodyPartHandle, BodySet};
use crate::solver::helper;
use crate::solver::{
    GenericNonlinearConstraint, IntegrationParameters, LinearConstraints,
    NonlinearConstraintGenerator,
};

/// A constraint that removes one relative translational degree of freedom, and all but one rotational degrees of freedom.
///
/// This ensures a body moves only on a plane wrt. its parent.
pub struct PlanarConstraint<N: RealField, Handle: BodyHandle> {
    b1: BodyPartHandle<Handle>,
    b2: BodyPartHandle<Handle>,
    anchor1: Point<N>,
    anchor2: Point<N>,
    axis1: Unit<AngularVector<N>>,
    axis2: Unit<AngularVector<N>>,
    lin_impulse: N,
    ang_impulses: [N; 2],
    break_torque_squared: N,
    break_force_squared: N,
    broken: bool,
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,
}

impl<N: RealField, Handle: BodyHandle> PlanarConstraint<N, Handle> {
    /// Create a new planar constraint which ensures the two provided axii always coincide.
    ///
    /// All anchros and axii are expressed in their corresponding body part local coordinate frame.
    pub fn new(
        b1: BodyPartHandle<Handle>,
        b2: BodyPartHandle<Handle>,
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
            break_force_squared: N::max_value(),
            break_torque_squared: N::max_value(),
            broken: false,
            bilateral_ground_rng: 0..0,
            bilateral_rng: 0..0,
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
}

impl<N: RealField, Handle: BodyHandle> JointConstraint<N, Handle> for PlanarConstraint<N, Handle> {
    fn is_broken(&self) -> bool {
        self.broken
    }

    fn num_velocity_constraints(&self) -> usize {
        3
    }

    fn anchors(&self) -> (BodyPartHandle<Handle>, BodyPartHandle<Handle>) {
        (self.b1, self.b2)
    }

    fn velocity_constraints(
        &mut self,
        _: &IntegrationParameters<N>,
        bodies: &dyn BodySet<N, Handle = Handle>,
        ext_vels: &DVector<N>,
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut LinearConstraints<N, usize>,
    ) {
        let body1 = try_ret!(bodies.get(self.b1.0));
        let body2 = try_ret!(bodies.get(self.b2.0));
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

        let first_bilateral_ground = constraints.bilateral_ground.len();
        let first_bilateral = constraints.bilateral.len();

        let axis1 = pos1 * self.axis1;

        helper::cancel_relative_linear_velocity_wrt_axis(
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
            self.b1,
            body2,
            part2,
            self.b2,
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

        self.bilateral_ground_rng = first_bilateral_ground..constraints.bilateral_ground.len();
        self.bilateral_rng = first_bilateral..constraints.bilateral.len();
    }

    fn cache_impulses(&mut self, constraints: &LinearConstraints<N, usize>, inv_dt: N) {
        for c in &constraints.bilateral_ground[self.bilateral_ground_rng.clone()] {
            if c.impulse_id == 0 {
                self.lin_impulse = c.impulse
            } else {
                self.ang_impulses[c.impulse_id - 1] = c.impulse;
            }
        }

        for c in &constraints.bilateral[self.bilateral_rng.clone()] {
            if c.impulse_id == 0 {
                self.lin_impulse = c.impulse
            } else {
                self.ang_impulses[c.impulse_id - 1] = c.impulse;
            }
        }

        let inv_dt2 = inv_dt * inv_dt;

        if self.lin_impulse * self.lin_impulse * inv_dt2 > self.break_force_squared
            || self.ang_impulses[0] * self.ang_impulses[0] * inv_dt2
                + self.ang_impulses[1] * self.ang_impulses[1] * inv_dt2
                > self.break_torque_squared
        {
            self.broken = true;
        }
    }
}

impl<N: RealField, Handle: BodyHandle> NonlinearConstraintGenerator<N, Handle>
    for PlanarConstraint<N, Handle>
{
    fn num_position_constraints(&self, bodies: &dyn BodySet<N, Handle = Handle>) -> usize {
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
        bodies: &mut dyn BodySet<N, Handle = Handle>,
        jacobians: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N, Handle>> {
        let body1 = bodies.get(self.b1.0)?;
        let body2 = bodies.get(self.b2.0)?;
        let part1 = body1.part(self.b1.1)?;
        let part2 = body2.part(self.b2.1)?;

        let pos1 = body1.position_at_material_point(part1, &self.anchor1);
        let pos2 = body2.position_at_material_point(part2, &self.anchor2);

        let anchor1 = Point::from(pos1.translation.vector);
        let anchor2 = Point::from(pos2.translation.vector);

        let axis1 = pos1 * self.axis1;

        if i == 0 {
            return helper::cancel_relative_translation_wrt_axis(
                parameters, body1, part1, self.b1, body2, part2, self.b2, &anchor1, &anchor2,
                &axis1, jacobians,
            );
        }

        if i == 1 {
            let axis2 = pos2 * self.axis2;

            return helper::align_axis(
                parameters, body1, part1, self.b1, body2, part2, self.b2, &anchor1, &anchor2,
                &axis1, &axis2, jacobians,
            );
        }

        None
    }
}
