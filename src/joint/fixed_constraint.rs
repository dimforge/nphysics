use na::{DVector, RealField};
use std::ops::Range;

use crate::joint::JointConstraint;
use crate::math::{AngularVector, Point, Rotation, Vector, DIM, SPATIAL_DIM};
use crate::object::{BodyHandle, BodyPartHandle, BodySet};
use crate::solver::helper;
use crate::solver::{
    GenericNonlinearConstraint, IntegrationParameters, LinearConstraints,
    NonlinearConstraintGenerator,
};

/// A constraint that removes all degrees of freedom between two body parts.
pub struct FixedConstraint<N: RealField, Handle: BodyHandle> {
    b1: BodyPartHandle<Handle>,
    b2: BodyPartHandle<Handle>,
    anchor1: Point<N>,
    ref_frame1: Rotation<N>,
    anchor2: Point<N>,
    ref_frame2: Rotation<N>,
    lin_impulses: Vector<N>,
    ang_impulses: AngularVector<N>,
    break_force_squared: N,
    break_torque_squared: N,
    broken: bool,
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,
}

impl<N: RealField, Handle: BodyHandle> FixedConstraint<N, Handle> {
    /// Create a fixed constraint between two body parts.
    ///
    /// This will ensure the frames `joint_to_b1` and `joint_to_b2` attached to the
    /// body parts `b1` adn `b2` respectively always coincide.
    pub fn new(
        b1: BodyPartHandle<Handle>,
        b2: BodyPartHandle<Handle>,
        anchor1: Point<N>,
        ref_frame1: Rotation<N>,
        anchor2: Point<N>,
        ref_frame2: Rotation<N>,
    ) -> Self {
        FixedConstraint {
            b1,
            b2,
            anchor1,
            ref_frame1,
            anchor2,
            ref_frame2,
            lin_impulses: Vector::zeros(),
            ang_impulses: AngularVector::zeros(),
            break_force_squared: N::max_value(),
            break_torque_squared: N::max_value(),
            broken: false,
            bilateral_ground_rng: 0..0,
            bilateral_rng: 0..0,
        }
    }

    /// Changes the reference frame for the first body part.
    pub fn set_reference_frame_1(&mut self, ref_frame1: Rotation<N>) {
        self.ref_frame1 = ref_frame1
    }

    /// Changes the reference frame for the second body part.
    pub fn set_reference_frame_2(&mut self, ref_frame2: Rotation<N>) {
        self.ref_frame2 = ref_frame2
    }

    /// Changes the attached material point from the first body part.
    pub fn set_anchor_1(&mut self, anchor1: Point<N>) {
        self.anchor1 = anchor1
    }

    /// Changes the attached material point from the second body part.
    pub fn set_anchor_2(&mut self, anchor2: Point<N>) {
        self.anchor2 = anchor2
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

impl<N: RealField, Handle: BodyHandle> JointConstraint<N, Handle> for FixedConstraint<N, Handle> {
    fn is_broken(&self) -> bool {
        self.broken
    }

    fn num_velocity_constraints(&self) -> usize {
        SPATIAL_DIM
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

        let pos1 = body1.position_at_material_point(part1, &self.anchor1) * self.ref_frame1;
        let pos2 = body2.position_at_material_point(part2, &self.anchor2) * self.ref_frame2;

        let anchor1 = Point::from(pos1.translation.vector);
        let anchor2 = Point::from(pos2.translation.vector);

        let assembly_id1 = body1.companion_id();
        let assembly_id2 = body2.companion_id();

        let first_bilateral_ground = constraints.bilateral_ground.len();
        let first_bilateral = constraints.bilateral.len();

        helper::cancel_relative_linear_velocity(
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
            ext_vels,
            &self.lin_impulses,
            0,
            ground_j_id,
            j_id,
            jacobians,
            constraints,
        );

        helper::cancel_relative_angular_velocity(
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
            ext_vels,
            &self.ang_impulses,
            DIM,
            ground_j_id,
            j_id,
            jacobians,
            constraints,
        );

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

impl<N: RealField, Handle: BodyHandle> NonlinearConstraintGenerator<N, Handle>
    for FixedConstraint<N, Handle>
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

        let pos1 = body1.position_at_material_point(part1, &self.anchor1) * self.ref_frame1;
        let pos2 = body2.position_at_material_point(part2, &self.anchor2) * self.ref_frame2;

        let anchor1 = Point::from(pos1.translation.vector);
        let anchor2 = Point::from(pos2.translation.vector);

        if i == 0 {
            let rotation1 = pos1.rotation;
            let rotation2 = pos2.rotation;

            helper::cancel_relative_rotation(
                parameters, body1, part1, self.b1, body2, part2, self.b2, &anchor1, &anchor2,
                &rotation1, &rotation2, jacobians,
            )
        } else if i == 1 {
            helper::cancel_relative_translation(
                parameters, body1, part1, self.b1, body2, part2, self.b2, &anchor1, &anchor2,
                jacobians,
            )
        } else {
            None
        }
    }
}
