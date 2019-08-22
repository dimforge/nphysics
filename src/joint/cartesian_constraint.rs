use na::{DVector, RealField};
use std::ops::Range;

use crate::joint::JointConstraint;
use crate::math::{AngularVector, Point, ANGULAR_DIM, Rotation};
use crate::object::{BodyPartHandle, BodySet, Body, BodyHandle};
use crate::solver::helper;
use crate::solver::{LinearConstraints, GenericNonlinearConstraint, IntegrationParameters,
             NonlinearConstraintGenerator};

/// A constraint that removes all relative angular motion between two body parts.
pub struct CartesianConstraint<N: RealField, Handle: BodyHandle> {
    b1: BodyPartHandle<Handle>,
    b2: BodyPartHandle<Handle>,
    anchor1: Point<N>,
    ref_frame1: Rotation<N>,
    anchor2: Point<N>,
    ref_frame2: Rotation<N>,
    ang_impulses: AngularVector<N>,
    break_torque_squared: N,
    broken: bool,
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,
}

impl<N: RealField, Handle: BodyHandle> CartesianConstraint<N, Handle> {
    /// Creates a cartesian constraint between two body parts.
    /// 
    /// This will ensure the rotational parts of the frames given identified by `ref_frame1` and
    /// `ref_frame2` and attached to the corresponding bodies will coincide.
    pub fn new(
        b1: BodyPartHandle<Handle>,
        b2: BodyPartHandle<Handle>,
        anchor1: Point<N>,
        ref_frame1: Rotation<N>,
        anchor2: Point<N>,
        ref_frame2: Rotation<N>,
    ) -> Self {
        CartesianConstraint {
            b1,
            b2,
            anchor1,
            ref_frame1,
            anchor2,
            ref_frame2,
            break_torque_squared: N::max_value(),
            broken: false,
            ang_impulses: AngularVector::zeros(),
            bilateral_ground_rng: 0..0,
            bilateral_rng: 0..0,
        }
    }

    /// Changes the reference frame for the first body part.
    pub fn set_reference_frame_1(&mut self, ref_frame1: Rotation<N>) {
        self.ref_frame1 = ref_frame1
    }

    /// Changes the reference frame for the second body part.
    pub fn set_reference_frame_2(&mut self, frame2: Rotation<N>) {
        self.ref_frame2 = frame2
    }

    /// Changes the attach point for the first body part.
    pub fn set_anchor_1(&mut self, anchor1: Point<N>) {
        self.anchor1 = anchor1
    }

    /// Changes the attach point for the second body part.
    pub fn set_anchor_2(&mut self, anchor2: Point<N>) {
        self.anchor2 = anchor2
    }

    /// The maximum torque this joint can absorb before breaking.
    pub fn set_break_torque(&mut self, break_torque: N) {
        self.break_torque_squared = break_torque * break_torque;
    }
}

impl<N: RealField, Handle: BodyHandle, Bodies: BodySet<N, Handle = Handle>> JointConstraint<N, Bodies> for CartesianConstraint<N, Handle> {
    fn is_broken(&self) -> bool {
        self.broken
    }

    fn num_velocity_constraints(&self) -> usize {
        ANGULAR_DIM
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
            0,
            ground_j_id,
            j_id,
            jacobians,
            constraints,
        );

        self.bilateral_ground_rng =
            first_bilateral_ground..constraints.bilateral_ground.len();
        self.bilateral_rng = first_bilateral..constraints.bilateral.len();
    }

    fn cache_impulses(&mut self, constraints: &LinearConstraints<N, usize>) {
        for c in &constraints.bilateral_ground[self.bilateral_ground_rng.clone()] {
            self.ang_impulses[c.impulse_id] = c.impulse;
        }

        for c in &constraints.bilateral[self.bilateral_rng.clone()] {
            self.ang_impulses[c.impulse_id] = c.impulse;
        }

        if self.ang_impulses.norm_squared() > self.break_torque_squared {
            self.broken = true;
        }
    }
}

impl<N: RealField, Handle: BodyHandle, Bodies: BodySet<N, Handle = Handle>> NonlinearConstraintGenerator<N, Bodies> for CartesianConstraint<N, Handle> {
    fn num_position_constraints(&self, bodies: &Bodies) -> usize {
        // FIXME: calling this at each iteration of the non-linear resolution is costly.
        if self.is_active(bodies) {
            1
        } else {
            0
        }
    }

    fn position_constraint(
        &self,
        parameters: &IntegrationParameters<N>,
        _: usize,
        bodies: &mut Bodies,
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

        let rotation1 = pos1.rotation;
        let rotation2 = pos2.rotation;

        helper::cancel_relative_rotation(
            parameters,
            body1,
            part1,
            self.b1,
            body2,
            part2,
            self.b2,
            &anchor1,
            &anchor2,
            &rotation1,
            &rotation2,
            jacobians,
        )
    }
}
