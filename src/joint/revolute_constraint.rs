#[cfg(feature = "dim3")]
use na::Unit;
use na::{DVector, RealField};
use std::ops::Range;

use crate::joint::JointConstraint;
use crate::math::{AngularVector, Point, Vector, DIM, SPATIAL_DIM};
use crate::object::{BodyPartHandle, BodySet, Body, BodyHandle};
use crate::solver::helper;
use crate::solver::{LinearConstraints, GenericNonlinearConstraint, IntegrationParameters,
             NonlinearConstraintGenerator};

/// A constraint that removes all relative motions except the rotation between two body parts.
#[cfg(feature = "dim2")]
pub struct RevoluteConstraint<N: RealField, Handle: BodyHandle> {
    b1: BodyPartHandle<Handle>,
    b2: BodyPartHandle<Handle>,
    anchor1: Point<N>,
    anchor2: Point<N>,
    lin_impulses: Vector<N>,
    ang_impulses: AngularVector<N>,
    break_force_squared: N,
    break_torque_squared: N,
    broken: bool,
    // FIXME: not actually needed in 2D.
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,
    // min_angle: Option<N>,
    // max_angle: Option<N>,
}

/// A constraint that removes all relative motions except one rotation between two body parts.
#[cfg(feature = "dim3")]
pub struct RevoluteConstraint<N: RealField, Handle: BodyHandle> {
    b1: BodyPartHandle<Handle>,
    b2: BodyPartHandle<Handle>,
    anchor1: Point<N>,
    anchor2: Point<N>,
    axis1: Unit<AngularVector<N>>,
    axis2: Unit<AngularVector<N>>,
    lin_impulses: Vector<N>,
    ang_impulses: AngularVector<N>,
    break_force_squared: N,
    break_torque_squared: N,
    broken: bool,
    bilateral_ground_rng: Range<usize>,
    bilateral_rng: Range<usize>,
    // min_angle: Option<N>,
    // max_angle: Option<N>,
}

impl<N: RealField, Handle: BodyHandle> RevoluteConstraint<N, Handle> {
    /// Create a new revolute constraint which ensures the provided axii and anchors always coincide.
    ///
    /// All axii and achors are expressed in the local coordinate system of the corresponding body parts.
    #[cfg(feature = "dim3")]
    pub fn new(
        b1: BodyPartHandle<Handle>,
        b2: BodyPartHandle<Handle>,
        anchor1: Point<N>,
        axis1: Unit<AngularVector<N>>,
        anchor2: Point<N>,
        axis2: Unit<AngularVector<N>>,
    ) -> Self {
        // let min_angle = None;
        // let max_angle = None;
        RevoluteConstraint {
            b1,
            b2,
            anchor1,
            anchor2,
            axis1,
            axis2,
            lin_impulses: Vector::zeros(),
            ang_impulses: AngularVector::zeros(),
            break_force_squared: N::max_value(),
            break_torque_squared: N::max_value(),
            broken: false,
            bilateral_ground_rng: 0..0,
            bilateral_rng: 0..0,
            // min_angle,
            // max_angle,
        }
    }

    /// Create a new revolute constraint which ensures the provided anchors always coincide.
    ///
    /// Both achors are expressed in the local coordinate system of the corresponding body parts.
    #[cfg(feature = "dim2")]
    pub fn new(b1: BodyPartHandle<Handle>, b2: BodyPartHandle<Handle>, anchor1: Point<N>, anchor2: Point<N>) -> Self {
        // let min_angle = None;
        // let max_angle = None;

        RevoluteConstraint {
            b1,
            b2,
            anchor1,
            anchor2,
            lin_impulses: Vector::zeros(),
            ang_impulses: AngularVector::zeros(),
            break_force_squared: N::max_value(),
            break_torque_squared: N::max_value(),
            broken: false,
            bilateral_ground_rng: 0..0,
            bilateral_rng: 0..0,
            // min_angle,
            // max_angle,
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

    // pub fn min_angle(&self) -> Option<N> {
    //     self.min_angle
    // }

    // pub fn max_angle(&self) -> Option<N> {
    //     self.max_angle
    // }

    // pub fn disable_min_angle(&mut self) {
    //     self.min_angle = None;
    // }

    // pub fn disable_max_angle(&mut self) {
    //     self.max_angle = None;
    // }

    // pub fn enable_min_angle(&mut self, limit: N) {
    //     self.min_angle = Some(limit);
    //     self.assert_limits();
    // }

    // pub fn enable_max_angle(&mut self, limit: N) {
    //     self.max_angle = Some(limit);
    //     self.assert_limits();
    // }

    // fn assert_limits(&self) {
    //     if let (Some(min_angle), Some(max_angle)) = (self.min_angle, self.max_angle) {
    //         assert!(
    //             min_angle <= max_angle,
    //             "RevoluteJoint constraint limits: the min angle must be larger than (or equal to) the max angle.");
    //     }
    // }
}

impl<N: RealField, Handle: BodyHandle, Bodies: BodySet<N, Handle = Handle>> JointConstraint<N, Bodies> for RevoluteConstraint<N, Handle> {
    fn is_broken(&self) -> bool {
        self.broken
    }

    fn num_velocity_constraints(&self) -> usize {
        SPATIAL_DIM - 1
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

        #[cfg(feature = "dim3")]
            {
                let axis1 = pos1 * self.axis1;

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
                    self.ang_impulses.as_slice(),
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
            first_bilateral_ground..constraints.bilateral_ground.len();
        self.bilateral_rng = first_bilateral..constraints.bilateral.len();
    }

    fn cache_impulses(&mut self, constraints: &LinearConstraints<N, usize>) {
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

        if self.lin_impulses.norm_squared() > self.break_force_squared ||
            self.ang_impulses.norm_squared() > self.break_torque_squared {
            self.broken = true;
        }
    }
}

impl<N: RealField, Handle: BodyHandle, Bodies: BodySet<N, Handle = Handle>> NonlinearConstraintGenerator<N, Bodies> for RevoluteConstraint<N, Handle> {
    fn num_position_constraints(&self, bodies: &Bodies) -> usize {
        // FIXME: calling this at each iteration of the non-linear resolution is costly.
        if self.is_active(bodies) {
            if DIM == 3 {
                2
            } else {
                1
            }
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
    ) -> Option<GenericNonlinearConstraint<N, Handle>> {
        let body1 = bodies.get(self.b1.0)?;
        let body2 = bodies.get(self.b2.0)?;
        let part1 = body1.part(self.b1.1)?;
        let part2 = body2.part(self.b2.1)?;

        let pos1 = body1.position_at_material_point(part1, &self.anchor1);
        let pos2 = body2.position_at_material_point(part2, &self.anchor2);

        let anchor1 = Point::from(pos1.translation.vector);
        let anchor2 = Point::from(pos2.translation.vector);

        if i == 0 {
            return helper::cancel_relative_translation(
                parameters,
                body1,
                part1,
                self.b1,
                body2,
                part2,
                self.b2,
                &anchor1,
                &anchor2,
                jacobians,
            );
        }

        #[cfg(feature = "dim3")]
            {
                if i == 1 {
                    let axis1 = pos1 * self.axis1;
                    let axis2 = pos2 * self.axis2;

                    return helper::align_axis(
                        parameters,
                        body1,
                        part1,
                        self.b1,
                        body2,
                        part2,
                        self.b2,
                        &anchor1,
                        &anchor2,
                        &axis1,
                        &axis2,
                        jacobians,
                    );
                }
            }

        return None;
    }
}
