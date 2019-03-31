//! Utilities for computing velocity and position constraints.

use alga::linear::{FiniteDimInnerSpace, FiniteDimVectorSpace};
#[cfg(feature = "dim3")]
use na;
use na::{DVector, DVectorSlice, RealField, Unit};
use std::ops::Neg;

use crate::math::{AngularVector, Force, Point, Rotation, Vector};
use crate::object::{Body, BodyPart};
use crate::solver::{BilateralConstraint, BilateralGroundConstraint, ConstraintGeometry, ConstraintSet,
             GenericNonlinearConstraint, ImpulseLimits, IntegrationParameters};

/// The direction of a force in world-space.
#[derive(Copy, Clone, Debug)]
pub enum ForceDirection<N: RealField> {
    /// A linear force toward a direction.
    Linear(Unit<Vector<N>>),
    /// A torque wrt. an axis.
    Angular(Unit<AngularVector<N>>),
}

impl<N: RealField> ForceDirection<N> {
    /// The force (at the specified point) resulting from this unit force applied at the specified point.
    #[inline]
    pub fn at_point(&self, pt: &Point<N>) -> Force<N> {
        match self {
            ForceDirection::Linear(normal) => {
                Force::linear_at_point(**normal, pt)
            }
            ForceDirection::Angular(axis) => {
                Force::torque_from_vector(**axis)
            }
        }
    }
}

impl<N: RealField> Neg for ForceDirection<N> {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        match self {
            ForceDirection::Linear(n) => ForceDirection::Linear(-n),
            ForceDirection::Angular(a) => ForceDirection::Angular(-a),
        }
    }
}

/// Fills all the jacobians (and the jacobians multiplied by the invers augmented mass matricxs) for a
/// constraint applying a force at the points `center1, center2` and the direction `dir`.
///
/// If the force is a torque, it is applied at the centers of mass of the body parts.
/// Every input are expressed in world-space.
#[inline]
pub fn constraint_pair_geometry<N: RealField>(
    body1: &Body<N>,
    part1: &BodyPart<N>,
    body2: &Body<N>,
    part2: &BodyPart<N>,
    center1: &Point<N>,
    center2: &Point<N>,
    dir: &ForceDirection<N>,
    ground_j_id: &mut usize,
    j_id: &mut usize,
    jacobians: &mut [N],
    ext_vels1: Option<&DVectorSlice<N>>,
    ext_vels2: Option<&DVectorSlice<N>>,
    mut out_vel: Option<&mut N>
) -> ConstraintGeometry<N> {
    let mut res = ConstraintGeometry::new();

    res.ndofs1 = body1.status_dependent_ndofs();
    res.ndofs2 = body2.status_dependent_ndofs();

    let out_j_id;
    if res.ndofs1 == 0 || res.ndofs2 == 0 {
        res.j_id1 = *ground_j_id;
        out_j_id = ground_j_id;
    } else {
        res.j_id1 = *j_id;
        out_j_id = j_id;
    }

    res.j_id2 = res.j_id1 + res.ndofs1;
    res.wj_id1 = res.j_id2 + res.ndofs2;
    res.wj_id2 = res.wj_id1 + res.ndofs1;

    let mut inv_r = N::zero();

    body1.fill_constraint_geometry(
        part1,
        res.ndofs1,
        center1,
        dir,
        res.j_id1,
        res.wj_id1,
        jacobians,
        &mut inv_r,
        ext_vels1,
        // Unfortunate pattern we have to do to avoid borrowing issues.
        // See https://internals.rust-lang.org/t/should-option-mut-t-implement-copy/3715/6
        out_vel.as_mut().map(|v| &mut **v)
    );

    body2.fill_constraint_geometry(
        part2,
        res.ndofs2,
        center2,
        &dir.neg(),
        res.j_id2,
        res.wj_id2,
        jacobians,
        &mut inv_r,
        ext_vels2,
        out_vel
    );

    if body1.handle() == body2.handle() {
        let j1 = DVectorSlice::from_slice(&jacobians[res.j_id1..], res.ndofs1);
        let j2 = DVectorSlice::from_slice(&jacobians[res.j_id2..], res.ndofs2);
        let invm_j1 = DVectorSlice::from_slice(&jacobians[res.wj_id1..], res.ndofs1);
        let invm_j2 = DVectorSlice::from_slice(&jacobians[res.wj_id2..], res.ndofs2);

        inv_r += j2.dot(&invm_j1) + j1.dot(&invm_j2);
    }

    if !inv_r.is_zero() {
        res.r = N::one() / inv_r;
    } else {
        res.r = N::one()
    }

    *out_j_id += (res.ndofs1 + res.ndofs2) * 2;
    res
}

/// Test if a constraint between the two given bodies should be a ground
/// constraint (a constraint between a dynamic body and one without any degree of freedom).
#[inline]
pub fn constraints_are_ground_constraints<N: RealField>(
    body1: &Body<N>,
    body2: &Body<N>,
) -> bool {
    body1.status_dependent_ndofs() == 0 || body2.status_dependent_ndofs() == 0
}

/// Retrieve the external velocity subvectors for the given bodies.
#[inline(always)]
pub fn split_ext_vels<'a, N: RealField>(
    body1: &Body<N>,
    body2: &Body<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    ext_vels: &'a DVector<N>)
    -> (DVectorSlice<'a, N>, DVectorSlice<'a, N>){
    let ndofs1 = body1.status_dependent_ndofs();
    let ndofs2 = body2.status_dependent_ndofs();

    (ext_vels.rows(assembly_id1, ndofs1), ext_vels.rows(assembly_id2, ndofs2))
}

/// Generates velocity constraints to cancel the relative linear velocity of two body parts wrt the given axis.
///
/// All inputs mut be given in world-space.
pub fn cancel_relative_linear_velocity_wrt_axis<N: RealField>(
    body1: &Body<N>,
    part1: &BodyPart<N>,
    body2: &Body<N>,
    part2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    axis: &Unit<Vector<N>>,
    ext_vels: &DVector<N>,
    impulse: N,
    impulse_id: usize,
    ground_j_id: &mut usize,
    j_id: &mut usize,
    jacobians: &mut [N],
    constraints: &mut ConstraintSet<N>,
) {
    let limits = ImpulseLimits::Independent {
        min: -N::max_value(),
        max: N::max_value(),
    };

    let force = ForceDirection::Linear(*axis);
    let mut rhs = N::zero();
    let (ext_vels1, ext_vels2) = split_ext_vels(body1, body2, assembly_id1, assembly_id2, ext_vels);

    let geom = constraint_pair_geometry(
        body1,
        part1,
        body2,
        part2,
        anchor1,
        anchor2,
        &force,
        ground_j_id,
        j_id,
        jacobians,
        Some(&ext_vels1),
        Some(&ext_vels2),
        Some(&mut rhs)
    );

    if geom.ndofs1 == 0 || geom.ndofs2 == 0 {
        constraints
            .velocity
            .bilateral_ground
            .push(BilateralGroundConstraint::new(
                geom,
                assembly_id1,
                assembly_id2,
                limits,
                rhs,
                impulse,
                impulse_id,
            ));
    } else {
        constraints
            .velocity
            .bilateral
            .push(BilateralConstraint::new(
                geom,
                assembly_id1,
                assembly_id2,
                limits,
                rhs,
                impulse,
                impulse_id,
            ));
    }
}

/// Generates velocity constraints to cancel the relative linear velocity of two body parts.
///
/// All inputs mut be given in world-space.
pub fn cancel_relative_linear_velocity<N: RealField>(
    body1: &Body<N>,
    part1: &BodyPart<N>,
    body2: &Body<N>,
    part2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    ext_vels: &DVector<N>,
    impulses: &Vector<N>,
    impulse_id: usize,
    ground_j_id: &mut usize,
    j_id: &mut usize,
    jacobians: &mut [N],
    constraints: &mut ConstraintSet<N>,
) {
    let mut i = 0;
    Vector::canonical_basis(|dir| {
        cancel_relative_linear_velocity_wrt_axis(
            body1,
            part1,
            body2,
            part2,
            assembly_id1,
            assembly_id2,
            anchor1,
            anchor2,
            &Unit::new_unchecked(*dir),
            ext_vels,
            impulses[i],
            impulse_id + i,
            ground_j_id,
            j_id,
            jacobians,
            constraints,
        );

        i += 1;

        true
    });
}

/// Generate position constraints to cancel the relative translation of two bodies wrt the given axis.
///
/// All inputs mut be given in world-space.
pub fn cancel_relative_translation_wrt_axis<N: RealField>(
    params: &IntegrationParameters<N>,
    body1: &Body<N>,
    part1: &BodyPart<N>,
    body2: &Body<N>,
    part2: &BodyPart<N>,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    axis: &Unit<Vector<N>>,
    jacobians: &mut [N],
) -> Option<GenericNonlinearConstraint<N>> {
    let mut depth = axis.dot(&(anchor2 - anchor1));

    let force = if depth < N::zero() {
        depth = -depth;
        ForceDirection::Linear(-*axis)
    } else {
        ForceDirection::Linear(*axis)
    };

    if depth > params.allowed_linear_error {
        let mut j_id = 0;
        let mut ground_j_id = 0;

        let geom = constraint_pair_geometry(
            body1,
            part1,
            body2,
            part2,
            anchor1,
            anchor2,
            &force,
            &mut ground_j_id,
            &mut j_id,
            jacobians,
            None,
            None,
            None
        );

        let rhs = -depth;
        let constraint = GenericNonlinearConstraint::new(
            part1.part_handle(),
            part2.part_handle(),
            false,
            geom.ndofs1,
            geom.ndofs2,
            geom.wj_id1,
            geom.wj_id2,
            rhs,
            geom.r,
        );

        Some(constraint)
    } else {
        None
    }
}

/// Generate position constraints to cancel the relative translation of two bodies.
///
/// All inputs mut be given in world-space.
pub fn cancel_relative_translation<N: RealField>(
    params: &IntegrationParameters<N>,
    body1: &Body<N>,
    part1: &BodyPart<N>,
    body2: &Body<N>,
    part2: &BodyPart<N>,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    jacobians: &mut [N],
) -> Option<GenericNonlinearConstraint<N>> {
    let error = anchor2 - anchor1;

    if let Some((dir, depth)) = Unit::try_new_and_get(error, params.allowed_linear_error) {
        let mut j_id = 0;
        let mut ground_j_id = 0;

        let geom = constraint_pair_geometry(
            body1,
            part1,
            body2,
            part2,
            anchor1,
            anchor2,
            &ForceDirection::Linear(dir),
            &mut ground_j_id,
            &mut j_id,
            jacobians,
            None,
            None,
            None
        );

        let rhs = -depth;
        let constraint = GenericNonlinearConstraint::new(
            part1.part_handle(),
            part2.part_handle(),
            false,
            geom.ndofs1,
            geom.ndofs2,
            geom.wj_id1,
            geom.wj_id2,
            rhs,
            geom.r,
        );

        Some(constraint)
    } else {
        None
    }
}

/// Generate velocity constraints to cancel the relative angular velocity of two bodies wrt. the given axis.
///
/// All inputs mut be given in world-space.
pub fn cancel_relative_angular_velocity_wrt_axis<N: RealField>(
    body1: &Body<N>,
    part1: &BodyPart<N>,
    body2: &Body<N>,
    part2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    axis: &Unit<AngularVector<N>>,
    ext_vels: &DVector<N>,
    impulse: N,
    impulse_id: usize,
    ground_j_id: &mut usize,
    j_id: &mut usize,
    jacobians: &mut [N],
    constraints: &mut ConstraintSet<N>,
) {
    let limits = ImpulseLimits::Independent {
        min: -N::max_value(),
        max: N::max_value(),
    };

    let (ext_vels1, ext_vels2) = split_ext_vels(body1, body2, assembly_id1, assembly_id2, ext_vels);
    let force = ForceDirection::Angular(*axis);
    let mut rhs = N::zero();

    let geom = constraint_pair_geometry(
        body1,
        part1,
        body2,
        part2,
        anchor1,
        anchor2,
        &force,
        ground_j_id,
        j_id,
        jacobians,
        Some(&ext_vels1),
        Some(&ext_vels2),
        Some(&mut rhs)
    );

    if geom.ndofs1 == 0 || geom.ndofs2 == 0 {
        constraints
            .velocity
            .bilateral_ground
            .push(BilateralGroundConstraint::new(
                geom,
                assembly_id1,
                assembly_id2,
                limits,
                rhs,
                impulse,
                impulse_id,
            ));
    } else {
        constraints
            .velocity
            .bilateral
            .push(BilateralConstraint::new(
                geom,
                assembly_id1,
                assembly_id2,
                limits,
                rhs,
                impulse,
                impulse_id,
            ));
    }
}

/// Generate velocity constraints to cancel the relative angular velocity of two bodies.
///
/// All inputs mut be given in world-space.
pub fn cancel_relative_angular_velocity<N: RealField>(
    body1: &Body<N>,
    part1: &BodyPart<N>,
    body2: &Body<N>,
    part2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    ext_vels: &DVector<N>,
    impulses: &AngularVector<N>,
    impulse_id: usize,
    ground_j_id: &mut usize,
    j_id: &mut usize,
    jacobians: &mut [N],
    constraints: &mut ConstraintSet<N>,
) {
    let mut i = 0;
    AngularVector::canonical_basis(|dir| {
        cancel_relative_angular_velocity_wrt_axis(
            body1,
            part1,
            body2,
            part2,
            assembly_id1,
            assembly_id2,
            anchor1,
            anchor2,
            &Unit::new_unchecked(*dir),
            ext_vels,
            impulses[i],
            impulse_id + i,
            ground_j_id,
            j_id,
            jacobians,
            constraints,
        );

        i += 1;

        true
    });
}

/// Generate position constraints to cancel the relative rotation of two bodies.
///
/// All inputs mut be given in world-space.
pub fn cancel_relative_rotation<N: RealField>(
    params: &IntegrationParameters<N>,
    body1: &Body<N>,
    part1: &BodyPart<N>,
    body2: &Body<N>,
    part2: &BodyPart<N>,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    rotation1: &Rotation<N>,
    rotation2: &Rotation<N>,
    jacobians: &mut [N],
) -> Option<GenericNonlinearConstraint<N>> {
    let error = (rotation2 / rotation1).scaled_axis();

    if let Some((dir, depth)) = Unit::try_new_and_get(error, params.allowed_angular_error) {
        let mut j_id = 0;
        let mut ground_j_id = 0;

        let geom = constraint_pair_geometry(
            body1,
            part1,
            body2,
            part2,
            anchor1,
            anchor2,
            &ForceDirection::Angular(dir),
            &mut ground_j_id,
            &mut j_id,
            jacobians,
            None,
            None,
            None
        );

        let rhs = -depth;
        let constraint = GenericNonlinearConstraint::new(
            part1.part_handle(),
            part2.part_handle(),
            true,
            geom.ndofs1,
            geom.ndofs2,
            geom.wj_id1,
            geom.wj_id2,
            rhs,
            geom.r,
        );

        Some(constraint)
    } else {
        None
    }
}

/// Generate velocity constraints to cancel the relative angular velocity of two bodies along all axis except the one provided.
///
/// All inputs mut be given in world-space.
#[cfg(feature = "dim3")]
pub fn restrict_relative_angular_velocity_to_axis<N: RealField>(
    body1: &Body<N>,
    part1: &BodyPart<N>,
    body2: &Body<N>,
    part2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    axis: &Unit<AngularVector<N>>,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    ext_vels: &DVector<N>,
    impulses: &[N],
    impulse_id: usize,
    ground_j_id: &mut usize,
    j_id: &mut usize,
    jacobians: &mut [N],
    constraints: &mut ConstraintSet<N>,
) {
    let limits = ImpulseLimits::Independent {
        min: -N::max_value(),
        max: N::max_value(),
    };

    let (ext_vels1, ext_vels2) = split_ext_vels(body1, body2, assembly_id1, assembly_id2, ext_vels);

    let mut i = 0;
    AngularVector::orthonormal_subspace_basis(&[axis.into_inner()], |dir| {
        let dir = ForceDirection::Angular(Unit::new_unchecked(*dir));
        let mut rhs = N::zero();
        let geom = constraint_pair_geometry(
            body1,
            part1,
            body2,
            part2,
            anchor1,
            anchor2,
            &dir,
            ground_j_id,
            j_id,
            jacobians,
            Some(&ext_vels1),
            Some(&ext_vels2),
            Some(&mut rhs)
        );

        if geom.ndofs1 == 0 || geom.ndofs2 == 0 {
            constraints
                .velocity
                .bilateral_ground
                .push(BilateralGroundConstraint::new(
                    geom,
                    assembly_id1,
                    assembly_id2,
                    limits,
                    rhs,
                    impulses[i],
                    impulse_id + i,
                ));
        } else {
            constraints
                .velocity
                .bilateral
                .push(BilateralConstraint::new(
                    geom,
                    assembly_id1,
                    assembly_id2,
                    limits,
                    rhs,
                    impulses[i],
                    impulse_id + i,
                ));
        }

        i += 1;

        true
    });
}

/// Generate position constraints to moved the body parts such that the given axis will become aligned.
///
/// All inputs mut be given in world-space.
#[cfg(feature = "dim3")]
pub fn align_axis<N: RealField>(
    params: &IntegrationParameters<N>,
    body1: &Body<N>,
    part1: &BodyPart<N>,
    body2: &Body<N>,
    part2: &BodyPart<N>,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    axis1: &Unit<Vector<N>>,
    axis2: &Unit<Vector<N>>,
    jacobians: &mut [N],
) -> Option<GenericNonlinearConstraint<N>> {
    // Angular regularization for two coincident axis.
    let mut error;
    if let Some(error_rot) = Rotation::rotation_between_axis(&axis1, &axis2) {
        error = error_rot.scaled_axis();
    } else {
        // Error equal to Pi, select one orthogonal direction.
        let imin = axis1.iamin();
        error = Vector::zeros();
        error[imin] = N::one();
        error = error.cross(&axis1).normalize() * N::pi();
    }

    if let Some((dir, depth)) = Unit::try_new_and_get(error, params.allowed_angular_error) {
        let mut j_id = 0;
        let mut ground_j_id = 0;

        let geom = constraint_pair_geometry(
            body1,
            part1,
            body2,
            part2,
            anchor1,
            anchor2,
            &ForceDirection::Angular(dir),
            &mut ground_j_id,
            &mut j_id,
            jacobians,
            None,
            None,
            None
        );

        let rhs = -depth;
        let constraint = GenericNonlinearConstraint::new(
            part1.part_handle(),
            part2.part_handle(),
            true,
            geom.ndofs1,
            geom.ndofs2,
            geom.wj_id1,
            geom.wj_id2,
            rhs,
            geom.r,
        );

        Some(constraint)
    } else {
        None
    }
}

/// Generate velocity constraints to cancel the relative linear velocity of two bodies along all axis except the one provided.
///
/// All inputs mut be given in world-space.
pub fn restrict_relative_linear_velocity_to_axis<N: RealField>(
    body1: &Body<N>,
    part1: &BodyPart<N>,
    body2: &Body<N>,
    part2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    axis1: &Unit<Vector<N>>,
    ext_vels: &DVector<N>,
    impulses: &[N],
    impulse_id: usize,
    ground_j_id: &mut usize,
    j_id: &mut usize,
    jacobians: &mut [N],
    constraints: &mut ConstraintSet<N>,
) {
    let limits = ImpulseLimits::Independent {
        min: -N::max_value(),
        max: N::max_value(),
    };

    let (ext_vels1, ext_vels2) = split_ext_vels(body1, body2, assembly_id1, assembly_id2, ext_vels);

    let mut i = 0;
    Vector::orthonormal_subspace_basis(&[axis1.into_inner()], |dir| {
        let dir = ForceDirection::Linear(Unit::new_unchecked(*dir));
        let mut rhs = N::zero();

        let geom = constraint_pair_geometry(
            body1,
            part1,
            body2,
            part2,
            anchor1,
            anchor2,
            &dir,
            ground_j_id,
            j_id,
            jacobians,
            Some(&ext_vels1),
            Some(&ext_vels2),
            Some(&mut rhs)
        );

        if geom.ndofs1 == 0 || geom.ndofs2 == 0 {
            constraints
                .velocity
                .bilateral_ground
                .push(BilateralGroundConstraint::new(
                    geom,
                    assembly_id1,
                    assembly_id2,
                    limits,
                    rhs,
                    impulses[i],
                    impulse_id + i,
                ));
        } else {
            constraints
                .velocity
                .bilateral
                .push(BilateralConstraint::new(
                    geom,
                    assembly_id1,
                    assembly_id2,
                    limits,
                    rhs,
                    impulses[i],
                    impulse_id + i,
                ));
        }

        i += 1;

        true
    });
}

/// Generate position constraints to project `anchor2` into the axis with direction `axis1` and passing through the `anchor1`.
///
/// All inputs mut be given in world-space.
pub fn project_anchor_to_axis<N: RealField>(
    params: &IntegrationParameters<N>,
    body1: &Body<N>,
    part1: &BodyPart<N>,
    body2: &Body<N>,
    part2: &BodyPart<N>,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    axis1: &Unit<Vector<N>>,
    jacobians: &mut [N],
) -> Option<GenericNonlinearConstraint<N>> {
    // Linear regularization of a point on an axis.
    let dpt = anchor2 - anchor1;
    let proj = anchor1 + axis1.into_inner() * axis1.dot(&dpt);
    let error = anchor2 - proj;

    if let Some((dir, depth)) = Unit::try_new_and_get(error, params.allowed_linear_error) {
        let mut j_id = 0;
        let mut ground_j_id = 0;

        let geom = constraint_pair_geometry(
            body1,
            part1,
            body2,
            part2,
            anchor1,
            anchor2,
            &ForceDirection::Linear(dir),
            &mut ground_j_id,
            &mut j_id,
            jacobians,
            None,
            None,
            None
        );

        let rhs = -depth;
        let constraint = GenericNonlinearConstraint::new(
            part1.part_handle(),
            part2.part_handle(),
            false,
            geom.ndofs1,
            geom.ndofs2,
            geom.wj_id1,
            geom.wj_id2,
            rhs,
            geom.r,
        );

        Some(constraint)
    } else {
        None
    }
}

/// Generate position constraints to ensure the two given axis are seperated by the given angle.
///
/// All inputs mut be given in world-space.
#[cfg(feature = "dim3")]
pub fn restore_angle_between_axis<N: RealField>(
    params: &IntegrationParameters<N>,
    body1: &Body<N>,
    part1: &BodyPart<N>,
    body2: &Body<N>,
    part2: &BodyPart<N>,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    axis1: &Unit<Vector<N>>,
    axis2: &Unit<Vector<N>>,
    angle: N,
    jacobians: &mut [N],
) -> Option<GenericNonlinearConstraint<N>> {
    // Angular regularization for two coincident axis.
    let mut separation;
    if let Some(separation_rot) = Rotation::rotation_between_axis(&axis1, &axis2) {
        separation = separation_rot.scaled_axis();
    } else {
        // Separation equal to Pi, select one orthogonal direction.
        let imin = axis1.iamin();
        separation = Vector::zeros();
        separation[imin] = N::one();
        separation = separation.cross(&axis1).normalize() * N::pi();
    }

    if let Some((mut dir, curr_ang)) = Unit::try_new_and_get(separation, N::default_epsilon()) {
        let mut error = curr_ang - angle;

        if error < N::zero() {
            error = -error;
            dir = -dir;
        }

        if error < params.allowed_angular_error {
            return None;
        }

        let mut j_id = 0;
        let mut ground_j_id = 0;

        let geom = constraint_pair_geometry(
            body1,
            part1,
            body2,
            part2,
            anchor1,
            anchor2,
            &ForceDirection::Angular(dir),
            &mut ground_j_id,
            &mut j_id,
            jacobians,
            None,
            None,
            None
        );

        let rhs = -error;
        let constraint = GenericNonlinearConstraint::new(
            part1.part_handle(),
            part2.part_handle(),
            true,
            geom.ndofs1,
            geom.ndofs2,
            geom.wj_id1,
            geom.wj_id2,
            rhs,
            geom.r,
        );

        Some(constraint)
    } else {
        None
    }
}
