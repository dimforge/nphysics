//! Utilities for computing velocity and position constraints.

use alga::linear::{FiniteDimInnerSpace, FiniteDimVectorSpace};
#[cfg(feature = "dim3")]
use na;
use na::{DVector, DVectorSlice, Real, Unit};
use std::ops::Neg;

use math::{AngularVector, Force, Point, Rotation, Vector, SPATIAL_DIM};
use object::{BodyPart, RigidBody};
use solver::{BilateralConstraint, BilateralGroundConstraint, ConstraintGeometry, ConstraintSet,
             GenericNonlinearConstraint, ImpulseLimits, IntegrationParameters};

/// The direction of a force.
#[derive(Copy, Clone, Debug)]
pub enum ForceDirection<N: Real> {
    /// A linear force toward a direction.
    Linear(Unit<Vector<N>>),
    /// A torque wrt. an axis.
    Angular(Unit<AngularVector<N>>),
}

impl<N: Real> Neg for ForceDirection<N> {
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
/// constraint applying a force at the point `center` (relative to the body part's center of mass) and
/// the direction `dir`.
///
/// If the force is a torque, it is applied at the center of mass of the body part.
#[inline]
pub fn fill_constraint_geometry<N: Real>(
    body: &BodyPart<N>,
    ndofs: usize,
    center: &Point<N>,
    dir: &ForceDirection<N>,
    j_id: usize,
    wj_id: usize,
    jacobians: &mut [N],
    inv_r: &mut N,
) {
    if let BodyPart::RigidBody(rb) = *body {
        fill_constraint_geometry_rigid_body(rb, ndofs, center, dir, j_id, wj_id, jacobians, inv_r);
        return;
    }

    let force;
    let pos = center - body.center_of_mass().coords;

    match *dir {
        ForceDirection::Linear(normal) => {
            force = Force::linear_at_point(*normal, &pos);
        }
        ForceDirection::Angular(axis) => {
            force = Force::torque_from_vector(*axis);
            // force = Force::torque_from_vector_at_point(*axis, &pos);
        }
    }

    body.body_jacobian_mul_force(&force, &mut jacobians[j_id..]);
    // FIXME: this could be optimized with a copy_nonoverlapping.
    for i in 0..ndofs {
        jacobians[wj_id + i] = jacobians[j_id + i];
    }
    body.inv_mass_mul_generalized_forces(&mut jacobians[wj_id..]);

    let j = DVectorSlice::from_slice(&jacobians[j_id..], ndofs);
    let invm_j = DVectorSlice::from_slice(&jacobians[wj_id..], ndofs);

    *inv_r += j.dot(&invm_j);
}

#[inline(never)]
fn fill_constraint_geometry_rigid_body<N: Real>(
    body: &RigidBody<N>,
    ndofs: usize,
    center: &Point<N>,
    dir: &ForceDirection<N>,
    j_id: usize,
    wj_id: usize,
    jacobians: &mut [N],
    inv_r: &mut N,
) {
    let force;
    let pos = center - body.center_of_mass().coords;

    match *dir {
        ForceDirection::Linear(normal) => {
            force = Force::linear_at_point(*normal, &pos);
        }
        ForceDirection::Angular(axis) => {
            force = Force::torque_from_vector(*axis);
        }
    }

    jacobians[j_id..j_id + SPATIAL_DIM].copy_from_slice(force.as_slice());

    let inv_mass = body.inv_augmented_mass();
    let imf = *inv_mass * force;
    jacobians[wj_id..wj_id + SPATIAL_DIM].copy_from_slice(imf.as_slice());

    *inv_r += inv_mass.mass() + force.angular_vector().dot(&imf.angular_vector());
}

/// Fills all the jacobians (and the jacobians multiplied by the invers augmented mass matricxs) for a
/// constraint applying a force at the points `center1, center2` and the direction `dir`.
///
/// If the force is a torque, it is applied at the centers of mass of the body parts.
/// Every input are expressed in world-space.
#[inline]
pub fn constraint_pair_geometry<N: Real>(
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
    center1: &Point<N>,
    center2: &Point<N>,
    dir: &ForceDirection<N>,
    ground_j_id: &mut usize,
    j_id: &mut usize,
    jacobians: &mut [N],
) -> ConstraintGeometry<N> {
    let mut res = ConstraintGeometry::new();

    res.ndofs1 = body1.status_dependent_parent_ndofs();
    res.ndofs2 = body2.status_dependent_parent_ndofs();

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

    if res.ndofs1 != 0 {
        fill_constraint_geometry(
            body1,
            res.ndofs1,
            center1,
            dir,
            res.j_id1,
            res.wj_id1,
            jacobians,
            &mut inv_r,
        );
    }

    if res.ndofs2 != 0 {
        fill_constraint_geometry(
            body2,
            res.ndofs2,
            center2,
            &dir.neg(),
            res.j_id2,
            res.wj_id2,
            jacobians,
            &mut inv_r,
        );
    }

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

/// Compute the generalized relative velocity for at the given points and along a given direction.
///
/// Every input are expressed in world-space.
#[inline]
pub fn constraint_pair_velocity<N: Real>(
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    center1: &Point<N>,
    center2: &Point<N>,
    dir: &ForceDirection<N>,
    ext_vels: &DVector<N>,
    jacobians: &[N],
    geom: &ConstraintGeometry<N>,
) -> N {
    let mut vel = N::zero();

    if geom.ndofs1 != 0 {
        let j = DVectorSlice::from_slice(&jacobians[geom.j_id1..], geom.ndofs1);
        vel += j.dot(&body1.parent_generalized_velocity())
            + j.dot(&ext_vels.rows(assembly_id1, geom.ndofs1));
    } else {
        // Adjust the rhs for kinematic bodies.
        let vel1 = body1.status_dependent_velocity();
        match *dir {
            ForceDirection::Linear(ref normal) => {
                let dpos = center1 - body1.center_of_mass();
                vel += vel1.shift(&dpos).linear.dot(normal);
            }
            ForceDirection::Angular(ref axis) => {
                // FIXME: do we have to take dpos into account here?
                vel += vel1.angular_vector().dot(axis);
            }
        }
    }

    if geom.ndofs2 != 0 {
        let j = DVectorSlice::from_slice(&jacobians[geom.j_id2..], geom.ndofs2);
        vel += j.dot(&body2.parent_generalized_velocity())
            + j.dot(&ext_vels.rows(assembly_id2, geom.ndofs2));
    } else {
        // Adjust the rhs for kinematic bodies.
        let vel2 = body2.status_dependent_velocity();

        match *dir {
            ForceDirection::Linear(ref normal) => {
                let dpos = center2 - body2.center_of_mass();
                vel -= vel2.shift(&dpos).linear.dot(normal);
            }
            ForceDirection::Angular(ref axis) => {
                vel -= vel2.angular_vector().dot(axis);
            }
        }
    }

    vel
}

/// Test sif a constraint between the two given bodies should be a ground
/// constraint (a constraint between a dynamic body and one without any degree of freedom).
#[inline]
pub fn constraints_are_ground_constraints<N: Real>(
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
) -> bool {
    body1.status_dependent_parent_ndofs() == 0 || body2.status_dependent_parent_ndofs() == 0
}

/// Generates velocity constraints to cancel the relative linear velocity of two body parts wrt the given axis.
///
/// All inputs mut be given in world-space.
pub fn cancel_relative_linear_velocity_wrt_axis<N: Real>(
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
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
    let geom = constraint_pair_geometry(
        body1,
        body2,
        anchor1,
        anchor2,
        &force,
        ground_j_id,
        j_id,
        jacobians,
    );

    let rhs = constraint_pair_velocity(
        &body1,
        &body2,
        assembly_id1,
        assembly_id2,
        anchor1,
        anchor2,
        &force,
        ext_vels,
        jacobians,
        &geom,
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
pub fn cancel_relative_linear_velocity<N: Real>(
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
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
            body2,
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
pub fn cancel_relative_translation_wrt_axis<N: Real>(
    params: &IntegrationParameters<N>,
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
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
            body2,
            anchor1,
            anchor2,
            &force,
            &mut ground_j_id,
            &mut j_id,
            jacobians,
        );

        let rhs = -depth;
        let constraint = GenericNonlinearConstraint::new(
            body1.handle(),
            body2.handle(),
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
pub fn cancel_relative_translation<N: Real>(
    params: &IntegrationParameters<N>,
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
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
            body2,
            anchor1,
            anchor2,
            &ForceDirection::Linear(dir),
            &mut ground_j_id,
            &mut j_id,
            jacobians,
        );

        let rhs = -depth;
        let constraint = GenericNonlinearConstraint::new(
            body1.handle(),
            body2.handle(),
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
pub fn cancel_relative_angular_velocity_wrt_axis<N: Real>(
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
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

    let force = ForceDirection::Angular(*axis);
    let geom = constraint_pair_geometry(
        body1,
        body2,
        anchor1,
        anchor2,
        &force,
        ground_j_id,
        j_id,
        jacobians,
    );

    let rhs = constraint_pair_velocity(
        &body1,
        &body2,
        assembly_id1,
        assembly_id2,
        anchor1,
        anchor2,
        &force,
        ext_vels,
        jacobians,
        &geom,
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
pub fn cancel_relative_angular_velocity<N: Real>(
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
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
            body2,
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
pub fn cancel_relative_rotation<N: Real>(
    params: &IntegrationParameters<N>,
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
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
            body2,
            anchor1,
            anchor2,
            &ForceDirection::Angular(dir),
            &mut ground_j_id,
            &mut j_id,
            jacobians,
        );

        let rhs = -depth;
        let constraint = GenericNonlinearConstraint::new(
            body1.handle(),
            body2.handle(),
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
pub fn restrict_relative_angular_velocity_to_axis<N: Real>(
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
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

    let mut i = 0;
    AngularVector::orthonormal_subspace_basis(&[axis.unwrap()], |dir| {
        let dir = ForceDirection::Angular(Unit::new_unchecked(*dir));
        let geom = constraint_pair_geometry(
            body1,
            body2,
            anchor1,
            anchor2,
            &dir,
            ground_j_id,
            j_id,
            jacobians,
        );

        let rhs = constraint_pair_velocity(
            &body1,
            &body2,
            assembly_id1,
            assembly_id2,
            anchor1,
            anchor2,
            &dir,
            ext_vels,
            jacobians,
            &geom,
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
pub fn align_axis<N: Real>(
    params: &IntegrationParameters<N>,
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
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
        error = na::normalize(&error.cross(&axis1)) * N::pi();
    }

    if let Some((dir, depth)) = Unit::try_new_and_get(error, params.allowed_angular_error) {
        let mut j_id = 0;
        let mut ground_j_id = 0;

        let geom = constraint_pair_geometry(
            body1,
            body2,
            anchor1,
            anchor2,
            &ForceDirection::Angular(dir),
            &mut ground_j_id,
            &mut j_id,
            jacobians,
        );

        let rhs = -depth;
        let constraint = GenericNonlinearConstraint::new(
            body1.handle(),
            body2.handle(),
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
pub fn restrict_relative_linear_velocity_to_axis<N: Real>(
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
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

    let mut i = 0;
    Vector::orthonormal_subspace_basis(&[axis1.unwrap()], |dir| {
        let dir = ForceDirection::Linear(Unit::new_unchecked(*dir));
        let geom = constraint_pair_geometry(
            body1,
            body2,
            anchor1,
            anchor2,
            &dir,
            ground_j_id,
            j_id,
            jacobians,
        );

        let rhs = constraint_pair_velocity(
            &body1,
            &body2,
            assembly_id1,
            assembly_id2,
            anchor1,
            anchor2,
            &dir,
            ext_vels,
            jacobians,
            &geom,
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
pub fn project_anchor_to_axis<N: Real>(
    params: &IntegrationParameters<N>,
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    axis1: &Unit<Vector<N>>,
    jacobians: &mut [N],
) -> Option<GenericNonlinearConstraint<N>> {
    // Linear regularization of a point on an axis.
    let dpt = anchor2 - anchor1;
    let proj = anchor1 + axis1.unwrap() * axis1.dot(&dpt);
    let error = anchor2 - proj;

    if let Some((dir, depth)) = Unit::try_new_and_get(error, params.allowed_linear_error) {
        let mut j_id = 0;
        let mut ground_j_id = 0;

        let geom = constraint_pair_geometry(
            body1,
            body2,
            anchor1,
            anchor2,
            &ForceDirection::Linear(dir),
            &mut ground_j_id,
            &mut j_id,
            jacobians,
        );

        let rhs = -depth;
        let constraint = GenericNonlinearConstraint::new(
            body1.handle(),
            body2.handle(),
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
pub fn restore_angle_between_axis<N: Real>(
    params: &IntegrationParameters<N>,
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
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
        separation = na::normalize(&separation.cross(&axis1)) * N::pi();
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
            body2,
            anchor1,
            anchor2,
            &ForceDirection::Angular(dir),
            &mut ground_j_id,
            &mut j_id,
            jacobians,
        );

        let rhs = -error;
        let constraint = GenericNonlinearConstraint::new(
            body1.handle(),
            body2.handle(),
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
