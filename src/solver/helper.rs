use alga::linear::{FiniteDimInnerSpace, FiniteDimVectorSpace};
use na::{self, DVector, DVectorSlice, Real, Unit};
use std::ops::Neg;

use math::{AngularVector, Force, Point, Rotation, Vector};
use object::BodyPart;
use solver::{BilateralConstraint, BilateralGroundConstraint, ConstraintGeometry, ConstraintSet,
             GenericNonlinearConstraint, ImpulseLimits, IntegrationParameters};

#[derive(Copy, Clone, Debug)]
pub enum ForceDirection<N: Real> {
    Linear(Unit<Vector<N>>),
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

#[inline]
pub fn constraints_are_ground_constraints<N: Real>(
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
) -> bool {
    body1.status_dependent_parent_ndofs() == 0 || body2.status_dependent_parent_ndofs() == 0
}

pub fn cancel_relative_linear_velocity<N: Real>(
    params: &IntegrationParameters<N>,
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
    let limits = ImpulseLimits::Independent {
        min: -N::max_value(),
        max: N::max_value(),
    };

    let mut i = 0;
    Vector::canonical_basis(|dir| {
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

pub fn cancel_relative_angular_velocity<N: Real>(
    params: &IntegrationParameters<N>,
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    orientation1: &Rotation<N>,
    orientation2: &Rotation<N>,
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
    let limits = ImpulseLimits::Independent {
        min: -N::max_value(),
        max: N::max_value(),
    };

    let mut i = 0;
    AngularVector::canonical_basis(|dir| {
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

#[cfg(feature = "dim3")]
pub fn restrict_relative_angular_velocity_to_axis<N: Real>(
    params: &IntegrationParameters<N>,
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    axis1: &Unit<AngularVector<N>>,
    axis2: &Unit<AngularVector<N>>,
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
    AngularVector::orthonormal_subspace_basis(&[axis1.unwrap()], |dir| {
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

pub fn restrict_relative_linear_velocity_to_axis<N: Real>(
    params: &IntegrationParameters<N>,
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    axis1: &Unit<Vector<N>>,
    axis2: &Unit<Vector<N>>,
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

pub fn project_anchor_to_axis<N: Real>(
    params: &IntegrationParameters<N>,
    body1: &BodyPart<N>,
    body2: &BodyPart<N>,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    axis1: &Unit<Vector<N>>,
    axis2: &Unit<Vector<N>>,
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
