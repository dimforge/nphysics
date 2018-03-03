use std::ops::Neg;
use alga::linear::FiniteDimVectorSpace;
use na::{self, DVector, DVectorSlice, Real, Unit};

use solver::{BilateralConstraint, BilateralGroundConstraint, ConstraintGeometry, ConstraintSet,
             ImpulseLimits, IntegrationParameters};
use object::BodyPart;
use math::{AngularVector, Force, Point, Rotation, Vector};

#[cfg(feature = "dim3")]
use alga::linear::FiniteDimInnerSpace;

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
    jacobian_id: usize,
    weighted_jacobian_id: usize,
    jacobians: &mut [N],
    inv_r: &mut N,
) {
    let force;

    match *dir {
        ForceDirection::Linear(normal) => {
            let pos = center - body.center_of_mass();
            force = Force::shifted_linear_force(*normal, pos);
        }
        ForceDirection::Angular(axis) => {
            force = Force::torque_from_vector(*axis);
        }
    }

    body.body_jacobian_mul_force(&force, &mut jacobians[jacobian_id..]);
    // FIXME: this could be optimized with a copy_nonoverlapping.
    for i in 0..ndofs {
        jacobians[weighted_jacobian_id + i] = jacobians[jacobian_id + i];
    }
    body.inv_mass_mul_generalized_forces(&mut jacobians[weighted_jacobian_id..]);

    let j = DVectorSlice::new(&jacobians[jacobian_id..], ndofs);
    let invm_j = DVectorSlice::new(&jacobians[weighted_jacobian_id..], ndofs);

    *inv_r += j.dot(&invm_j);
}

#[inline]
fn velocity_constraint_rhs<N: Real>(
    body: &BodyPart<N>,
    ndofs: usize,
    ext_vels: &DVector<N>,
    jacobian_id: usize,
    assembly_id: usize,
    jacobians: &[N],
    rhs: &mut N,
) {
    let j = DVectorSlice::new(&jacobians[jacobian_id..], ndofs);
    *rhs += j.dot(&body.parent_generalized_velocity()) + j.dot(&ext_vels.rows(assembly_id, ndofs));
}

#[inline]
pub fn constraints_are_ground_constraints<N: Real>(b1: &BodyPart<N>, b2: &BodyPart<N>) -> bool {
    b1.status_dependent_parent_ndofs() == 0 || b2.status_dependent_parent_ndofs() == 0
}

// FIXME: take a Unit for the normal.
#[inline]
pub fn constraint_pair_geometry<N: Real>(
    b1: &BodyPart<N>,
    b2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    center1: &Point<N>,
    center2: &Point<N>,
    dir: &ForceDirection<N>,
    ext_vels: &DVector<N>,
    ground_jacobian_id: &mut usize,
    jacobian_id: &mut usize,
    jacobians: &mut [N],
) -> ConstraintGeometry<N> {
    let mut res = ConstraintGeometry::new();

    res.ndofs1 = b1.status_dependent_parent_ndofs();
    res.ndofs2 = b2.status_dependent_parent_ndofs();

    let out_jacobian_id;
    if res.ndofs1 == 0 || res.ndofs2 == 0 {
        res.jacobian_id1 = *ground_jacobian_id;
        out_jacobian_id = ground_jacobian_id;
    } else {
        res.jacobian_id1 = *jacobian_id;
        out_jacobian_id = jacobian_id;
    }

    res.jacobian_id2 = res.jacobian_id1 + res.ndofs1;
    res.weighted_jacobian_id1 = res.jacobian_id2 + res.ndofs2;
    res.weighted_jacobian_id2 = res.weighted_jacobian_id1 + res.ndofs1;

    let mut inv_r = N::zero();

    if res.ndofs1 != 0 {
        res.assembly_id1 = assembly_id1;

        fill_constraint_geometry(
            b1,
            res.ndofs1,
            center1,
            &dir.neg(),
            res.jacobian_id1,
            res.weighted_jacobian_id1,
            jacobians,
            &mut inv_r,
        );
        velocity_constraint_rhs(
            b1,
            res.ndofs1,
            ext_vels,
            res.jacobian_id1,
            res.assembly_id1,
            jacobians,
            &mut res.rhs,
        );
    } else {
        // Adjust the rhs for kinematic bodies.
        let vel = b1.status_dependent_velocity();

        match *dir {
            ForceDirection::Linear(ref normal) => {
                let dpos = center1 - b1.center_of_mass();
                res.rhs -= vel.shift(&dpos).linear.dot(normal);
            }
            ForceDirection::Angular(ref axis) => {
                res.rhs -= vel.angular_vector().dot(axis);
            }
        }
    }

    if res.ndofs2 != 0 {
        res.assembly_id2 = assembly_id2;

        fill_constraint_geometry(
            b2,
            res.ndofs2,
            center2,
            dir,
            res.jacobian_id2,
            res.weighted_jacobian_id2,
            jacobians,
            &mut inv_r,
        );
        velocity_constraint_rhs(
            b2,
            res.ndofs2,
            ext_vels,
            res.jacobian_id2,
            res.assembly_id2,
            jacobians,
            &mut res.rhs,
        );
    } else {
        // Adjust the rhs for kinematic bodies.
        let vel = b2.status_dependent_velocity();

        match *dir {
            ForceDirection::Linear(ref normal) => {
                let dpos = center2 - b2.center_of_mass();
                res.rhs += vel.shift(&dpos).linear.dot(normal);
            }
            ForceDirection::Angular(ref axis) => {
                res.rhs += vel.angular_vector().dot(axis);
            }
        }
    }

    if res.assembly_id1 == res.assembly_id2 {
        let j1 = DVectorSlice::new(&jacobians[res.jacobian_id1..], res.ndofs1);
        let j2 = DVectorSlice::new(&jacobians[res.jacobian_id2..], res.ndofs2);
        let invm_j1 = DVectorSlice::new(&jacobians[res.weighted_jacobian_id1..], res.ndofs1);
        let invm_j2 = DVectorSlice::new(&jacobians[res.weighted_jacobian_id2..], res.ndofs2);

        inv_r += j2.dot(&invm_j1) + j1.dot(&invm_j2);
    }

    if !inv_r.is_zero() {
        res.r = N::one() / inv_r;
    } else {
        res.r = N::one()
    }

    *out_jacobian_id += (res.ndofs1 + res.ndofs2) * 2;
    res
}

pub fn cancel_relative_linear_motion<N: Real>(
    params: &IntegrationParameters<N>,
    b1: &BodyPart<N>,
    b2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    ext_vels: &DVector<N>,
    ground_jacobian_id: &mut usize,
    jacobian_id: &mut usize,
    jacobians: &mut [N],
    constraints: &mut ConstraintSet<N>,
) {
    let error = anchor1 - anchor2;
    let limits = ImpulseLimits::Independent {
        min: -N::max_value(),
        max: N::max_value(),
    };

    Vector::canonical_basis(|dir| {
        let mut geom = constraint_pair_geometry(
            b1,
            b2,
            assembly_id1,
            assembly_id2,
            anchor1,
            anchor2,
            &ForceDirection::Linear(Unit::new_unchecked(*dir)),
            ext_vels,
            ground_jacobian_id,
            jacobian_id,
            jacobians,
        );

        let stabilization = -dir.dot(&error) / params.dt * params.erp;
        geom.rhs += stabilization;

        if geom.ndofs1 == 0 || geom.ndofs2 == 0 {
            constraints
                .velocity
                .bilateral_ground
                .push(BilateralGroundConstraint::new(geom, limits, na::zero(), 0));
        } else {
            constraints
                .velocity
                .bilateral
                .push(BilateralConstraint::new(geom, limits, na::zero(), 0));
        }

        true
    });
}

pub fn cancel_relative_angular_motion<N: Real>(
    params: &IntegrationParameters<N>,
    b1: &BodyPart<N>,
    b2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    orientation1: &Rotation<N>,
    orientation2: &Rotation<N>,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    ext_vels: &DVector<N>,
    ground_jacobian_id: &mut usize,
    jacobian_id: &mut usize,
    jacobians: &mut [N],
    constraints: &mut ConstraintSet<N>,
) {
    let error = (orientation1 / orientation2).scaled_axis();
    let limits = ImpulseLimits::Independent {
        min: -N::max_value(),
        max: N::max_value(),
    };

    AngularVector::canonical_basis(|dir| {
        let mut geom = constraint_pair_geometry(
            b1,
            b2,
            assembly_id1,
            assembly_id2,
            anchor1,
            anchor2,
            &ForceDirection::Angular(Unit::new_unchecked(*dir)),
            ext_vels,
            ground_jacobian_id,
            jacobian_id,
            jacobians,
        );

        let stabilization = -dir.dot(&error) / params.dt * params.erp;
        geom.rhs += stabilization;

        if geom.ndofs1 == 0 || geom.ndofs2 == 0 {
            constraints
                .velocity
                .bilateral_ground
                .push(BilateralGroundConstraint::new(geom, limits, na::zero(), 0));
        } else {
            constraints
                .velocity
                .bilateral
                .push(BilateralConstraint::new(geom, limits, na::zero(), 0));
        }

        true
    });
}

#[cfg(feature = "dim3")]
pub fn restrict_relative_angular_motion_to_axis<N: Real>(
    params: &IntegrationParameters<N>,
    b1: &BodyPart<N>,
    b2: &BodyPart<N>,
    assembly_id1: usize,
    assembly_id2: usize,
    axis1: &Unit<AngularVector<N>>,
    axis2: &Unit<AngularVector<N>>,
    anchor1: &Point<N>,
    anchor2: &Point<N>,
    ext_vels: &DVector<N>,
    ground_jacobian_id: &mut usize,
    jacobian_id: &mut usize,
    jacobians: &mut [N],
    constraints: &mut ConstraintSet<N>,
) {
    let limits = ImpulseLimits::Independent {
        min: -N::max_value(),
        max: N::max_value(),
    };

    // NOTE: error equal to Pi won't be corrected.
    let mut error;
    if let Some(error_rot) = Rotation::rotation_between_axis(&axis2, &axis1) {
        error = error_rot.scaled_axis();
    } else {
        // Error equal to Pi, select one orthogonal direction.
        let imin = axis1.iamin();
        error = Vector::zeros();
        error[imin] = N::one();
        error = na::normalize(&error.cross(&axis1)) * N::pi();
    }

    AngularVector::orthonormal_subspace_basis(&[axis1.unwrap()], |dir| {
        let mut geom = constraint_pair_geometry(
            b1,
            b2,
            assembly_id1,
            assembly_id2,
            anchor1,
            anchor2,
            &ForceDirection::Angular(Unit::new_unchecked(*dir)),
            ext_vels,
            ground_jacobian_id,
            jacobian_id,
            jacobians,
        );

        let stabilization = -dir.dot(&error) / params.dt * params.erp;
        geom.rhs += stabilization;

        if geom.ndofs1 == 0 || geom.ndofs2 == 0 {
            constraints
                .velocity
                .bilateral_ground
                .push(BilateralGroundConstraint::new(geom, limits, na::zero(), 0));
        } else {
            constraints
                .velocity
                .bilateral
                .push(BilateralConstraint::new(geom, limits, na::zero(), 0));
        }

        true
    });
}
