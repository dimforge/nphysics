use na::storage::StorageMut;
use na::{self, DVector, Dim, Dynamic, RealField, Vector, VectorSliceN, U1};

use ncollide::query::ContactId;

// FIXME: could we just merge UnilateralConstraint and Bilateral constraint into a single structure
// without performance impact due to clamping?
use crate::math::{SpatialDim, SPATIAL_DIM};
use crate::object::{BodyHandle, BodySet};
use crate::solver::{
    BilateralConstraint, BilateralGroundConstraint, ImpulseLimits, LinearConstraints,
    UnilateralConstraint, UnilateralGroundConstraint,
};

/// A SOR-Prox velocity-based constraints solver.
pub(crate) struct SORProx;

impl SORProx {
    fn warmstart_set<N: RealField, Handle: BodyHandle, Id>(
        _bodies: &mut dyn BodySet<N, Handle = Handle>,
        constraints: &mut LinearConstraints<N, Id>,
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
    ) {
        for c in constraints.unilateral.iter_mut() {
            let dim1 = Dynamic::new(c.ndofs1);
            let dim2 = Dynamic::new(c.ndofs2);
            Self::warmstart_unilateral(c, jacobians, mj_lambda, dim1, dim2);
        }

        for c in constraints.unilateral_ground.iter_mut() {
            let dim = Dynamic::new(c.ndofs);
            Self::warmstart_unilateral_ground(c, jacobians, mj_lambda, dim);
        }

        for c in constraints.bilateral.iter_mut() {
            let dim1 = Dynamic::new(c.ndofs1);
            let dim2 = Dynamic::new(c.ndofs2);
            Self::warmstart_bilateral(c, jacobians, mj_lambda, dim1, dim2);
        }

        for c in constraints.bilateral_ground.iter_mut() {
            Self::warmstart_bilateral_ground(c, jacobians, mj_lambda, Dynamic::new(c.ndofs));
        }
    }

    /// Solve the given set of constraints.
    pub fn solve<N: RealField, Handle: BodyHandle>(
        bodies: &mut dyn BodySet<N, Handle = Handle>,
        contact_constraints: &mut LinearConstraints<N, ContactId>,
        joint_constraints: &mut LinearConstraints<N, usize>,
        internal: &[Handle],
        mj_lambda: &mut DVector<N>,
        jacobians: &[N],
        max_iter: usize,
    ) {
        Self::warmstart_set(bodies, contact_constraints, jacobians, mj_lambda);
        Self::warmstart_set(bodies, joint_constraints, jacobians, mj_lambda);

        for handle in internal {
            if let Some(body) = bodies.get_mut(*handle) {
                let mut dvels = mj_lambda.rows_mut(body.companion_id(), body.ndofs());
                body.warmstart_internal_velocity_constraints(&mut dvels);
            }
        }

        /*
         * Solve.
         */
        for _ in 0..max_iter {
            Self::step(
                bodies,
                contact_constraints,
                joint_constraints,
                internal,
                jacobians,
                mj_lambda,
            )
        }
    }

    fn step_unilateral<N: RealField, Handle: BodyHandle, Id>(
        _bodies: &mut dyn BodySet<N, Handle = Handle>,
        constraints: &mut LinearConstraints<N, Id>,
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
    ) {
        for c in constraints.unilateral.iter_mut() {
            if c.ndofs1 == SPATIAL_DIM && c.ndofs2 == SPATIAL_DIM {
                // Most common case (between two free rigid bodies).
                Self::solve_unilateral(c, jacobians, mj_lambda, SpatialDim {}, SpatialDim {})
            } else {
                let dim1 = Dynamic::new(c.ndofs1);
                let dim2 = Dynamic::new(c.ndofs2);
                Self::solve_unilateral(c, jacobians, mj_lambda, dim1, dim2)
            }
        }

        for c in constraints.unilateral_ground.iter_mut() {
            if c.ndofs == SPATIAL_DIM {
                // Most common case (with one free rigid body).
                // NOTE: it's weird that the compiler requires the { } even though SpatialDim is the
                // alias of a marker type.
                Self::solve_unilateral_ground(c, jacobians, mj_lambda, SpatialDim {})
            } else {
                let dim = Dynamic::new(c.ndofs);
                Self::solve_unilateral_ground(c, jacobians, mj_lambda, dim)
            }
        }
    }

    fn step_bilateral<N: RealField, Handle: BodyHandle, Id>(
        _bodies: &mut dyn BodySet<N, Handle = Handle>,
        constraints: &mut LinearConstraints<N, Id>,
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
    ) {
        for c in constraints.bilateral.iter_mut() {
            if c.ndofs1 == SPATIAL_DIM && c.ndofs2 == SPATIAL_DIM {
                // Most common case (between two free rigid bodies).
                Self::solve_bilateral(
                    c,
                    &constraints.unilateral,
                    jacobians,
                    mj_lambda,
                    SpatialDim {},
                    SpatialDim {},
                )
            } else {
                let dim1 = Dynamic::new(c.ndofs1);
                let dim2 = Dynamic::new(c.ndofs2);
                Self::solve_bilateral(c, &constraints.unilateral, jacobians, mj_lambda, dim1, dim2)
            }
        }

        for c in constraints.bilateral_ground.iter_mut() {
            if c.ndofs == SPATIAL_DIM {
                // Most common case (with one free rigid body).
                Self::solve_bilateral_ground(
                    c,
                    &constraints.unilateral_ground,
                    jacobians,
                    mj_lambda,
                    SpatialDim {},
                )
            } else {
                let dim = Dynamic::new(c.ndofs);
                Self::solve_bilateral_ground(
                    c,
                    &constraints.unilateral_ground,
                    jacobians,
                    mj_lambda,
                    dim,
                )
            }
        }
    }

    fn step<N: RealField, Handle: BodyHandle>(
        bodies: &mut dyn BodySet<N, Handle = Handle>,
        contact_constraints: &mut LinearConstraints<N, ContactId>,
        joint_constraints: &mut LinearConstraints<N, usize>,
        internal: &[Handle],
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
    ) {
        Self::step_bilateral(bodies, joint_constraints, jacobians, mj_lambda);
        Self::step_bilateral(bodies, contact_constraints, jacobians, mj_lambda);

        for handle in internal {
            if let Some(body) = bodies.get_mut(*handle) {
                let mut dvels = mj_lambda.rows_mut(body.companion_id(), body.ndofs());
                body.step_solve_internal_velocity_constraints(&mut dvels);
            }
        }

        Self::step_unilateral(bodies, joint_constraints, jacobians, mj_lambda);
        Self::step_unilateral(bodies, contact_constraints, jacobians, mj_lambda);
    }

    fn solve_unilateral<N: RealField, D1: Dim, D2: Dim, Id>(
        c: &mut UnilateralConstraint<N, Id>,
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
        dim1: D1,
        dim2: D2,
    ) {
        let id1 = c.assembly_id1;
        let id2 = c.assembly_id2;

        let jacobian1 = VectorSliceN::from_slice_generic(&jacobians[c.j_id1..], dim1, U1);
        let jacobian2 = VectorSliceN::from_slice_generic(&jacobians[c.j_id2..], dim2, U1);
        let weighted_jacobian1 = VectorSliceN::from_slice_generic(&jacobians[c.wj_id1..], dim1, U1);
        let weighted_jacobian2 = VectorSliceN::from_slice_generic(&jacobians[c.wj_id2..], dim2, U1);

        let dimpulse = jacobian1.dot(&mj_lambda.rows_generic(id1, dim1))
            + jacobian2.dot(&mj_lambda.rows_generic(id2, dim2))
            + c.rhs;

        let new_impulse = na::sup(&N::zero(), &(c.impulse - c.r * dimpulse));
        let dlambda = new_impulse - c.impulse;

        c.impulse = new_impulse;
        mj_lambda
            .rows_generic_mut(id1, dim1)
            .axpy(dlambda, &weighted_jacobian1, N::one());
        mj_lambda
            .rows_generic_mut(id2, dim2)
            .axpy(dlambda, &weighted_jacobian2, N::one());
    }

    pub fn solve_unilateral_ground<N: RealField, D: Dim, DMJ: Dim, S: StorageMut<N, DMJ>, Id>(
        c: &mut UnilateralGroundConstraint<N, Id>,
        jacobians: &[N],
        mj_lambda: &mut Vector<N, DMJ, S>,
        dim: D,
    ) {
        let jacobian = VectorSliceN::from_slice_generic(&jacobians[c.j_id..], dim, U1);
        let weighted_jacobian = VectorSliceN::from_slice_generic(&jacobians[c.wj_id..], dim, U1);

        let dimpulse = jacobian.dot(&mj_lambda.rows_generic_mut(c.assembly_id, dim)) + c.rhs;

        let new_impulse = na::sup(&N::zero(), &(c.impulse - c.r * dimpulse));
        let dlambda = new_impulse - c.impulse;

        c.impulse = new_impulse;
        mj_lambda
            .rows_generic_mut(c.assembly_id, dim)
            .axpy(dlambda, &weighted_jacobian, N::one());
    }

    fn solve_bilateral<N: RealField, D1: Dim, D2: Dim, Id>(
        c: &mut BilateralConstraint<N, Id>,
        unilateral: &[UnilateralConstraint<N, Id>],
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
        dim1: D1,
        dim2: D2,
    ) {
        let id1 = c.assembly_id1;
        let id2 = c.assembly_id2;

        let min_impulse;
        let max_impulse;

        match c.limits {
            ImpulseLimits::Independent { min, max } => {
                min_impulse = min;
                max_impulse = max;
            }
            ImpulseLimits::Dependent { dependency, coeff } => {
                let impulse = unilateral[dependency].impulse;
                if impulse.is_zero() {
                    if !c.impulse.is_zero() {
                        let wj1 =
                            VectorSliceN::from_slice_generic(&jacobians[c.wj_id1..], dim1, U1);
                        let wj2 =
                            VectorSliceN::from_slice_generic(&jacobians[c.wj_id2..], dim2, U1);

                        mj_lambda
                            .rows_generic_mut(id1, dim1)
                            .axpy(-c.impulse, &wj1, N::one());
                        mj_lambda
                            .rows_generic_mut(id2, dim2)
                            .axpy(-c.impulse, &wj2, N::one());
                        c.impulse = N::zero();
                    }
                    return;
                }
                max_impulse = coeff * impulse;
                min_impulse = -max_impulse;
            }
        }

        let jacobian1 = VectorSliceN::from_slice_generic(&jacobians[c.j_id1..], dim1, U1);
        let jacobian2 = VectorSliceN::from_slice_generic(&jacobians[c.j_id2..], dim2, U1);
        let weighted_jacobian1 = VectorSliceN::from_slice_generic(&jacobians[c.wj_id1..], dim1, U1);
        let weighted_jacobian2 = VectorSliceN::from_slice_generic(&jacobians[c.wj_id2..], dim2, U1);

        let dimpulse = jacobian1.dot(&mj_lambda.rows_generic(id1, dim1))
            + jacobian2.dot(&mj_lambda.rows_generic(id2, dim2))
            + c.rhs;

        let new_impulse = na::clamp(c.impulse - c.r * dimpulse, min_impulse, max_impulse);
        let dlambda = new_impulse - c.impulse;

        c.impulse = new_impulse;
        mj_lambda
            .rows_generic_mut(id1, dim1)
            .axpy(dlambda, &weighted_jacobian1, N::one());
        mj_lambda
            .rows_generic_mut(id2, dim2)
            .axpy(dlambda, &weighted_jacobian2, N::one());
    }

    pub fn solve_bilateral_ground<N: RealField, D: Dim, DMJ: Dim, S: StorageMut<N, DMJ>, Id>(
        c: &mut BilateralGroundConstraint<N, Id>,
        unilateral: &[UnilateralGroundConstraint<N, Id>],
        jacobians: &[N],
        mj_lambda: &mut Vector<N, DMJ, S>,
        dim: D,
    ) {
        let min_impulse;
        let max_impulse;

        match c.limits {
            ImpulseLimits::Independent { min, max } => {
                min_impulse = min;
                max_impulse = max;
            }
            ImpulseLimits::Dependent { dependency, coeff } => {
                let impulse = unilateral[dependency].impulse;
                if impulse.is_zero() {
                    if !c.impulse.is_zero() {
                        let wj = VectorSliceN::from_slice_generic(&jacobians[c.wj_id..], dim, U1);

                        mj_lambda.rows_generic_mut(c.assembly_id, dim).axpy(
                            -c.impulse,
                            &wj,
                            N::one(),
                        );
                        c.impulse = N::zero();
                    }
                    return;
                }
                max_impulse = coeff * impulse;
                min_impulse = -max_impulse;
            }
        }

        let jacobian = VectorSliceN::from_slice_generic(&jacobians[c.j_id..], dim, U1);
        let weighted_jacobian = VectorSliceN::from_slice_generic(&jacobians[c.wj_id..], dim, U1);

        let dimpulse = jacobian.dot(&mj_lambda.rows_generic(c.assembly_id, dim)) + c.rhs;

        let new_impulse = na::clamp(c.impulse - c.r * dimpulse, min_impulse, max_impulse);
        let dlambda = new_impulse - c.impulse;

        c.impulse = new_impulse;
        mj_lambda
            .rows_generic_mut(c.assembly_id, dim)
            .axpy(dlambda, &weighted_jacobian, N::one());
    }

    fn warmstart_unilateral<N: RealField, D1: Dim, D2: Dim, Id>(
        c: &UnilateralConstraint<N, Id>,
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
        dim1: D1,
        dim2: D2,
    ) {
        if !c.impulse.is_zero() {
            let id1 = c.assembly_id1;
            let id2 = c.assembly_id2;

            let weighted_jacobian1 =
                VectorSliceN::from_slice_generic(&jacobians[c.wj_id1..], dim1, U1);
            let weighted_jacobian2 =
                VectorSliceN::from_slice_generic(&jacobians[c.wj_id2..], dim2, U1);

            mj_lambda
                .rows_generic_mut(id1, dim1)
                .axpy(c.impulse, &weighted_jacobian1, N::one());
            mj_lambda
                .rows_generic_mut(id2, dim2)
                .axpy(c.impulse, &weighted_jacobian2, N::one());
        }
    }

    pub fn warmstart_unilateral_ground<
        N: RealField,
        D: Dim,
        DMJ: Dim,
        S: StorageMut<N, DMJ>,
        Id,
    >(
        c: &UnilateralGroundConstraint<N, Id>,
        jacobians: &[N],
        mj_lambda: &mut Vector<N, DMJ, S>,
        dim: D,
    ) {
        if !c.impulse.is_zero() {
            let weighted_jacobian =
                VectorSliceN::from_slice_generic(&jacobians[c.wj_id..], dim, U1);

            mj_lambda.rows_generic_mut(c.assembly_id, dim).axpy(
                c.impulse,
                &weighted_jacobian,
                N::one(),
            );
        }
    }

    fn warmstart_bilateral<N: RealField, D1: Dim, D2: Dim, Id>(
        c: &BilateralConstraint<N, Id>,
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
        dim1: D1,
        dim2: D2,
    ) {
        if !c.impulse.is_zero() {
            let id1 = c.assembly_id1;
            let id2 = c.assembly_id2;

            let weighted_jacobian1 =
                VectorSliceN::from_slice_generic(&jacobians[c.wj_id1..], dim1, U1);
            let weighted_jacobian2 =
                VectorSliceN::from_slice_generic(&jacobians[c.wj_id2..], dim2, U1);

            mj_lambda
                .rows_generic_mut(id1, dim1)
                .axpy(c.impulse, &weighted_jacobian1, N::one());
            mj_lambda
                .rows_generic_mut(id2, dim2)
                .axpy(c.impulse, &weighted_jacobian2, N::one());
        }
    }

    pub fn warmstart_bilateral_ground<N: RealField, D: Dim, DMJ: Dim, S: StorageMut<N, DMJ>, Id>(
        c: &BilateralGroundConstraint<N, Id>,
        jacobians: &[N],
        mj_lambda: &mut Vector<N, DMJ, S>,
        dim: D,
    ) {
        if !c.impulse.is_zero() {
            let weighted_jacobian =
                VectorSliceN::from_slice_generic(&jacobians[c.wj_id..], dim, U1);

            mj_lambda.rows_generic_mut(c.assembly_id, dim).axpy(
                c.impulse,
                &weighted_jacobian,
                N::one(),
            );
        }
    }
}
