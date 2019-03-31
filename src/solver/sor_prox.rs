use na::{self, DVector, Dim, Dynamic, RealField, U1, VectorSliceN, Vector};
use na::storage::StorageMut;

// FIXME: could we just merge UnilateralConstraint and Bilateral constraint into a single structure
// without performance impact due to clamping?
use crate::math::{SpatialDim, SPATIAL_DIM};
use crate::object::{BodySet, BodyHandle};
use crate::solver::{BilateralConstraint, BilateralGroundConstraint, ImpulseLimits, UnilateralConstraint,
             UnilateralGroundConstraint};

/// A SOR-Prox velocity-based constraints solver.
pub(crate) struct SORProx;

impl SORProx {
    /// Solve the given set of constraints.
    pub fn solve<N: RealField>(
        bodies: &mut BodySet<N>,
        unilateral_ground: &mut [UnilateralGroundConstraint<N>],
        unilateral: &mut [UnilateralConstraint<N>],
        bilateral_ground: &mut [BilateralGroundConstraint<N>],
        bilateral: &mut [BilateralConstraint<N>],
        internal: &[BodyHandle],
        mj_lambda: &mut DVector<N>,
        jacobians: &[N],
        max_iter: usize,
    ) {
        /*
         * Setup constraints.
         */
        for c in unilateral.iter_mut() {
            let dim1 = Dynamic::new(c.ndofs1);
            let dim2 = Dynamic::new(c.ndofs2);
            Self::warmstart_unilateral(c, jacobians, mj_lambda, dim1, dim2);
        }

        for c in unilateral_ground.iter_mut() {
            let dim = Dynamic::new(c.ndofs);
            Self::warmstart_unilateral_ground(c, jacobians, mj_lambda, dim);
        }

        for c in bilateral.iter_mut() {
            let dim1 = Dynamic::new(c.ndofs1);
            let dim2 = Dynamic::new(c.ndofs2);
            Self::warmstart_bilateral(c, jacobians, mj_lambda, dim1, dim2);
        }

        for c in bilateral_ground.iter_mut() {
            Self::warmstart_bilateral_ground(c, jacobians, mj_lambda, Dynamic::new(c.ndofs));
        }

        for handle in internal {
            if let Some(body) = bodies.body_mut(*handle) {
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
                unilateral_ground,
                unilateral,
                bilateral_ground,
                bilateral,
                internal,
                jacobians,
                mj_lambda,
            )
        }
    }

    fn step<N: RealField>(
        bodies: &mut BodySet<N>,
        unilateral_ground: &mut [UnilateralGroundConstraint<N>],
        unilateral: &mut [UnilateralConstraint<N>],
        bilateral_ground: &mut [BilateralGroundConstraint<N>],
        bilateral: &mut [BilateralConstraint<N>],
        internal: &[BodyHandle],
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
    ) {
        for c in unilateral.iter_mut() {
            if c.ndofs1 == SPATIAL_DIM && c.ndofs2 == SPATIAL_DIM {
                // Most common case (between two free rigid bodies).
                Self::solve_unilateral(c, jacobians, mj_lambda, SpatialDim {}, SpatialDim {})
            } else {
                let dim1 = Dynamic::new(c.ndofs1);
                let dim2 = Dynamic::new(c.ndofs2);
                Self::solve_unilateral(c, jacobians, mj_lambda, dim1, dim2)
            }
        }

        for c in unilateral_ground.iter_mut() {
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

        for c in bilateral.iter_mut() {
            if c.ndofs1 == SPATIAL_DIM && c.ndofs2 == SPATIAL_DIM {
                // Most common case (between two free rigid bodies).
                Self::solve_bilateral(
                    c,
                    unilateral,
                    jacobians,
                    mj_lambda,
                    SpatialDim {},
                    SpatialDim {},
                )
            } else {
                let dim1 = Dynamic::new(c.ndofs1);
                let dim2 = Dynamic::new(c.ndofs2);
                Self::solve_bilateral(c, unilateral, jacobians, mj_lambda, dim1, dim2)
            }
        }

        for c in bilateral_ground.iter_mut() {
            if c.ndofs == SPATIAL_DIM {
                // Most common case (with one free rigid body).
                Self::solve_bilateral_ground(
                    c,
                    unilateral_ground,
                    jacobians,
                    mj_lambda,
                    SpatialDim {},
                )
            } else {
                let dim = Dynamic::new(c.ndofs);
                Self::solve_bilateral_ground(c, unilateral_ground, jacobians, mj_lambda, dim)
            }
        }

        for handle in internal {
            if let Some(body) = bodies.body_mut(*handle) {
                let mut dvels = mj_lambda.rows_mut(body.companion_id(), body.ndofs());
                body.step_solve_internal_velocity_constraints(&mut dvels);
            }
        }
    }

    fn solve_unilateral<N: RealField, D1: Dim, D2: Dim>(
        c: &mut UnilateralConstraint<N>,
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
            + jacobian2.dot(&mj_lambda.rows_generic(id2, dim2)) + c.rhs;

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

    pub fn solve_unilateral_ground<N: RealField, D: Dim, DMJ: Dim, S: StorageMut<N, DMJ>>(
        c: &mut UnilateralGroundConstraint<N>,
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

    fn solve_bilateral<N: RealField, D1: Dim, D2: Dim>(
        c: &mut BilateralConstraint<N>,
        unilateral: &[UnilateralConstraint<N>],
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
            + jacobian2.dot(&mj_lambda.rows_generic(id2, dim2)) + c.rhs;

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

    pub fn solve_bilateral_ground<N: RealField, D: Dim, DMJ: Dim, S: StorageMut<N, DMJ>>(
        c: &mut BilateralGroundConstraint<N>,
        unilateral: &[UnilateralGroundConstraint<N>],
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

    fn warmstart_unilateral<N: RealField, D1: Dim, D2: Dim>(
        c: &UnilateralConstraint<N>,
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

    pub fn warmstart_unilateral_ground<N: RealField, D: Dim,  DMJ: Dim, S: StorageMut<N, DMJ>>(
        c: &UnilateralGroundConstraint<N>,
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

    fn warmstart_bilateral<N: RealField, D1: Dim, D2: Dim>(
        c: &BilateralConstraint<N>,
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

    pub fn warmstart_bilateral_ground<N: RealField, D: Dim, DMJ: Dim, S: StorageMut<N, DMJ>>(
        c: &BilateralGroundConstraint<N>,
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
