use std::marker::PhantomData;

use na::{self, DVector, Dim, Dynamic, Real, U1, VectorSliceN};

// FIXME: could we just merge UnilateralConstraint and Bilateral constraint into a single structure
// without performance impact due to clamping?
use solver::{BilateralConstraint, BilateralGroundConstraint, ImpulseLimits, UnilateralConstraint,
             UnilateralGroundConstraint};
use math::{SpatialDim, SPATIAL_DIM};

pub struct SORProx<N: Real> {
    _phantom: PhantomData<N>,
}

impl<N: Real> SORProx<N> {
    pub fn new() -> Self {
        SORProx {
            _phantom: PhantomData,
        }
    }

    pub fn solve(
        &self,
        unilateral_ground: &mut [UnilateralGroundConstraint<N>],
        unilateral: &mut [UnilateralConstraint<N>],
        bilateral_ground: &mut [BilateralGroundConstraint<N>],
        bilateral: &mut [BilateralConstraint<N>],
        mj_lambda: &mut DVector<N>,
        jacobians: &[N],
        max_iter: usize,
    ) {
        /*
         * Setup constraints.
         */
        let mut warmstarted = 0;
        for c in unilateral.iter_mut() {
            let dim1 = Dynamic::new(c.ndofs1);
            let dim2 = Dynamic::new(c.ndofs2);
            self.setup_unilateral(c, jacobians, mj_lambda, dim1, dim2);
            if c.impulse != N::zero() {
                warmstarted += 1;
            }
        }

        for c in unilateral_ground.iter_mut() {
            let dim = Dynamic::new(c.ndofs);
            self.setup_unilateral_ground(c, jacobians, mj_lambda, dim);
            if c.impulse != N::zero() {
                warmstarted += 1;
            }
        }

        for c in bilateral.iter_mut() {
            let dim1 = Dynamic::new(c.ndofs1);
            let dim2 = Dynamic::new(c.ndofs2);
            self.setup_bilateral(c, jacobians, mj_lambda, dim1, dim2);
            if c.impulse != N::zero() {
                warmstarted += 1;
            }
        }

        for c in bilateral_ground.iter_mut() {
            self.setup_bilateral_ground(c, jacobians, mj_lambda, Dynamic::new(c.ndofs));
            if c.impulse != N::zero() {
                warmstarted += 1;
            }
        }

        /*
         * Solve.
         */
        for _ in 0..max_iter {
            self.step(
                unilateral_ground,
                unilateral,
                bilateral_ground,
                bilateral,
                jacobians,
                mj_lambda,
            )
        }
    }

    pub fn step(
        &self,
        unilateral_ground: &mut [UnilateralGroundConstraint<N>],
        unilateral: &mut [UnilateralConstraint<N>],
        bilateral_ground: &mut [BilateralGroundConstraint<N>],
        bilateral: &mut [BilateralConstraint<N>],
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
    ) {
        for c in unilateral.iter_mut() {
            if c.ndofs1 == SPATIAL_DIM && c.ndofs2 == SPATIAL_DIM {
                // Most common case (between two free rigid bodies).
                self.solve_unilateral(c, jacobians, mj_lambda, SpatialDim {}, SpatialDim {})
            } else {
                let dim1 = Dynamic::new(c.ndofs1);
                let dim2 = Dynamic::new(c.ndofs2);
                self.solve_unilateral(c, jacobians, mj_lambda, dim1, dim2)
            }
        }

        for c in unilateral_ground.iter_mut() {
            if c.ndofs == SPATIAL_DIM {
                // Most common case (with one free rigid body).
                // NOTE: it's weird that the compiler requires the { } even thouh SpatialDim is the
                // alias of a marker type.
                self.solve_unilateral_ground(c, jacobians, mj_lambda, SpatialDim {})
            } else {
                let dim = Dynamic::new(c.ndofs);
                self.solve_unilateral_ground(c, jacobians, mj_lambda, dim)
            }
        }

        for c in bilateral.iter_mut() {
            if c.ndofs1 == SPATIAL_DIM && c.ndofs2 == SPATIAL_DIM {
                // Most common case (between two free rigid bodies).
                self.solve_bilateral(
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
                self.solve_bilateral(c, unilateral, jacobians, mj_lambda, dim1, dim2)
            }
        }

        for c in bilateral_ground.iter_mut() {
            if c.ndofs == SPATIAL_DIM {
                // Most common case (with one free rigid body).
                self.solve_bilateral_ground(
                    c,
                    unilateral_ground,
                    jacobians,
                    mj_lambda,
                    SpatialDim {},
                )
            } else {
                let dim = Dynamic::new(c.ndofs);
                self.solve_bilateral_ground(c, unilateral_ground, jacobians, mj_lambda, dim)
            }
        }
    }

    pub fn solve_unilateral<D1: Dim, D2: Dim>(
        &self,
        c: &mut UnilateralConstraint<N>,
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
        dim1: D1,
        dim2: D2,
    ) {
        let id1 = c.assembly_id1;
        let id2 = c.assembly_id2;

        let jacobian1 = VectorSliceN::new_generic(&jacobians[c.jacobian_id1..], dim1, U1);
        let jacobian2 = VectorSliceN::new_generic(&jacobians[c.jacobian_id2..], dim2, U1);
        let weighted_jacobian1 =
            VectorSliceN::new_generic(&jacobians[c.weighted_jacobian_id1..], dim1, U1);
        let weighted_jacobian2 =
            VectorSliceN::new_generic(&jacobians[c.weighted_jacobian_id2..], dim2, U1);

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

    pub fn solve_unilateral_ground<D: Dim>(
        &self,
        c: &mut UnilateralGroundConstraint<N>,
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
        dim: D,
    ) {
        let jacobian = VectorSliceN::new_generic(&jacobians[c.jacobian_id..], dim, U1);
        let weighted_jacobian =
            VectorSliceN::new_generic(&jacobians[c.weighted_jacobian_id..], dim, U1);

        let dimpulse = jacobian.dot(&mj_lambda.rows_generic_mut(c.assembly_id, dim)) + c.rhs;

        let new_impulse = na::sup(&N::zero(), &(c.impulse - c.r * dimpulse));
        let dlambda = new_impulse - c.impulse;

        c.impulse = new_impulse;
        mj_lambda
            .rows_generic_mut(c.assembly_id, dim)
            .axpy(dlambda, &weighted_jacobian, N::one());
    }

    pub fn solve_bilateral<D1: Dim, D2: Dim>(
        &self,
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
                        let wj1 = VectorSliceN::new_generic(
                            &jacobians[c.weighted_jacobian_id1..],
                            dim1,
                            U1,
                        );
                        let wj2 = VectorSliceN::new_generic(
                            &jacobians[c.weighted_jacobian_id2..],
                            dim2,
                            U1,
                        );

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

        let jacobian1 = VectorSliceN::new_generic(&jacobians[c.jacobian_id1..], dim1, U1);
        let jacobian2 = VectorSliceN::new_generic(&jacobians[c.jacobian_id2..], dim2, U1);
        let weighted_jacobian1 =
            VectorSliceN::new_generic(&jacobians[c.weighted_jacobian_id1..], dim1, U1);
        let weighted_jacobian2 =
            VectorSliceN::new_generic(&jacobians[c.weighted_jacobian_id2..], dim2, U1);

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

    pub fn solve_bilateral_ground<D: Dim>(
        &self,
        c: &mut BilateralGroundConstraint<N>,
        unilateral: &[UnilateralGroundConstraint<N>],
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
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
                        let wj = VectorSliceN::new_generic(
                            &jacobians[c.weighted_jacobian_id..],
                            dim,
                            U1,
                        );

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

        let jacobian = VectorSliceN::new_generic(&jacobians[c.jacobian_id..], dim, U1);
        let weighted_jacobian =
            VectorSliceN::new_generic(&jacobians[c.weighted_jacobian_id..], dim, U1);

        let dimpulse = jacobian.dot(&mj_lambda.rows_generic(c.assembly_id, dim)) + c.rhs;

        let new_impulse = na::clamp(c.impulse - c.r * dimpulse, min_impulse, max_impulse);
        let dlambda = new_impulse - c.impulse;

        c.impulse = new_impulse;
        mj_lambda
            .rows_generic_mut(c.assembly_id, dim)
            .axpy(dlambda, &weighted_jacobian, N::one());
    }

    pub fn setup_unilateral<D1: Dim, D2: Dim>(
        &self,
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
                VectorSliceN::new_generic(&jacobians[c.weighted_jacobian_id1..], dim1, U1);
            let weighted_jacobian2 =
                VectorSliceN::new_generic(&jacobians[c.weighted_jacobian_id2..], dim2, U1);

            mj_lambda
                .rows_generic_mut(id1, dim1)
                .axpy(c.impulse, &weighted_jacobian1, N::one());
            mj_lambda
                .rows_generic_mut(id2, dim2)
                .axpy(c.impulse, &weighted_jacobian2, N::one());
        }
    }

    pub fn setup_unilateral_ground<D: Dim>(
        &self,
        c: &UnilateralGroundConstraint<N>,
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
        dim: D,
    ) {
        if !c.impulse.is_zero() {
            let weighted_jacobian =
                VectorSliceN::new_generic(&jacobians[c.weighted_jacobian_id..], dim, U1);

            mj_lambda.rows_generic_mut(c.assembly_id, dim).axpy(
                c.impulse,
                &weighted_jacobian,
                N::one(),
            );
        }
    }

    pub fn setup_bilateral<D1: Dim, D2: Dim>(
        &self,
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
                VectorSliceN::new_generic(&jacobians[c.weighted_jacobian_id1..], dim1, U1);
            let weighted_jacobian2 =
                VectorSliceN::new_generic(&jacobians[c.weighted_jacobian_id2..], dim2, U1);

            mj_lambda
                .rows_generic_mut(id1, dim1)
                .axpy(c.impulse, &weighted_jacobian1, N::one());
            mj_lambda
                .rows_generic_mut(id2, dim2)
                .axpy(c.impulse, &weighted_jacobian2, N::one());
        }
    }

    pub fn setup_bilateral_ground<D: Dim>(
        &self,
        c: &BilateralGroundConstraint<N>,
        jacobians: &[N],
        mj_lambda: &mut DVector<N>,
        dim: D,
    ) {
        if !c.impulse.is_zero() {
            let weighted_jacobian =
                VectorSliceN::new_generic(&jacobians[c.weighted_jacobian_id..], dim, U1);

            mj_lambda.rows_generic_mut(c.assembly_id, dim).axpy(
                c.impulse,
                &weighted_jacobian,
                N::one(),
            );
        }
    }
}
