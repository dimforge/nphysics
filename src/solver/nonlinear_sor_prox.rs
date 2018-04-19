use std::marker::PhantomData;
use std::ops::MulAssign;
use approx::ApproxEq;
use slab::Slab;
use alga::linear::Transformation;
use alga::linear::ProjectiveTransformation;
use na::{self, DVector, Dim, Dynamic, Real, U1, Unit, VectorSliceMutN, DVectorSlice};
use ncollide::query::ContactKinematic;
use ncollide::query::closest_points_internal;

use object::{BodyHandle, BodyPart, BodySet};
use joint::JointConstraint;
use solver::helper;
use solver::{ForceDirection, IntegrationParameters,
             MultibodyJointLimitsNonlinearConstraintGenerator, NonlinearConstraintGenerator,
             NonlinearUnilateralConstraint};
use math::{Isometry, Point, Rotation, Vector};

pub struct NonlinearSORProx<N: Real> {
    _phantom: PhantomData<N>,
}

impl<N: Real> NonlinearSORProx<N> {
    pub fn new() -> Self {
        NonlinearSORProx {
            _phantom: PhantomData,
        }
    }

    pub fn solve(
        &self,
        params: &IntegrationParameters<N>,
        bodies: &mut BodySet<N>,
        constraints: &mut [NonlinearUnilateralConstraint<N>],
        multibody_limits: &[MultibodyJointLimitsNonlinearConstraintGenerator],
        joints_constraints: &Slab<Box<JointConstraint<N>>>, // FIXME: ugly, use a slice of refs instead.
        jacobians: &mut [N],
        max_iter: usize,
    ) {
        for _ in 0..max_iter {
            for constraint in constraints.iter_mut() {
                // FIXME: specialize for SPATIAL_DIM.
                let dim1 = Dynamic::new(constraint.ndofs1);
                let dim2 = Dynamic::new(constraint.ndofs2);
                self.solve_unilateral(params, bodies, constraint, jacobians, dim1, dim2);
            }

            for generator in multibody_limits {
                self.solve_generic(params, bodies, generator, jacobians)
            }

            for joint in &*joints_constraints {
                self.solve_generic(params, bodies, &**joint.1, jacobians)
            }
        }
    }

    fn solve_generic<Gen: ?Sized + NonlinearConstraintGenerator<N>>(
        &self,
        params: &IntegrationParameters<N>,
        bodies: &mut BodySet<N>,
        generator: &Gen,
        jacobians: &mut [N],
    ) {
        let nconstraints = generator.num_position_constraints(bodies);

        for i in 0..nconstraints {
            if let Some(mut constraint) =
                generator.position_constraint(params, i, bodies, jacobians)
            {
                let dim1 = Dynamic::new(constraint.dim1);
                let dim2 = Dynamic::new(constraint.dim2);

                let rhs = if constraint.is_angular {
                    na::sup(
                        &((constraint.rhs + params.allowed_angular_error) * params.erp),
                        &(-params.max_angular_correction),
                    )
                } else {
                    na::sup(
                        &((constraint.rhs + params.allowed_linear_error) * params.erp),
                        &(-params.max_linear_correction),
                    )
                };

                // Avoid overshoot when the penetration vector is close to the null-space
                // of a multibody link jacobian.
                // FIXME: will this cause issue with very light objects?
                // Should this be done depending on the jacobian magnitude instead
                // (instead of JM-1J)?
                if false { // constraint.r > params.max_stabilization_multiplier {
                    constraint.r = params.max_stabilization_multiplier;
                }

                if rhs < N::zero() {
                    let impulse = -rhs * constraint.r;

                    VectorSliceMutN::new_generic_mut(&mut jacobians[constraint.wj_id1..], dim1, U1)
                        .mul_assign(impulse);

                    VectorSliceMutN::new_generic_mut(&mut jacobians[constraint.wj_id2..], dim2, U1)
                        .mul_assign(impulse);

                    // FIXME: the body update should be performed lazily, especially because
                    // we dont actually need to update the kinematic of a multibody until
                    // we have to solve a contact involvoing one of its links.
                    bodies.body_mut(constraint.body1).apply_displacement(
                        &jacobians[constraint.wj_id1..constraint.wj_id1 + constraint.dim1],
                    );
                    bodies.body_mut(constraint.body2).apply_displacement(
                        &jacobians[constraint.wj_id2..constraint.wj_id2 + constraint.dim2],
                    );
                }
            }
        }
    }

    fn solve_unilateral<D1: Dim, D2: Dim>(
        &self,
        params: &IntegrationParameters<N>,
        bodies: &mut BodySet<N>,
        constraint: &mut NonlinearUnilateralConstraint<N>,
        jacobians: &mut [N],
        dim1: D1,
        dim2: D2,
    ) {
        if self.update_contact_constraint(params, bodies, constraint, jacobians) {
            let impulse = -constraint.rhs * constraint.r;

            VectorSliceMutN::new_generic_mut(jacobians, dim1, U1).mul_assign(impulse);
            VectorSliceMutN::new_generic_mut(&mut jacobians[dim1.value()..], dim2, U1)
                .mul_assign(impulse);

            if dim1.value() != 0 {
                bodies
                    .body_mut(constraint.body1)
                    .apply_displacement(&jacobians[0..dim1.value()]);
            }
            if dim2.value() != 0 {
                bodies
                    .body_mut(constraint.body2)
                    .apply_displacement(&jacobians[dim1.value()..dim1.value() + dim2.value()]);
            }
        }
    }

    fn update_contact_constraint(
        &self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        constraint: &mut NonlinearUnilateralConstraint<N>,
        jacobians: &mut [N],
    ) -> bool {
        let body1 = bodies.body_part(constraint.body1);
        let body2 = bodies.body_part(constraint.body2);
        let pos1 = body1.position();
        let pos2 = body2.position();

        if let Some(contact) = constraint
            .kinematic
            .contact(&pos1, &pos2, &constraint.normal1)
        {
            constraint.rhs = na::sup(
                &((-contact.depth + params.allowed_linear_error) * params.erp),
                &(-params.max_linear_correction),
            );

            if constraint.rhs >= N::zero() {
                return false;
            }

            // XXX: should use constraint_pair_geometry to properly handle multibodies.
            let mut inv_r = N::zero();
            let j_id1 = constraint.ndofs1 + constraint.ndofs2;
            let j_id2 = (constraint.ndofs1 * 2) + constraint.ndofs2;

            if constraint.ndofs1 != 0 {
                helper::fill_constraint_geometry(
                    &body1,
                    constraint.ndofs1,
                    &contact.world1,
                    &ForceDirection::Linear(-contact.normal),
                    j_id1,
                    0,
                    jacobians,
                    &mut inv_r,
                );
            }

            if constraint.ndofs2 != 0 {
                helper::fill_constraint_geometry(
                    &body2,
                    constraint.ndofs2,
                    &contact.world2,
                    &ForceDirection::Linear(contact.normal),
                    j_id2,
                    constraint.ndofs1,
                    jacobians,
                    &mut inv_r,
                );
            }

            // Avoid overshoot when the penetration vector is close to the null-space
            // of a multibody link jacobian.
            // FIXME: will this cause issue with very heavy objects?
            // Should this be done depending on the jacobian magnitude instead
            // (instead of JM-1J)?

            // let j1 = DVectorSlice::new(&jacobians[j_id1..], constraint.ndofs1);
            // let j2 = DVectorSlice::new(&jacobians[j_id2..], constraint.ndofs2);

            if false { // j1.dot(&j1) + j2.dot(&j2) < N::one() / params.max_stabilization_multiplier {
                constraint.r = params.max_stabilization_multiplier;
            } else {
                constraint.r = N::one() / inv_r
            }

            true
        } else {
            false
        }
    }
}
