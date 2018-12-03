use na::{self, Dim, Dynamic, Real, U1, VectorSliceMutN};
use slab::Slab;
use std::marker::PhantomData;
use std::ops::MulAssign;

use world::CollisionWorld;
use joint::JointConstraint;
use object::{BodySet, ColliderAnchor, BodyHandle};
use solver::helper;
use solver::{ForceDirection, IntegrationParameters,
             MultibodyJointLimitsNonlinearConstraintGenerator, NonlinearConstraintGenerator,
             NonlinearUnilateralConstraint};
use math::Isometry;

/// Non-linear position-based constraint solver using the SOR-Prox approach.
pub struct NonlinearSORProx<N: Real> {
    _phantom: PhantomData<N>,
}

impl<N: Real> NonlinearSORProx<N> {
    /// Initialize a new nonlinear SOR-Prox solver.
    pub fn new() -> Self {
        NonlinearSORProx {
            _phantom: PhantomData,
        }
    }

    /// Solve a set of nonlinear position-based constraints.
    pub fn solve(
        &self,
        params: &IntegrationParameters<N>,
        cworld: &CollisionWorld<N>,
        bodies: &mut BodySet<N>,
        constraints: &mut [NonlinearUnilateralConstraint<N>],
        multibody_limits: &[MultibodyJointLimitsNonlinearConstraintGenerator],
        joints_constraints: &Slab<Box<JointConstraint<N>>>, // FIXME: ugly, use a slice of refs instead.
        internal_constraints: &[BodyHandle],
        jacobians: &mut [N],
        max_iter: usize,
    ) {
        for _ in 0..max_iter {
            for constraint in constraints.iter_mut() {
                // FIXME: specialize for SPATIAL_DIM.
                let dim1 = Dynamic::new(constraint.ndofs1);
                let dim2 = Dynamic::new(constraint.ndofs2);
                self.solve_unilateral(params, cworld, bodies, constraint, jacobians, dim1, dim2);
            }

            for generator in multibody_limits {
                self.solve_generic(params, bodies, generator, jacobians)
            }

            for joint in &*joints_constraints {
                self.solve_generic(params, bodies, &**joint.1, jacobians)
            }

            for constraint in internal_constraints {
                let body = bodies.body_mut(*constraint);
                body.step_solve_internal_position_constraints(params);
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
                    if false {
                        // constraint.r > params.max_stabilization_multiplier {
                        constraint.r = params.max_stabilization_multiplier;
                    }

                    if rhs < N::zero() {
                        let impulse = -rhs * constraint.r;

                        VectorSliceMutN::from_slice_generic(
                            &mut jacobians[constraint.wj_id1..],
                            dim1,
                            U1,
                        ).mul_assign(impulse);

                        VectorSliceMutN::from_slice_generic(
                            &mut jacobians[constraint.wj_id2..],
                            dim2,
                            U1,
                        ).mul_assign(impulse);

                        // FIXME: the body update should be performed lazily, especially because
                        // we dont actually need to update the kinematic of a multibody until
                        // we have to solve a contact involvoing one of its links.
                        bodies.body_mut(constraint.body1.body_handle).apply_displacement(
                            &jacobians[constraint.wj_id1..constraint.wj_id1 + constraint.dim1],
                        );
                        bodies.body_mut(constraint.body2.body_handle).apply_displacement(
                            &jacobians[constraint.wj_id2..constraint.wj_id2 + constraint.dim2],
                        );
                    }
                }
        }
    }

    fn solve_unilateral<D1: Dim, D2: Dim>(
        &self,
        params: &IntegrationParameters<N>,
        cworld: &CollisionWorld<N>,
        bodies: &mut BodySet<N>,
        constraint: &mut NonlinearUnilateralConstraint<N>,
        jacobians: &mut [N],
        dim1: D1,
        dim2: D2,
    ) {
        if self.update_contact_constraint(params, cworld, bodies, constraint, jacobians) {
            let impulse = -constraint.rhs * constraint.r;

            VectorSliceMutN::from_slice_generic(jacobians, dim1, U1).mul_assign(impulse);
            VectorSliceMutN::from_slice_generic(&mut jacobians[dim1.value()..], dim2, U1)
                .mul_assign(impulse);

            if dim1.value() != 0 {
                bodies
                    .body_mut(constraint.body1.body_handle)
                    .apply_displacement(&jacobians[0..dim1.value()]);
            }
            if dim2.value() != 0 {
                bodies
                    .body_mut(constraint.body2.body_handle)
                    .apply_displacement(&jacobians[dim1.value()..dim1.value() + dim2.value()]);
            }
        }
    }

    fn update_contact_constraint(
        &self,
        params: &IntegrationParameters<N>,
        cworld: &CollisionWorld<N>,
        bodies: &BodySet<N>,
        constraint: &mut NonlinearUnilateralConstraint<N>,
        jacobians: &mut [N],
    ) -> bool {
        let body1 = bodies.body(constraint.body1.body_handle);
        let body2 = bodies.body(constraint.body2.body_handle);
        let part1 = body1.part(constraint.body1);
        let part2 = body2.part(constraint.body2);

        let collider1 = cworld.collision_object(constraint.collider1).unwrap();
        let collider2 = cworld.collision_object(constraint.collider2).unwrap();

        let pos1;
        let pos2;
        let coords1;
        let coords2;

        match collider1.data().anchor() {
            ColliderAnchor::OnDeformableBody { indices, .. } => {
                let coords = body1.deformed_positions().unwrap().1;
                collider1.shape().as_deformable_shape().unwrap().update_local_approximation(
                    coords,
                    indices.as_ref().map(|idx| &idx[..]),
                    constraint.kinematic.approx1_mut());
                // FIXME: is this really the identity?
                pos1 = Isometry::identity();
                coords1 = Some(coords);
            }
            ColliderAnchor::OnBodyPart { position_wrt_body_part, .. } => {
                pos1 = part1.position() * position_wrt_body_part;
                coords1 = None;
            }
        }

        match collider2.data().anchor() {
            ColliderAnchor::OnDeformableBody { indices, .. } => {
                let coords = body2.deformed_positions().unwrap().1;
                collider2.shape().as_deformable_shape().unwrap().update_local_approximation(
                    coords,
                    indices.as_ref().map(|idx| &idx[..]),
                    constraint.kinematic.approx2_mut());
                // FIXME: is this really the identity?
                pos2 = Isometry::identity();
                coords2 = Some(coords);
            }
            ColliderAnchor::OnBodyPart { position_wrt_body_part, .. } => {
                pos2 = part2.position() * position_wrt_body_part;
                coords2 = None;
            }
        }


        if let Some(contact) = constraint
            .kinematic
            .contact(&pos1, &**collider1.shape(), coords1, &pos2, &**collider2.shape(), coords2, &constraint.normal1)
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
                    body1.fill_constraint_geometry(
                        part1,
                        constraint.ndofs1,
                        &contact.world1,
                        &ForceDirection::Linear(-contact.normal),
                        j_id1,
                        0,
                        jacobians,
                        &mut inv_r,
                        None,
                        None
                    );
                }

                if constraint.ndofs2 != 0 {
                    body2.fill_constraint_geometry(
                        part2,
                        constraint.ndofs2,
                        &contact.world2,
                        &ForceDirection::Linear(contact.normal),
                        j_id2,
                        constraint.ndofs1,
                        jacobians,
                        &mut inv_r,
                        None,
                        None
                    );
                }

                // Avoid overshoot when the penetration vector is close to the null-space
                // of a multibody link jacobian.
                // FIXME: will this cause issue with very heavy objects?
                // Should this be done depending on the jacobian magnitude instead
                // (instead of JM-1J)?

                // let j1 = DVectorSlice::from_slice(&jacobians[j_id1..], constraint.ndofs1);
                // let j2 = DVectorSlice::from_slice(&jacobians[j_id2..], constraint.ndofs2);

                if false {
                    // j1.dot(&j1) + j2.dot(&j2) < N::one() / params.max_stabilization_multiplier {
                    constraint.r = params.max_stabilization_multiplier;
                } else {
                    if inv_r == N::zero() {
                        return false;
                    }
                    constraint.r = N::one() / inv_r
                }

                true
            } else {
            false
        }
    }
}
