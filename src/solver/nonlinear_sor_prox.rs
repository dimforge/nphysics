use std::marker::PhantomData;
use std::ops::MulAssign;
use approx::ApproxEq;
use slab::Slab;
use alga::linear::Transformation;
use alga::linear::ProjectiveTransformation;
use na::{self, DVector, Dim, Dynamic, Real, U1, Unit, VectorSliceMutN};
use ncollide::query::ContactKinematic;
use ncollide::query::closest_points_internal;

use object::{BodyHandle, BodyPart, BodySet};
use joint::JointConstraint;
use solver::helper;
use solver::{ForceDirection, IntegrationParameters,
             MultibodyJointLimitsNonlinearConstraintGenerator, NonlinearConstraintGenerator,
             NonlinearUnilateralConstraint};
use math::{Isometry, Point, Rotation, Vector};

struct ContactGeometry<N: Real> {
    world1: Point<N>,
    world2: Point<N>,
    normal: Unit<Vector<N>>,
    depth: N,
}

impl<N: Real> ContactGeometry<N> {
    pub fn new(world1: Point<N>, world2: Point<N>, normal: Unit<Vector<N>>, depth: N) -> Self {
        ContactGeometry {
            world1,
            world2,
            normal,
            depth,
        }
    }
}

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
            if let Some(constraint) = generator.position_constraint(params, i, bodies, jacobians) {
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

            bodies
                .body_mut(constraint.body1)
                .apply_displacement(&jacobians[0..dim1.value()]);
            bodies
                .body_mut(constraint.body2)
                .apply_displacement(&jacobians[dim1.value()..dim1.value() + dim2.value()]);
        }
    }

    fn compute_contact_geometry(
        &self,
        body1: &BodyPart<N>,
        body2: &BodyPart<N>,
        constraint: &NonlinearUnilateralConstraint<N>,
    ) -> ContactGeometry<N> {
        let m1 = body1.position();
        let m2 = body2.position();

        let mut world1 = m1.transform_point(&constraint.local1);
        let mut world2 = m2.transform_point(&constraint.local2);
        let normal;
        let mut depth;

        match constraint.kinematic {
            ContactKinematic::PlanePoint => {
                normal = m1 * constraint.normal1;
                depth = -na::dot(normal.as_ref(), &(world2 - world1));
                world1 = world2 + normal.as_ref() * depth;
            }
            ContactKinematic::PointPlane => {
                let world_normal2 = m2 * constraint.normal2.as_ref();
                depth = -na::dot(&world_normal2, &(world1 - world2));
                world2 = world1 + &world_normal2 * depth;
                normal = Unit::new_unchecked(-world_normal2);
            }
            ContactKinematic::PointPoint => {
                if let Some((n, d)) = Unit::try_new_and_get(world2 - world1, N::default_epsilon()) {
                    depth = -d;
                    normal = n;
                } else {
                    depth = -constraint.margin1 - constraint.margin2;
                    normal = m1 * constraint.normal1;
                }
            }
            ContactKinematic::LinePoint(dir1) => {
                let world_dir1 = m1 * dir1;                
                let mut shift = world2 - world1;
                let proj = na::dot(world_dir1.as_ref(), &shift);
                shift -= dir1.as_ref() * proj;

                if let Some((n, d)) = Unit::try_new_and_get(shift, N::zero()) {
                    let local_n1 = m1.inverse_transform_vector(n.as_ref());
                    let local_n2 = m2.inverse_transform_vector(&-*n);
                    world1 = world2 - shift;

                    if constraint
                        .ncone1
                        .contains_dir(&-Unit::new_unchecked(local_n1))
                        && constraint
                            .ncone2
                            .contains_dir(&-Unit::new_unchecked(local_n2))
                    {
                        depth = d;
                        normal = -n;
                    } else {
                        depth = -d;
                        normal = n;
                    }
                } else {
                    depth = na::zero();
                    normal = m1 * constraint.normal1;
                }
            }
            ContactKinematic::PointLine(dir2) => {
                let world_dir2 = m2 * dir2;                
                let mut shift = world1 - world2;
                let proj = na::dot(world_dir2.as_ref(), &shift);
                shift -= dir2.as_ref() * proj;
                // NOTE: we set:
                // shift = world2 - world1
                let shift = -shift;

                if let Some((n, d)) = Unit::try_new_and_get(shift, N::zero()) {
                    let local_n1 = m1.inverse_transform_vector(n.as_ref());
                    let local_n2 = m2.inverse_transform_vector(&-*n);
                    world2 = world1 + shift;

                    if constraint
                        .ncone1
                        .contains_dir(&-Unit::new_unchecked(local_n1))
                        && constraint
                            .ncone2
                            .contains_dir(&-Unit::new_unchecked(local_n2))
                    {
                        depth = d;
                        normal = -n;
                    } else {
                        depth = -d;
                        normal = n;
                    }
                } else {
                    depth = na::zero();
                    normal = m1 * constraint.normal1;
                }
            }
            ContactKinematic::LineLine(dir1, dir2) => {
                let world_dir1 = m1 * dir1.unwrap();
                let world_dir2 = m2 * dir2.unwrap();
                let (pt1, pt2) = closest_points_internal::line_against_line(
                    &world1,
                    &world_dir1,
                    &world2,
                    &world_dir2,
                );
                world1 = pt1;
                world2 = pt2;

                if let Some((n, d)) = Unit::try_new_and_get(world2 - world1, N::zero()) {
                    let local_n1 = m1.inverse_transform_vector(n.as_ref());
                    let local_n2 = m2.inverse_transform_vector(&-*n);

                    if constraint
                        .ncone1
                        .contains_dir(&-Unit::new_unchecked(local_n1))
                        && constraint
                            .ncone2
                            .contains_dir(&-Unit::new_unchecked(local_n2))
                    {
                        depth = d;
                        normal = -n;
                    } else {
                        depth = -d;
                        normal = n;
                    }
                } else {
                    depth = na::zero();
                    normal = m1 * constraint.normal1;
                }
            }
            ContactKinematic::Unknown => {
                depth = N::zero();
                normal = m1 * constraint.normal1;
            }
        }

        world1 += normal.unwrap() * constraint.margin1;
        world2 -= normal.unwrap() * constraint.margin2;
        depth += constraint.margin1 + constraint.margin2;

        ContactGeometry::new(world1, world2, normal, depth)
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

        let geom = self.compute_contact_geometry(&body1, &body2, constraint);

        constraint.rhs = na::sup(
            &((-geom.depth + params.allowed_linear_error) * params.erp),
            &(-params.max_linear_correction),
        );

        if constraint.rhs >= N::zero() {
            return false;
        }

        // XXX: should use constraint_pair_geometry to properly handle multibodies.
        let mut inv_r = N::zero();
        helper::fill_constraint_geometry(
            &body1,
            constraint.ndofs1,
            &geom.world1,
            &ForceDirection::Linear(-geom.normal),
            constraint.ndofs1 + constraint.ndofs2,
            0,
            jacobians,
            &mut inv_r,
        );

        helper::fill_constraint_geometry(
            &body2,
            constraint.ndofs2,
            &geom.world2,
            &ForceDirection::Linear(geom.normal),
            (constraint.ndofs1 * 2) + constraint.ndofs2,
            constraint.ndofs1,
            jacobians,
            &mut inv_r,
        );

        // May happen sometimes for self-collisions (e.g. a multibody with itself).
        if inv_r.is_zero() {
            constraint.r = N::one();
        } else {
            constraint.r = N::one() / inv_r
        }

        true
    }
}
