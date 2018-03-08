use std::marker::PhantomData;
use std::ops::MulAssign;
use approx::ApproxEq;
use alga::linear::Transformation;
use alga::linear::ProjectiveTransformation;
use na::{self, DVector, Dim, Dynamic, Real, U1, Unit, VectorSliceMutN};
use ncollide::query::ContactKinematic;

use object::{BodyHandle, BodySet};
use solver::helper;
use solver::{ForceDirection, NonlinearNormalConeConstraint, NonlinearUnilateralConstraint};
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
        bodies: &mut BodySet<N>,
        constraints: &mut [NonlinearUnilateralConstraint<N>],
        ncone_constraints: &mut [NonlinearNormalConeConstraint<N>],
        mj_lambda: &mut DVector<N>,
        jacobians: &mut [N],
        max_iter: usize,
    ) {
        for _ in 0..max_iter {
            for constraint in constraints.iter_mut() {
                // FIXME: specialize for SPATIAL_DIM.
                let dim1 = Dynamic::new(constraint.ndofs1);
                let dim2 = Dynamic::new(constraint.ndofs2);
                self.solve_unilateral(bodies, constraint, mj_lambda, jacobians, dim1, dim2);

                self.solve_normal(
                    bodies,
                    constraint,
                    &mut ncone_constraints[constraint.normal_constraint_id],
                    mj_lambda,
                    jacobians,
                    dim1,
                    dim2,
                );
            }

            /*
            for constraint in ncone_constraints.iter_mut() {
                // FIXME: specialize for SPATIAL_DIM.
                let contact = &constraints[constraint.contact_id];
                let dim1 = Dynamic::new(contact.ndofs1);
                let dim2 = Dynamic::new(contact.ndofs2);

                self.solve_normal(
                    bodies,
                    contact,
                    constraint,
                    mj_lambda,
                    jacobians,
                    dim1,
                    dim2,
                );
            }*/
        }
    }

    fn solve_unilateral<D1: Dim, D2: Dim>(
        &self,
        bodies: &mut BodySet<N>,
        constraint: &mut NonlinearUnilateralConstraint<N>,
        mj_lambda: &mut DVector<N>,
        jacobians: &mut [N],
        dim1: D1,
        dim2: D2,
    ) {
        self.update_contact_constraint(bodies, constraint, jacobians);

        let impulse = -constraint.rhs / constraint.inv_r;

        if impulse > N::zero() {
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

    fn solve_normal<D1: Dim, D2: Dim>(
        &self,
        bodies: &mut BodySet<N>,
        contact: &NonlinearUnilateralConstraint<N>,
        constraint: &mut NonlinearNormalConeConstraint<N>,
        mj_lambda: &mut DVector<N>,
        jacobians: &mut [N],
        dim1: D1,
        dim2: D2,
    ) {
        if self.update_normal_cone_constraint(bodies, contact, constraint, jacobians) {
            let impulse = -constraint.rhs / constraint.inv_r;

            VectorSliceMutN::new_generic_mut(jacobians, dim1, U1).mul_assign(impulse);
            VectorSliceMutN::new_generic_mut(&mut jacobians[dim1.value()..], dim2, U1)
                .mul_assign(impulse);

            bodies
                .body_mut(contact.body1)
                .apply_displacement(&jacobians[0..dim1.value()]);
            bodies
                .body_mut(contact.body2)
                .apply_displacement(&jacobians[dim1.value()..dim1.value() + dim2.value()]);
        }
    }

    fn update_contact_constraint(
        &self,
        bodies: &BodySet<N>,
        constraint: &mut NonlinearUnilateralConstraint<N>,
        jacobians: &mut [N],
    ) {
        let body1 = bodies.body_part(constraint.body1);
        let body2 = bodies.body_part(constraint.body2);

        let m1 = body1.position();
        let m2 = body2.position();

        let mut world1 = m1.transform_point(&constraint.local1);
        let mut world2 = m2.transform_point(&constraint.local2);
        let normal;
        let depth;

        match constraint.kinematic {
            ContactKinematic::PlanePoint => {
                normal = Unit::new_unchecked(m1.transform_vector(constraint.normal1.as_ref()));
                depth = -na::dot(normal.as_ref(), &(world2 - world1));
                world1 = world2 + normal.as_ref() * depth;
            }
            ContactKinematic::PointPlane => {
                let normal2 = m2.transform_vector(constraint.normal2.as_ref());
                depth = -na::dot(&normal2, &(world1 - world2));
                world2 = world1 + &normal2 * depth;
                normal = Unit::new_unchecked(-normal2);
            }
            ContactKinematic::PointPoint => {
                if let Some((n, d)) = Unit::try_new_and_get(world2 - world1, N::default_epsilon()) {
                    depth = -d;
                    normal = n;
                } else {
                    depth = -constraint.margin1 - constraint.margin2;
                    normal = Unit::new_unchecked(m1.transform_vector(constraint.normal1.as_ref()));
                }
            }
            ContactKinematic::LinePoint(dir1) => {
                let mut shift = world2 - world1;
                let proj = na::dot(dir1.as_ref(), &shift);
                shift -= dir1.as_ref() * proj;
                world1 = world2 - shift;

                if let Some((n, d)) = Unit::try_new_and_get(shift, N::zero()) {
                    depth = d;
                    normal = n;
                } else {
                    depth = na::zero();
                    normal = Unit::new_unchecked(m1.transform_vector(constraint.normal1.as_ref()));
                }
            }
            ContactKinematic::PointLine(dir2) => {
                let mut shift = world1 - world2;
                let proj = na::dot(dir2.as_ref(), &shift);
                shift -= dir2.as_ref() * proj;
                world2 = world1 - shift;

                if let Some((n, d)) = Unit::try_new_and_get(shift, N::zero()) {
                    depth = d;
                    normal = n;
                } else {
                    depth = na::zero();
                    normal = Unit::new_unchecked(m1.transform_vector(constraint.normal1.as_ref()));
                }
            }
            ContactKinematic::LineLine(dir1, dir2) => unimplemented!(),
            ContactKinematic::Unknown => {
                depth = N::zero();
                normal = Unit::new_unchecked(m1.transform_vector(constraint.normal1.as_ref()));
            }
        }

        let erp: N = na::convert(0.2); // XXX: don't hard-code this.s
        let allowed_error: N = na::convert(0.005); // XXX don't hard-code this.
        let max_correction: N = na::convert(0.2); // XXX don't hard-code this.
        constraint.rhs = na::sup(
            &((-(depth + constraint.margin1 + constraint.margin2)) * erp),
            &(-max_correction),
        );

        world1 += normal.unwrap() * constraint.margin1;
        world2 -= normal.unwrap() * constraint.margin2;

        constraint.inv_r = N::zero();
        helper::fill_constraint_geometry(
            &body1,
            constraint.ndofs1,
            &world1,
            &ForceDirection::Linear(-normal),
            constraint.ndofs1 + constraint.ndofs2,
            0,
            jacobians,
            &mut constraint.inv_r,
        );

        helper::fill_constraint_geometry(
            &body2,
            constraint.ndofs2,
            &world2,
            &ForceDirection::Linear(normal),
            (constraint.ndofs1 * 2) + constraint.ndofs2,
            constraint.ndofs1,
            jacobians,
            &mut constraint.inv_r,
        );

        // May happen sometimes for self-collisions (e.g. a multibody with itself).
        if constraint.inv_r.is_zero() {
            constraint.inv_r = N::one();
        }
    }

    fn update_normal_cone_constraint(
        &self,
        bodies: &BodySet<N>,
        contact: &NonlinearUnilateralConstraint<N>,
        constraint: &mut NonlinearNormalConeConstraint<N>,
        jacobians: &mut [N],
    ) -> bool {
        if contact.rhs >= N::zero() {
            return false;
        }

        let body1 = bodies.body_part(contact.body1);
        let body2 = bodies.body_part(contact.body2);

        let m1 = body1.position();
        let m2 = body2.position();

        let (n1, n2) = self.local_contact_normals(bodies, contact, &m1, &m2);
        let closest_n1 = constraint.ncone1.project(&n1);
        let closest_n2 = constraint.ncone2.project(&n2);

        // FIXME: could rot1 and rot2 be obtained as a by-product of the projection?

        let rot1;
        let rot2;

        #[cfg(feature = "dim3")]
        {
            // NOTE: we assume the case where both vectors are antipodal
            // (i.e. the result is None) happens too rarely to be handled properly.
            // So if it does, no correction will be made for this iteration.
            let id = Rotation::identity();
            rot1 = Rotation::rotation_between_axis(&n1, &closest_n1).unwrap_or(id);
            rot2 = Rotation::rotation_between_axis(&n2, &closest_n2).unwrap_or(id);
        }

        #[cfg(feature = "dim2")]
        {
            rot1 = Rotation::rotation_between_axis(&n1, &closest_n1);
            rot2 = Rotation::rotation_between_axis(&n2, &closest_n2);
        }

        let (axis, ang, m) = match (rot1.axis_angle(), rot2.axis_angle()) {
            (None, None) => {
                return false;
            }
            (Some((axis1, ang1)), None) => (axis1, ang1, &m1),
            (None, Some((axis2, ang2))) => (-axis2, ang2, &m2),
            (Some((axis1, ang1)), Some((axis2, ang2))) => {
                if ang1 > ang2 {
                    (axis1, ang1, &m1)
                } else {
                    (-axis2, ang2, &m2)
                }
            }
        };

        let world_axis;
        #[cfg(feature = "dim2")]
        {
            world_axis = axis;
        }

        #[cfg(feature = "dim3")]
        {
            world_axis = Unit::new_unchecked(m.transform_vector(axis.as_ref()));
        }

        constraint.rhs = -ang; // na::sup(&-ang, &na::convert(-0.02));
                               // println!("Angular stabilization: {}", ang);
        if constraint.rhs >= N::zero() {
            return false;
        }

        // FIXME: for 2D rigid bodies, there is no need to recompute the
        // constraint geometry at each frame because the jacobians will
        // always be the same (except maybe for the sign).
        constraint.inv_r = N::zero();
        helper::fill_constraint_geometry(
            &body1,
            contact.ndofs1,
            &Point::origin(),
            &ForceDirection::Angular(world_axis),
            contact.ndofs1 + contact.ndofs2,
            0,
            jacobians,
            &mut constraint.inv_r,
        );

        helper::fill_constraint_geometry(
            &body2,
            contact.ndofs2,
            &Point::origin(),
            &ForceDirection::Angular(-world_axis),
            (contact.ndofs1 * 2) + contact.ndofs2,
            contact.ndofs1,
            jacobians,
            &mut constraint.inv_r,
        );

        // May happen sometimes for self-collisions (e.g. a multibody with itself).
        if constraint.inv_r.is_zero() {
            constraint.inv_r = N::one();
        }

        return true;
    }

    fn local_contact_normals(
        &self,
        bodies: &BodySet<N>,
        constraint: &NonlinearUnilateralConstraint<N>,
        m1: &Isometry<N>,
        m2: &Isometry<N>,
    ) -> (Unit<Vector<N>>, Unit<Vector<N>>) {
        match constraint.kinematic {
            ContactKinematic::PlanePoint => {
                let world_normal1 = m1.transform_vector(constraint.normal1.as_ref());
                let normal2 = m2.inverse_transform_vector(&-world_normal1);
                (constraint.normal1, Unit::new_unchecked(normal2))
            }
            ContactKinematic::PointPlane => {
                let world_normal2 = m2.transform_vector(constraint.normal2.as_ref());
                let normal1 = m1.inverse_transform_vector(&-world_normal2);
                (Unit::new_unchecked(normal1), constraint.normal2)
            }
            ContactKinematic::PointPoint => {
                let world1 = m1.transform_point(&constraint.local1);
                let world2 = m2.transform_point(&constraint.local2);

                if let Some(n) = Unit::try_new(world2 - world1, N::default_epsilon()) {
                    (
                        Unit::new_unchecked(m1.inverse_transform_vector(n.as_ref())),
                        Unit::new_unchecked(m2.inverse_transform_vector(&-*n)),
                    )
                } else {
                    (constraint.normal1, constraint.normal2)
                }
            }
            ContactKinematic::LinePoint(dir1) => unimplemented!(),
            ContactKinematic::PointLine(dir2) => unimplemented!(),
            ContactKinematic::LineLine(dir1, dir2) => unimplemented!(),
            ContactKinematic::Unknown => (constraint.normal1, constraint.normal2),
        }
    }
}
