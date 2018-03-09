use std::marker::PhantomData;
use std::ops::MulAssign;
use approx::ApproxEq;
use alga::linear::Transformation;
use alga::linear::ProjectiveTransformation;
use na::{self, DVector, Dim, Dynamic, Real, U1, Unit, VectorSliceMutN};
use ncollide::query::ContactKinematic;

use object::{BodyHandle, BodyPart, BodySet};
use solver::helper;
use solver::{ForceDirection, NonlinearNormalConeConstraint, NonlinearUnilateralConstraint};
use math::{Isometry, Point, Rotation, Vector};

struct ContactGeometry<N: Real> {
    world1: Point<N>,
    world2: Point<N>,
    normal1: Unit<Vector<N>>,
    normal2: Unit<Vector<N>>,
    normal: Unit<Vector<N>>,
    depth: N,
}

impl<N: Real> ContactGeometry<N> {
    pub fn new(
        world1: Point<N>,
        world2: Point<N>,
        normal1: Unit<Vector<N>>,
        normal2: Unit<Vector<N>>,
        normal: Unit<Vector<N>>,
        depth: N,
    ) -> Self {
        ContactGeometry {
            world1,
            world2,
            normal1,
            normal2,
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

                // self.solve_normal(
                //     bodies,
                //     constraint,
                //     &mut ncone_constraints[constraint.normal_constraint_id],
                //     mj_lambda,
                //     jacobians,
                //     dim1,
                //     dim2,
                // );
            }
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
        let normal1;
        let normal2;
        let mut depth;

        match constraint.kinematic {
            ContactKinematic::PlanePoint => {
                normal = m1 * constraint.normal1;
                depth = -na::dot(normal.as_ref(), &(world2 - world1));
                world1 = world2 + normal.as_ref() * depth;
                normal1 = constraint.normal1;
                normal2 = -Unit::new_unchecked(m2.inverse_transform_vector(normal.as_ref()));
            }
            ContactKinematic::PointPlane => {
                let world_normal2 = m2 * constraint.normal2.as_ref();
                depth = -na::dot(&world_normal2, &(world1 - world2));
                world2 = world1 + &world_normal2 * depth;
                normal = Unit::new_unchecked(-world_normal2);
                normal1 = Unit::new_unchecked(m1.inverse_transform_vector(normal.as_ref()));
                normal2 = constraint.normal2;
            }
            ContactKinematic::PointPoint => {
                if let Some((n, d)) = Unit::try_new_and_get(world2 - world1, N::default_epsilon()) {
                    depth = -d;
                    normal = n;
                } else {
                    depth = -constraint.margin1 - constraint.margin2;
                    normal = m1 * constraint.normal1;
                }
                normal1 = Unit::new_unchecked(m1.inverse_transform_vector(normal.as_ref()));
                normal2 = -Unit::new_unchecked(m2.inverse_transform_vector(normal.as_ref()));
            }
            ContactKinematic::LinePoint(dir1) => {
                /*
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
                */
                unimplemented!()
            }
            ContactKinematic::PointLine(dir2) => {
                /*
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
                */
                unimplemented!()
            }
            ContactKinematic::LineLine(dir1, dir2) => {
                let world_dir1 = m1 * dir1;
                let world_dir2 = m2 * dir2;
                // let (pt1, pt2) = closest_points(world1, world_dir1, world2, world_dir2);
                // world1 = pt1;
                // world2 = pt2;

                if let Some((n, d)) = Unit::try_new_and_get(world2 - world1, N::zero()) {
                    let local_n1 = m1.inverse_transform_vector(n.as_ref());
                    let local_n2 = m2.inverse_transform_vector(&-*n);

                    if constraint.ncone1.contains(&local_n1)
                        && constraint.ncone2.contains(&local_n2)
                    {
                        depth = d;
                        normal = n;
                    } else {
                        depth = -d;
                        normal = -n;
                    }
                } else {
                    depth = na::zero();
                    normal = Unit::new_unchecked(m1.transform_vector(constraint.normal1.as_ref()));
                }

                normal1 = Unit::new_unchecked(m1.inverse_transform_vector(normal.as_ref()));
                normal2 = -Unit::new_unchecked(m2.inverse_transform_vector(normal.as_ref()));
            }
            ContactKinematic::Unknown => {
                depth = N::zero();
                normal = m1 * constraint.normal1;
                normal1 = constraint.normal1;
                normal2 = -Unit::new_unchecked(m2.inverse_transform_vector(normal.as_ref()));
            }
        }

        world1 += normal.unwrap() * constraint.margin1;
        world2 -= normal.unwrap() * constraint.margin2;
        depth += constraint.margin1 + constraint.margin2;

        ContactGeometry::new(world1, world2, normal1, normal2, normal, depth)
    }

    fn update_contact_constraint(
        &self,
        bodies: &BodySet<N>,
        constraint: &mut NonlinearUnilateralConstraint<N>,
        jacobians: &mut [N],
    ) {
        let body1 = bodies.body_part(constraint.body1);
        let body2 = bodies.body_part(constraint.body2);

        let geom = self.compute_contact_geometry(&body1, &body2, constraint);

        let erp: N = na::convert(0.2); // XXX: don't hard-code this.s
        let allowed_error: N = na::convert(0.005); // XXX don't hard-code this.
        let max_correction: N = na::convert(0.2); // XXX don't hard-code this.
        constraint.rhs = na::sup(&(-geom.depth * erp), &(-max_correction)) + allowed_error;

        constraint.inv_r = N::zero();
        helper::fill_constraint_geometry(
            &body1,
            constraint.ndofs1,
            &geom.world1,
            &ForceDirection::Linear(-geom.normal),
            constraint.ndofs1 + constraint.ndofs2,
            0,
            jacobians,
            &mut constraint.inv_r,
        );

        helper::fill_constraint_geometry(
            &body2,
            constraint.ndofs2,
            &geom.world2,
            &ForceDirection::Linear(geom.normal),
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

        let geom = self.compute_contact_geometry(&body1, &body2, contact);

        let closest_n1 = constraint.ncone1.project(&geom.normal1);
        let closest_n2 = constraint.ncone2.project(&geom.normal2);

        // FIXME: could rot1 and rot2 be obtained as a by-product of the projection?
        let rot1;
        let rot2;

        #[cfg(feature = "dim3")]
        {
            // NOTE: we assume the case where both vectors are antipodal
            // (i.e. the result is None) happens too rarely to be handled properly.
            // So if it does, no correction will be made for this iteration.
            let id = Rotation::identity();
            rot1 = Rotation::rotation_between_axis(&geom.normal1, &closest_n1).unwrap_or(id);
            rot2 = Rotation::rotation_between_axis(&geom.normal2, &closest_n2).unwrap_or(id);
        }

        #[cfg(feature = "dim2")]
        {
            rot1 = Rotation::rotation_between_axis(&geom.normal1, &closest_n1);
            rot2 = Rotation::rotation_between_axis(&geom.normal2, &closest_n2);
        }

        let m1 = body1.position();
        let m2 = body2.position();

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
            world_axis = m * axis;
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
            &body1.center_of_mass(),
            &ForceDirection::Angular(world_axis),
            contact.ndofs1 + contact.ndofs2,
            0,
            jacobians,
            &mut constraint.inv_r,
        );

        helper::fill_constraint_geometry(
            &body2,
            contact.ndofs2,
            &body2.center_of_mass(),
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
}
