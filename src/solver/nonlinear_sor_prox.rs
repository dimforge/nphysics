use std::marker::PhantomData;
use std::ops::MulAssign;
use approx::ApproxEq;
use alga::linear::Transformation;
use na::{self, DVector, Dim, Dynamic, Real, U1, Unit, VectorSliceMutN};
use ncollide::query::ContactKinematic;

use object::{BodyHandle, BodySet};
use solver::helper;
use solver::{ForceDirection, NonlinearUnilateralConstraint};
use math::{Point, Vector};

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
        mj_lambda: &mut DVector<N>,
        jacobians: &mut [N],
        max_iter: usize,
    ) {
        for _ in 0..max_iter {
            for constraint in constraints.iter_mut() {
                // FIXME: specialize for SPATIAL_DIM.
                let dim1 = Dynamic::new(constraint.ndofs1);
                let dim2 = Dynamic::new(constraint.ndofs2);
                self.solve_unilateral(bodies, constraint, mj_lambda, jacobians, dim1, dim2)
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
        self.update_constraint(bodies, constraint, jacobians);

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

    fn update_constraint(
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
                // if let Some((n, d)) = Unit::try_new_and_get(world2 - world1, N::default_epsilon()) {
                //     depth = -d;
                //     normal = n;
                // } else {
                depth = -constraint.margin1 - constraint.margin2;
                normal = Unit::new_unchecked(m1.transform_vector(constraint.normal1.as_ref()));
                // }
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
}
