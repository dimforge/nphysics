use std::marker::PhantomData;
use std::ops::MulAssign;
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
        self.update_constraint(bodies, constraint, mj_lambda, jacobians);

        let impulse = na::sup(&N::zero(), &(-constraint.rhs / constraint.inv_r));

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

    fn update_constraint(
        &self,
        bodies: &BodySet<N>,
        constraint: &mut NonlinearUnilateralConstraint<N>,
        mj_lambda: &mut DVector<N>,
        jacobians: &mut [N],
    ) {
        let body1 = bodies.body_part(constraint.body1);
        let body2 = bodies.body_part(constraint.body2);

        let m1 = body1.position();
        let m2 = body2.position();

        let mut world1 = m1.transform_point(&constraint.local1);
        let mut world2 = m2.transform_point(&constraint.local2);
        let mut normal1 = Unit::new_unchecked(m1.transform_vector(constraint.normal1.as_ref()));
        let mut normal2 = Unit::new_unchecked(m2.transform_vector(constraint.normal2.as_ref()));
        let depth;

        match constraint.kinematic {
            ContactKinematic::PlanePoint => {
                depth = na::dot(normal1.as_ref(), &(world2 - world1));
                world1 = world2 - normal1.as_ref() * depth;
            }
            ContactKinematic::PointPlane => {
                depth = na::dot(normal2.as_ref(), &(world1 - world2));
                world2 = world1 - normal2.as_ref() * depth;
            }
            ContactKinematic::PointPoint => {
                depth = na::distance(&world1, &world2);
            }
            ContactKinematic::LinePoint(dir1) => {
                let mut shift = world2 - world1;
                let proj = na::dot(dir1.as_ref(), &shift);
                shift -= dir1.as_ref() * proj;
                world1 = world2 - shift;
                depth = na::norm(&shift);
            }
            ContactKinematic::PointLine(dir2) => {
                let mut shift = world1 - world2;
                let proj = na::dot(dir2.as_ref(), &shift);
                shift -= dir2.as_ref() * proj;
                world2 = world1 - shift;
                depth = na::norm(&shift);
            }
            ContactKinematic::LineLine(dir1, dir2) => {
                let a = N::zero();
                depth = N::zero();
            }
            ContactKinematic::Unknown => {
                let a = N::zero();
                depth = N::zero();
            }
        }

        constraint.rhs = depth;

        helper::fill_constraint_geometry(
            &body1,
            constraint.ndofs1,
            &world1,
            &ForceDirection::Linear(normal1),
            constraint.ndofs1 + constraint.ndofs2,
            0,
            jacobians,
            &mut constraint.inv_r,
        );

        helper::fill_constraint_geometry(
            &body2,
            constraint.ndofs2,
            &world2,
            &ForceDirection::Linear(normal2),
            (constraint.ndofs1 * 2) + constraint.ndofs2,
            constraint.ndofs1,
            jacobians,
            &mut constraint.inv_r,
        );
    }
}
