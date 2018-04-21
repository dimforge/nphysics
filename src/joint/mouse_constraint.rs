use alga::linear::FiniteDimVectorSpace;
use na::{DVector, Real, Unit};

use object::{BodyHandle, BodySet};
use solver::{ConstraintSet, GenericNonlinearConstraint, IntegrationParameters,
             NonlinearConstraintGenerator};
use solver::{helper, BilateralConstraint, BilateralGroundConstraint, ForceDirection, ImpulseLimits};
use joint::JointConstraint;
use math::{Point, Vector, DIM};

pub struct MouseConstraint<N: Real> {
    b1: BodyHandle,
    b2: BodyHandle,
    anchor1: Point<N>,
    anchor2: Point<N>,
    limit: N,
}

impl<N: Real> MouseConstraint<N> {
    pub fn new(
        b1: BodyHandle,
        b2: BodyHandle,
        anchor1: Point<N>,
        anchor2: Point<N>,
        limit: N,
    ) -> Self {
        MouseConstraint {
            b1,
            b2,
            anchor1,
            anchor2,
            limit,
        }
    }

    pub fn set_anchor_1(&mut self, anchor1: Point<N>) {
        self.anchor1 = anchor1;
    }

    pub fn set_anchor_2(&mut self, anchor2: Point<N>) {
        self.anchor2 = anchor2;
    }
}

impl<N: Real> JointConstraint<N> for MouseConstraint<N> {
    fn num_velocity_constraints(&self) -> usize {
        DIM
    }

    fn anchors(&self) -> (BodyHandle, BodyHandle) {
        (self.b1, self.b2)
    }

    fn velocity_constraints(
        &mut self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    ) {
        let body1 = bodies.body_part(self.b1);
        let body2 = bodies.body_part(self.b2);

        /*
         *
         * Joint constraints.
         *
         */
        let pos1 = body1.position();
        let pos2 = body2.position();

        let anchor1 = pos1 * self.anchor1;
        let anchor2 = pos2 * self.anchor2;

        let assembly_id1 = body1.parent_companion_id();
        let assembly_id2 = body2.parent_companion_id();

        let limits = ImpulseLimits::Independent {
            min: -self.limit,
            max: self.limit,
        };

        let error = anchor2 - anchor1;

        let mut i = 0;
        Vector::canonical_basis(|dir| {
            let fdir = ForceDirection::Linear(Unit::new_unchecked(*dir));
            let geom = helper::constraint_pair_geometry(
                &body1,
                &body2,
                &anchor1,
                &anchor2,
                &fdir,
                ground_j_id,
                j_id,
                jacobians,
            );

            let rhs = helper::constraint_pair_velocity(
                &body1,
                &body2,
                assembly_id1,
                assembly_id2,
                &anchor1,
                &anchor2,
                &fdir,
                ext_vels,
                jacobians,
                &geom,
            ) - error.dot(&*dir) * params.erp / params.dt;

            if geom.ndofs1 == 0 || geom.ndofs2 == 0 {
                constraints
                    .velocity
                    .bilateral_ground
                    .push(BilateralGroundConstraint::new(
                        geom,
                        assembly_id1,
                        assembly_id2,
                        limits,
                        rhs,
                        N::zero(),
                        0,
                    ));
            } else {
                constraints
                    .velocity
                    .bilateral
                    .push(BilateralConstraint::new(
                        geom,
                        assembly_id1,
                        assembly_id2,
                        limits,
                        rhs,
                        N::zero(),
                        0,
                    ));
            }

            i += 1;

            true
        });
    }

    fn cache_impulses(&mut self, _: &ConstraintSet<N>) {}
}

impl<N: Real> NonlinearConstraintGenerator<N> for MouseConstraint<N> {
    fn num_position_constraints(&self, _: &BodySet<N>) -> usize {
        0
    }

    fn position_constraint(
        &self,
        _: &IntegrationParameters<N>,
        _: usize,
        _: &mut BodySet<N>,
        _: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N>> {
        None
    }
}
