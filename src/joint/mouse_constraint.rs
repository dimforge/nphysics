use na::{DVector, RealField, Unit};

use crate::joint::JointConstraint;
use crate::math::{Point, Vector, DIM};
use crate::object::{BodyHandle, BodyPartHandle, BodySet};
use crate::solver::{
    helper, BilateralConstraint, BilateralGroundConstraint, ForceDirection, ImpulseLimits,
};
use crate::solver::{
    GenericNonlinearConstraint, IntegrationParameters, LinearConstraints,
    NonlinearConstraintGenerator,
};

/// A spring-like constraint to be used to drag a body part with the mouse.
pub struct MouseConstraint<N: RealField, Handle: BodyHandle> {
    b1: BodyPartHandle<Handle>,
    b2: BodyPartHandle<Handle>,
    anchor1: Point<N>,
    anchor2: Point<N>,
    limit: N,
}

impl<N: RealField, Handle: BodyHandle> MouseConstraint<N, Handle> {
    /// Initialize a mouse constraint between two bodies.getPartHandle
    ///
    /// Typically, `b1` will be the ground and the anchor the position of the mouse.
    /// Both anchors are expressed in the local coordinate frames of the corresponding body parts.
    pub fn new(
        b1: BodyPartHandle<Handle>,
        b2: BodyPartHandle<Handle>,
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

    /// Change the first anchor, expressed in the local space of the first body part.
    pub fn set_anchor_1(&mut self, anchor1: Point<N>) {
        self.anchor1 = anchor1;
    }

    /// Change the first anchor, expressed in the local space of the second body part.
    pub fn set_anchor_2(&mut self, anchor2: Point<N>) {
        self.anchor2 = anchor2;
    }
}

impl<N: RealField, Handle: BodyHandle> JointConstraint<N, Handle> for MouseConstraint<N, Handle> {
    fn num_velocity_constraints(&self) -> usize {
        DIM
    }

    fn anchors(&self) -> (BodyPartHandle<Handle>, BodyPartHandle<Handle>) {
        (self.b1, self.b2)
    }

    fn velocity_constraints(
        &mut self,
        parameters: &IntegrationParameters<N>,
        bodies: &dyn BodySet<N, Handle = Handle>,
        ext_vels: &DVector<N>,
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut LinearConstraints<N, usize>,
    ) {
        let body1 = try_ret!(bodies.get(self.b1.0));
        let body2 = try_ret!(bodies.get(self.b2.0));
        let part1 = try_ret!(body1.part(self.b1.1));
        let part2 = try_ret!(body2.part(self.b2.1));

        /*
         *
         * Joint constraints.
         *
         */
        let anchor1 = body1.world_point_at_material_point(part1, &self.anchor1);
        let anchor2 = body2.world_point_at_material_point(part2, &self.anchor2);

        let assembly_id1 = body1.companion_id();
        let assembly_id2 = body2.companion_id();

        let limits = ImpulseLimits::Independent {
            min: -self.limit,
            max: self.limit,
        };

        let error = anchor2 - anchor1;
        let (ext_vels1, ext_vels2) =
            helper::split_ext_vels(body1, body2, assembly_id1, assembly_id2, ext_vels);

        #[cfg(feature = "dim2")]
        let canonical_basis = [Vector::x(), Vector::y()];
        #[cfg(feature = "dim3")]
        let canonical_basis = [Vector::x(), Vector::y(), Vector::z()];

        for dir in &canonical_basis {
            let fdir = ForceDirection::Linear(Unit::new_unchecked(*dir));
            let mut rhs = -error.dot(&*dir) * parameters.erp * parameters.inv_dt();
            let geom = helper::constraint_pair_geometry(
                body1,
                part1,
                self.b1,
                body2,
                part2,
                self.b2,
                &anchor1,
                &anchor2,
                &fdir,
                ground_j_id,
                j_id,
                jacobians,
                Some(&ext_vels1),
                Some(&ext_vels2),
                Some(&mut rhs),
            );

            if geom.ndofs1 == 0 || geom.ndofs2 == 0 {
                constraints
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
                constraints.bilateral.push(BilateralConstraint::new(
                    geom,
                    assembly_id1,
                    assembly_id2,
                    limits,
                    rhs,
                    N::zero(),
                    0,
                ));
            }
        }
    }

    fn cache_impulses(&mut self, _: &LinearConstraints<N, usize>, _: N) {}
}

impl<N: RealField, Handle: BodyHandle> NonlinearConstraintGenerator<N, Handle>
    for MouseConstraint<N, Handle>
{
    fn num_position_constraints(&self, _: &dyn BodySet<N, Handle = Handle>) -> usize {
        0
    }

    fn position_constraint(
        &self,
        _: &IntegrationParameters<N>,
        _: usize,
        _: &mut dyn BodySet<N, Handle = Handle>,
        _: &mut [N],
    ) -> Option<GenericNonlinearConstraint<N, Handle>> {
        None
    }
}
