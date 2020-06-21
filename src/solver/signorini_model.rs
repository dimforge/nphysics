use na::{self, DVector, RealField};
use std::ops::Range;

use crate::detection::ColliderContactManifold;
use crate::material::{
    LocalMaterialProperties, Material, MaterialContext, MaterialsCoefficientsTable,
};
use crate::object::{Body, BodyHandle, BodyPart, BodyPartHandle, BodySet, ColliderHandle};
use crate::solver::helper;
use crate::solver::{
    ConstraintSet, ContactModel, ForceDirection, ImpulseCache, IntegrationParameters,
    NonlinearUnilateralConstraint, UnilateralConstraint, UnilateralGroundConstraint,
};
use ncollide::query::{ContactId, TrackedContact};
use ncollide::utils::IsometryOps;

/// A contact model generating one non-penetration constraint per contact.
///
/// This is a frictionless contact model.
pub struct SignoriniModel<N: RealField> {
    impulses: ImpulseCache<N>,
    vel_ground_rng: Range<usize>,
    vel_rng: Range<usize>,
}

impl<N: RealField> SignoriniModel<N> {
    /// Create a new Signorini contact model.
    pub fn new() -> Self {
        SignoriniModel {
            impulses: ImpulseCache::new(),
            vel_ground_rng: 0..0,
            vel_rng: 0..0,
        }
    }

    /// Build a non-penetration velocity-based constraint for the given contact.
    pub fn build_velocity_constraint<
        B: Body<N> + ?Sized,
        Handle: BodyHandle,
        CollHandle: ColliderHandle,
    >(
        parameters: &IntegrationParameters<N>,
        body1: &B,
        part1: &dyn BodyPart<N>,
        handle1: BodyPartHandle<Handle>,
        body2: &B,
        part2: &dyn BodyPart<N>,
        handle2: BodyPartHandle<Handle>,
        props: &LocalMaterialProperties<N>,
        manifold: &ColliderContactManifold<N, Handle, CollHandle>,
        ext_vels: &DVector<N>,
        c: &TrackedContact<N>,
        impulse: N,
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N, Handle, CollHandle, ContactId>,
    ) -> bool {
        let data1 = manifold.collider1;
        let data2 = manifold.collider2;

        let assembly_id1 = body1.companion_id();
        let assembly_id2 = body2.companion_id();

        let center1 = c.contact.world1 + c.contact.normal.into_inner() * data1.margin();
        let center2 = c.contact.world2 - c.contact.normal.into_inner() * data2.margin();
        let dir = ForceDirection::Linear(-c.contact.normal);
        let (ext_vels1, ext_vels2) =
            helper::split_ext_vels(body1, body2, assembly_id1, assembly_id2, ext_vels);
        let mut rhs = c.contact.normal.dot(&props.surface_velocity);

        let geom = helper::constraint_pair_geometry(
            body1,
            part1,
            handle1,
            body2,
            part2,
            handle2,
            &center1,
            &center2,
            &dir,
            ground_j_id,
            j_id,
            jacobians,
            Some(&ext_vels1),
            Some(&ext_vels2),
            Some(&mut rhs),
        );

        //        println!("rhs before: {}", rhs);

        // Handle restitution.
        if rhs <= -parameters.restitution_velocity_threshold {
            rhs += props.restitution.0 * rhs;
        }

        // Handle predictive contact if no penetration.
        let depth = c.contact.depth + data1.margin() + data2.margin();
        if depth < N::zero() {
            rhs += (-depth) * parameters.inv_dt();
        }

        //        println!("rhs after 2: {}", rhs);

        // FIXME: would it be more efficient to consider the contact active iff. the rhs
        // is still negative at this point?

        let warmstart = impulse * parameters.warmstart_coeff;
        if geom.is_ground_constraint() {
            constraints
                .velocity
                .unilateral_ground
                .push(UnilateralGroundConstraint::new(
                    geom,
                    assembly_id1,
                    assembly_id2,
                    rhs,
                    warmstart,
                    c.id,
                ));

            true
        } else {
            constraints
                .velocity
                .unilateral
                .push(UnilateralConstraint::new(
                    geom,
                    assembly_id1,
                    assembly_id2,
                    rhs,
                    warmstart,
                    c.id,
                ));

            false
        }
    }

    /// Checks if the given constraint is active.
    pub fn is_constraint_active<Handle: BodyHandle, CollHandle: ColliderHandle>(
        c: &TrackedContact<N>,
        manifold: &ColliderContactManifold<N, Handle, CollHandle>,
    ) -> bool {
        let depth = c.contact.depth + manifold.collider1.margin() + manifold.collider2.margin();

        // NOTE: for now we consider non-penetrating
        // constraints as inactive.
        depth >= N::zero()
    }

    /// Builds non-linear position-based non-penetration constraints for the given contact manifold.
    pub fn build_position_constraint<Handle: BodyHandle, CollHandle: ColliderHandle>(
        bodies: &dyn BodySet<N, Handle = Handle>,
        manifold: &ColliderContactManifold<N, Handle, CollHandle>,
        c: &TrackedContact<N>,
        constraints: &mut ConstraintSet<N, Handle, CollHandle, ContactId>,
    ) {
        let data1 = manifold.collider1;
        let data2 = manifold.collider2;

        let b1 = manifold.body_part1(c.kinematic.feature1());
        let b2 = manifold.body_part2(c.kinematic.feature2());

        let body1 = try_ret!(bodies.get(b1.0));
        let body2 = try_ret!(bodies.get(b2.0));
        let part1 = try_ret!(body1.part(b1.1));
        let part2 = try_ret!(body2.part(b2.1));

        // XXX: don't do this with deformable.
        let pos1 = part1.position();
        let pos2 = part2.position();

        let normal1 = pos1.inverse_transform_unit_vector(&c.contact.normal);
        let normal2 = -pos2.inverse_transform_unit_vector(&c.contact.normal);

        let mut kinematic = c.kinematic.clone();
        let total_margin1 = kinematic.dilation1() + data1.margin();
        let total_margin2 = kinematic.dilation2() + data2.margin();
        kinematic.set_dilation1(total_margin1);
        kinematic.set_dilation2(total_margin2);

        constraints
            .position
            .unilateral
            .push(NonlinearUnilateralConstraint::new(
                b1,
                manifold.handle1,
                body1.status_dependent_ndofs(),
                b2,
                manifold.handle2,
                body2.status_dependent_ndofs(),
                normal1,
                normal2,
                kinematic,
            ));
    }
}

impl<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle>
    ContactModel<N, Handle, CollHandle> for SignoriniModel<N>
{
    fn num_velocity_constraints(
        &self,
        c: &ColliderContactManifold<N, Handle, CollHandle>,
    ) -> usize {
        c.manifold.len()
    }

    fn constraints(
        &mut self,
        parameters: &IntegrationParameters<N>,
        coefficients: &MaterialsCoefficientsTable<N>,
        bodies: &dyn BodySet<N, Handle = Handle>,
        ext_vels: &DVector<N>,
        manifolds: &[ColliderContactManifold<N, Handle, CollHandle>],
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N, Handle, CollHandle, ContactId>,
    ) {
        let id_vel_ground = constraints.velocity.unilateral_ground.len();
        let id_vel = constraints.velocity.unilateral.len();

        for manifold in manifolds {
            let body1 = try_ret!(bodies.get(manifold.body1()));
            let body2 = try_ret!(bodies.get(manifold.body2()));

            for c in manifold.contacts() {
                if !Self::is_constraint_active(c, manifold) {
                    continue;
                }

                let handle1 = manifold.body_part1(c.kinematic.feature1());
                let handle2 = manifold.body_part2(c.kinematic.feature2());
                let part1 = try_ret!(body1.part(handle1.1));
                let part2 = try_ret!(body2.part(handle2.1));

                let material1 = manifold.collider1.material();
                let material2 = manifold.collider2.material();
                let context1 = MaterialContext::new(
                    manifold.collider1.shape(),
                    manifold.collider1.position(),
                    c,
                    true,
                );
                let context2 = MaterialContext::new(
                    manifold.collider2.shape(),
                    manifold.collider2.position(),
                    c,
                    false,
                );
                let props =
                    Material::combine(coefficients, material1, context1, material2, context2);

                let _ = Self::build_velocity_constraint(
                    parameters,
                    body1,
                    part1,
                    handle1,
                    body2,
                    part2,
                    handle2,
                    &props,
                    manifold,
                    ext_vels,
                    c,
                    self.impulses.get(c.id).cloned().unwrap_or_else(N::zero),
                    ground_j_id,
                    j_id,
                    jacobians,
                    constraints,
                );

                Self::build_position_constraint(bodies, manifold, c, constraints);
            }
        }

        self.vel_ground_rng = id_vel_ground..constraints.velocity.unilateral_ground.len();
        self.vel_rng = id_vel..constraints.velocity.unilateral.len();
    }

    fn cache_impulses(&mut self, constraints: &ConstraintSet<N, Handle, CollHandle, ContactId>) {
        let ground_contacts = &constraints.velocity.unilateral_ground[self.vel_ground_rng.clone()];
        let contacts = &constraints.velocity.unilateral[self.vel_rng.clone()];

        for c in ground_contacts {
            let _ = self.impulses.insert(c.impulse_id, c.impulse);
        }

        for c in contacts {
            let _ = self.impulses.insert(c.impulse_id, c.impulse);
        }
    }
}
