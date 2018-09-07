use na::{self, DVector, Real};
use std::ops::Range;

use detection::ColliderContactManifold;
use ncollide::query::TrackedContact;
use ncollide::utils::IsometryOps;
use object::BodySet;
use solver::helper;
use solver::{ConstraintSet, ContactModel, ForceDirection, ImpulseCache, IntegrationParameters,
             NonlinearUnilateralConstraint, UnilateralConstraint, UnilateralGroundConstraint};

/// A contact model generating one non-penetration constraint per contact.
///
/// This is a frictionless contact model.
pub struct SignoriniModel<N: Real> {
    impulses: ImpulseCache<N>,
    vel_ground_rng: Range<usize>,
    vel_rng: Range<usize>,
}

impl<N: Real> SignoriniModel<N> {
    /// Create a new signorini contact model.
    pub fn new() -> Self {
        SignoriniModel {
            impulses: ImpulseCache::new(),
            vel_ground_rng: 0..0,
            vel_rng: 0..0,
        }
    }

    /// Build a non-penetration velocity-based constraint for the given contact.
    pub fn build_velocity_constraint(
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        manifold: &ColliderContactManifold<N>,
        ext_vels: &DVector<N>,
        c: &TrackedContact<N>,
        impulse: N,
        impulse_id: usize,
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    ) -> bool {
        let data1 = manifold.collider1.data();
        let data2 = manifold.collider2.data();

        let body1 = bodies.body(manifold.body1());
        let body2 = bodies.body(manifold.body2());
        let part1 = body1.part(manifold.body_part1());
        let part2 = body2.part(manifold.body_part2());

        let assembly_id1 = body1.companion_id();
        let assembly_id2 = body2.companion_id();

        let center1 = c.contact.world1 + c.contact.normal.unwrap() * data1.margin();
        let center2 = c.contact.world2 - c.contact.normal.unwrap() * data2.margin();
        let dir = ForceDirection::Linear(-c.contact.normal);

        let geom = helper::constraint_pair_geometry(
            body1,
            part1,
            body2,
            part2,
            &center1,
            &center2,
            &dir,
            ground_j_id,
            j_id,
            jacobians,
        );

        let mut rhs = helper::constraint_pair_velocity(
            body1,
            part1,
            body2,
            part2,
            assembly_id1,
            assembly_id2,
            &center1,
            &center2,
            &dir,
            ext_vels,
            jacobians,
            &geom,
        );

        if rhs > -params.restitution_velocity_threshold {
            let depth = c.contact.depth + data1.margin() + data2.margin();

            // No penetration, use predictive contact.
            if depth < N::zero() {
                rhs += (-depth) / params.dt;
            }
        } else {
            let rest1 = data1.material().restitution;
            let rest2 = data2.material().restitution;
            rhs += (rest1 + rest2) * na::convert(0.5) * rhs;
        }

        let warmstart = impulse * params.warmstart_coeff;
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
                    impulse_id,
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
                    impulse_id,
                ));

            false
        }
    }

    /// Checks if the given constraint is active.
    pub fn is_constraint_active(
        c: &TrackedContact<N>,
        manifold: &ColliderContactManifold<N>,
    ) -> bool {
        let depth = c.contact.depth + manifold.collider1.data().margin()
            + manifold.collider2.data().margin();

        // NOTE: for now we consider non-penetrating
        // constraints as inactive.
        depth >= N::zero()
    }

    /// Builds non-linear position-based non-penetration constraints for the given contact manifold.
    pub fn build_position_constraint(
        bodies: &BodySet<N>,
        manifold: &ColliderContactManifold<N>,
        c: &TrackedContact<N>,
        constraints: &mut ConstraintSet<N>,
    ) {
        let data1 = manifold.collider1.data();
        let data2 = manifold.collider2.data();

        let b1 = manifold.body_part1();
        let b2 = manifold.body_part2();

        let body1 = bodies.body(b1.body_handle);
        let body2 = bodies.body(b2.body_handle);
        let part1 = body1.part(b1);
        let part2 = body2.part(b2);

        let pos1 = part1.position();
        let pos2 = part2.position();
        let normal1 = pos1.inverse_transform_unit_vector(&c.contact.normal);
        let normal2 = -pos2.inverse_transform_unit_vector(&c.contact.normal);

        let mut kinematic = c.kinematic.clone();
        let total_margin1 = kinematic.dilation1() + data1.margin();
        let total_margin2 = kinematic.dilation2() + data2.margin();
        kinematic.set_dilation1(total_margin1);
        kinematic.set_dilation2(total_margin2);
        kinematic.transform1(&manifold.pos_wrt_body1);
        kinematic.transform2(&manifold.pos_wrt_body2);

        constraints
            .position
            .unilateral
            .push(NonlinearUnilateralConstraint::new(
                b1,
                body1.status_dependent_ndofs(),
                b2,
                body2.status_dependent_ndofs(),
                normal1,
                normal2,
                kinematic,
            ));
    }
}

impl<N: Real> ContactModel<N> for SignoriniModel<N> {
    fn num_velocity_constraints(&self, c: &ColliderContactManifold<N>) -> usize {
        c.manifold.len()
    }

    fn constraints(
        &mut self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        manifolds: &[ColliderContactManifold<N>],
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    ) {
        let id_vel_ground = constraints.velocity.unilateral_ground.len();
        let id_vel = constraints.velocity.unilateral.len();

        for manifold in manifolds {
            for c in manifold.contacts() {
                // if !Self::is_constraint_active(c, manifold) {
                //     continue;
                // }

                let _ = Self::build_velocity_constraint(
                    params,
                    bodies,
                    manifold,
                    ext_vels,
                    c,
                    self.impulses.get(c.id),
                    self.impulses.entry_id(c.id),
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

    fn cache_impulses(&mut self, constraints: &ConstraintSet<N>) {
        let ground_contacts = &constraints.velocity.unilateral_ground[self.vel_ground_rng.clone()];
        let contacts = &constraints.velocity.unilateral[self.vel_rng.clone()];

        for c in ground_contacts {
            self.impulses[c.impulse_id] = c.impulse;
        }

        for c in contacts {
            self.impulses[c.impulse_id] = c.impulse;
        }
    }
}
