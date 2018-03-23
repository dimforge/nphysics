use std::ops::Range;
use alga::linear::ProjectiveTransformation;
use na::{self, DVector, Real, Unit};

use ncollide::query::TrackedContact;
use ncollide::math::Isometry as NCollideIsometry;
use detection::BodyContactManifold;
use solver::helper;
use solver::{ConstraintSet, ContactModel, ForceDirection, ImpulseCache, IntegrationParameters,
             NonlinearUnilateralConstraint, UnilateralConstraint, UnilateralGroundConstraint};
use object::{BodyHandle, BodySet};
use math::{Isometry, Point};

pub struct SignoriniModel<N: Real> {
    impulses: ImpulseCache<N>,
    vel_ground_rng: Range<usize>,
    vel_rng: Range<usize>,
}

impl<N: Real> SignoriniModel<N> {
    pub fn new() -> Self {
        SignoriniModel {
            impulses: ImpulseCache::new(),
            vel_ground_rng: 0..0,
            vel_rng: 0..0,
        }
    }

    pub fn build_velocity_constraint(
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        b1: BodyHandle,
        b2: BodyHandle,
        c: &TrackedContact<Point<N>>,
        margin1: N,
        margin2: N,
        impulse: N,
        impulse_id: usize,
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    ) -> bool {
        let body1 = bodies.body_part(b1);
        let body2 = bodies.body_part(b2);

        let assembly_id1 = body1.parent_companion_id();
        let assembly_id2 = body2.parent_companion_id();

        let center1 = c.contact.world1 + c.contact.normal.unwrap() * margin1;
        let center2 = c.contact.world2 - c.contact.normal.unwrap() * margin2;
        let dir = ForceDirection::Linear(-c.contact.normal);

        let geom = helper::constraint_pair_geometry(
            &body1,
            &body2,
            &center1,
            &center2,
            &dir,
            ground_j_id,
            j_id,
            jacobians,
        );

        let mut rhs = helper::constraint_pair_velocity(
            &body1,
            &body2,
            assembly_id1,
            assembly_id2,
            &center1,
            &center2,
            &dir,
            ext_vels,
            jacobians,
            &geom,
        );

        rhs += N::zero(); // FIXME: (rb1.restitution() + rb2.restitution()) * na::convert(0.5) * dvel;

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

    pub fn is_constraint_active(
        c: &TrackedContact<Point<N>>,
        manifold: &BodyContactManifold<N>,
    ) -> bool {
        let depth = c.contact.depth + manifold.margin1 + manifold.margin2;

        // NOTE: for now we consider non-penetrating
        // constraints as inactive.
        depth >= N::zero()
    }

    pub fn build_position_constraint(
        bodies: &BodySet<N>,
        b1: BodyHandle,
        b2: BodyHandle,
        collider_pos_wrt_body1: &Isometry<N>,
        collider_pos_wrt_body2: &Isometry<N>,
        c: &TrackedContact<Point<N>>,
        margin1: N,
        margin2: N,
        constraints: &mut ConstraintSet<N>,
    ) {
        let body1 = bodies.body_part(b1);
        let body2 = bodies.body_part(b2);

        // FIXME: the points must be expressed in the local space of the body.
        // Thus, c.normal1/c.normal2 are not correct since they express the
        // contact in the local space of the collider instead of the body itself.
        // Could there be a way to design this differently in order to avoid this
        // difference of local spaces?
        let pos1 = body1.position();
        let pos2 = body2.position();
        let normal1 = pos1.inverse_transform_unit_vector(&c.contact.normal);
        let normal2 = -pos2.inverse_transform_unit_vector(&c.contact.normal);

        // XXX: we have to transform the contact kinematic
        // if the collider orientation is not the same
        // as the body's.
        let mut kinematic = c.kinematic.clone();
        let total_margin1 = kinematic.dilation1() + margin1;
        let total_margin2 = kinematic.dilation2() + margin2;
        kinematic.set_dilation1(total_margin1);
        kinematic.set_dilation2(total_margin2);
        kinematic.transform1(collider_pos_wrt_body1);
        kinematic.transform2(collider_pos_wrt_body2);

        constraints
            .position
            .unilateral
            .push(NonlinearUnilateralConstraint::new(
                b1,
                body1.status_dependent_parent_ndofs(),
                b2,
                body2.status_dependent_parent_ndofs(),
                normal1,
                normal2,
                kinematic,
            ));
    }
}

impl<N: Real> ContactModel<N> for SignoriniModel<N> {
    fn num_velocity_constraints(&self, c: &BodyContactManifold<N>) -> usize {
        c.manifold.len()
    }

    fn constraints(
        &mut self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        manifolds: &[BodyContactManifold<N>],
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    ) {
        let id_vel_ground = constraints.velocity.unilateral_ground.len();
        let id_vel = constraints.velocity.unilateral.len();

        for manifold in manifolds {
            for c in manifold.contacts() {
                if !Self::is_constraint_active(c, manifold) {
                    continue;
                }

                let _ = Self::build_velocity_constraint(
                    params,
                    bodies,
                    ext_vels,
                    manifold.body1,
                    manifold.body2,
                    c,
                    manifold.margin1,
                    manifold.margin2,
                    self.impulses.get(c.id),
                    self.impulses.entry_id(c.id),
                    ground_j_id,
                    j_id,
                    jacobians,
                    constraints,
                );

                Self::build_position_constraint(
                    bodies,
                    manifold.body1,
                    manifold.body2,
                    manifold.collider_pos_wrt_body1,
                    manifold.collider_pos_wrt_body2,
                    c,
                    manifold.margin1,
                    manifold.margin2,
                    constraints,
                );
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
