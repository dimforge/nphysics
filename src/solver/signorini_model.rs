use std::ops::Range;
use alga::linear::ProjectiveTransformation;
use na::{self, DVector, Real, Unit};

use ncollide::query::TrackedContact;
use detection::BodyContactManifold;
use solver::helper;
use solver::{ConstraintSet, ContactModel, ForceDirection, ImpulseCache, IntegrationParameters,
             NonlinearUnilateralConstraint, UnilateralConstraint, UnilateralGroundConstraint};
use object::{BodyHandle, BodySet};
use math::{Point, Vector};

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
        deepest_normal: &Unit<Vector<N>>,
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

        let mut depth = c.contact.depth + margin1 + margin2;
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

        if depth < N::zero() {
            rhs += -depth / params.dt;
        } else {
            rhs += N::zero(); // FIXME: (rb1.restitution() + rb2.restitution()) * na::convert(0.5) * dvel;
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

    pub fn build_position_constraint(
        bodies: &BodySet<N>,
        b1: BodyHandle,
        b2: BodyHandle,
        c: &TrackedContact<Point<N>>,
        margin1: N,
        margin2: N,
        constraints: &mut ConstraintSet<N>,
    ) {
        let body1 = bodies.body_part(b1);
        let body2 = bodies.body_part(b2);

        let assembly_id1 = body1.parent_companion_id();
        let assembly_id2 = body2.parent_companion_id();
        // FIXME: the points must be expressed in the local space of the body.
        // Thus, c.local1/c.local2/c.normal1/c.normal2 are not correct since they express the
        // contact in the local space of the collider instead of the body itself.
        // Could there be a way to design this differently in order to avoid this
        // difference of local spaces?
        let pos1 = body1.position();
        let pos2 = body2.position();
        let local1 = pos1.inverse_transform_point(&c.contact.world1);
        let local2 = pos2.inverse_transform_point(&c.contact.world2);
        let normal1 = Unit::new_unchecked(pos1.inverse_transform_vector(c.contact.normal.as_ref()));
        let normal2 =
            -Unit::new_unchecked(pos2.inverse_transform_vector(c.contact.normal.as_ref()));

        let contact_id = constraints.position.unilateral.len();

        // XXX: we have to change the coordinate system
        // if the collider orientation is not the same
        // as the body's.
        // XXX: the same remark applies to the contact kinematic
        // information (e.g. for the line directions).
        let ncone1 = c.normals1.clone();
        let ncone2 = c.normals2.clone();

        constraints
            .position
            .unilateral
            .push(NonlinearUnilateralConstraint::new(
                b1,
                body1.status_dependent_parent_ndofs(),
                b2,
                body2.status_dependent_parent_ndofs(),
                local1,
                normal1,
                ncone1,
                margin1,
                local2,
                normal2,
                ncone2,
                margin2,
                c.kinematic,
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
            let deepest_contact = &manifold.deepest_contact();
            let deepest_contact_normal = &deepest_contact.contact.normal;

            for c in manifold.contacts() {
                let _ = Self::build_velocity_constraint(
                    params,
                    bodies,
                    ext_vels,
                    manifold.b1,
                    manifold.b2,
                    c,
                    deepest_contact_normal,
                    manifold.margin1,
                    manifold.margin2,
                    self.impulses.get(c.id),
                    self.impulses.entry_id(c.id),
                    ground_j_id,
                    j_id,
                    jacobians,
                    constraints,
                );
            }

            Self::build_position_constraint(
                bodies,
                manifold.b1,
                manifold.b2,
                deepest_contact,
                manifold.margin1,
                manifold.margin2,
                constraints,
            );
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
