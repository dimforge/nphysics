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

    pub fn build_constraint(
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
        cache_id: usize,
        ground_jacobian_id: &mut usize,
        jacobian_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    ) -> bool {
        let body1 = bodies.body_part(b1);
        let body2 = bodies.body_part(b2);

        let assembly_id1 = body1.parent_companion_id();
        let assembly_id2 = body2.parent_companion_id();

        let mut depth = c.contact.depth + margin1 + margin2;

        let mut geom = helper::constraint_pair_geometry(
            &body1,
            &body2,
            assembly_id1,
            assembly_id2,
            &(c.contact.world1 + c.contact.normal.unwrap() * margin1),
            &(c.contact.world2 - c.contact.normal.unwrap() * margin2),
            &ForceDirection::Linear(c.contact.normal),
            ext_vels,
            ground_jacobian_id,
            jacobian_id,
            jacobians,
        );

        if depth < N::zero() {
            geom.rhs += -depth / params.dt;
        } else {
            let restitution = N::zero(); // FIXME: (rb1.restitution() + rb2.restitution()) * na::convert(0.5) * dvel;
            let stabilization = if depth > na::convert(0.005) {
                -depth * params.erp / params.dt
            } else {
                N::zero()
            };

            geom.rhs += na::inf(&restitution, &stabilization);
        }

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

        constraints
            .position
            .unilateral
            .push(NonlinearUnilateralConstraint::new(
                b1,
                geom.ndofs1,
                b2,
                geom.ndofs2,
                local1,
                normal1,
                margin1,
                local2,
                normal2,
                margin2,
                c.kinematic,
            ));

        let warmstart = impulse * params.warmstart_coeff;
        if geom.is_ground_constraint() {
            constraints
                .velocity
                .unilateral_ground
                .push(UnilateralGroundConstraint::new(geom, warmstart, cache_id));

            true
        } else {
            constraints
                .velocity
                .unilateral
                .push(UnilateralConstraint::new(geom, warmstart, cache_id));

            false
        }
    }
}

impl<N: Real> ContactModel<N> for SignoriniModel<N> {
    fn nconstraints(&self, c: &BodyContactManifold<N>) -> usize {
        c.manifold.len()
    }

    fn build_constraints(
        &mut self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        manifolds: &[BodyContactManifold<N>],
        ground_jacobian_id: &mut usize,
        jacobian_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N>,
    ) {
        let id_vel_ground = constraints.velocity.unilateral_ground.len();
        let id_vel = constraints.velocity.unilateral.len();

        for manifold in manifolds {
            let deepest_contact_normal = &manifold.deepest_contact().contact.normal;
            for c in manifold.contacts() {
                let _ = Self::build_constraint(
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
                    ground_jacobian_id,
                    jacobian_id,
                    jacobians,
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
            self.impulses[c.cache_id] = c.impulse;
        }

        for c in contacts {
            self.impulses[c.cache_id] = c.impulse;
        }
    }
}
