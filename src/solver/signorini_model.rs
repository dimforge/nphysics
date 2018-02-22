use std::marker::PhantomData;
use na::{self, DVector, Real};

use ncollide::query::TrackedContact;
use detection::BodyContactManifold;
use solver::helper;
use solver::{BilateralConstraint, BilateralGroundConstraint, ContactModel, ForceDirection,
             ImpulseCache, IntegrationParameters, UnilateralConstraint, UnilateralGroundConstraint};
use object::{BodyHandle, BodySet};
use math::Point;

pub struct SignoriniModel<N: Real> {
    impulses: ImpulseCache<N>,
}

impl<N: Real> SignoriniModel<N> {
    pub fn new() -> Self {
        SignoriniModel {
            impulses: ImpulseCache::new(),
        }
    }

    pub fn build_constraint(
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        b1: BodyHandle,
        b2: BodyHandle,
        c: &TrackedContact<Point<N>>,
        margin: N,
        impulse: N,
        ground_jacobian_id: &mut usize,
        jacobian_id: &mut usize,
        jacobians: &mut [N],
        out_ground_contacts: &mut Vec<UnilateralGroundConstraint<N>>,
        out_contacts: &mut Vec<UnilateralConstraint<N>>,
    ) {
        let b1 = bodies.body_part(b1);
        let b2 = bodies.body_part(b2);

        let assembly_id1 = b1.parent_companion_id();
        let assembly_id2 = b2.parent_companion_id();

        let mut geom = helper::constraint_pair_geometry(
            &b1,
            &b2,
            assembly_id1,
            assembly_id2,
            &c.contact.world1,
            &c.contact.world2,
            &ForceDirection::Linear(c.contact.normal),
            ext_vels,
            ground_jacobian_id,
            jacobian_id,
            jacobians,
        );

        let depth = c.contact.depth + margin;
        if depth < N::zero() {
            geom.rhs += -depth / params.dt;
        } else {
            let restitution = N::zero(); // FIXME: (rb1.restitution() + rb2.restitution()) * na::convert(0.5) * dvel;
            let stabilization = -depth / params.dt * params.erp;

            geom.rhs += na::inf(&restitution, &stabilization);
        }

        let warmstart = if geom.rhs < na::zero() {
            impulse * params.warmstart_coeff
        } else {
            na::zero()
        };

        if geom.is_ground_constraint() {
            out_ground_contacts.push(UnilateralGroundConstraint::new(geom, warmstart));
        } else {
            out_contacts.push(UnilateralConstraint::new(geom, warmstart));
        }
    }
}

impl<N: Real> ContactModel<N> for SignoriniModel<N> {
    fn nconstraints(&self, c: &BodyContactManifold<N>) -> usize {
        c.manifold.len()
    }

    fn build_constraints(
        &self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        manifolds: &[BodyContactManifold<N>],
        ground_jacobian_id: &mut usize,
        jacobian_id: &mut usize,
        jacobians: &mut [N],
        out_ground_contacts: &mut Vec<UnilateralGroundConstraint<N>>,
        out_contacts: &mut Vec<UnilateralConstraint<N>>,
        _: &mut Vec<BilateralGroundConstraint<N>>,
        _: &mut Vec<BilateralConstraint<N>>,
    ) {
        for manifold in manifolds {
            for c in manifold.contacts() {
                Self::build_constraint(
                    params,
                    bodies,
                    ext_vels,
                    manifold.b1,
                    manifold.b2,
                    c,
                    manifold.margin,
                    self.impulses.get(c.id),
                    ground_jacobian_id,
                    jacobian_id,
                    jacobians,
                    out_ground_contacts,
                    out_contacts,
                )
            }
        }
    }

    fn cache_impulses(
        &mut self,
        bodies: &BodySet<N>,
        manifolds: &[BodyContactManifold<N>],
        ground_contacts: &[UnilateralGroundConstraint<N>],
        contacts: &[UnilateralConstraint<N>],
        _: &[BilateralGroundConstraint<N>],
        _: &[BilateralConstraint<N>],
    ) {
        let mut curr_contact = 0;
        let mut curr_ground_contact = 0;

        for m in manifolds {
            let b1 = bodies.body_part(m.b1);
            let b2 = bodies.body_part(m.b2);

            if helper::constraints_are_ground_constraints(&b1, &b2) {
                for c in m.contacts() {
                    let impulse = ground_contacts[curr_ground_contact].impulse;
                    self.impulses.set(c.id, impulse);
                    curr_ground_contact += 1;
                }
            } else {
                for c in m.contacts() {
                    let impulse = contacts[curr_contact].impulse;
                    self.impulses.set(c.id, impulse);
                    curr_contact += 1;
                }
            }
        }
    }
}
