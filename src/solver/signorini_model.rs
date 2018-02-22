use std::marker::PhantomData;
use na::{self, DVector, Real};

use ncollide::query::Contact;
use detection::BodyContactManifold;
use solver::helper;
use solver::{BilateralConstraint, BilateralGroundConstraint, ContactModel, ForceDirection,
             IntegrationParameters, UnilateralConstraint, UnilateralGroundConstraint};
use object::{BodyHandle, BodySet};
use math::Point;

pub struct SignoriniModel<N: Real> {
    impulses: Vec<N>,
}

impl<N: Real> SignoriniModel<N> {
    pub fn new() -> Self {
        SignoriniModel {
            impulses: Vec::new(),
        }
    }

    pub fn build_constraint(
        &self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        b1: BodyHandle,
        b2: BodyHandle,
        c: &Contact<Point<N>>,
        ground_jacobian_id: &mut usize,
        jacobian_id: &mut usize,
        jacobians: &mut [N],
        out_ground_contacts: &mut Vec<UnilateralGroundConstraint<N>>,
        out_contacts: &mut Vec<UnilateralConstraint<N>>,
    ) -> bool {
        let b1 = bodies.body_part(b1);
        let b2 = bodies.body_part(b2);

        let assembly_id1 = b1.parent_companion_id();
        let assembly_id2 = b2.parent_companion_id();

        let geom = helper::constraint_pair_geometry(
            &b1,
            &b2,
            assembly_id1,
            assembly_id2,
            &c.world1,
            &c.world2,
            &ForceDirection::Linear(c.normal),
            ext_vels,
            ground_jacobian_id,
            jacobian_id,
            jacobians,
        );

        if let Some(mut geom) = geom {
            if c.depth < N::zero() {
                geom.rhs += -c.depth / params.dt;
            } else {
                let restitution = N::zero(); // FIXME: (rb1.restitution() + rb2.restitution()) * na::convert(0.5) * dvel;
                let stabilization = -c.depth / params.dt * params.erp;

                geom.rhs += na::inf(&restitution, &stabilization);
            }

            if geom.ndofs1 == 0 || geom.ndofs2 == 0 {
                out_ground_contacts.push(UnilateralGroundConstraint::new(geom));
            } else {
                out_contacts.push(UnilateralConstraint::new(geom));
            }

            true
        } else {
            false
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
        manifold: &BodyContactManifold<N>,
        ground_jacobian_id: &mut usize,
        jacobian_id: &mut usize,
        jacobians: &mut [N],
        out_ground_contacts: &mut Vec<UnilateralGroundConstraint<N>>,
        out_contacts: &mut Vec<UnilateralConstraint<N>>,
        _: &mut Vec<BilateralGroundConstraint<N>>,
        _: &mut Vec<BilateralConstraint<N>>,
    ) {
        for c in manifold.contacts() {
            let _ = self.build_constraint(
                params,
                bodies,
                ext_vels,
                manifold.b1,
                manifold.b2,
                &c.contact,
                ground_jacobian_id,
                jacobian_id,
                jacobians,
                out_ground_contacts,
                out_contacts,
            );
        }
    }
}
