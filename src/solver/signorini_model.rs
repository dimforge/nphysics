use std::marker::PhantomData;
use na::{self, DVector, Real};

use detection::ContactConstraint;
use solver::helper;
use solver::{BilateralConstraint2, BilateralGroundConstraint, ContactModel, ForceDirection,
             IntegrationParameters, UnilateralConstraint2, UnilateralGroundConstraint};
use object::BodySet;

pub struct SignoriniModel<N: Real> {
    _phantom: PhantomData<N>,
}

impl<N: Real> SignoriniModel<N> {
    pub fn new() -> Self {
        SignoriniModel {
            _phantom: PhantomData,
        }
    }
}

impl<N: Real> ContactModel<N> for SignoriniModel<N> {
    fn nconstraints(&self) -> usize {
        1
    }

    fn build_constraints(
        &self,
        params: &IntegrationParameters<N>,
        bodies: &BodySet<N>,
        ext_vels: &DVector<N>,
        ground_jacobian_id: &mut usize,
        jacobian_id: &mut usize,
        jacobians: &mut [N],
        c: &ContactConstraint<N>,
        out_ground_contacts: &mut Vec<UnilateralGroundConstraint<N>>,
        out_contacts: &mut Vec<UnilateralConstraint2<N>>,
        _: &mut Vec<BilateralGroundConstraint<N>>,
        _: &mut Vec<BilateralConstraint2<N>>,
    ) -> bool {
        let b1 = bodies.body_part(c.b1);
        let b2 = bodies.body_part(c.b2);

        let assembly_id1 = b1.parent_companion_id();
        let assembly_id2 = b2.parent_companion_id();

        let geom = helper::constraint_pair_geometry(
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

        if let Some(mut geom) = geom {
            if c.contact.depth < N::zero() {
                geom.rhs += -c.contact.depth / params.dt;
            } else {
                let restitution = N::zero(); // FIXME: (rb1.restitution() + rb2.restitution()) * na::convert(0.5) * dvel;
                let stabilization = -c.contact.depth / params.dt * params.erp;

                geom.rhs += na::inf(&restitution, &stabilization);
            }

            if geom.ndofs1 == 0 || geom.ndofs2 == 0 {
                out_ground_contacts.push(UnilateralGroundConstraint::new(geom));
            } else {
                out_contacts.push(UnilateralConstraint2::new(geom));
            }

            true
        } else {
            false
        }
    }
}
