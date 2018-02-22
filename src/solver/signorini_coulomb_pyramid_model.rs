use alga::linear::FiniteDimInnerSpace;
use na::{self, DVector, Real, Unit};

use detection::BodyContactManifold;
use solver::helper;
use solver::{BilateralConstraint2, BilateralGroundConstraint, ContactModel, ForceDirection,
             ImpulseLimits, IntegrationParameters, SignoriniModel, UnilateralConstraint2,
             UnilateralGroundConstraint};
use object::BodySet;
use math::{Vector, DIM};

pub struct SignoriniCoulombPyramidModel<N: Real> {
    signorini: SignoriniModel<N>,
}

impl<N: Real> SignoriniCoulombPyramidModel<N> {
    pub fn new() -> Self {
        SignoriniCoulombPyramidModel {
            signorini: SignoriniModel::new(),
        }
    }
}

impl<N: Real> ContactModel<N> for SignoriniCoulombPyramidModel<N> {
    fn nconstraints(&self, c: &BodyContactManifold<N>) -> usize {
        DIM * c.len()
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
        out_contacts: &mut Vec<UnilateralConstraint2<N>>,
        out_ground_frictions: &mut Vec<BilateralGroundConstraint<N>>,
        out_frictions: &mut Vec<BilateralConstraint2<N>>,
    ) {
        let b1 = bodies.body_part(manifold.b1);
        let b2 = bodies.body_part(manifold.b2);

        for c in manifold.contacts() {
            let nctcts = out_contacts.len();
            let active = self.signorini.build_constraint(
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

            if !active {
                continue;
            }

            let dependency;
            if out_contacts.len() == nctcts {
                dependency = out_ground_contacts.len() - 1;
            } else {
                dependency = out_contacts.len() - 1;
            }

            let assembly_id1 = b1.parent_companion_id();
            let assembly_id2 = b2.parent_companion_id();

            // Generate friction constraints.
            let friction_coeff = na::convert(0.5); // XXX hard-coded friction coefficient.
            let limits = ImpulseLimits::Dependent {
                dependency: dependency,
                coeff: friction_coeff,
            };

            Vector::orthonormal_subspace_basis(&[c.contact.normal.unwrap()], |friction_dir| {
                let geom = helper::constraint_pair_geometry(
                    &b1,
                    &b2,
                    assembly_id1,
                    assembly_id2,
                    &c.contact.world1,
                    &c.contact.world2,
                    &ForceDirection::Linear(Unit::new_unchecked(*friction_dir)),
                    ext_vels,
                    ground_jacobian_id,
                    jacobian_id,
                    jacobians,
                );

                if let Some(geom) = geom {
                    if geom.ndofs1 == 0 || geom.ndofs2 == 0 {
                        let constraint = BilateralGroundConstraint::new(geom, limits);
                        out_ground_frictions.push(constraint);
                    } else {
                        let constraint = BilateralConstraint2::new(geom, limits);
                        out_frictions.push(constraint);
                    }
                }

                true
            });
        }
    }
}
