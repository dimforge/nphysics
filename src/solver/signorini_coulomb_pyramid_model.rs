use alga::linear::FiniteDimInnerSpace;
use na::{self, DVector, Real, Unit};

use detection::BodyContactManifold;
use solver::helper;
use solver::{BilateralConstraint, BilateralGroundConstraint, ContactModel, ForceDirection,
             ImpulseCache, ImpulseLimits, IntegrationParameters, SignoriniModel,
             UnilateralConstraint, UnilateralGroundConstraint};
use object::BodySet;
use math::{Vector, DIM};

pub struct SignoriniCoulombPyramidModel<N: Real> {
    impulses: ImpulseCache<Vector<N>>,
}

impl<N: Real> SignoriniCoulombPyramidModel<N> {
    pub fn new() -> Self {
        SignoriniCoulombPyramidModel {
            impulses: ImpulseCache::new(),
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
        manifolds: &[BodyContactManifold<N>],
        ground_jacobian_id: &mut usize,
        jacobian_id: &mut usize,
        jacobians: &mut [N],
        out_ground_contacts: &mut Vec<UnilateralGroundConstraint<N>>,
        out_contacts: &mut Vec<UnilateralConstraint<N>>,
        out_ground_frictions: &mut Vec<BilateralGroundConstraint<N>>,
        out_frictions: &mut Vec<BilateralConstraint<N>>,
    ) {
        for manifold in manifolds {
            let b1 = bodies.body_part(manifold.b1);
            let b2 = bodies.body_part(manifold.b2);

            for c in manifold.contacts() {
                let impulse = self.impulses.get(c.id);
                let nctcts = out_contacts.len();

                SignoriniModel::build_constraint(
                    params,
                    bodies,
                    ext_vels,
                    manifold.b1,
                    manifold.b2,
                    c,
                    manifold.margin,
                    impulse[0],
                    ground_jacobian_id,
                    jacobian_id,
                    jacobians,
                    out_ground_contacts,
                    out_contacts,
                );

                let dependency;
                let warmstart_enabled;

                if out_contacts.len() == nctcts {
                    dependency = out_ground_contacts.len() - 1;
                    warmstart_enabled = !out_ground_contacts[dependency].impulse.is_zero();
                } else {
                    dependency = out_contacts.len() - 1;
                    warmstart_enabled = !out_contacts[dependency].impulse.is_zero();
                }

                let assembly_id1 = b1.parent_companion_id();
                let assembly_id2 = b2.parent_companion_id();

                // Generate friction constraints.
                let friction_coeff = na::convert(0.5); // XXX hard-coded friction coefficient.
                let limits = ImpulseLimits::Dependent {
                    dependency: dependency,
                    coeff: friction_coeff,
                };

                let mut i = 1;
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

                    let warmstart = if warmstart_enabled {
                        impulse[i] * params.warmstart_coeff
                    } else {
                        na::zero()
                    };

                    if geom.is_ground_constraint() {
                        let constraint = BilateralGroundConstraint::new(geom, limits, warmstart);
                        out_ground_frictions.push(constraint);
                    } else {
                        let constraint = BilateralConstraint::new(geom, limits, warmstart);
                        out_frictions.push(constraint);
                    }

                    i += 1;

                    true
                });
            }
        }
    }

    fn cache_impulses(
        &mut self,
        bodies: &BodySet<N>,
        manifolds: &[BodyContactManifold<N>],
        ground_contacts: &[UnilateralGroundConstraint<N>],
        contacts: &[UnilateralConstraint<N>],
        friction_ground_constraints: &[BilateralGroundConstraint<N>],
        friction_constraints: &[BilateralConstraint<N>],
    ) {
        let mut curr_contact = 0;
        let mut curr_ground_contact = 0;
        const DIM_FRICTION: usize = DIM - 1;

        for m in manifolds {
            let b1 = bodies.body_part(m.b1);
            let b2 = bodies.body_part(m.b2);

            if helper::constraints_are_ground_constraints(&b1, &b2) {
                for c in m.contacts() {
                    let mut impulse = Vector::zeros();
                    impulse[0] = ground_contacts[curr_ground_contact].impulse;

                    for i in 0..DIM_FRICTION {
                        let constraint_id = curr_ground_contact * DIM_FRICTION + i;
                        impulse[i + 1] = friction_ground_constraints[constraint_id].impulse;
                    }

                    self.impulses.set(c.id, impulse);
                    curr_ground_contact += 1;
                }
            } else {
                for c in m.contacts() {
                    let mut impulse = Vector::zeros();
                    impulse[0] = contacts[curr_contact].impulse;

                    for i in 0..DIM - 1 {
                        let constraint_id = curr_contact * DIM_FRICTION + i;
                        impulse[i + 1] = friction_constraints[constraint_id].impulse;
                    }

                    self.impulses.set(c.id, impulse);
                    curr_contact += 1;
                }
            }
        }
    }
}
