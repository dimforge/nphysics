use std::ops::Range;
use alga::linear::FiniteDimInnerSpace;
use na::{self, DVector, Real, Unit};

use detection::BodyContactManifold;
use solver::helper;
use solver::{BilateralConstraint, BilateralGroundConstraint, ConstraintSet, ContactModel,
             ForceDirection, ImpulseCache, ImpulseLimits, IntegrationParameters, SignoriniModel};
use object::BodySet;
use math::{Vector, DIM};

pub struct SignoriniCoulombPyramidModel<N: Real> {
    impulses: ImpulseCache<Vector<N>>,
    vel_ground_rng: Range<usize>,
    vel_rng: Range<usize>,
    friction_ground_rng: Range<usize>,
    friction_rng: Range<usize>,
}

impl<N: Real> SignoriniCoulombPyramidModel<N> {
    pub fn new() -> Self {
        SignoriniCoulombPyramidModel {
            impulses: ImpulseCache::new(),
            vel_ground_rng: 0..0,
            vel_rng: 0..0,
            friction_ground_rng: 0..0,
            friction_rng: 0..0,
        }
    }
}

impl<N: Real> ContactModel<N> for SignoriniCoulombPyramidModel<N> {
    fn nconstraints(&self, c: &BodyContactManifold<N>) -> usize {
        DIM * c.len()
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
            let b1 = bodies.body_part(manifold.b1);
            let b2 = bodies.body_part(manifold.b2);

            for c in manifold.contacts() {
                let impulse = self.impulses.get(c.id);
                let impulse_id = self.impulses.entry_id(c.id);

                let ground_constraint = SignoriniModel::build_constraint(
                    params,
                    bodies,
                    ext_vels,
                    manifold.b1,
                    manifold.b2,
                    c,
                    manifold.margin,
                    impulse[0],
                    impulse_id,
                    ground_jacobian_id,
                    jacobian_id,
                    jacobians,
                    constraints,
                );

                let dependency;
                let warmstart_enabled;

                if ground_constraint {
                    let constraints = &constraints.velocity.unilateral_ground;
                    dependency = constraints.len() - 1;
                    warmstart_enabled = !constraints[dependency].impulse.is_zero();
                } else {
                    let constraints = &constraints.velocity.unilateral;
                    dependency = constraints.len() - 1;
                    warmstart_enabled = !constraints[dependency].impulse.is_zero();
                }

                let assembly_id1 = b1.parent_companion_id();
                let assembly_id2 = b2.parent_companion_id();

                // Generate friction constraints.
                let friction_coeff = na::convert(0.3); // XXX hard-coded friction coefficient.
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
                        let constraint =
                            BilateralGroundConstraint::new(geom, limits, warmstart, impulse_id + i);
                        constraints.velocity.bilateral_ground.push(constraint);
                    } else {
                        let constraint =
                            BilateralConstraint::new(geom, limits, warmstart, impulse_id + i);
                        constraints.velocity.bilateral.push(constraint);
                    }

                    i += 1;

                    true
                });
            }
        }

        self.vel_ground_rng = id_vel_ground..constraints.velocity.unilateral_ground.len();
        self.vel_rng = id_vel..constraints.velocity.unilateral.len();
    }

    fn cache_impulses(&mut self, constraints: &ConstraintSet<N>) {
        let ground_contacts = &constraints.velocity.unilateral_ground[self.vel_ground_rng.clone()];
        let contacts = &constraints.velocity.unilateral[self.vel_rng.clone()];
        let ground_friction =
            &constraints.velocity.unilateral_ground[self.friction_ground_rng.clone()];
        let friction = &constraints.velocity.unilateral[self.friction_rng.clone()];

        for c in ground_contacts {
            self.impulses[c.cache_id][0] = c.impulse;
        }

        for c in contacts {
            self.impulses[c.cache_id][0] = c.impulse;
        }

        for c in ground_friction {
            self.impulses[c.cache_id / DIM][c.cache_id % DIM] = c.impulse;
        }

        for c in friction {
            self.impulses[c.cache_id / DIM][c.cache_id % DIM] = c.impulse;
        }
    }
}
