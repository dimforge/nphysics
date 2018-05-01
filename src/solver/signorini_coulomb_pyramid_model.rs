use std::ops::Range;
use alga::linear::FiniteDimInnerSpace;
use na::{self, DVector, Real, Unit};

use detection::ColliderContactManifold;
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
    fn num_velocity_constraints(&self, c: &ColliderContactManifold<N>) -> usize {
        DIM * c.len()
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
        let id_friction_ground = constraints.velocity.bilateral_ground.len();
        let id_friction = constraints.velocity.bilateral.len();

        for manifold in manifolds {
            let body1 = bodies.body_part(manifold.collider1.data().body());
            let body2 = bodies.body_part(manifold.collider2.data().body());

            let friction1 = manifold.collider1.data().material().friction;
            let friction2 = manifold.collider2.data().material().friction;
            let friction = (friction1 + friction2) * na::convert(0.5);

            for c in manifold.contacts() {
                // if !SignoriniModel::is_constraint_active(c, manifold) {
                //     continue;
                // }

                let impulse = self.impulses.get(c.id);
                let impulse_id = self.impulses.entry_id(c.id);

                let ground_constraint = SignoriniModel::build_velocity_constraint(
                    params,
                    bodies,
                    manifold,
                    ext_vels,
                    c,
                    impulse[0],
                    impulse_id,
                    ground_j_id,
                    j_id,
                    jacobians,
                    constraints,
                );

                SignoriniModel::build_position_constraint(bodies, manifold, c, constraints);

                let dependency;

                if ground_constraint {
                    let constraints = &constraints.velocity.unilateral_ground;
                    dependency = constraints.len() - 1;
                } else {
                    let constraints = &constraints.velocity.unilateral;
                    dependency = constraints.len() - 1;
                }

                let assembly_id1 = body1.parent_companion_id();
                let assembly_id2 = body2.parent_companion_id();

                // Generate friction constraints.
                let limits = ImpulseLimits::Dependent {
                    dependency: dependency,
                    coeff: friction,
                };

                let mut i = 1;

                // FIXME: this compute the contact point locations (with margins) several times,
                // it was already computed for the signorini law.
                let center1 = c.contact.world1
                    + c.contact.normal.unwrap() * manifold.collider1.data().margin();
                let center2 = c.contact.world2
                    - c.contact.normal.unwrap() * manifold.collider2.data().margin();

                Vector::orthonormal_subspace_basis(&[c.contact.normal.unwrap()], |friction_dir| {
                    let dir = ForceDirection::Linear(Unit::new_unchecked(*friction_dir));

                    // FIXME: will this compute the momentum twice ?
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

                    let rhs = helper::constraint_pair_velocity(
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

                    let warmstart = impulse[i] * params.warmstart_coeff;

                    if geom.is_ground_constraint() {
                        let constraint = BilateralGroundConstraint::new(
                            geom,
                            assembly_id1,
                            assembly_id2,
                            limits,
                            rhs,
                            warmstart,
                            impulse_id * DIM + i,
                        );
                        constraints.velocity.bilateral_ground.push(constraint);
                    } else {
                        let constraint = BilateralConstraint::new(
                            geom,
                            assembly_id1,
                            assembly_id2,
                            limits,
                            rhs,
                            warmstart,
                            impulse_id * DIM + i,
                        );
                        constraints.velocity.bilateral.push(constraint);
                    }

                    i += 1;

                    true
                });
            }
        }

        self.vel_ground_rng = id_vel_ground..constraints.velocity.unilateral_ground.len();
        self.vel_rng = id_vel..constraints.velocity.unilateral.len();
        self.friction_ground_rng = id_friction_ground..constraints.velocity.bilateral_ground.len();
        self.friction_rng = id_friction..constraints.velocity.bilateral.len();
    }

    fn cache_impulses(&mut self, constraints: &ConstraintSet<N>) {
        let ground_contacts = &constraints.velocity.unilateral_ground[self.vel_ground_rng.clone()];
        let contacts = &constraints.velocity.unilateral[self.vel_rng.clone()];
        let ground_friction =
            &constraints.velocity.bilateral_ground[self.friction_ground_rng.clone()];
        let friction = &constraints.velocity.bilateral[self.friction_rng.clone()];

        for c in ground_contacts {
            self.impulses[c.impulse_id][0] = c.impulse;
        }

        for c in contacts {
            self.impulses[c.impulse_id][0] = c.impulse;
        }

        for c in ground_friction {
            self.impulses[c.impulse_id / DIM][c.impulse_id % DIM] = c.impulse;
        }

        for c in friction {
            self.impulses[c.impulse_id / DIM][c.impulse_id % DIM] = c.impulse;
        }
    }
}
