use alga::linear::FiniteDimInnerSpace;
use na::{self, DVector, RealField, Unit};
use ncollide::query::ContactId;
use slotmap::Key;
use std::ops::Range;

use crate::detection::ColliderContactManifold;
use crate::material::{Material, MaterialContext, MaterialsCoefficientsTable};
use crate::math::{Vector, DIM};
use crate::object::{Body, BodySet, ColliderHandle};
use crate::solver::helper;
use crate::solver::{
    BilateralConstraint, BilateralGroundConstraint, ConstraintSet, ContactModel, ForceDirection,
    ImpulseCache, ImpulseLimits, IntegrationParameters, SignoriniModel,
};

/// A contact model generating one non-penetration constraint and two friction constraints per contact.
///
/// This contact model approximates the friction cone at a contact with pyramid.
pub struct SignoriniCoulombPyramidModel<N: RealField> {
    impulses: ImpulseCache<Vector<N>>,
    vel_ground_rng: Range<usize>,
    vel_rng: Range<usize>,
    friction_ground_rng: Range<usize>,
    friction_rng: Range<usize>,
}

impl<N: RealField> SignoriniCoulombPyramidModel<N> {
    /// Initialize a new signorini-coulomb-pyramid contact model.
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

impl<N: RealField> Default for SignoriniCoulombPyramidModel<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<N: RealField, Bodies: BodySet<N>, CollHandle: ColliderHandle>
    ContactModel<N, Bodies, CollHandle> for SignoriniCoulombPyramidModel<N>
{
    fn num_velocity_constraints(
        &self,
        c: &ColliderContactManifold<N, Bodies::Handle, CollHandle>,
    ) -> usize {
        DIM * c.len()
    }

    fn constraints(
        &mut self,
        parameters: &IntegrationParameters<N>,
        coefficients: &MaterialsCoefficientsTable<N>,
        bodies: &Bodies,
        ext_vels: &DVector<N>,
        manifolds: &[ColliderContactManifold<N, Bodies::Handle, CollHandle>],
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        constraints: &mut ConstraintSet<N, Bodies::Handle, CollHandle, ContactId>,
    ) {
        let id_vel_ground = constraints.velocity.unilateral_ground.len();
        let id_vel = constraints.velocity.unilateral.len();
        let id_friction_ground = constraints.velocity.bilateral_ground.len();
        let id_friction = constraints.velocity.bilateral.len();

        for manifold in manifolds {
            let body1 = try_continue!(bodies.get(manifold.body1()));
            let body2 = try_continue!(bodies.get(manifold.body2()));

            for c in manifold.contacts() {
                let handle1 = manifold.body_part1(c.kinematic.feature1());
                let handle2 = manifold.body_part2(c.kinematic.feature2());
                let part1 = try_continue!(body1.part(handle1.1));
                let part2 = try_continue!(body2.part(handle2.1));

                let material1 = manifold.collider1.material();
                let material2 = manifold.collider2.material();
                let context1 = MaterialContext::new(
                    manifold.collider1.shape(),
                    manifold.collider1.position(),
                    c,
                    true,
                );
                let context2 = MaterialContext::new(
                    manifold.collider2.shape(),
                    manifold.collider2.position(),
                    c,
                    false,
                );
                let props =
                    Material::combine(coefficients, material1, context1, material2, context2);

                // if !SignoriniModel::is_constraint_active(c, manifold) {
                //     continue;
                // }

                let impulse = self.impulses.get(c.id).cloned().unwrap_or(Vector::zeros());

                let ground_constraint = SignoriniModel::build_velocity_constraint(
                    parameters,
                    body1,
                    part1,
                    handle1,
                    body2,
                    part2,
                    handle2,
                    &props,
                    manifold,
                    ext_vels,
                    c,
                    impulse[0],
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

                let assembly_id1 = body1.companion_id();
                let assembly_id2 = body2.companion_id();

                // Generate friction constraints.
                let limits = ImpulseLimits::Dependent {
                    dependency,
                    coeff: props.friction.0,
                };

                let mut i = 1;

                // FIXME: this compute the contact point locations (with margins) several times,
                // it was already computed for the signorini law.
                let center1 =
                    c.contact.world1 + c.contact.normal.into_inner() * manifold.collider1.margin();
                let center2 =
                    c.contact.world2 - c.contact.normal.into_inner() * manifold.collider2.margin();
                let (ext_vels1, ext_vels2) =
                    helper::split_ext_vels(body1, body2, assembly_id1, assembly_id2, ext_vels);

                Vector::orthonormal_subspace_basis(
                    &[c.contact.normal.into_inner()],
                    |friction_dir| {
                        let dir = ForceDirection::Linear(Unit::new_unchecked(*friction_dir));
                        let mut rhs = friction_dir.dot(&props.surface_velocity);

                        // FIXME: will this compute the momentum twice ?
                        let geom = helper::constraint_pair_geometry(
                            body1,
                            part1,
                            handle1,
                            body2,
                            part2,
                            handle2,
                            &center1,
                            &center2,
                            &dir,
                            ground_j_id,
                            j_id,
                            jacobians,
                            Some(&ext_vels1),
                            Some(&ext_vels2),
                            Some(&mut rhs),
                        );

                        let warmstart = impulse[i] * parameters.warmstart_coeff;

                        if geom.is_ground_constraint() {
                            let constraint = BilateralGroundConstraint::new(
                                geom,
                                assembly_id1,
                                assembly_id2,
                                limits,
                                rhs,
                                warmstart,
                                c.id,
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
                                c.id,
                            );
                            constraints.velocity.bilateral.push(constraint);
                        }

                        i += 1;

                        true
                    },
                );
            }
        }

        self.vel_ground_rng = id_vel_ground..constraints.velocity.unilateral_ground.len();
        self.vel_rng = id_vel..constraints.velocity.unilateral.len();
        self.friction_ground_rng = id_friction_ground..constraints.velocity.bilateral_ground.len();
        self.friction_rng = id_friction..constraints.velocity.bilateral.len();
    }

    fn cache_impulses(
        &mut self,
        constraints: &ConstraintSet<N, Bodies::Handle, CollHandle, ContactId>,
    ) {
        let ground_contacts = &constraints.velocity.unilateral_ground[self.vel_ground_rng.clone()];
        let contacts = &constraints.velocity.unilateral[self.vel_rng.clone()];
        let ground_friction =
            &constraints.velocity.bilateral_ground[self.friction_ground_rng.clone()];
        let friction = &constraints.velocity.bilateral[self.friction_rng.clone()];

        for c in ground_contacts {
            if !c.impulse_id.is_null() {
                let _ = self.impulses.insert(c.impulse_id, Vector::zeros());
                self.impulses[c.impulse_id][0] = c.impulse;
            }
        }

        for c in contacts {
            if !c.impulse_id.is_null() {
                let _ = self.impulses.insert(c.impulse_id, Vector::zeros());
                self.impulses[c.impulse_id][0] = c.impulse;
            }
        }

        let mut dim = 0;
        for c in ground_friction {
            if !c.impulse_id.is_null() {
                self.impulses[c.impulse_id][1 + dim % (DIM - 1)] = c.impulse;
                dim += 1;
            }
        }

        for c in friction {
            if !c.impulse_id.is_null() {
                self.impulses[c.impulse_id][1 + dim % (DIM - 1)] = c.impulse;
                dim += 1;
            }
        }
    }
}
