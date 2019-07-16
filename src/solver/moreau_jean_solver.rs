use std::collections::HashSet;
use na::{DVector, RealField};
use ncollide::query::ContactId;

use crate::counters::Counters;
use crate::detection::ColliderContactManifold;
use crate::joint::{JointConstraint, JointConstraintSet};
use crate::object::{BodySet, Body, BodyHandle, ColliderHandle, ColliderSet};
use crate::material::MaterialsCoefficientsTable;
use crate::solver::{ConstraintSet, ContactModel, IntegrationParameters, NonlinearSORProx, SORProx};
use crate::world::ColliderWorld;

/// Moreau-Jean time-stepping scheme.
pub struct MoreauJeanSolver<N: RealField, Bodies: BodySet<N>, CollHandle: ColliderHandle> {
    jacobians: Vec<N>,
    // FIXME: use a Vec or a DVector?
    mj_lambda_vel: DVector<N>,
    ext_vels: DVector<N>,
    contact_model: Box<ContactModel<N, Bodies, CollHandle>>,
    contact_constraints: ConstraintSet<N, Bodies::Handle, CollHandle, ContactId>,
    joint_constraints: ConstraintSet<N, Bodies::Handle, CollHandle, usize>,
    internal_constraints: Vec<Bodies::Handle>,
}

impl<N: RealField, Bodies: BodySet<N>, CollHandle: ColliderHandle> MoreauJeanSolver<N, Bodies, CollHandle> {
    /// Create a new time-stepping scheme with the given contact model.
    pub fn new(contact_model: Box<ContactModel<N, Bodies, CollHandle>>) -> Self {
        MoreauJeanSolver {
            jacobians: Vec::new(),
            mj_lambda_vel: DVector::zeros(0),
            ext_vels: DVector::zeros(0),
            contact_model,
            contact_constraints: ConstraintSet::new(),
            joint_constraints: ConstraintSet::new(),
            internal_constraints: Vec::new(),
        }
    }

    /// Sets the contact model.
    pub fn set_contact_model(&mut self, model: Box<ContactModel<N, Bodies, CollHandle>>) {
        self.contact_model = model
    }

    /// Perform one step of the time-stepping scheme.
    pub fn step<Colliders: ColliderSet<N, Bodies::Handle, Handle = CollHandle>, Constraints: JointConstraintSet<N, Bodies>>(
        &mut self,
        counters: &mut Counters,
        bodies: &mut Bodies,
        colliders: &Colliders,
        joints: &mut Constraints,
        manifolds: &[ColliderContactManifold<N, Bodies::Handle, CollHandle>],
        island: &[Bodies::Handle],
        parameters: &IntegrationParameters<N>,
        coefficients: &MaterialsCoefficientsTable<N>,
    ) {
        counters.assembly_started();
        self.assemble_system(counters, parameters, coefficients, bodies, joints, manifolds, island);
        counters.assembly_completed();
        counters.set_nconstraints(self.contact_constraints.velocity.len() + self.joint_constraints.velocity.len());

        counters.velocity_resolution_started();
        self.solve_velocity_constraints(parameters, bodies);
        self.cache_impulses(bodies, joints);
        counters.velocity_resolution_completed();

        counters.velocity_update_started();
        self.update_velocities_and_integrate(parameters, bodies, island);
        counters.velocity_update_completed();

        counters.position_resolution_started();
        self.solve_position_constraints(parameters, bodies, colliders, joints);
        counters.position_resolution_completed();
    }

    // FIXME: this comment is bad.
    /// Perform one sub-step of the time-stepping scheme as part of a CCD integration.
    pub fn step_ccd<Colliders: ColliderSet<N, Bodies::Handle, Handle = CollHandle>, Constraints: JointConstraintSet<N, Bodies>>(
        &mut self,
        counters: &mut Counters,
        bodies: &mut Bodies,
        colliders: &Colliders,
        joints: &mut Constraints,
        manifolds: &[ColliderContactManifold<N, Bodies::Handle, CollHandle>],
        ccd_pair: [Bodies::Handle; 2],
        island: &[Bodies::Handle],
        parameters: &IntegrationParameters<N>,
        coefficients: &MaterialsCoefficientsTable<N>,
        locked_bodies: &HashSet<Bodies::Handle>,
    ) {
        self.assemble_system(counters, parameters, coefficients, bodies, joints, manifolds, island);

//        for constraint in &mut self.contact_constraints.position.unilateral {
//            if constraint.body1.0 != ccd_pair[0] && constraint.body1.0 != ccd_pair[1] {
//                constraint.ndofs1 = 0;
//            }
//
//            if constraint.body2.0 != ccd_pair[0] && constraint.body2.0 != ccd_pair[1] {
//                constraint.ndofs2 = 0;
//            }
//        }

        self.solve_position_constraints(parameters, bodies, colliders, joints);
        if !locked_bodies.contains(&ccd_pair[0]) {
            bodies.get_mut(ccd_pair[0]).unwrap().validate_advancement();
        }

        if !locked_bodies.contains(&ccd_pair[1]) {
            bodies.get_mut(ccd_pair[1]).unwrap().validate_advancement();
        }

//        for handle in island {
//            let body = try_continue!(bodies.get_mut(*handle));
//            body.validate_advancement();
//        }

        self.solve_velocity_constraints(parameters, bodies);
//        self.cache_impulses(bodies, joints);
        self.update_velocities_and_integrate(parameters, bodies, island);
    }

    fn assemble_system<Constraints: JointConstraintSet<N, Bodies>>(
        &mut self,
        counters: &mut Counters,
        parameters: &IntegrationParameters<N>,
        coefficients: &MaterialsCoefficientsTable<N>,
        bodies: &mut Bodies,
        joints: &mut Constraints,
        manifolds: &[ColliderContactManifold<N, Bodies::Handle, CollHandle>],
        island: &[Bodies::Handle],
    ) {
        self.internal_constraints.clear();
        let mut system_ndofs = 0;

        for handle in island {
            let body = try_continue!(bodies.get_mut(*handle));
            body.set_companion_id(system_ndofs);
            let ndofs = body.status_dependent_ndofs();
            assert!(
                ndofs != 0,
                "Internal error: an island cannot contain a non-dynamic body."
            );

            system_ndofs += ndofs;

            if ndofs != 0 && body.has_active_internal_constraints() {
                self.internal_constraints.push(*handle)
            }
        }

        self.resize_buffers(system_ndofs);
        self.contact_constraints.clear();
        self.joint_constraints.clear();

        /*
         * Initialize M^{-1} h * dt
         */
        for handle in island {
            let body = try_continue!(bodies.get(*handle));
            let id = body.companion_id();
            let accs = body.generalized_acceleration();

            self.ext_vels
                .rows_mut(id, accs.len())
                .axpy(parameters.dt(), &accs, N::zero());
        }

        /*
         *
         * Compute jacobian sizes.
         *
         */
        let mut jacobian_sz = 0;
        let mut ground_jacobian_sz = 0;


        joints.foreach(|_, g| {
            if g.is_active(bodies) {
                let (b1, b2) = g.anchors();
                if let (Some(body1), Some(body2)) = (bodies.get(b1.0), bodies.get(b2.0)) {
                    let ndofs1 = body1.status_dependent_ndofs();
                    let ndofs2 = body2.status_dependent_ndofs();

                    let nconstraints = g.num_velocity_constraints();
                    let sz = nconstraints * 2 * (ndofs1 + ndofs2);

                    if ndofs1 == 0 || ndofs2 == 0 {
                        ground_jacobian_sz += sz;
                    } else {
                        jacobian_sz += sz;
                    }
                }
            }
        });

        for m in manifolds {
            let ndofs1 = try_continue!(bodies.get(m.body1())).status_dependent_ndofs();
            let ndofs2 = try_continue!(bodies.get(m.body2())).status_dependent_ndofs();
            let sz = self.contact_model.num_velocity_constraints(m) * (ndofs1 + ndofs2) * 2;

            if ndofs1 == 0 || ndofs2 == 0 {
                ground_jacobian_sz += sz;
            } else {
                jacobian_sz += sz;
            }
        }

        self.jacobians
            .resize(jacobian_sz + ground_jacobian_sz, N::zero());

        /*
         *
         * Initialize constraints.
         *
         */
        let mut j_id = 0;
        let mut ground_j_id = jacobian_sz;

        joints.foreach_mut(|_, g| {
            if g.is_active(bodies) {
                g.velocity_constraints(
                    parameters,
                    bodies,
                    &self.ext_vels,
                    &mut ground_j_id,
                    &mut j_id,
                    &mut self.jacobians,
                    &mut self.joint_constraints.velocity,
                );
            }
        });

        counters.custom_started();
        self.contact_model.constraints(
            parameters,
            coefficients,
            bodies,
            &self.ext_vels,
            manifolds,
            &mut ground_j_id,
            &mut j_id,
            &mut self.jacobians,
            &mut self.contact_constraints,
        );
        counters.custom_completed();


        for handle in &self.internal_constraints {
            if let Some(body) = bodies.get_mut(*handle) {
                let ext_vels = self.ext_vels.rows(body.companion_id(), body.ndofs());
                body.setup_internal_velocity_constraints(&ext_vels, parameters);
            }
        }
    }

    fn solve_velocity_constraints(&mut self, parameters: &IntegrationParameters<N>, bodies: &mut Bodies) {
        SORProx::solve(
            bodies,
            &mut self.contact_constraints.velocity,
            &mut self.joint_constraints.velocity,
            &self.internal_constraints,
            &mut self.mj_lambda_vel,
            &self.jacobians,
            parameters.max_velocity_iterations,
        );
    }

    fn solve_position_constraints<Colliders: ColliderSet<N, Bodies::Handle, Handle = CollHandle>, Constraints: JointConstraintSet<N, Bodies>>(
        &mut self,
        parameters: &IntegrationParameters<N>,
        bodies: &mut Bodies,
        colliders: &Colliders,
        joints: &mut Constraints,
    ) {
        // XXX: avoid the systematic clone.
        // This is needed for cases where we perform the position resolution
        // before the velocity resolution.
        let mut jacobians = self.jacobians.clone();
        NonlinearSORProx::solve(
            parameters,
            bodies,
            colliders,
            &mut self.contact_constraints.position.unilateral,
            joints,
            &self.internal_constraints,
            &mut jacobians,
            parameters.max_position_iterations,
        );
    }

    fn cache_impulses<Constraints: JointConstraintSet<N, Bodies>>(
        &mut self,
        bodies: &mut Bodies,
        joints: &mut Constraints,
    ) {
        self.contact_model.cache_impulses(&self.contact_constraints);

        joints.foreach_mut(|_, j| {
            if j.is_active(bodies) {
                j.cache_impulses(&self.joint_constraints.velocity);
            }
        });
    }

    fn resize_buffers(&mut self, ndofs: usize) {
        // XXX: use resize functions instead of reallocating.
        self.mj_lambda_vel = DVector::zeros(ndofs);
        self.ext_vels = DVector::zeros(ndofs);
    }

    fn update_velocities_and_integrate(
        &mut self,
        parameters: &IntegrationParameters<N>,
        bodies: &mut Bodies,
        island: &[Bodies::Handle],
    ) {
        for handle in island {
            let body = try_continue!(bodies.get_mut(*handle));
            let id = body.companion_id();
            let ndofs = body.ndofs();

            {
//                println!("ext_vels: {:?}, mj_lambda: {:?}", self.ext_vels, self.mj_lambda_vel);
                let mut mb_vels = body.generalized_velocity_mut();
                mb_vels += self.ext_vels.rows(id, ndofs);
                mb_vels += self.mj_lambda_vel.rows(id, ndofs);
            }

            body.integrate(parameters);
        }
    }
}
