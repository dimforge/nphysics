use slab::Slab;

use na::{DVector, DVectorSlice, Real};
use na::storage::Storage;

use counters::Counters;
use detection::ColliderContactManifold;
use object::{BodyHandle, BodySet};
use joint::JointConstraint;
use solver::{ConstraintSet, ContactModel, IntegrationParameters,
             MultibodyJointLimitsNonlinearConstraintGenerator, NonlinearSORProx, SORProx};

/// Moreau-Jean time-stepping scheme.
pub struct MoreauJeanSolver<N: Real> {
    jacobians: Vec<N>, // FIXME: use a Vec or a DVector?
    mj_lambda_vel: DVector<N>,
    ext_vels: DVector<N>,
    contact_model: Box<ContactModel<N>>,
    constraints: ConstraintSet<N>,
}

impl<N: Real> MoreauJeanSolver<N> {
    pub fn new(contact_model: Box<ContactModel<N>>) -> Self {
        let constraints = ConstraintSet::new();

        MoreauJeanSolver {
            jacobians: Vec::new(),
            mj_lambda_vel: DVector::zeros(0),
            ext_vels: DVector::zeros(0),
            contact_model: contact_model,
            constraints: constraints,
        }
    }

    pub fn set_contact_model(&mut self, model: Box<ContactModel<N>>) {
        self.contact_model = model
    }

    pub fn step(
        &mut self,
        counters: &mut Counters,
        bodies: &mut BodySet<N>,
        joints: &mut Slab<Box<JointConstraint<N>>>,
        manifolds: &[ColliderContactManifold<N>],
        island: &[BodyHandle],
        params: &IntegrationParameters<N>,
    ) {
        counters.assembly_started();
        self.assemble_system(params, bodies, joints, manifolds, island);
        counters.assembly_completed();

        counters.set_nconstraints(self.constraints.velocity.len());

        counters.resolution_started();
        self.solve_velocity_constraints(params);
        self.save_cache(bodies, joints, island);
        counters.resolution_completed();

        counters.position_update_started();
        self.update_velocities_and_integrate(params, bodies, island);
        self.solve_position_constraints(params, bodies, joints);
        counters.position_update_completed();
    }

    fn assemble_system(
        &mut self,
        params: &IntegrationParameters<N>,
        bodies: &mut BodySet<N>,
        joints: &mut Slab<Box<JointConstraint<N>>>,
        manifolds: &[ColliderContactManifold<N>],
        island: &[BodyHandle],
    ) {
        let mut system_ndofs = 0;

        for handle in island {
            let mut body = bodies.body_mut(*handle);
            body.set_companion_id(system_ndofs);
            let ndofs = body.status_dependent_ndofs();
            assert!(
                ndofs != 0,
                "Internal error: an island cannot contain a non-dynamic body."
            );

            system_ndofs += ndofs;
        }

        self.resize_buffers(system_ndofs);
        self.constraints.clear();

        /*
         * Initialize M^{-1} h * dt
         */
        for handle in island {
            let body = bodies.body(*handle);
            let id = body.companion_id();
            let accs = body.generalized_acceleration();

            self.ext_vels
                .rows_mut(id, accs.len())
                .axpy(params.dt, &accs, N::zero());
        }

        /*
         *
         * Compute jacobian sizes.
         *
         */
        let mut jacobian_sz = 0;
        let mut ground_jacobian_sz = 0;

        for (_, g) in joints.iter() {
            if g.is_active(bodies) {
                let (b1, b2) = g.anchors();
                let body1 = bodies.body(b1);
                let body2 = bodies.body(b2);

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

        for m in manifolds {
            let ndofs1 = bodies.body(m.body1()).status_dependent_ndofs();
            let ndofs2 = bodies.body(m.body2()).status_dependent_ndofs();
            let sz = self.contact_model.num_velocity_constraints(m) * (ndofs1 + ndofs2) * 2;

            if ndofs1 == 0 || ndofs2 == 0 {
                ground_jacobian_sz += sz;
            } else {
                jacobian_sz += sz;
            }
        }

        for handle in island {
            if let Some(mb) = bodies.multibody(*handle) {
                for rb in mb.rbs().iter() {
                    ground_jacobian_sz += rb.dof.num_velocity_constraints() * mb.ndofs() * 2;
                }
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

        for (_, g) in joints {
            if g.is_active(bodies) {
                g.velocity_constraints(
                    params,
                    bodies,
                    &self.ext_vels,
                    &mut ground_j_id,
                    &mut j_id,
                    &mut self.jacobians,
                    &mut self.constraints,
                );
            }
        }

        for handle in island {
            if let Some(mb) = bodies.multibody_mut(*handle) {
                mb.constraints(
                    params,
                    &self.ext_vels,
                    &mut ground_j_id,
                    &mut self.jacobians,
                    &mut self.constraints,
                );
            }
        }

        self.contact_model.constraints(
            params,
            bodies,
            &self.ext_vels,
            manifolds,
            &mut ground_j_id,
            &mut j_id,
            &mut self.jacobians,
            &mut self.constraints,
        );
    }

    fn solve_velocity_constraints(&mut self, params: &IntegrationParameters<N>) {
        let solver = SORProx::new();

        solver.solve(
            &mut self.constraints.velocity.unilateral_ground,
            &mut self.constraints.velocity.unilateral,
            &mut self.constraints.velocity.bilateral_ground,
            &mut self.constraints.velocity.bilateral,
            &mut self.mj_lambda_vel,
            &self.jacobians,
            params.max_velocity_iterations,
        );
    }

    fn solve_position_constraints(
        &mut self,
        params: &IntegrationParameters<N>,
        bodies: &mut BodySet<N>,
        joints: &mut Slab<Box<JointConstraint<N>>>,
    ) {
        let solver = NonlinearSORProx::new();

        solver.solve(
            params,
            bodies,
            &mut self.constraints.position.unilateral,
            &mut self.constraints.position.multibody_limits,
            joints,
            &mut self.jacobians,
            params.max_position_iterations,
        );
    }

    fn save_cache(
        &mut self,
        bodies: &mut BodySet<N>,
        joints: &mut Slab<Box<JointConstraint<N>>>,
        island: &[BodyHandle],
    ) {
        for handle in island {
            if let Some(mb) = bodies.multibody_mut(*handle) {
                mb.cache_impulses(&self.constraints)
            }
        }

        self.contact_model.cache_impulses(&self.constraints);

        for (_, g) in joints {
            if g.is_active(bodies) {
                g.cache_impulses(&self.constraints);
            }
        }
    }

    fn resize_buffers(&mut self, ndofs: usize) {
        // XXX: use resize functions instead of reallocating.
        self.mj_lambda_vel = DVector::zeros(ndofs);
        self.ext_vels = DVector::zeros(ndofs);
    }

    fn update_velocities_and_integrate(
        &mut self,
        params: &IntegrationParameters<N>,
        bodies: &mut BodySet<N>,
        island: &[BodyHandle],
    ) {
        for handle in island {
            let mut body = bodies.body_mut(*handle);
            let id = body.companion_id();
            let ndofs = body.ndofs();

            {
                let mut mb_vels = body.generalized_velocity_mut();
                mb_vels += self.ext_vels.rows(id, ndofs);
                mb_vels += self.mj_lambda_vel.rows(id, ndofs);
            }

            body.integrate(params);
        }
    }
}
