use slab::Slab;

use na::{DVector, DVectorSlice, Real};
use na::storage::Storage;

use counters::Counters;
use detection::BodyContactManifold;
use object::{BodyHandle, BodySet};
use joint::ConstraintGenerator;
use solver::{ConstraintSet, ContactModel, IntegrationParameters, NonlinearSORProx, SORProx};

/// Moreau-Jean time-stepping scheme.
pub struct MoreauJeanSolver<N: Real> {
    jacobians: Vec<N>, // FIXME: use a Vec or a DVector?
    mj_lambda_vel: DVector<N>,
    mj_lambda_pos: DVector<N>,
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
            mj_lambda_pos: DVector::zeros(0),
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
        gens: &Slab<Box<ConstraintGenerator<N>>>,
        manifolds: &[BodyContactManifold<N>],
        island: &[BodyHandle],
        params: &IntegrationParameters<N>,
    ) {
        counters.assembly_started();
        self.assemble_system(bodies, gens, manifolds, island, params);
        counters.assembly_completed();

        counters.set_nconstraints(self.constraints.velocity.len());

        counters.resolution_started();
        self.solve_velocity_constraints(params);
        self.save_cache();
        counters.resolution_completed();

        counters.position_update_started();
        self.update_velocities_and_integrate(bodies, island, params);
        self.solve_position_constraints(bodies, params);
        counters.position_update_completed();
    }

    fn assemble_system(
        &mut self,
        bodies: &mut BodySet<N>,
        gens: &Slab<Box<ConstraintGenerator<N>>>,
        manifolds: &[BodyContactManifold<N>],
        island: &[BodyHandle],
        params: &IntegrationParameters<N>,
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

        for (_, g) in gens {
            let (b1, b2) = g.anchors();
            let body1 = bodies.body(b1);
            let body2 = bodies.body(b2);

            let ndofs1 = body1.status_dependent_ndofs();
            let ndofs2 = body2.status_dependent_ndofs();

            if (ndofs1 == 0 || !body1.is_active()) && (ndofs2 == 0 || !body2.is_active()) {
                continue;
            }

            let nconstraints = g.nconstraints();
            let sz = nconstraints * 2 * (ndofs1 + ndofs2);

            if ndofs1 == 0 || ndofs2 == 0 {
                ground_jacobian_sz += sz;
            } else {
                jacobian_sz += sz;
            }
        }

        for c in manifolds {
            let ndofs1 = bodies.body(c.b1).status_dependent_ndofs();
            let ndofs2 = bodies.body(c.b2).status_dependent_ndofs();
            let sz = self.contact_model.nconstraints(c) * (ndofs1 + ndofs2) * 2;

            if ndofs1 == 0 || ndofs2 == 0 {
                ground_jacobian_sz += sz;
            } else {
                jacobian_sz += sz;
            }
        }

        for handle in island {
            if let Some(mb) = bodies.multibody(*handle) {
                for rb in mb.rbs().iter() {
                    ground_jacobian_sz += rb.dof.nconstraints() * mb.ndofs() * 2;
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
        let mut jacobian_id = 0;
        let mut ground_jacobian_id = jacobian_sz;

        for (_, g) in gens {
            let (b1, b2) = g.anchors();
            let body1 = bodies.body(b1);
            let body2 = bodies.body(b2);

            let ndofs1 = body1.status_dependent_ndofs();
            let ndofs2 = body2.status_dependent_ndofs();

            if (ndofs1 == 0 || !body1.is_active()) && (ndofs2 == 0 || !body2.is_active()) {
                continue;
            }

            g.build_constraints(
                params,
                bodies,
                &self.ext_vels,
                &mut ground_jacobian_id,
                &mut jacobian_id,
                &mut self.jacobians,
                &mut self.constraints,
            );
        }

        for handle in island {
            if let Some(mb) = bodies.multibody(*handle) {
                let assembly_id = mb.companion_id();

                for link in mb.links() {
                    link.joint().build_constraints(
                        params,
                        mb,
                        &link,
                        assembly_id,
                        0,
                        &self.ext_vels.as_slice(),
                        &mut ground_jacobian_id,
                        &mut self.jacobians,
                        &mut self.constraints,
                    );
                }
            }
        }

        self.contact_model.build_constraints(
            params,
            bodies,
            &self.ext_vels,
            manifolds,
            &mut ground_jacobian_id,
            &mut jacobian_id,
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
        bodies: &mut BodySet<N>,
        params: &IntegrationParameters<N>,
    ) {
        let solver = NonlinearSORProx::new();

        solver.solve(
            bodies,
            &mut self.constraints.position.unilateral,
            &mut self.mj_lambda_pos,
            &mut self.jacobians,
            params.max_position_iterations,
        );
    }

    fn save_cache(&mut self) {
        self.contact_model.cache_impulses(&self.constraints)
    }

    fn resize_buffers(&mut self, ndofs: usize) {
        // XXX: use resize functions instead of reallocating.
        self.mj_lambda_vel = DVector::zeros(ndofs);
        self.mj_lambda_pos = DVector::zeros(ndofs);
        self.ext_vels = DVector::zeros(ndofs);
    }

    fn update_velocities_and_integrate(
        &mut self,
        bodies: &mut BodySet<N>,
        island: &[BodyHandle],
        params: &IntegrationParameters<N>,
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
