use slab::Slab;

use na::{DVector, Real};

use counters::Counters;
use detection::ContactConstraint;
use object::{BodyHandle, BodySet};
use joint::ConstraintGenerator;
use solver::{BilateralConstraint2, BilateralGroundConstraint, ContactModel, IntegrationParameters,
             SORProx, UnilateralConstraint2, UnilateralGroundConstraint};

/// Moreau-Jean time-stepping scheme.
pub struct MoreauJeanSolver<N: Real> {
    jacobians: Vec<N>, // FIXME: use a Vec or a DVector?
    mj_lambda: DVector<N>,
    ext_vels: DVector<N>,
    unilateral_ground_constraints: Vec<UnilateralGroundConstraint<N>>,
    unilateral_constraints: Vec<UnilateralConstraint2<N>>,
    bilateral_ground_constraints: Vec<BilateralGroundConstraint<N>>,
    bilateral_constraints: Vec<BilateralConstraint2<N>>,
}

impl<N: Real> MoreauJeanSolver<N> {
    pub fn new() -> Self {
        MoreauJeanSolver {
            jacobians: Vec::new(),
            mj_lambda: DVector::zeros(0),
            ext_vels: DVector::zeros(0),
            unilateral_constraints: Vec::new(),
            unilateral_ground_constraints: Vec::new(),
            bilateral_constraints: Vec::new(),
            bilateral_ground_constraints: Vec::new(),
        }
    }

    pub fn step(
        &mut self,
        counters: &mut Counters,
        bodies: &mut BodySet<N>,
        gens: &Slab<Box<ConstraintGenerator<N>>>,
        contacts: &[ContactConstraint<N>],
        island: &[BodyHandle],
        contact_model: &ContactModel<N>,
        params: &IntegrationParameters<N>,
    ) {
        counters.assembly_started();
        self.assemble_system(bodies, gens, contacts, island, contact_model, params);
        counters.assembly_completed();

        counters.set_nconstraints(
            self.unilateral_constraints.len() + self.unilateral_ground_constraints.len()
                + self.bilateral_ground_constraints.len()
                + self.bilateral_constraints.len(),
        );

        counters.resolution_started();
        self.solve_constraints(params);
        counters.resolution_completed();

        counters.position_update_started();
        self.update_positions(bodies, island, params);
        counters.position_update_completed();
    }

    fn assemble_system(
        &mut self,
        bodies: &mut BodySet<N>,
        gens: &Slab<Box<ConstraintGenerator<N>>>,
        contacts: &[ContactConstraint<N>],
        island: &[BodyHandle],
        contact_model: &ContactModel<N>,
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
         * Setup contact contsraints.
         */
        self.unilateral_ground_constraints.clear();
        self.unilateral_constraints.clear();
        self.bilateral_ground_constraints.clear();
        self.bilateral_constraints.clear();

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

        for c in contacts {
            let ndofs1 = bodies.body(c.b1).status_dependent_ndofs();
            let ndofs2 = bodies.body(c.b2).status_dependent_ndofs();
            let sz = contact_model.nconstraints() * (ndofs1 + ndofs2) * 2;

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
                &mut self.unilateral_ground_constraints,
                &mut self.unilateral_constraints,
                &mut self.bilateral_ground_constraints,
                &mut self.bilateral_constraints,
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
                        &mut self.unilateral_ground_constraints,
                        &mut self.bilateral_ground_constraints,
                    );
                }
            }
        }

        for c in contacts {
            let _ = contact_model.build_constraints(
                params,
                bodies,
                &self.ext_vels,
                &mut ground_jacobian_id,
                &mut jacobian_id,
                &mut self.jacobians,
                c,
                &mut self.unilateral_ground_constraints,
                &mut self.unilateral_constraints,
                &mut self.bilateral_ground_constraints,
                &mut self.bilateral_constraints,
            );
        }
    }

    fn solve_constraints(&mut self, params: &IntegrationParameters<N>) {
        let solver = SORProx::new();

        solver.solve(
            &mut self.unilateral_ground_constraints,
            &mut self.unilateral_constraints,
            &mut self.bilateral_ground_constraints,
            &mut self.bilateral_constraints,
            &mut self.mj_lambda,
            &self.jacobians,
            params.niter,
        );
    }

    fn resize_buffers(&mut self, ndofs: usize) {
        // XXX: use resize functions instead of reallocating.
        self.mj_lambda = DVector::zeros(ndofs);
        self.ext_vels = DVector::zeros(ndofs);
    }

    fn update_positions(
        &self,
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
                mb_vels += self.mj_lambda.rows(id, ndofs);
            }

            body.apply_displacements(params);
        }
    }
}
