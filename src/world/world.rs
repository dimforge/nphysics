use slab::Slab;
use std::collections::{HashMap, HashSet};

use na::{self, RealField};
use ncollide;
use ncollide::query::{self, NonlinearTOIStatus};
use ncollide::narrow_phase::{Interaction, ContactEvents, ProximityEvents};
use ncollide::interpolation::ConstantVelocityRigidMotion;

use crate::counters::Counters;
use crate::detection::{ActivationManager, ColliderContactManifold};
use crate::force_generator::{ForceGenerator, ForceGeneratorHandle, ForceGeneratorSet};
use crate::joint::{JointConstraintHandle, JointConstraint, JointConstraintSet};
use crate::math::Vector;
use crate::object::{
    Body, BodySlab, BodyDesc, BodyStatus, Collider, ColliderAnchor,
    ColliderHandle, Multibody, RigidBody, BodySlabHandle, BodySet, BodyHandle
};
use crate::material::MaterialsCoefficientsTable;
use crate::solver::{ContactModel, IntegrationParameters, MoreauJeanSolver, SignoriniCoulombPyramidModel};
use crate::world::ColliderWorld;

struct SubstepState<N: RealField> {
    active: bool,
    dt: N,
    // FIXME: can we avoid the use of a hash-map to save the
    // number of time a contact pair generated a CCD event?
    ccd_counts: HashMap<(ColliderHandle, ColliderHandle), usize>,
    body_times: HashMap<BodySlabHandle, N>,
    body_hit: HashSet<BodySlabHandle>,
}

/// The physics world.
pub struct World<N: RealField, Bodies: BodySet<N>> {
    counters: Counters,

    bodies: BodySlab<N>,
    constraints: Slab<Box<JointConstraint<N, BodySlab<N>>>>,
    forces: Slab<Box<ForceGenerator<N, BodySlab<N>>>>,

    cworld: ColliderWorld<N, BodySlabHandle>,
    solver: MoreauJeanSolver<N, Bodies>,
    activation_manager: ActivationManager<N, Bodies::Handle>,

    // FIXME: set this parameter per-colliders?
    prediction: N,
    gravity: Vector<N>,
    params: IntegrationParameters<N>,
    material_coefficients: MaterialsCoefficientsTable<N>,
    substep: SubstepState<N>,
}

impl<N: RealField, Bodies: BodySet<N>> World<N, Bodies> {
    /// Creates a new physics world with default parameters.
    ///
    /// The ground body is automatically created and added to the world without any colliders attached.
    pub fn new() -> Self {
        let counters = Counters::new(false);
        let bv_margin = na::convert(0.01f64);
        let prediction = na::convert(0.002);
        let bodies = BodySlab::new();
        let constraints = Slab::new();
        let forces = Slab::new();
        let cworld = ColliderWorld::new(bv_margin);
        let contact_model = Box::new(SignoriniCoulombPyramidModel::new());
        let solver = MoreauJeanSolver::new(contact_model);
        let activation_manager = ActivationManager::new(na::convert(0.01f64));
        let gravity = Vector::zeros();
        let params = IntegrationParameters::default();
        let material_coefficients = MaterialsCoefficientsTable::new();
        let substep = SubstepState {
            active: false,
            dt: params.dt(),
            ccd_counts: HashMap::new(),
            body_times: HashMap::new(),
            body_hit: HashSet::new(),
        };

        World {
            counters,
            bodies,
            cworld,
            solver,
            activation_manager,
            material_coefficients,
            prediction,
            gravity,
            constraints,
            forces,
            params,
            substep,
        }
    }

    /// Prediction distance used internally for collision detection.
    pub fn prediction(&self) -> N {
        self.prediction
    }

    /// Disable the perfomance counters that measure various times and statistics during a timestep.
    pub fn disable_performance_counters(&mut self) {
        self.counters.disable();
    }

    /// Enable the perfomance counters that measure various times and statistics during a timestep.
    pub fn enable_performance_counters(&mut self) {
        self.counters.enable();
    }

    /// Retrieve the perfomance counters that measure various times and statistics during a timestep.
    pub fn performance_counters(&self) -> &Counters {
        &self.counters
    }

    /// Set the contact model for all contacts.
    pub fn set_contact_model<C: ContactModel<N, Bodies>>(&mut self, model: C) {
        self.solver.set_contact_model(Box::new(model))
    }

    /// Retrieve a reference to the parameters for the integration.
    pub fn integration_parameters(&self) -> &IntegrationParameters<N> {
        &self.params
    }

    /// Retrieve a mutable reference to the parameters for the integration.
    pub fn integration_parameters_mut(&mut self) -> &mut IntegrationParameters<N> {
        &mut self.params
    }

    /// Reference to the lookup table for friction and restitution coefficients.
    pub fn materials_coefficients_table(&self) -> &MaterialsCoefficientsTable<N> {
        &self.material_coefficients
    }

    /// Mutable reference to the lookup table for friction and restitution coefficients.
    pub fn materials_coefficients_table_mut(&mut self) -> &mut MaterialsCoefficientsTable<N> {
        &mut self.material_coefficients
    }

    /// Retrieve the timestep used for the integration.
    pub fn timestep(&self) -> N {
        self.params.dt()
    }

    /// Sets the timestep used for the integration.
    pub fn set_timestep(&mut self, dt: N) {
        self.params.set_dt(dt);
    }

    /// Activate the given body.
    pub fn activate_body(&mut self, handle: BodySlabHandle) {
        Self::activate_body_at(&mut self.bodies, handle)
    }

    // NOTE: static method used to avoid borrowing issues.
    fn activate_body_at(bodies: &mut BodySlab<N>, handle: BodySlabHandle) {
        if let Some(body) = bodies.get_mut(handle) {
            if body.status_dependent_ndofs() != 0 {
                body.activate();
            }
        }
    }

    /// Add a constraints to the physics world and retrieves its handle.
    pub fn add_constraint<C: JointConstraint<N, BodySlab<N>>>(&mut self, constraint: C) -> JointConstraintHandle {
        let (anchor1, anchor2) = constraint.anchors();
        self.activate_body(anchor1.0);
        self.activate_body(anchor2.0);
        self.constraints.insert(Box::new(constraint))
    }

    /// Get a reference to the specified constraint.
    pub fn constraint(&self, handle: JointConstraintHandle) -> &JointConstraint<N, BodySlab<N>> {
        &*self.constraints[handle]
    }

    /// Get a mutable reference to the specified constraint.
    pub fn constraint_mut(&mut self, handle: JointConstraintHandle) -> &mut JointConstraint<N, BodySlab<N>> {
        let (anchor1, anchor2) = self.constraints[handle].anchors();
        self.activate_body(anchor1.0);
        self.activate_body(anchor2.0);
        &mut *self.constraints[handle]
    }

    /// Remove the specified constraint from the world.
    pub fn remove_constraint(&mut self, handle: JointConstraintHandle) -> Box<JointConstraint<N, BodySlab<N>>> {
        let constraint = self.constraints.remove(handle);
        let (anchor1, anchor2) = constraint.anchors();
        self.activate_body(anchor1.0);
        self.activate_body(anchor2.0);

        constraint
    }

    /// Remove the specified collider from the world.
    pub fn remove_colliders(&mut self, handles: &[ColliderHandle]) {
        let bodies = &mut self.bodies;

        for handle in handles {
            if let Some(it) = self.cworld.colliders_in_contact_with(*handle) {
                it.for_each(|coll| {
                    if let Some(b) = bodies.get_mut(coll.body()) {
                        b.activate()
                    }
                });
            }
        }

        self.cworld.remove(handles);
    }

    /// Add a force generator to the world.
    pub fn add_force_generator<G: ForceGenerator<N, BodySlab<N>>>(
        &mut self,
        force_generator: G,
    ) -> ForceGeneratorHandle {
        self.forces.insert(Box::new(force_generator))
    }

    /// Retrieve a reference to the specified force generator.
    pub fn force_generator(&self, handle: ForceGeneratorHandle) -> &ForceGenerator<N, BodySlab<N>> {
        &*self.forces[handle]
    }

    /// Retrieve a mutable reference to the specified force generator.
    pub fn force_generator_mut(&mut self, handle: ForceGeneratorHandle) -> &mut ForceGenerator<N, BodySlab<N>> {
        &mut *self.forces[handle]
    }

    /// Remove the specified force generator from the world.
    pub fn remove_force_generator(
        &mut self,
        handle: ForceGeneratorHandle,
    ) -> Box<ForceGenerator<N, BodySlab<N>>> {
        self.forces.remove(handle)
    }

    /// Set the gravity.
    pub fn set_gravity(&mut self, gravity: Vector<N>) {
        self.gravity = gravity
    }

    /// The gravity applied to all dynamic bodies.
    pub fn gravity(&self) -> &Vector<N> {
        &self.gravity
    }

    /// Execute one time step of the physics simulation.
    pub fn step<Constraints, Forces>(&mut self,
                                     cworld: &mut ColliderWorld<N, Bodies::Handle>,
                                     bodies: &mut Bodies,
                                     constraints: &mut Constraints,
                                     forces: &mut Forces)
    where Constraints: JointConstraintSet<N, Bodies>,
          Forces: ForceGeneratorSet<N, Bodies> {
        if !self.substep.active {
            println!("##### Loop");
            self.counters.step_started();

            /*
             *
             * Update body dynamics and accelerations.
             *
             */

            bodies.foreach_mut(|_, b| {
                b.step_started();
                b.update_kinematics();
                b.update_dynamics(self.params.dt());
            });

            // FIXME: how to make force generators work
            // with the external body set?
            let params = &self.params;
            forces.foreach_mut(|_, f| {
                f.apply(params, bodies)
            });

            bodies.foreach_mut(|_, b| {
                b.update_acceleration(&self.gravity, params);
            });

            /*
             *
             * Sync colliders and perform CD if the user moved
             * manually some bodies.
             */
            cworld.clear_events();
            cworld.sync_colliders(bodies);
            cworld.perform_broad_phase();
            cworld.perform_narrow_phase();

            /*
             *
             * Handle sleeping and collision
             * islands.
             *
             */
            // FIXME: for now, no island is built.
            self.counters.island_construction_started();
            let mut active_bodies = Vec::new();
            self.activation_manager.update(
                bodies,
                cworld,
                constraints,
                &mut active_bodies,
            );
            self.counters.island_construction_completed();

            /*
             *
             * Collect contact manifolds.
             *
             */
            let mut contact_manifolds = Vec::new(); // FIXME: avoid allocations.
            for (c1, c2, _, manifold) in cworld.contact_pairs(false) {
                let b1 = try_continue!(bodies.get(c1.body()));
                let b2 = try_continue!(bodies.get(c2.body()));

                if manifold.len() > 0
                    && b1.status() != BodyStatus::Disabled && b2.status() != BodyStatus::Disabled
                    && ((b1.status_dependent_ndofs() != 0 && b1.is_active())
                    || (b2.status_dependent_ndofs() != 0 && b2.is_active()))
                {
                    contact_manifolds.push(ColliderContactManifold::new(c1, c2, manifold));
                }
            }

            /*
             *
             * Solve the system and integrate.
             *
             */
            bodies.foreach_mut(|_, b| {
                // FIXME This is currently needed by the solver because otherwise
                // some kinematic bodies may end up with a companion_id (used as
                // an assembly_id) that it out of bounds of the velocity vector.
                // Note sure what the best place for this is though.
                b.set_companion_id(0);
            });

            self.counters.solver_started();
            self.solver.step(
                &mut self.counters,
                bodies,
                constraints,
                &contact_manifolds[..],
                &active_bodies[..],
                params,
                &self.material_coefficients,
                cworld,
            );

            bodies.foreach_mut(|_, b| {
                if b.status() == BodyStatus::Kinematic {
                    b.integrate(params)
                }
            });
            self.counters.solver_completed();

            /*
             *
             * Update body kinematics and dynamics
             * after the contact resolution step.
             *
             */
            // FIXME: objects involved in a non-linear position stabilization already
            // updated their kinematics.
            bodies.foreach_mut(|_, b| {
                b.update_kinematics();
                b.update_dynamics(params.dt());
            });
        }
/*
        /*
         *
         * Handle CCD
         *
         */
        if self.params.ccd_enabled {
            self.solve_ccd();
        }
*/
        if !self.substep.active {
            /*
             *
             * Finally, clear the update flag of every body.
             *
             */
            bodies.foreach_mut(|_, b| {
                b.clear_forces();
                b.validate_advancement();
            });


            /*
             *
             * Update colliders and perform CD with the new
             * body positions.
             *
             */
            self.counters.collision_detection_started();
            cworld.sync_colliders(bodies);

            self.counters.broad_phase_started();
            cworld.perform_broad_phase();
            self.counters.broad_phase_completed();

            self.counters.narrow_phase_started();
            cworld.perform_narrow_phase();
            self.counters.narrow_phase_completed();
            self.counters.collision_detection_completed();

            self.params.t += self.params.dt();
            self.counters.step_completed();

            bodies.foreach_mut(|_, b| {
                b.clear_update_flags();
            });
        }
    }

    /*
    // NOTE: this is an approach very similar to Box2D's.
    fn solve_ccd(&mut self) {
        let dt0 = self.params.dt();
        let inv_dt0 = self.params.inv_dt();

        if dt0 == N::zero() {
            return;
        }

        let mut params = self.params.clone();
//        params.max_velocity_iterations = 20;
        params.max_position_iterations = 20;
        params.warmstart_coeff = N::zero();

        let max_substeps = 8;

        for k in 0.. {
            self.cworld.sync_colliders(&self.bodies);
            self.cworld.perform_broad_phase();
            self.cworld.perform_narrow_phase();

            /*
             *
             *
             * Search for the first TOI.
             *
             *
             */
            let mut min_toi = N::max_value();
            let mut found_toi = false;
            let mut ccd_handles = None;

            'outer: for coll in self.cworld.colliders() {
                if coll.is_ccd_enabled() {
                    // Find a TOI
                    for (c1, c2, inter) in self.cworld.interactions_with(coll.handle(), false).unwrap() {
                        use crate::object::BodyPart;
                        let handle1 = c1.body();
                        let handle2 = c2.body();
                        let (b1, b2) = self.bodies.get_pair_mut(handle1, handle2);
                        let (b1, b2) = (b1.unwrap(), b2.unwrap());

                        let count = self.substep.ccd_counts.entry((c1.handle(), c2.handle())).or_insert(0);

                        if *count >= max_substeps {
                            let _ = self.substep.body_hit.insert(handle1);
                            let _ = self.substep.body_hit.insert(handle2);
                            continue;
                        }

                        match inter {
                            Interaction::Contact(alg, manifold) => {
                                let margins = c1.margin() + c2.margin();
                                let target = self.params.allowed_linear_error; // self.params.allowed_linear_error.max(margins - self.params.allowed_linear_error * na::convert(3.0));

                                let time1 = *self.substep.body_times.entry(c1.body()).or_insert(N::zero());
                                let time2 = *self.substep.body_times.entry(c2.body()).or_insert(N::zero());
                                let start_time = time1.max(time2);

                                if time1 < time2 {
                                    if b1.is_dynamic() && !self.substep.body_hit.contains(&handle1) {
                                        // Advance b1 so it is at the same internal time as b2.
                                        b1.advance(time2 - time1); // (time2 - time1) / (dt0 - time1));
                                        let _ = self.substep.body_times.insert(handle1, time2);
                                    }
                                } else if time2 < time1 {
                                    if b2.is_dynamic() && !self.substep.body_hit.contains(&c2.body()) {
                                        b2.advance(time1 - time2); // (time1 - time2) / (dt0 - time2));
                                        let _ = self.substep.body_times.insert(handle2, time1);
                                    }
                                }

                                // Compute the TOI.
                                let p1 = b1.part(0).unwrap();
                                let p2 = b2.part(0).unwrap();

                                let start1 = p1.safe_position() * c1.position_wrt_body();
                                let start2 = p2.safe_position() * c2.position_wrt_body();
                                let end1 = p1.position() * c1.position_wrt_body();
                                let end2 = p2.position() * c2.position_wrt_body();
                                let vel1 = p1.velocity();
                                let vel2 = p2.velocity();
                                let motion1 = ConstantVelocityRigidMotion::new(&start1, vel1.linear, vel1.angular);
                                let motion2 = ConstantVelocityRigidMotion::new(&start2, vel2.linear, vel2.angular);
                                let remaining_time = dt0 - start_time;

//                                if let Some(toi) = query::time_of_impact(&pos1, &v1, &**c1.shape(), &pos2, &v2, &**c2.shape(), target) {
                                 if let Some(toi) = query::nonlinear_time_of_impact(&motion1, &**c1.shape(), &motion2, &**c2.shape(), remaining_time, target) {
                                     // Don't use the TOI if the colliders are already penetrating.
                                     println!("toi: {:?}", toi);
                                     if toi.status != NonlinearTOIStatus::Penetrating {
                                         let toi = start_time + toi.toi;
//                                     println!("Toi: {} vs time: {}", toi, start_time);
                                         if toi < min_toi {
                                             println!("start time: {}", start_time);
                                             found_toi = true;
                                             min_toi = toi;
                                             ccd_handles = Some((c1.handle(), c1.body(), c2.handle(), c2.body()))
                                         }
                                     }
                                 }
                            }
                            Interaction::Proximity(prox) => unimplemented!()
                        }
                    }
                }
            }

//            println!("Min toi: {}", min_toi);

            /*
             *
             *
             * Resolve the minimum TOI event, if any.
             *
             *
             */
            if let Some((c1, b1, c2, b2)) = ccd_handles {
                println!("Found min_toi: {}", min_toi);
                params.set_dt(dt0 - min_toi);
                // We will use the companion ID to know which body is already on the island.
                for b in self.bodies.bodies_mut() {
                    b.clear_forces();
                    b.update_kinematics();
                    b.update_dynamics(params.dt());
                    b.update_acceleration(&Vector::zeros(), &params);
                    b.set_companion_id(0);
                }

                *self.substep.ccd_counts.get_mut(&(c1, c2)).unwrap() += 1;

                let mut island = Vec::new();
                let mut contact_manifolds = Vec::new();

                let cworld = &mut self.cworld.cworld;

/*
                for b in self.bodies.bodies_mut() {
                    *self.substep.body_times.entry(b.handle()).or_insert(N::zero()) += min_toi;
                    b.advance(min_toi);
                    b.clamp_advancement();
                    island.push(b.handle());
                }

                self.cworld.sync_colliders(&self.bodies);
                self.cworld.perform_broad_phase();
                self.cworld.perform_narrow_phase();

                for (c1, c2, _, manifold) in self.cworld.contact_pairs(false) {
                    let b1 = try_continue!(self.bodies.get(c1.body()));
                    let b2 = try_continue!(self.bodies.get(c2.body()));

                    if manifold.len() > 0
                        && b1.status() != BodyStatus::Disabled && b2.status() != BodyStatus::Disabled
                        && ((b1.status_dependent_ndofs() != 0 && b1.is_active())
                        || (b2.status_dependent_ndofs() != 0 && b2.is_active()))
                    {
                        contact_manifolds.push(ColliderContactManifold::new(c1, c2, manifold));
                    }
                }*/


                let mut colliders = vec![c1, c2]; // FIXME: should contain all the colliders attached to b1 and b2.
                let mut interaction_ids = Vec::new();
                let (ca, cb) = (c1, c2);

                // Update contact manifolds.
                while let Some(c) = colliders.pop() {
                    let graph_id = cworld.objects.get(c).unwrap().graph_index();

                    for (c1, c2, eid, inter) in cworld.interactions.interactions_with_mut(graph_id) {
                        if !(c1 == c && (c2 == ca || c2 == cb)) {
                            // This interaction will be reported twice.
                            continue;
                        }

                        match inter {
                            Interaction::Contact(alg, manifold) => {
                                let (c1, c2) = cworld.objects.get_pair_mut(c1, c2);
                                let (c1, c2) = (Collider::from_mut(c1.unwrap()), Collider::from_mut(c2.unwrap()));
                                let (handle1, handle2) = (c1.body(), c2.body());

                                if let (Some(b1), Some(b2)) = self.bodies.get_pair_mut(handle1, handle2) {
//                                    if !((c1.is_ccd_enabled() || !b1.is_dynamic()) || (c2.is_ccd_enabled() || !b2.is_dynamic())) {
//                                        continue;
//                                    }

                                    if b1.companion_id() == 0 && b1.is_dynamic() {
                                        let time1 = self.substep.body_times.entry(handle1).or_insert(N::zero());

                                        if min_toi != *time1 && !self.substep.body_hit.contains(&handle1) {
                                            if min_toi < *time1 {
                                                continue;
                                            }
                                            assert!(min_toi >= *time1);
//                                            println!("Advancing by: {}", (min_toi - *time1) / (dt0 - *time1));
                                            b1.advance(min_toi - *time1); // (min_toi - *time1) / (dt0 - *time1));
                                            *time1 = min_toi;
                                        }

                                        b1.set_companion_id(1);
                                        b1.activate();
                                        b1.clamp_advancement();
                                        island.push(c1.body());
                                        c1.set_position(b1.part(0).unwrap().position() * c1.position_wrt_body());
                                    }

                                    if b2.companion_id() == 0 && b2.is_dynamic() {
                                        let time2 = self.substep.body_times.entry(handle2).or_insert(N::zero());

                                        if min_toi != *time2 && !self.substep.body_hit.contains(&handle2) {
                                            if min_toi < *time2 {
                                                continue;
                                            }
                                            assert!(min_toi >= *time2, format!("min_toi: {}, time2: {}", min_toi, *time2));
//                                            println!("Advancing by: {}", (min_toi - *time2) / (dt0 - *time2));
                                            b2.advance(min_toi - *time2); // (min_toi - *time2) / (dt0 - *time2));
                                            *time2 = min_toi;
                                        }

                                        b2.set_companion_id(1);
                                        b2.activate();
                                        b2.clamp_advancement();
                                        island.push(c2.body());
                                        c2.set_position(b2.part(0).unwrap().position() * c2.position_wrt_body());
                                    }

                                    cworld.narrow_phase.update_contact(c1.as_collision_object(), c2.as_collision_object(), &mut **alg, manifold);

                                    if manifold.len() > 0 {
                                        interaction_ids.push(eid);

//                                        if c1.handle() != c {
//                                            colliders.push(c1.handle());
//                                        }
//
//                                        if c2.handle() != c {
//                                            colliders.push(c2.handle());
//                                        }
//                                    }
                                    }
                                }
                            }
                            Interaction::Proximity(prox) => unimplemented!()
                        }
                    }
                }


                for eid in interaction_ids {
                    let (c1, c2, inter) = self.cworld.cworld.interactions.index_interaction(eid).unwrap();
                    match inter {
                        Interaction::Contact(_, manifold) => {
                            let c1 = self.cworld.collider(c1).unwrap();
                            let c2 = self.cworld.collider(c2).unwrap();
                            contact_manifolds.push(ColliderContactManifold::new(c1, c2, manifold));
                        }
                        Interaction::Proximity(_) => unimplemented!()
                    }
                }


                // Solve the system and integrate.
                for b in self.bodies.bodies_mut() {
                    b.set_companion_id(0);
                }

                // XXX: should joint constraints be taken into account here?
                let mut empty_constraints = Slab::new();
                params.set_dt(dt0 - min_toi);
                println!("Time-stepping length: {}", params.dt());
                println!("Num contacts: {}", contact_manifolds.len());
                println!("Island len: {}", island.len());

                self.solver.step_ccd(
                    &mut self.counters,
                    &mut self.bodies,
                    &mut empty_constraints,
                    &contact_manifolds[..],
                    [b1, b2],
                    &island[..],
                    &params,
                    &self.material_coefficients,
                    &self.cworld,
                );
                println!("CCD step completed.");


                // Update body kinematics and dynamics
                // after the contact resolution step.
                let gravity = &self.gravity;
                self.bodies.bodies_mut().for_each(|b| {
                    b.clear_forces();
                    b.update_kinematics();
                    b.update_dynamics(params.dt());
                });

                if self.params.substepping_enabled {
                    println!("Substep completed.");
                    self.substep.active = true;
                    self.substep.dt = params.dt();
                    break;
                }
            } else {
                println!("Full step completed.");
                let mut substep = &mut self.substep;
                self.bodies.foreach_mut(|handle, body| {
                    if substep.body_hit.contains(&handle) {
                        body.clamp_advancement();
                    }
                });

//                self.bodies.bodies_mut().for_each(|b| b.clamp_advancement());
                self.substep.active = false;
                self.substep.ccd_counts.clear();
                self.substep.body_times.clear();
                self.substep.body_hit.clear();
                self.cworld.sync_colliders(&self.bodies);
                self.cworld.perform_broad_phase();
                self.cworld.perform_narrow_phase();
                break;
            }
        }
    }
*/
    /// Remove the specified bodies.
    pub fn remove_bodies(&mut self, handles: &[BodySlabHandle]) {
        for handle in handles {
            self.bodies.remove_body(*handle);
        }

        self.cleanup_after_body_removal();

        for handle in handles {
            self.cworld.remove_body(*handle);
        }
    }

    fn cleanup_after_body_removal(&mut self) {
        self.activate_bodies_touching_deleted_bodies();
        self.cleanup_constraints_with_deleted_anchors();
    }

    fn activate_bodies_touching_deleted_bodies(&mut self) {
        let bodies = &mut self.bodies;

        for (c1, c2, _, _) in self.cworld.contact_pairs(true) {
            let b1_exists = bodies.get(c1.body()).is_some();
            let b2_exists = bodies.get(c2.body()).is_some();

            if !b1_exists {
                if b2_exists {
                    Self::activate_body_at(bodies, c2.body());
                }
            } else if !b2_exists {
                Self::activate_body_at(bodies, c1.body());
            }
        }
    }

    fn cleanup_constraints_with_deleted_anchors(&mut self) {
        let bodies = &mut self.bodies;

        self.constraints.retain(|_, constraint| {
            let (b1, b2) = constraint.anchors();
            let b1_exists = bodies.get(b1.0).and_then(|b| b.part(b1.1)).is_some();
            let b2_exists = bodies.get(b2.0).and_then(|b| b.part(b2.1)).is_some();

            if !b1_exists {
                if b2_exists {
                    Self::activate_body_at(bodies, b2.0);
                }
            } else if !b2_exists {
                Self::activate_body_at(bodies, b1.0);
            }

            b1_exists && b2_exists
        })
    }

    /// Adds a body to the world.
    pub fn add_body<B: BodyDesc<N>>(&mut self, desc: &B) -> &mut B::Body {
        self.bodies.add_body(desc, &mut self.cworld)
    }

    /// Get a reference to the specified body.
    pub fn body(&self, handle: BodySlabHandle) -> Option<&Body<N>> {
        self.bodies.get(handle)
    }

    /// Get a mutable reference to the specified body.
    pub fn body_mut(&mut self, handle: BodySlabHandle) -> Option<&mut Body<N>> {
        self.bodies.get_mut(handle)
    }

    /// Get a reference to the multibody containing the specified multibody link.
    ///
    /// Returns `None` if the handle does not correspond to a multibody link in this world.
    pub fn multibody(&self, handle: BodySlabHandle) -> Option<&Multibody<N>> {
        self.bodies.get(handle)?.downcast_ref::<Multibody<N>>()
    }

    /// Get a mutable reference to the multibody containing the specified multibody link.
    ///
    /// Returns `None` if the handle does not correspond to a multibody link in this world.
    pub fn multibody_mut(&mut self, handle: BodySlabHandle) -> Option<&mut Multibody<N>> {
        self.bodies.get_mut(handle)?.downcast_mut::<Multibody<N>>()
    }

    /// Get a reference to the specified rigid body.
    ///
    /// Returns `None` if the handle does not correspond to a rigid body in this world.
    pub fn rigid_body(&self, handle: BodySlabHandle) -> Option<&RigidBody<N>> {
        self.bodies.get(handle)?.downcast_ref::<RigidBody<N>>()
    }

    /// Get a mutable reference to the specified rigid body.
    ///
    /// Returns `None` if the handle does not correspond to a rigid body in this world.
    pub fn rigid_body_mut(&mut self, handle: BodySlabHandle) -> Option<&mut RigidBody<N>> {
        self.bodies.get_mut(handle)?.downcast_mut::<RigidBody<N>>()
    }

    /// Reference to the underlying collision world.
    pub fn collider_world(&self) -> &ColliderWorld<N, BodySlabHandle> {
        &self.cworld
    }

    /// Mutable reference to the underlying collision world.
    pub fn collider_world_mut(&mut self) -> &mut ColliderWorld<N, BodySlabHandle> {
        &mut self.cworld
    }


    /// Mutable reference to the underlying collision world.
    #[doc(hidden)]
    pub fn bodies_mut_and_collider_world_mut(&mut self) -> (&mut BodySlab<N>, &mut ColliderWorld<N, BodySlabHandle>) {
        (&mut self.bodies, &mut self.cworld)
    }

    /// Get a reference to the specified collider.
    ///
    /// Returns `None` if the handle does not correspond to a collider in this world.
    pub fn collider(&self, handle: ColliderHandle) -> Option<&Collider<N, BodySlabHandle>> {
        self.cworld.collider(handle)
    }

    /// Get a mutable reference to the specified collider.
    ///
    /// Returns `None` if the handle does not correspond to a collider in this world.
    pub fn collider_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider<N, BodySlabHandle>> {
        self.cworld.collider_mut(handle)
    }

    /// Gets the handle of the body the specified collider is attached to.
    pub fn collider_body_handle(&self, handle: ColliderHandle) -> Option<BodySlabHandle> {
        self.collider_anchor(handle).map(|anchor| anchor.body())
    }

    /// Gets the anchor attaching this collider to a body or body part.
    pub fn collider_anchor(&self, handle: ColliderHandle) -> Option<&ColliderAnchor<N, BodySlabHandle>> {
        self.cworld
            .collider(handle)
            .map(|co| co.anchor())
    }

    /// An iterator through all the colliders on this collision world.
    pub fn colliders(&self) -> impl Iterator<Item = &Collider<N, BodySlabHandle>> {
        self.cworld.colliders()
    }

    /// An iterator through all the bodies on this world.
    pub fn bodies(&self) -> impl Iterator<Item = &Body<N>> { self.bodies.bodies() }

    /// A mutable iterator through all the bodies on this world.
    pub fn bodies_mut(&mut self) -> impl Iterator<Item = &mut Body<N>> { self.bodies.bodies_mut() }

    /// An iterator through all the bodies with the given name.
    pub fn bodies_with_name<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Body<N>> {
        self.bodies().filter(move |b| b.name() == name)
    }

    /// An iterator through all the bodies with the given name.
    pub fn bodies_with_name_mut<'a>(&'a mut self, name: &'a str) -> impl Iterator<Item = &'a mut Body<N>> {
        self.bodies_mut().filter(move |b| b.name() == name)
    }

    /// An iterator through all the contact events generated during the last execution of `self.step()`.
    pub fn contact_events(&self) -> &ContactEvents<ColliderHandle> {
        self.cworld.contact_events()
    }

    /// An iterator through all the proximity events generated during the last execution of `self.step()`.
    pub fn proximity_events(&self) -> &ProximityEvents<ColliderHandle> {
        self.cworld.proximity_events()
    }
}

impl<N: RealField> Default for World<N, BodySlab<N>> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod test {
    use crate::world::World;

    #[test]
    fn world_is_send_sync() {
        let _ = Box::new(World::<f32>::new()) as Box<Send + Sync>;
    }
}
