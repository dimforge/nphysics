use std::collections::{HashMap, HashSet};

use na::{self, RealField};
use ncollide;
use ncollide::query::{self, NonlinearTOIStatus};
use ncollide::narrow_phase::{Interaction, ContactEvents, ProximityEvents};
use ncollide::interpolation::ConstantVelocityRigidMotion;

use crate::counters::Counters;
use crate::detection::{ActivationManager, ColliderContactManifold};
use crate::force_generator::{ForceGenerator, DefaultForceGeneratorHandle, ForceGeneratorSet};
use crate::joint::{DefaultJointConstraintHandle, JointConstraint, JointConstraintSet};
use crate::math::Vector;
use crate::object::{
    Body, DefaultBodySet, BodyDesc, BodyStatus, Collider, ColliderAnchor, ColliderHandle,
    DefaultColliderHandle, Multibody, RigidBody, DefaultBodyHandle, BodySet, BodyHandle, ColliderSet,
};
use crate::material::MaterialsCoefficientsTable;
use crate::solver::{ContactModel, IntegrationParameters, MoreauJeanSolver, SignoriniCoulombPyramidModel};
use crate::world::ColliderWorld;

pub type DefaultDynamicWorld<N> = DynamicWorld<N, DefaultBodySet<N>, DefaultColliderHandle>;

struct SubstepState<N: RealField> {
    active: bool,
    dt: N,
    // FIXME: can we avoid the use of a hash-map to save the
    // number of time a contact pair generated a CCD event?
    ccd_counts: HashMap<(DefaultColliderHandle, DefaultColliderHandle), usize>,
    body_times: HashMap<DefaultBodyHandle, N>,
    body_hit: HashSet<DefaultBodyHandle>,
}

/// The physics world.
pub struct DynamicWorld<N: RealField, Bodies: BodySet<N>, CollHandle: ColliderHandle> {
    pub counters: Counters,
    pub solver: MoreauJeanSolver<N, Bodies, CollHandle>,
    pub parameters: IntegrationParameters<N>,
    pub material_coefficients: MaterialsCoefficientsTable<N>,
    pub gravity: Vector<N>,
    activation_manager: ActivationManager<N, Bodies::Handle>,
    substep: SubstepState<N>,
}

impl<N: RealField, Bodies: BodySet<N>, CollHandle: ColliderHandle> DynamicWorld<N, Bodies, CollHandle> {
    /// Creates a new physics world with default parameters.
    ///
    /// The ground body is automatically created and added to the world without any colliders attached.
    pub fn new(gravity: Vector<N>) -> Self {
        let counters = Counters::new(false);
        let contact_model = Box::new(SignoriniCoulombPyramidModel::new());
        let solver = MoreauJeanSolver::new(contact_model);
        let activation_manager = ActivationManager::new(na::convert(0.01f64));
        let parameters = IntegrationParameters::default();
        let material_coefficients = MaterialsCoefficientsTable::new();
        let substep = SubstepState {
            active: false,
            dt: parameters.dt(),
            ccd_counts: HashMap::new(),
            body_times: HashMap::new(),
            body_hit: HashSet::new(),
        };

        DynamicWorld {
            counters,
            solver,
            activation_manager,
            material_coefficients,
            gravity,
            parameters,
            substep,
        }
    }

    /// Retrieve the timestep used for the integration.
    pub fn timestep(&self) -> N {
        self.parameters.dt()
    }

    /// Sets the timestep used for the integration.
    pub fn set_timestep(&mut self, dt: N) {
        self.parameters.set_dt(dt);
    }

    pub fn maintain<Colliders, Constraints>(&mut self,
                                            cworld: &mut ColliderWorld<N, Bodies::Handle, CollHandle>,
                                            bodies: &mut Bodies,
                                            colliders: &mut Colliders,
                                            constraints: &mut Constraints)
        where Colliders: ColliderSet<N, Bodies::Handle, Handle = CollHandle>,
              Constraints: JointConstraintSet<N, Bodies> {
        // NOTE: the order of handling events matters.
        // In particular, handling body removal events must be done first because it
        // can itself cause constraints or colliders to be automatically removed too.

        let mut at_least_one_body_removed = false; // If at least one body is removed we need to find the constraints to remove too.

        while let Some(handle) = bodies.pop_removal_event() {
            // Remove every colliders attached to this body.
            if let Some(colls_to_remove) = cworld.body_colliders(handle) {
                for collider in colls_to_remove {
                    let _ = colliders.remove(*collider);
                }
            }

            at_least_one_body_removed = true;
        }

        // Remove constraints with a missing body.
        if at_least_one_body_removed {
            let mut constraints_to_remove = Vec::new();
            constraints.foreach(|h, c| {
                let (b1, b2) = c.anchors();
                if !bodies.contains(b1.0) || !bodies.contains(b2.0) {
                    constraints_to_remove.push(h)
                }
            });

            for to_remove in constraints_to_remove {
                constraints.remove(to_remove);
            }
        }

        while let Some((_, body1, body2)) = constraints.pop_removal_event() {
            // Wake-up the bodies this was attached to.
            if let Some(body1) = bodies.get_mut(body1.0) {
                body1.activate()
            }

            if let Some(body2) = bodies.get_mut(body2.0) {
                body2.activate()
            }
        }

        while let Some((_, body1, body2)) = constraints.pop_insertion_event() {
            // Wake-up the bodies this was attached to.
            if let Some(body1) = bodies.get_mut(body1.0) {
                body1.activate()
            }

            if let Some(body2) = bodies.get_mut(body2.0) {
                body2.activate()
            }
        }

        cworld.maintain(bodies, colliders);
    }

    /// Execute one time step of the physics simulation.
    pub fn step<Colliders, Constraints, Forces>(&mut self,
                                                cworld: &mut ColliderWorld<N, Bodies::Handle, CollHandle>,
                                                bodies: &mut Bodies,
                                                colliders: &mut Colliders,
                                                constraints: &mut Constraints,
                                                forces: &mut Forces)
    where Colliders: ColliderSet<N, Bodies::Handle, Handle = CollHandle>,
          Constraints: JointConstraintSet<N, Bodies>,
          Forces: ForceGeneratorSet<N, Bodies> {
        if !self.substep.active {
            self.counters.step_started();

            /*
             *
             * Handle insertions/removals.
             *
             */
            self.maintain(cworld, bodies, colliders, constraints);

            /*
             *
             * Update body dynamics and accelerations.
             *
             */
            bodies.foreach_mut(|_, b| {
                b.step_started();
                b.update_kinematics();
                b.update_dynamics(self.parameters.dt());
            });

            // FIXME: how to make force generators work
            // with the external body set?
            let parameters = &self.parameters;
            forces.foreach_mut(|_, f| {
                f.apply(parameters, bodies)
            });

            bodies.foreach_mut(|_, b| {
                b.update_acceleration(&self.gravity, parameters);
            });

            /*
             *
             * Sync colliders and perform CD if the user moved
             * manually some bodies.
             */
            cworld.clear_events();
            cworld.sync_colliders(bodies, colliders);
            cworld.perform_broad_phase(colliders);
            cworld.perform_narrow_phase(colliders);

            colliders.foreach_mut(|_, c| {
                c.clear_update_flags()
            });

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
                colliders,
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
            for (h1, c1, h2, c2, _, manifold) in cworld.contact_pairs(colliders, false) {
                let b1 = try_continue!(bodies.get(c1.body()));
                let b2 = try_continue!(bodies.get(c2.body()));

                if manifold.len() > 0
                    && b1.status() != BodyStatus::Disabled && b2.status() != BodyStatus::Disabled
                    && ((b1.status_dependent_ndofs() != 0 && b1.is_active())
                    || (b2.status_dependent_ndofs() != 0 && b2.is_active()))
                {
                    contact_manifolds.push(ColliderContactManifold::new(h1, c1, h2, c2, manifold));
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
                colliders,
                constraints,
                &contact_manifolds[..],
                &active_bodies[..],
                parameters,
                &self.material_coefficients,
            );

            bodies.foreach_mut(|_, b| {
                if b.status() == BodyStatus::Kinematic {
                    b.integrate(parameters)
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
                b.update_dynamics(parameters.dt());
            });
        }
/*
        /*
         *
         * Handle CCD
         *
         */
        if self.parameters.ccd_enabled {
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
            cworld.sync_colliders(bodies, colliders);

            self.counters.broad_phase_started();
            cworld.perform_broad_phase(colliders);
            self.counters.broad_phase_completed();

            self.counters.narrow_phase_started();
            cworld.perform_narrow_phase(colliders);
            self.counters.narrow_phase_completed();
            self.counters.collision_detection_completed();

            self.parameters.t += self.parameters.dt();
            self.counters.step_completed();

            bodies.foreach_mut(|_, b| {
                b.clear_update_flags();
            });

            colliders.foreach_mut(|_, c| {
                c.clear_update_flags()
            })
        }
    }

    /*
    // NOTE: this is an approach very similar to Box2D's.
    fn solve_ccd(&mut self) {
        let dt0 = self.parameters.dt();
        let inv_dt0 = self.parameters.inv_dt();

        if dt0 == N::zero() {
            return;
        }

        let mut parameters = self.parameters.clone();
//        parameters.max_velocity_iterations = 20;
        parameters.max_position_iterations = 20;
        parameters.warmstart_coeff = N::zero();

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
                                let target = self.parameters.allowed_linear_error; // self.parameters.allowed_linear_error.max(margins - self.parameters.allowed_linear_error * na::convert(3.0));

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
                parameters.set_dt(dt0 - min_toi);
                // We will use the companion ID to know which body is already on the island.
                for b in self.bodies.bodies_mut() {
                    b.clear_forces();
                    b.update_kinematics();
                    b.update_dynamics(parameters.dt());
                    b.update_acceleration(&Vector::zeros(), &parameters);
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
                parameters.set_dt(dt0 - min_toi);
                println!("Time-stepping length: {}", parameters.dt());
                println!("Num contacts: {}", contact_manifolds.len());
                println!("Island len: {}", island.len());

                self.solver.step_ccd(
                    &mut self.counters,
                    &mut self.bodies,
                    &mut empty_constraints,
                    &contact_manifolds[..],
                    [b1, b2],
                    &island[..],
                    &parameters,
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
                    b.update_dynamics(parameters.dt());
                });

                if self.parameters.substepping_enabled {
                    println!("Substep completed.");
                    self.substep.active = true;
                    self.substep.dt = parameters.dt();
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

    /*
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
    */
}
