use std::collections::{HashMap, HashSet};

use na::{self, RealField};
use ncollide;
use ncollide::query::{self, TOIStatus, Proximity};
use ncollide::narrow_phase::{Interaction};
use ncollide::interpolation::{RigidMotion, RigidMotionComposition};

use crate::counters::Counters;
use crate::detection::{ActivationManager, ColliderContactManifold};
use crate::force_generator::{ForceGenerator, ForceGeneratorSet};
use crate::joint::{JointConstraint, JointConstraintSet};
use crate::math::{Vector};
use crate::object::{
    Body, DefaultBodySet, BodyStatus, BodyPartMotion, Collider, ColliderHandle,
    DefaultColliderHandle, BodySet, BodyHandle, ColliderSet,
};
use crate::material::MaterialsCoefficientsTable;
use crate::solver::{IntegrationParameters, MoreauJeanSolver, SignoriniCoulombPyramidModel};
use crate::world::ColliderWorld;

/// The default dynamic world, that can be used with a `DefaultBodySet` and `DefaultColliderHandle`.
pub type DefaultDynamicWorld<N> = DynamicWorld<N, DefaultBodySet<N>, DefaultColliderHandle>;

enum PredictedImpacts<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle> {
    Impacts(Vec<TOIEntry<N, Handle, CollHandle>>, HashMap<Handle, N>),
    ImpactsAfterEndTime(N),
    NoImpacts,
}

impl<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle> PredictedImpacts<N, Handle, CollHandle> {
    fn unwrap_impacts(self) -> (Vec<TOIEntry<N, Handle, CollHandle>>, HashMap<Handle, N>) {
        match self {
            PredictedImpacts::Impacts(entries, frozen) => (entries, frozen),
            PredictedImpacts::ImpactsAfterEndTime(_) => panic!("Impacts have not been computed."),
            PredictedImpacts::NoImpacts => (Vec::new(), HashMap::new())
        }
    }
}


struct SubstepState<N: RealField, Handle: BodyHandle> {
    active: bool,
    dt: N,
    end_time: N,
    next_substep: usize,
    // FIXME: can we avoid the use of a hash-map to save the
    // number of time a contact pair generated a CCD event?
    body_times: HashMap<Handle, N>,
}

/// The physics world.
pub struct DynamicWorld<N: RealField, Bodies: BodySet<N>, CollHandle: ColliderHandle> {
    /// Performance counters used for debugging and benchmarking nphysics.
    pub counters: Counters,
    /// The constraints solver.
    pub solver: MoreauJeanSolver<N, Bodies, CollHandle>,
    /// Parameters of the whole simulation.
    pub integration_parameters: IntegrationParameters<N>,
    /// Coefficient table used for resolving material properties to apply at one contact.
    pub material_coefficients: MaterialsCoefficientsTable<N>,
    /// The acting on this dynamic world.
    pub gravity: Vector<N>,
    activation_manager: ActivationManager<N, Bodies::Handle>,
    substep: SubstepState<N, Bodies::Handle>,
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
        let integration_parameters = IntegrationParameters::default();
        let material_coefficients = MaterialsCoefficientsTable::new();
        let substep = SubstepState {
            active: false,
            dt: integration_parameters.dt(),
            end_time: N::zero(),
            next_substep: 0,
            body_times: HashMap::new(),
        };

        DynamicWorld {
            counters,
            solver,
            activation_manager,
            material_coefficients,
            gravity,
            integration_parameters,
            substep,
        }
    }

    /// Retrieve the timestep used for the integration.
    pub fn timestep(&self) -> N {
        self.integration_parameters.dt()
    }

    /// Sets the timestep used for the integration.
    pub fn set_timestep(&mut self, dt: N) {
        self.integration_parameters.set_dt(dt);
    }

    /// Maintain the internal structures of the dynamic world by handling insersion and removal
    /// events from every sets this dynamic world interacts with.
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
                b.update_dynamics(self.integration_parameters.dt());
            });

            // FIXME: how to make force generators work
            // with the external body set?
            let parameters = &self.integration_parameters;
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

            let mut active_joints = Vec::new();
            constraints.foreach(|h, j| {
                if j.is_active(bodies) {
                    active_joints.push(h)
                }
            });
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
                &active_joints[..],
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
         *
         * Handle CCD
         *
         */
        self.counters.ccd_started();
        self.solve_ccd(cworld, bodies, colliders, constraints, forces);
        self.counters.ccd_completed();

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

            self.integration_parameters.t += self.integration_parameters.dt();
            self.counters.step_completed();

            bodies.foreach_mut(|_, b| {
                b.clear_update_flags();
            });

            colliders.foreach_mut(|_, c| {
                c.clear_update_flags()
            })
        }
    }

    // Outputs a sorted list of TOI event (in ascending order) for the given time interval,
    // assuming body motions clamped at their first TOI.
    fn predict_next_impacts<Colliders>(&self,
                                       cworld: &ColliderWorld<N, Bodies::Handle, CollHandle>,
                                       bodies: &Bodies,
                                       colliders: &Colliders,
                                       end_time: N)
                                       -> PredictedImpacts<N, Bodies::Handle, CollHandle>
        where Colliders: ColliderSet<N, Bodies::Handle, Handle = CollHandle> {

        let mut impacts = Vec::new();
        let mut frozen = HashMap::<_, N>::new();
        let mut timestamps = HashMap::new();
        let mut all_toi = std::collections::BinaryHeap::new();
        let mut pairs_seen = HashSet::new();
        let mut min_overstep = self.integration_parameters.dt();

        ColliderSet::foreach(colliders, |coll_handle, coll| {
            if coll.is_ccd_enabled() {
                for (ch1, c1, ch2, c2, inter) in cworld.interactions_with(colliders, coll_handle, false).unwrap() {
                    if pairs_seen.insert((ch1, ch2)) {
                        
                        let handle1 = c1.body();
                        let handle2 = c2.body();

                        let (b1, b2) = bodies.get_pair(handle1, handle2);
                        let (b1, b2) = (b1.unwrap(), b2.unwrap());

                        if let Some(toi) = TOIEntry::try_from_colliders(
                            ch1, ch2, c1, c2, b1, b2, inter.is_proximity(), None, None, &self.integration_parameters, min_overstep, &self.substep.body_times) {

                            if toi.toi > end_time {
                                min_overstep = min_overstep.min(toi.toi);
                            } else {
                                min_overstep = end_time;
                                all_toi.push(toi);
                                let _ = timestamps.insert((ch1, ch2), 0);
                            }
                        }
                    }
                }
            }
        });

        if min_overstep == self.integration_parameters.dt() && all_toi.is_empty() {
            return PredictedImpacts::NoImpacts;
        } else if min_overstep > end_time {
            return PredictedImpacts::ImpactsAfterEndTime(min_overstep);
        }

        // NOTE: all static bodies should be considered as "frozen", this
        // may avoid some resweeps.

        while let Some(toi) = all_toi.pop() {
            assert!(toi.toi <= end_time);

            if toi.timestamp != timestamps[&(toi.c1, toi.c2)] {
                // This TOI has been updated and is no longer valid.
                continue;
            }

            if let (Some(t1), Some(t2)) = (frozen.get(&toi.b1), frozen.get(&toi.b2)) {
                // If both objects are frozen, they won't move anymore
                // so they can't touch in the future anymore (and there is
                // no need for any resweep since the resweep already happened
                // the time it has been frozen).

                // There is actually an impact if it is known to happen at the time the
                // latest object was frozen (thought that sounds like a very unlikely case).
                if toi.toi == t1.max(*t2) {
                    impacts.push(toi);
                }
                continue;
            }

            if toi.is_proximity {
                // This is only a proximity so we don't have to freeze and there is no need to resweep.
                impacts.push(toi);
                continue;
            }

            let _ = frozen.entry(toi.b1).or_insert(toi.toi);
            let _ = frozen.entry(toi.b2).or_insert(toi.toi);

            let to_traverse = [ toi.c1, toi.c2 ];

            for c in to_traverse.iter() {
                let c = colliders.get(*c).unwrap();

                // No need to resweep if the collider is static.
                if c.body_status_dependent_ndofs() == 0 {
                    continue;
                }
                let graph_id = c.graph_index().unwrap();

                // Update any TOI involving c1 or c2.
                for (ch1, ch2, inter) in cworld.interactions.interactions_with(graph_id, false) {
                    let c1 = colliders.get(ch1).unwrap();
                    let c2 = colliders.get(ch2).unwrap();

                    if !c1.is_ccd_enabled() && !c2.is_ccd_enabled() {
                        continue;
                    }

                    let frozen1 = frozen.get(&c1.body()).cloned();
                    let frozen2 = frozen.get(&c2.body()).cloned();

                    if (frozen1.is_some() || c1.body_status_dependent_ndofs() == 0) &&
                        (frozen2.is_some() || c2.body_status_dependent_ndofs() == 0) {
                        continue;
                    }

                    let b1 = bodies.get(c1.body()).unwrap();
                    let b2 = bodies.get(c2.body()).unwrap();

                    let timestamp = timestamps.entry((ch1, ch2)).or_insert(0);
                    *timestamp += 1;


                    if let Some(mut toi) = TOIEntry::try_from_colliders(
                        ch1, ch2, c1, c2, b1, b2, inter.is_proximity(), frozen1, frozen2, &self.integration_parameters, end_time, &self.substep.body_times) {
                        toi.timestamp = *timestamp;
                        all_toi.push(toi)
                    }
                }
            }

            impacts.push(toi);
        }

        PredictedImpacts::Impacts(impacts, frozen)
    }



    fn solve_ccd<Colliders, Constraints, Forces>(&mut self,
                                                cworld: &mut ColliderWorld<N, Bodies::Handle, CollHandle>,
                                                bodies: &mut Bodies,
                                                colliders: &mut Colliders,
                                                constraints: &mut Constraints,
                                                _forces: &mut Forces)
        where Colliders: ColliderSet<N, Bodies::Handle, Handle = CollHandle>,
              Constraints: JointConstraintSet<N, Bodies>,
              Forces: ForceGeneratorSet<N, Bodies> {
        let dt0 = self.integration_parameters.dt();
        let _inv_dt0 = self.integration_parameters.inv_dt();

        if dt0 == N::zero() || self.integration_parameters.max_ccd_substeps == 0 {
            return;
        }

        let substep_length = self.integration_parameters.dt() / na::convert(self.integration_parameters.max_ccd_substeps as f64);

        let mut parameters = self.integration_parameters.clone();

        parameters.max_position_iterations = parameters.max_ccd_position_iterations;
        parameters.warmstart_coeff = N::zero();

        self.counters.ccd.reset();

        let mut last_iter = false;

        loop {
            if self.substep.end_time < dt0 {
                self.substep.end_time = (self.substep.end_time + substep_length).min(dt0);
            } else {
                last_iter = true;
            }

            // Update the broad phase.
            self.counters.ccd.broad_phase_time.resume();
            cworld.sync_colliders(bodies, colliders);
            cworld.perform_broad_phase(colliders);
            self.counters.ccd.broad_phase_time.pause();

            // Compute the next TOI.
            self.counters.ccd.toi_computation_time.resume();

            let (toi_entries, frozen) = match self.predict_next_impacts(cworld, bodies, colliders, self.substep.end_time) {
                PredictedImpacts::Impacts(toi_entries, frozen) => {
                    (toi_entries, frozen)
                },
                PredictedImpacts::ImpactsAfterEndTime(min_toi) => {
                    self.substep.end_time = (min_toi / substep_length).floor() * substep_length;
                    self.substep.end_time = (self.substep.end_time + substep_length).min(dt0);

                    self.predict_next_impacts(cworld, bodies, colliders, self.substep.end_time).unwrap_impacts()
                },
                PredictedImpacts::NoImpacts => {
                    (Vec::new(), HashMap::new())
                }
            };

            self.counters.ccd.toi_computation_time.pause();

            // Resolve the minimum TOI events, if any.
            if !last_iter && !toi_entries.is_empty() {
                self.counters.ccd.num_substeps += 1;

                let mut island = Vec::new();
                let mut contact_manifolds = Vec::new();

                // We will start the integration at the date of the latest TOI before the end_time.
                // (We don't start at end_time directly to avoid clamping some motion needlessly).
                let last_toi = toi_entries.last().unwrap().toi;
                parameters.set_dt(dt0 - last_toi);

                // We will use the companion ID to know which body is already on the island.
                bodies.foreach_mut(|_h, b| {
                    b.set_companion_id(0);
                });

                let mut colliders_to_traverse = Vec::new();
                let mut ccd_bodies = Vec::new();

                for entry in &toi_entries {
                    if !entry.is_proximity {
                        colliders_to_traverse.push(entry.c1);
                        colliders_to_traverse.push(entry.c2);

                        ccd_bodies.push(entry.b1);
                        ccd_bodies.push(entry.b2);
                    }
                }

                let mut interaction_ids = Vec::new();
                let mut visited = HashSet::new();

                self.counters.ccd.narrow_phase_time.resume();

                // Advance colliders and update contact manifolds.
                while let Some(c) = colliders_to_traverse.pop() {
                    let graph_id = colliders.get(c).unwrap().graph_index().unwrap();

                    for (ch1, ch2, eid, inter) in cworld.interactions.interactions_with_mut(graph_id) {
                        if (ch1 == c && visited.contains(&ch2)) || (ch2 == c && visited.contains(&ch1)) {
                            continue;
                        }

                        match inter {
                            Interaction::Contact(alg, manifold) => {
                                let (c1, c2) = colliders.get_pair_mut(ch1, ch2);
                                let (c1, c2) = (c1.unwrap(), c2.unwrap());
                                let (handle1, handle2) = (c1.body(), c2.body());

                                if let (Some(b1), Some(b2)) = bodies.get_pair_mut(handle1, handle2) {
                                    let mut prepare_body = |h, b: &mut Bodies::Body, c: &mut Collider<N, Bodies::Handle>| {
                                        b.clear_forces();
                                        b.update_kinematics();
                                        b.update_dynamics(parameters.dt());
                                        b.update_acceleration(&Vector::zeros(), &parameters);

                                        let curr_body_time = self.substep.body_times.entry(h).or_insert(N::zero());

                                        let target_time = frozen.get(&h).cloned().unwrap_or(last_toi);
                                        b.advance(target_time - *curr_body_time);
                                        b.clamp_advancement();

                                        // This body will have its motion clamped between the times target_time and last_time.
                                        *curr_body_time = last_toi;

                                        b.set_companion_id(1);
                                        island.push(h);
                                        c.set_position(b.part(0).unwrap().position() * c.position_wrt_body());
                                    };

                                    if b1.companion_id() == 0 && b1.is_dynamic() {
                                        prepare_body(handle1, b1, c1)
                                    }

                                    if b2.companion_id() == 0 && b2.is_dynamic() {
                                        prepare_body(handle2, b2, c2)
                                    }

                                    cworld.narrow_phase.update_contact(c1, c2, ch1, ch2, &mut **alg, manifold);

                                    if manifold.len() > 0 {
                                        interaction_ids.push(eid);

                                        if ch1 != c {
                                            colliders_to_traverse.push(ch1);
                                        }

                                        if ch2 != c {
                                            colliders_to_traverse.push(ch2);
                                        }
                                    }
                                }
                            }
                            Interaction::Proximity(..) => {
                                // Proximities are handled in another loop,
                                // after all the bodies have been advanced.
                            }
                        }
                    }

                    let _ = visited.insert(c);
                }

                // Handle proximities.
                for toi in &toi_entries {
                    if toi.is_proximity {
                        let mut c1 = colliders.get(toi.c1).unwrap();
                        let mut c2 = colliders.get(toi.c2).unwrap();
                        let (ch1, ch2, detector, prox) = cworld.interactions.proximity_pair_mut(c1.graph_index().unwrap(), c2.graph_index().unwrap()).unwrap();

                        if ch1 != toi.c1 {
                            // The order of the colliders may not be the same in the interaction.
                            std::mem::swap(&mut c1, &mut c2)
                        }

                        // Emit an event (the case where we already have *prox == Intersecting will be filtered out automatically).
                        cworld.narrow_phase.emit_proximity_event(ch1, ch2, *prox, Proximity::Intersecting);

                        // Set the proximity as intersecting.
                        *prox = Proximity::Intersecting;

                        // If we mulitple events is disabled, there is nothing more to do as the
                        // final proximity event will only depend on the final position of the
                        // colliders (and thus will be handled by the final narrow-phase update of
                        // the non-ccd step).
                        if self.integration_parameters.multiple_ccd_trigger_events_enabled {
                            // If the bodies of the colliders have been
                            // teleported at a time of impact, then we have to update the proximity now
                            // to account for multiple on/off proximity events during consecutive substeps.
                            //
                            // If the bodies have not been teleported, they will be handled later during the
                            // final narrow-phase update of the non-ccd step.
                            //
                            // FIXME: In the case where this is the last substep to
                            // be performed, and the related bodies remain frozen at the end of CCD
                            // handling, then this proximity update will happen twice. The results will
                            // still be correct, but this will unfortunately cost two proximity updates
                            // instead of just one.
                            let b1 = bodies.get(c1.body()).unwrap();
                            let b2 = bodies.get(c2.body()).unwrap();

                            if b1.companion_id() == 1 || b2.companion_id() == 1 {
                                cworld.narrow_phase.update_proximity(
                                    c1,
                                    c2,
                                    ch1,
                                    ch2,
                                    detector,
                                    prox
                                )
                            }
                        }
                    }
                }

                // Collect contact manifolds.
                for eid in interaction_ids {
                    let (ch1, ch2, inter) = cworld.interactions.index_interaction(eid).unwrap();
                    match inter {
                        Interaction::Contact(_, manifold) => {
                            let c1 = colliders.get(ch1).unwrap();
                            let c2 = colliders.get(ch2).unwrap();
                            contact_manifolds.push(ColliderContactManifold::new(ch1, c1, ch2, c2, manifold));
                        }
                        Interaction::Proximity(..) => {}
                    }
                }

                self.counters.ccd.narrow_phase_time.pause();

                // Solve the system and integrate.
                bodies.foreach_mut(|_, b| {
                    b.set_companion_id(0);
                });

                self.counters.ccd.solver_time.resume();
                self.solver.step_ccd(
                    &mut self.counters,
                    bodies,
                    colliders,
                    constraints,
                    &contact_manifolds[..],
                    &ccd_bodies[..],
                    &island[..],
                    &[], // FIXME: take constraints into account?
                    &parameters,
                    &self.material_coefficients,
                );
                self.counters.ccd.solver_time.pause();

                // Update body kinematics and dynamics
                // after the contact resolution step.
                let _gravity = &self.gravity;
                bodies.foreach_mut(|_, b| {
                    b.clear_forces();
                    b.update_kinematics();
                    b.update_dynamics(parameters.dt());
                });

                if self.integration_parameters.return_after_ccd_substep {
                    self.substep.active = true;
                    self.substep.dt = parameters.dt();
                    break;
                }
            } else {
                let _substep = &mut self.substep;
                bodies.foreach_mut(|handle, body| {
                    if !body.is_static() && frozen.contains_key(&handle) {
                        body.clamp_advancement();
                    }
                });

                self.substep.active = false;
                self.substep.body_times.clear();
                self.substep.next_substep = 0;
                self.substep.end_time = N::zero();
                break;
            }
        }
    }
}


struct TOIEntry<N: RealField, Handle, CollHandle> {
    toi: N,
    c1: CollHandle,
    b1: Handle,
    c2: CollHandle,
    b2: Handle,
    is_proximity: bool,
    timestamp: usize
}

impl<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle> TOIEntry<N, Handle, CollHandle> {
    fn new(toi: N,
           c1: CollHandle,
           b1: Handle,
           c2: CollHandle,
           b2: Handle,
           is_proximity: bool,
           timestamp: usize)
    -> Self {
        Self {
            toi, c1, b1, c2, b2, is_proximity, timestamp
        }
    }

    fn try_from_colliders(ch1: CollHandle,
                          ch2: CollHandle,
                          c1: &Collider<N, Handle>,
                          c2: &Collider<N, Handle>,
                          b1: &(impl Body<N> + ?Sized),
                          b2: &(impl Body<N> + ?Sized),
                          is_proximity: bool,
                          frozen1: Option<N>,
                          frozen2: Option<N>,
                          params: &IntegrationParameters<N>,
                          end_time: N,
                          body_times: &HashMap<Handle, N>)
                        -> Option<Self> {
        let _margins = c1.margin() + c2.margin();
        let target = params.allowed_linear_error; // self.integration_parameters.allowed_linear_error.max(margins - self.integration_parameters.allowed_linear_error * na::convert(3.0));

        let body_time1 = body_times.get(&c1.body()).cloned().unwrap_or(N::zero());
        let body_time2 = body_times.get(&c2.body()).cloned().unwrap_or(N::zero());
        let time1 = frozen1.unwrap_or(body_time1);
        let time2 = frozen2.unwrap_or(body_time2);

        let start_time = time1.max(time2);
        let time_origin1 = time1 - start_time;
        let time_origin2 = time2 - start_time;

        // Compute the TOI.
        let mut motion1 = b1.part_motion(0, time_origin1)?;
        let mut motion2 = b2.part_motion(0, time_origin2)?;

        if let Some(t) = frozen1 {
            let pos = motion1.position_at_time(t + time_origin1 - body_time1);
            motion1 = BodyPartMotion::Static(pos);
        }

        if let Some(t) = frozen2 {
            let pos = motion2.position_at_time(t + time_origin2 - body_time2);
            motion2 = BodyPartMotion::Static(pos);
        }

        let remaining_time = end_time - start_time;
        let toi;

        if motion1.is_static_or_linear() && motion2.is_static_or_linear() {
            let pos1 = motion1.position_at_time(N::zero()) * c1.position_wrt_body();
            let pos2 = motion2.position_at_time(N::zero()) * c2.position_wrt_body();
            toi = query::time_of_impact(&pos1, &motion1.linvel(), c1.shape(), &pos2, &motion2.linvel(), c2.shape(), target)?
        } else {
            let motion1 = motion1.prepend_transformation(c1.position_wrt_body());
            let motion2 = motion2.prepend_transformation(c2.position_wrt_body());
            toi = query::nonlinear_time_of_impact(&motion1, c1.shape(), &motion2, c2.shape(), remaining_time, target)?
        }

        if params.ccd_on_penetration_enabled || toi.status != TOIStatus::Penetrating {
            let toi = start_time + toi.toi;
            Some(Self::new(toi, ch1, c1.body(), ch2, c2.body(), is_proximity, 0))
        } else {
            None
        }
    }
}

impl<N: RealField, CollHandle, BodyHandle> PartialOrd for TOIEntry<N, CollHandle, BodyHandle> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        (-self.toi).partial_cmp(&(-other.toi))
    }
}

impl<N: RealField, CollHandle, BodyHandle> Ord for TOIEntry<N, CollHandle, BodyHandle> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).unwrap()
    }
}

impl<N: RealField, CollHandle, BodyHandle> PartialEq for TOIEntry<N, CollHandle, BodyHandle> {
    fn eq(&self, other: &Self) -> bool {
        self.toi == other.toi
    }
}

impl<N: RealField, CollHandle, BodyHandle> Eq for TOIEntry<N, CollHandle, BodyHandle> {
}