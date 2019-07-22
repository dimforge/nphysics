use std::collections::{HashMap, HashSet};

use na::{self, RealField};
use ncollide;
use ncollide::query::{self, NonlinearTOIStatus};
use ncollide::narrow_phase::{Interaction, ContactEvents, ProximityEvents};
use ncollide::interpolation::{RigidMotion, ConstantVelocityRigidMotion};

use crate::counters::Counters;
use crate::detection::{ActivationManager, ColliderContactManifold};
use crate::force_generator::{ForceGenerator, DefaultForceGeneratorHandle, ForceGeneratorSet};
use crate::joint::{DefaultJointConstraintHandle, JointConstraint, JointConstraintSet};
use crate::math::{Vector, Velocity};
use crate::object::{
    Body, DefaultBodySet, BodyDesc, BodyStatus, Collider, ColliderAnchor, ColliderHandle,
    DefaultColliderHandle, Multibody, RigidBody, DefaultBodyHandle, BodySet, BodyHandle, ColliderSet,
};
use crate::material::MaterialsCoefficientsTable;
use crate::solver::{ContactModel, IntegrationParameters, MoreauJeanSolver, SignoriniCoulombPyramidModel};
use crate::world::ColliderWorld;

pub type DefaultDynamicWorld<N> = DynamicWorld<N, DefaultBodySet<N>, DefaultColliderHandle>;

struct SubstepState<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle> {
    active: bool,
    dt: N,
    // FIXME: can we avoid the use of a hash-map to save the
    // number of time a contact pair generated a CCD event?
    ccd_counts: HashMap<(CollHandle, CollHandle), usize>,
    body_times: HashMap<Handle, N>,
    locked_bodies: HashSet<Handle>,
}

/// The physics world.
pub struct DynamicWorld<N: RealField, Bodies: BodySet<N>, CollHandle: ColliderHandle> {
    pub counters: Counters,
    pub solver: MoreauJeanSolver<N, Bodies, CollHandle>,
    pub parameters: IntegrationParameters<N>,
    pub material_coefficients: MaterialsCoefficientsTable<N>,
    pub gravity: Vector<N>,
    activation_manager: ActivationManager<N, Bodies::Handle>,
    substep: SubstepState<N, Bodies::Handle, CollHandle>,
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
            locked_bodies: HashSet::new(),
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
         *
         * Handle CCD
         *
         */
        if self.parameters.ccd_enabled {
            self.counters.ccd_started();
            self.solve_ccd(cworld, bodies, colliders, constraints, forces);
            self.counters.ccd_completed();
        }

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

    // Outputs a sorted list of TOI event for the given time interval, assuming montions clamped at the first TOI.
    fn predict_next_impacts<Colliders>(&mut self,
                                    cworld: &ColliderWorld<N, Bodies::Handle, CollHandle>,
                                    bodies: &Bodies,
                                    colliders: &Colliders,
                                    time_interval: (N, N))
                                    -> (Vec<TOIEntry<N, Bodies::Handle, CollHandle>>, HashMap<Bodies::Handle, N>)
        where Colliders: ColliderSet<N, Bodies::Handle, Handle = CollHandle> {

        let mut impacts = Vec::new();
        let mut frozen = HashMap::new();
        let mut timestamps = HashMap::new();
        let mut all_toi = std::collections::BinaryHeap::new();

        let dt0 = self.parameters.dt();

        ColliderSet::foreach(colliders, |coll_handle, coll| {
            if coll.is_ccd_enabled() {
                for (ch1, c1, ch2, c2, inter) in cworld.interactions_with(colliders, coll_handle, false).unwrap() {
                    use crate::object::BodyPart;
                    let handle1 = c1.body();
                    let handle2 = c2.body();

                    let (b1, b2) = bodies.get_pair(handle1, handle2);
                    let (b1, b2) = (b1.unwrap(), b2.unwrap());

                    match inter {
                        Interaction::Contact(alg, manifold) => {
                            if let Some(toi) = TOIEntry::try_from_colliders(
                                ch1, ch2, c1, c2, b1, b2, None, None, &self.parameters, dt0, &self.substep.body_times) {
                                all_toi.push(toi);
                                let _ = timestamps.insert((ch1, ch2), 0);
                            }
                        }
                        Interaction::Proximity(prox) => unimplemented!()
                    }
                }
            }
        });

        let max_substeps = 1;
        let min_substep_size = 1.0;

        self.substep.locked_bodies.clear();

//        for toi in all_toi.iter() {
//            println!("Found initial toi: {}", toi.toi);
//        }

        while let Some(toi) = all_toi.pop() {
            if toi.timestamp != timestamps[&(toi.c1, toi.c2)] {
                // This TOI has been updated and is no longer valid.
                continue;
            }

            let count = self.substep.ccd_counts.entry((toi.c1, toi.c2)).or_insert(0);
            if false { // *count == max_substeps {
                let _ = self.substep.locked_bodies.insert(toi.b1);
                let _ = self.substep.locked_bodies.insert(toi.b2);
            } else {
                *count += 1;

                let _ = frozen.entry(toi.b1).or_insert(toi.toi);
                let _ = frozen.entry(toi.b2).or_insert(toi.toi);

                let to_traverse = [ toi.c1, toi.c2 ];

                for c in to_traverse.iter() {
                    let graph_id = colliders.get(*c).unwrap().graph_index().unwrap();

                    // Update any TOI involving c1 or c2.
                    for (ch1, ch2, inter) in cworld.interactions.interactions_with(graph_id, false) {
                        let c1 = colliders.get(ch1).unwrap();
                        let c2 = colliders.get(ch2).unwrap();

                        if !c1.is_ccd_enabled() && !c2.is_ccd_enabled() {
                            continue;
                        }

                        if frozen.contains_key(&c1.body()) && frozen.contains_key(&c2.body()) {
                            continue;
                        }

                        let b1 = bodies.get(c1.body()).unwrap();
                        let b2 = bodies.get(c2.body()).unwrap();

                        let timestamp = timestamps.entry((ch1, ch2)).or_insert(0);
                        *timestamp += 1;

                        let frozen1 = frozen.get(&c1.body()).cloned();
                        let frozen2 = frozen.get(&c2.body()).cloned();

                        if let Some(mut toi) = TOIEntry::try_from_colliders(
                            ch1, ch2, c1, c2, b1, b2, frozen1, frozen2, &self.parameters, dt0, &self.substep.body_times) {
                            toi.timestamp = *timestamp;
                            all_toi.push(toi)
                        }
                    }
                }

                impacts.push(toi);
            }
        }

        (impacts, frozen)
    }



    pub fn solve_ccd<Colliders, Constraints, Forces>(&mut self,
                                                cworld: &mut ColliderWorld<N, Bodies::Handle, CollHandle>,
                                                bodies: &mut Bodies,
                                                colliders: &mut Colliders,
                                                constraints: &mut Constraints,
                                                forces: &mut Forces)
        where Colliders: ColliderSet<N, Bodies::Handle, Handle = CollHandle>,
              Constraints: JointConstraintSet<N, Bodies>,
              Forces: ForceGeneratorSet<N, Bodies> {
        let dt0 = self.parameters.dt();
        let inv_dt0 = self.parameters.inv_dt();

        if dt0 == N::zero() {
            return;
        }
        println!("Loop");

        let mut parameters = self.parameters.clone();
//        parameters.max_velocity_iterations = 20;
        parameters.max_position_iterations = 20;
        parameters.warmstart_coeff = N::zero();

        let time_interval = (N::zero(), self.parameters.dt());

        self.counters.ccd.reset();

        for k in 0.. {
            // Update the broad phase.
            self.counters.ccd.broad_phase_time.resume();
            cworld.sync_colliders(bodies, colliders);
            cworld.perform_broad_phase(colliders);
            self.counters.ccd.broad_phase_time.pause();

            // Compute the next TOI.
            self.counters.ccd.toi_computation_time.resume();
            let (toi_entries, frozen) =
                self.predict_next_impacts(cworld, bodies, colliders, time_interval);
            self.counters.ccd.toi_computation_time.pause();

            // Resolve the minimum TOI events, if any.
            if !toi_entries.is_empty() && k != 1 {
                self.counters.ccd.num_substeps += 1;

                let mut island = Vec::new();
                let mut contact_manifolds = Vec::new();

                parameters.set_dt(dt0 - time_interval.1);


                // We will use the companion ID to know which body is already on the island.
                bodies.foreach_mut(|h, b| {
                    b.set_companion_id(0);
                });

                let mut colliders_to_traverse = Vec::new();
                let mut ccd_bodies = Vec::new();

                for entry in &toi_entries {
                    colliders_to_traverse.push(entry.c1);
                    colliders_to_traverse.push(entry.c2);

                    if !self.substep.locked_bodies.contains(&entry.b1) {
                        ccd_bodies.push(entry.b1);
                    }

                    if !self.substep.locked_bodies.contains(&entry.b2) {
                        ccd_bodies.push(entry.b2);
                    }
                }

                let mut interaction_ids = Vec::new();
                let mut visited = HashSet::new();

                self.counters.ccd.narrow_phase_time.resume();

                // Update contact manifolds.
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
//                                    if !((c1.is_ccd_enabled() || !b1.is_dynamic()) || (c2.is_ccd_enabled() || !b2.is_dynamic())) {
//                                        continue;
//                                    }

                                    let mut prepare_body = |h, b: &mut Bodies::Body, c: &mut Collider<N, Bodies::Handle>| {
                                        b.clear_forces();
                                        b.update_kinematics();
                                        b.update_dynamics(parameters.dt());
                                        b.update_acceleration(&Vector::zeros(), &parameters);

                                        let curr_body_time = self.substep.body_times.entry(h).or_insert(N::zero());

                                        let target_time = frozen.get(&h).cloned().unwrap_or(time_interval.1);
                                        println!("Preparing body with traget time: {}", target_time);

                                        if !self.substep.locked_bodies.contains(&h) {
                                            println!("Advancing to: {}", target_time - *curr_body_time);
                                            b.advance(target_time - *curr_body_time);
                                            b.clamp_advancement();
                                        }

                                        *curr_body_time = target_time;

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
                            Interaction::Proximity(prox) => unimplemented!()
                        }
                    }

                    let _ = visited.insert(c);
                }

                for eid in interaction_ids {
                    let (ch1, ch2, inter) = cworld.interactions.index_interaction(eid).unwrap();
                    match inter {
                        Interaction::Contact(_, manifold) => {
                            let c1 = colliders.get(ch1).unwrap();
                            let c2 = colliders.get(ch2).unwrap();
                            contact_manifolds.push(ColliderContactManifold::new(ch1, c1, ch2, c2, manifold));
                        }
                        Interaction::Proximity(_) => unimplemented!()
                    }
                }
                self.counters.ccd.narrow_phase_time.pause();

                // Solve the system and integrate.
                bodies.foreach_mut(|_, b| {
                    b.set_companion_id(0);
                });

                parameters.set_dt(dt0 - time_interval.1);
                println!("Integrating remaining time: {}", dt0 - time_interval.1);

                self.counters.ccd.solver_time.resume();
                self.solver.step_ccd(
                    &mut self.counters,
                    bodies,
                    colliders,
                    constraints,
                    &contact_manifolds[..],
                    &ccd_bodies[..],
                    &island[..],
                    &parameters,
                    &self.material_coefficients,
                );
                self.counters.ccd.solver_time.pause();

                // Update body kinematics and dynamics
                // after the contact resolution step.
                let gravity = &self.gravity;
                bodies.foreach_mut(|_, b| {
                    b.clear_forces();
                    b.update_kinematics();
                    b.update_dynamics(parameters.dt());
                });

                bodies.foreach_mut(|handle, body| {
                    if !body.is_static() && self.substep.locked_bodies.contains(&handle) {
                        body.clamp_advancement();
                    }
                });

                if self.parameters.substepping_enabled {
                    self.substep.active = true;
                    self.substep.dt = parameters.dt();
                    break;
                }
            } else {
                let mut substep = &mut self.substep;
                bodies.foreach_mut(|handle, body| {
                    if !body.is_static() && frozen.contains_key(&handle) { // substep.locked_bodies.contains(&handle) {
                        body.clamp_advancement();
                    }
                });

                self.substep.active = false;
                self.substep.ccd_counts.clear();
                self.substep.body_times.clear();
                self.substep.locked_bodies.clear();
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
    timestamp: usize
}

impl<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle> TOIEntry<N, Handle, CollHandle> {
    fn new(toi: N,
           c1: CollHandle,
           b1: Handle,
           c2: CollHandle,
           b2: Handle,
           timestamp: usize)
    -> Self {
        Self {
            toi, c1, b1, c2, b2, timestamp
        }
    }

    fn try_from_colliders(ch1: CollHandle,
                          ch2: CollHandle,
                          c1: &Collider<N, Handle>,
                          c2: &Collider<N, Handle>,
                          b1: &(impl Body<N> + ?Sized),
                          b2: &(impl Body<N> + ?Sized),
                          frozen1: Option<N>,
                          frozen2: Option<N>,
                          params: &IntegrationParameters<N>,
                          dt0: N,
                          body_times: &HashMap<Handle, N>,)
                        -> Option<Self> {
        let margins = c1.margin() + c2.margin();
        let target = params.allowed_linear_error; // self.parameters.allowed_linear_error.max(margins - self.parameters.allowed_linear_error * na::convert(3.0));

        let time1 = frozen1.unwrap_or(body_times.get(&c1.body()).cloned().unwrap_or(N::zero()));
        let time2 = frozen2.unwrap_or(body_times.get(&c2.body()).cloned().unwrap_or(N::zero()));
        let start_time = time1.max(time2);
        let time_origin1 = time1 - start_time;
        let time_origin2 = time2 - start_time;

        // Compute the TOI.
        let p1 = b1.part(0).unwrap();
        let p2 = b2.part(0).unwrap();

        let start1 = p1.safe_position() * c1.position_wrt_body();
        let start2 = p2.safe_position() * c2.position_wrt_body();
        let vel1 = p1.velocity();
        let vel2 = p2.velocity();
        let mut motion1 = ConstantVelocityRigidMotion::new(time_origin1, start1, vel1.linear, vel1.angular);
        let mut motion2 = ConstantVelocityRigidMotion::new(time_origin2, start2, vel2.linear, vel2.angular);

        if let Some(t) = frozen1 {
            motion1.t0 = N::zero();
            let pos = motion1.position_at_time(t);
            motion1.start = pos;
            motion1.linvel = na::zero();
            motion1.angvel = na::zero();
        }

        if let Some(t) = frozen2 {
            motion2.t0 = N::zero();
            let pos = motion2.position_at_time(t);
            motion2.start = pos;
            motion2.linvel = na::zero();
            motion2.angvel = na::zero();
        }

        let remaining_time = dt0 - start_time;

//                                if let Some(toi) = query::time_of_impact(&pos1, &v1, &**c1.shape(), &pos2, &v2, &**c2.shape(), target) {
        let toi = query::nonlinear_time_of_impact(&motion1, c1.shape(), &motion2, c2.shape(), remaining_time, target)?;
        // SUGGESTION: Don't use the TOI if the colliders are already penetrating?
        if true { // toi.status != NonlinearTOIStatus::Penetrating {
            let toi = start_time + toi.toi;
            Some(Self::new(toi, ch1, c1.body(), ch2, c2.body(), 0))
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