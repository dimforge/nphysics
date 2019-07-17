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
            self.solve_ccd(cworld, bodies, colliders, constraints, forces);
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

    pub fn compute_next_toi<Colliders>(&mut self,
                                        cworld: &mut ColliderWorld<N, Bodies::Handle, CollHandle>,
                                        bodies: &mut Bodies,
                                        colliders: &mut Colliders)
        -> Option<(N, CollHandle, Bodies::Handle, CollHandle, Bodies::Handle)>
        where Colliders: ColliderSet<N, Bodies::Handle, Handle = CollHandle> {

        let mut all_toi = std::collections::BinaryHeap::new();

        let dt0 = self.parameters.dt();

        ColliderSet::foreach(colliders, |coll_handle, coll| {
            if coll.is_ccd_enabled() {
                // Find a TOI
                for (ch1, c1, ch2, c2, inter) in cworld.interactions_with(colliders, coll_handle, false).unwrap() {
                    use crate::object::BodyPart;
                    let handle1 = c1.body();
                    let handle2 = c2.body();

                    let (b1, b2) = bodies.get_pair_mut(handle1, handle2);
                    let (b1, b2) = (b1.unwrap(), b2.unwrap());

                    match inter {
                        Interaction::Contact(alg, manifold) => {
                            let margins = c1.margin() + c2.margin();
                            let target = self.parameters.allowed_linear_error; // self.parameters.allowed_linear_error.max(margins - self.parameters.allowed_linear_error * na::convert(3.0));

                            let time1 = *self.substep.body_times.entry(c1.body()).or_insert(N::zero());
                            let time2 = *self.substep.body_times.entry(c2.body()).or_insert(N::zero());
                            let start_time = time1.max(time2);
                            let time_origin1 = time1 - start_time;
                            let time_origin2 = time2 - start_time;

                            // Compute the TOI.
                            let p1 = b1.part(0).unwrap();
                            let p2 = b2.part(0).unwrap();

                            let start1 = p1.safe_position() * c1.position_wrt_body();
                            let start2 = p2.safe_position() * c2.position_wrt_body();
//                            let end1 = p1.position() * c1.position_wrt_body();
//                            let end2 = p2.position() * c2.position_wrt_body();
                            let vel1 = p1.velocity();
                            let vel2 = p2.velocity();
                            let motion1 = ConstantVelocityRigidMotion::new(time_origin1, &start1, vel1.linear, vel1.angular);
                            let motion2 = ConstantVelocityRigidMotion::new(time_origin2, &start2, vel2.linear, vel2.angular);
                            let remaining_time = dt0 - start_time;

//                                if let Some(toi) = query::time_of_impact(&pos1, &v1, &**c1.shape(), &pos2, &v2, &**c2.shape(), target) {
                            if let Some(toi) = query::nonlinear_time_of_impact(&motion1, c1.shape(), &motion2, c2.shape(), remaining_time, target) {
                                // Don't use the TOI if the colliders are already penetrating.
                                if true { // toi.status != NonlinearTOIStatus::Penetrating {
                                    let toi = start_time + toi.toi;
                                    all_toi.push(TOIEntry(toi, ch1, c1.body(), ch2, c2.body()));
                                }
                            }
                        }
                        Interaction::Proximity(prox) => unimplemented!()
                    }
                }
            }
        });

        let max_substeps = 1;

        self.substep.locked_bodies.clear();

        while let Some(toi) = all_toi.pop() {
            let count = self.substep.ccd_counts.entry((toi.1, toi.3)).or_insert(0);
            if *count == max_substeps {
                let _ = self.substep.locked_bodies.insert(toi.2);
                let _ = self.substep.locked_bodies.insert(toi.4);
            } else {
                *count += 1;
                return Some((toi.0, toi.1, toi.2, toi.3, toi.4));
            }
        }

        None
    }

    // NOTE: this is an approach very similar to Box2D's.
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

        let mut parameters = self.parameters.clone();
//        parameters.max_velocity_iterations = 20;
        parameters.max_position_iterations = 20;
        parameters.warmstart_coeff = N::zero();

        for k in 0.. {
            // Perform collision detection.
            cworld.sync_colliders(bodies, colliders);
            cworld.perform_broad_phase(colliders);

            // Compute the next TOI.
            let ccd_handles = self.compute_next_toi(cworld, bodies, colliders);

            // Resolve the minimum TOI event, if any.
            if let Some((min_toi, c1, b1, c2, b2)) = ccd_handles {
                let mut island = Vec::new();
                let mut contact_manifolds = Vec::new();

                parameters.set_dt(dt0 - min_toi);


                /*
                // We will use the companion ID to know which body is already on the island.
                bodies.foreach_mut(|h, b| {
                    b.clear_forces();
                    b.update_kinematics();
                    b.update_dynamics(parameters.dt());
                    b.update_acceleration(&Vector::zeros(), &parameters);
                    b.set_companion_id(0);

                    let curr_body_time = self.substep.body_times.entry(h).or_insert(N::zero());

                    if !self.substep.locked_bodies.contains(&h) {
                        b.advance(min_toi - *curr_body_time);
                        b.clamp_advancement();
                    }
                    *curr_body_time = min_toi;

                    if !b.is_static() {
                        island.push(h);
                    }
                });

                cworld.sync_colliders(bodies, colliders);
                cworld.perform_broad_phase(colliders);
                cworld.perform_narrow_phase(colliders);

                for (ch1, c1, ch2, c2, _, manifold) in cworld.contact_pairs(colliders, false) {
                    let b1 = try_continue!(bodies.get(c1.body()));
                    let b2 = try_continue!(bodies.get(c2.body()));

                    if manifold.len() > 0
                        && b1.status() != BodyStatus::Disabled && b2.status() != BodyStatus::Disabled
                        && ((b1.status_dependent_ndofs() != 0 && b1.is_active())
                        || (b2.status_dependent_ndofs() != 0 && b2.is_active()))
                    {
                        contact_manifolds.push(ColliderContactManifold::new(ch1, c1, ch2, c2, manifold));
                    }
                }
                */

                // We will use the companion ID to know which body is already on the island.
                bodies.foreach_mut(|h, b| {
                    b.clear_forces();
                    b.update_kinematics();
                    b.update_dynamics(parameters.dt());
                    b.update_acceleration(&Vector::zeros(), &parameters);
                    b.set_companion_id(0);

                    let curr_body_time = self.substep.body_times.entry(h).or_insert(N::zero());

                    if !self.substep.locked_bodies.contains(&h) {
                        b.advance(min_toi - *curr_body_time);
                        b.clamp_advancement();
                    }
                    *curr_body_time = min_toi;
                });

                let mut colliders_to_traverse = vec![c1, c2]; // FIXME: should contain all the colliders attached to b1 and b2.
                let mut interaction_ids = Vec::new();
                let (ca, cb) = (c1, c2);

                let mut visited = HashSet::new();

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

                                    if b1.companion_id() == 0 && b1.is_dynamic() {
                                        b1.set_companion_id(1);
                                        island.push(c1.body());
                                        c1.set_position(b1.part(0).unwrap().position() * c1.position_wrt_body());
                                    }

                                    if b2.companion_id() == 0 && b2.is_dynamic() {
                                        b2.set_companion_id(1);
                                        island.push(c2.body());
                                        c2.set_position(b2.part(0).unwrap().position() * c2.position_wrt_body());
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


                // Solve the system and integrate.
                bodies.foreach_mut(|_, b| {
                    b.set_companion_id(0);
                });

                parameters.set_dt(dt0 - min_toi);

                self.solver.step_ccd(
                    &mut self.counters,
                    bodies,
                    colliders,
                    constraints,
                    &contact_manifolds[..],
                    [b1, b2],
                    &island[..],
                    &parameters,
                    &self.material_coefficients,
                    &self.substep.locked_bodies,
                );

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
                    if !body.is_static() && substep.locked_bodies.contains(&handle) {
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


struct TOIEntry<N: RealField, CollHandle, BodyHandle>(N, CollHandle, BodyHandle, CollHandle, BodyHandle);

impl<N: RealField, CollHandle, BodyHandle> PartialOrd for TOIEntry<N, CollHandle, BodyHandle> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        (-self.0).partial_cmp(&(-other.0))
    }
}

impl<N: RealField, CollHandle, BodyHandle> Ord for TOIEntry<N, CollHandle, BodyHandle> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).unwrap()
    }
}

impl<N: RealField, CollHandle, BodyHandle> PartialEq for TOIEntry<N, CollHandle, BodyHandle> {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<N: RealField, CollHandle, BodyHandle> Eq for TOIEntry<N, CollHandle, BodyHandle> {
}