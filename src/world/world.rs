use slab::Slab;

use na::{self, RealField};
use ncollide;
use ncollide::events::{ContactEvents, ProximityEvents};

use crate::counters::Counters;
use crate::detection::{ActivationManager, ColliderContactManifold};
use crate::force_generator::{ForceGenerator, ForceGeneratorHandle};
use crate::joint::{ConstraintHandle, JointConstraint};
use crate::math::Vector;
use crate::object::{
    Body, BodySet, BodyDesc, BodyStatus, Collider, ColliderAnchor,
    ColliderHandle, Multibody, RigidBody, BodyHandle,
};
use crate::material::MaterialsCoefficientsTable;
use crate::solver::{ContactModel, IntegrationParameters, MoreauJeanSolver, SignoriniCoulombPyramidModel};
use crate::world::ColliderWorld;


/// The physics world.
pub struct World<N: RealField> {
    counters: Counters,
    bodies: BodySet<N>,
    active_bodies: Vec<BodyHandle>,
    cworld: ColliderWorld<N>,
    solver: MoreauJeanSolver<N>,
    activation_manager: ActivationManager<N>,
    material_coefficients: MaterialsCoefficientsTable<N>,
    // FIXME: set those two parameters per-collider?
    prediction: N,
    gravity: Vector<N>,
    constraints: Slab<Box<JointConstraint<N>>>,
    forces: Slab<Box<ForceGenerator<N>>>,
    params: IntegrationParameters<N>,
}

impl<N: RealField> World<N> {
    /// Creates a new physics world with default parameters.
    ///
    /// The ground body is automatically created and added to the world without any colliders attached.
    pub fn new() -> Self {
        let counters = Counters::new(false);
        let bv_margin = na::convert(0.01f64);
        let prediction = na::convert(0.002);
        let bodies = BodySet::new();
        let active_bodies = Vec::new();
        let constraints = Slab::new();
        let forces = Slab::new();
        let cworld = ColliderWorld::new(bv_margin);
        let contact_model = Box::new(SignoriniCoulombPyramidModel::new());
        let solver = MoreauJeanSolver::new(contact_model);
        let activation_manager = ActivationManager::new(na::convert(0.01f64));
        let gravity = Vector::zeros();
        let params = IntegrationParameters::default();
        let material_coefficients = MaterialsCoefficientsTable::new();

        World {
            counters,
            bodies,
            active_bodies,
            cworld,
            solver,
            activation_manager,
            material_coefficients,
            prediction,
            gravity,
            constraints,
            forces,
            params
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
    pub fn set_contact_model<C: ContactModel<N>>(&mut self, model: C) {
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
        self.params.dt
    }

    /// Sets the timestep used for the integration.
    pub fn set_timestep(&mut self, dt: N) {
        self.params.dt = dt;
    }

    /// Activate the given body.
    pub fn activate_body(&mut self, handle: BodyHandle) {
        Self::activate_body_at(&mut self.bodies, handle)
    }

    // NOTE: static method used to avoid borrowing issues.
    fn activate_body_at(bodies: &mut BodySet<N>, handle: BodyHandle) {
        if let Some(body) = bodies.body_mut(handle) {
            if body.status_dependent_ndofs() != 0 {
                body.activate();
            }
        }
    }

    /// Add a constraints to the physics world and retrieves its handle.
    pub fn add_constraint<C: JointConstraint<N>>(&mut self, constraint: C) -> ConstraintHandle {
        let (anchor1, anchor2) = constraint.anchors();
        self.activate_body(anchor1.0);
        self.activate_body(anchor2.0);
        self.constraints.insert(Box::new(constraint))
    }

    /// Get a reference to the specified constraint.
    pub fn constraint(&self, handle: ConstraintHandle) -> &JointConstraint<N> {
        &*self.constraints[handle]
    }

    /// Get a mutable reference to the specified constraint.
    pub fn constraint_mut(&mut self, handle: ConstraintHandle) -> &mut JointConstraint<N> {
        let (anchor1, anchor2) = self.constraints[handle].anchors();
        self.activate_body(anchor1.0);
        self.activate_body(anchor2.0);
        &mut *self.constraints[handle]
    }

    /// Remove the specified constraint from the world.
    pub fn remove_constraint(&mut self, handle: ConstraintHandle) -> Box<JointConstraint<N>> {
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
                    if let Some(b) = bodies.body_mut(coll.body()) {
                        b.activate()
                    }
                });
            }
        }

        self.cworld.remove(handles);
    }

    /// Add a force generator to the world.
    pub fn add_force_generator<G: ForceGenerator<N>>(
        &mut self,
        force_generator: G,
    ) -> ForceGeneratorHandle {
        self.forces.insert(Box::new(force_generator))
    }

    /// Retrieve a reference to the specified force generator.
    pub fn force_generator(&self, handle: ForceGeneratorHandle) -> &ForceGenerator<N> {
        &*self.forces[handle]
    }

    /// Retrieve a mutable reference to the specified force generator.
    pub fn force_generator_mut(&mut self, handle: ForceGeneratorHandle) -> &mut ForceGenerator<N> {
        &mut *self.forces[handle]
    }

    /// Remove the specified force generator from the world.
    pub fn remove_force_generator(
        &mut self,
        handle: ForceGeneratorHandle,
    ) -> Box<ForceGenerator<N>> {
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
    pub fn step(&mut self) {
        self.counters.step_started();

        /*
         *
         * Update body dynamics and accelerations.
         *
         */
        for b in self.bodies.bodies_mut() {
            b.update_kinematics();
            b.update_dynamics(self.params.dt);
        }

        let params = &self.params;
        let bodies = &mut self.bodies;
        self.forces.retain(|_, f| {
            f.apply(params, bodies)
        });

        for b in self.bodies.bodies_mut() {
            b.update_acceleration(&self.gravity, &self.params);
        }

        /*
         *
         * Sync colliders and perform CD if the user moved
         * manually some bodies.
         */
        self.cworld.clear_events();
        self.cworld.sync_colliders(&self.bodies);
        self.cworld.perform_broad_phase();
        self.cworld.perform_narrow_phase();

        /*
         *
         * Handle sleeping and collision
         * islands.
         *
         */
        // FIXME: for now, no island is built.
        self.counters.island_construction_started();
        self.active_bodies.clear();
        self.activation_manager.update(
            &mut self.bodies,
            &self.cworld,
            &self.constraints,
            &mut self.active_bodies,
        );
        self.counters.island_construction_completed();

        /*
         *
         * Collect contact manifolds.
         *
         */
        let mut contact_manifolds = Vec::new(); // FIXME: avoid allocations.
        for (c1, c2, _, manifold) in self.cworld.contact_pairs(false) {
            let b1 = try_continue!(self.bodies.body(c1.body()));
            let b2 = try_continue!(self.bodies.body(c2.body()));

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
        for b in self.bodies.bodies_mut() {
            // FIXME This is currently needed by the solver because otherwise
            // some kinematic bodies may end up with a companion_id (used as
            // an assembly_id) that it out of bounds of the velocity vector.
            // Note sure what the best place for this is though.
            b.set_companion_id(0);
        }

        self.solver.step(
            &mut self.counters,
            &mut self.bodies,
            &mut self.constraints,
            &contact_manifolds[..],
            &self.active_bodies[..],
            &self.params,
            &self.material_coefficients,
            &self.cworld,
        );

        for b in self.bodies.bodies_mut() {
            if b.status() == BodyStatus::Kinematic {
                b.integrate(&self.params)
            }
        }


        /*
         *
         * Update body kinematics and dynamics
         * after the contact resolution step.
         *
         */
        // FIXME: objects involved in a non-linear position stabilization already
        // updated their kinematics.
        self.bodies.bodies_mut().for_each(|b| {
            b.update_kinematics();
            b.update_dynamics(params.dt);
        });

        /*
         *
         * Update colliders and perform CD with the new
         * body positions.
         *
         */
        self.cworld.sync_colliders(&self.bodies);
        self.counters.collision_detection_started();
        self.cworld.perform_broad_phase();
        self.cworld.perform_narrow_phase();
        self.counters.collision_detection_completed();

        /*
         *
         * Finally, clear the update flag of every body.
         *
         */
        self.bodies.bodies_mut().for_each(|b| {
            b.clear_forces();
            b.clear_update_flags();
        });

        self.params.t += self.params.dt;
        self.counters.step_completed();
    }

    /// Remove the specified bodies.
    pub fn remove_bodies(&mut self, handles: &[BodyHandle]) {
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
            let b1_exists = bodies.body(c1.body()).is_some();
            let b2_exists = bodies.body(c2.body()).is_some();

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
            let b1_exists = bodies.body(b1.0).and_then(|b| b.part(b1.1)).is_some();
            let b2_exists = bodies.body(b2.0).and_then(|b| b.part(b2.1)).is_some();

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
    pub fn body(&self, handle: BodyHandle) -> Option<&Body<N>> {
        self.bodies.body(handle)
    }

    /// Get a mutable reference to the specified body.
    pub fn body_mut(&mut self, handle: BodyHandle) -> Option<&mut Body<N>> {
        self.bodies.body_mut(handle)
    }

    /// Get a reference to the multibody containing the specified multibody link.
    ///
    /// Returns `None` if the handle does not correspond to a multibody link in this world.
    pub fn multibody(&self, handle: BodyHandle) -> Option<&Multibody<N>> {
        self.bodies.body(handle)?.downcast_ref::<Multibody<N>>()
    }

    /// Get a mutable reference to the multibody containing the specified multibody link.
    ///
    /// Returns `None` if the handle does not correspond to a multibody link in this world.
    pub fn multibody_mut(&mut self, handle: BodyHandle) -> Option<&mut Multibody<N>> {
        self.bodies.body_mut(handle)?.downcast_mut::<Multibody<N>>()
    }

    /// Get a reference to the specified rigid body.
    ///
    /// Returns `None` if the handle does not correspond to a rigid body in this world.
    pub fn rigid_body(&self, handle: BodyHandle) -> Option<&RigidBody<N>> {
        self.bodies.body(handle)?.downcast_ref::<RigidBody<N>>()
    }

    /// Get a mutable reference to the specified rigid body.
    ///
    /// Returns `None` if the handle does not correspond to a rigid body in this world.
    pub fn rigid_body_mut(&mut self, handle: BodyHandle) -> Option<&mut RigidBody<N>> {
        self.bodies.body_mut(handle)?.downcast_mut::<RigidBody<N>>()
    }

    /// Reference to the underlying collision world.
    pub fn collider_world(&self) -> &ColliderWorld<N> {
        &self.cworld
    }

    /// Mutable reference to the underlying collision world.
    pub fn collider_world_mut(&mut self) -> &mut ColliderWorld<N> {
        &mut self.cworld
    }


    /// Mutable reference to the underlying collision world.
    #[doc(hidden)]
    pub fn bodies_mut_and_collider_world_mut(&mut self) -> (&mut BodySet<N>, &mut ColliderWorld<N>) {
        (&mut self.bodies, &mut self.cworld)
    }

    /// Get a reference to the specified collider.
    ///
    /// Returns `None` if the handle does not correspond to a collider in this world.
    pub fn collider(&self, handle: ColliderHandle) -> Option<&Collider<N>> {
        self.cworld.collider(handle)
    }

    /// Get a mutable reference to the specified collider.
    ///
    /// Returns `None` if the handle does not correspond to a collider in this world.
    pub fn collider_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider<N>> {
        self.cworld.collider_mut(handle)
    }

    /// Gets the handle of the body the specified collider is attached to.
    pub fn collider_body_handle(&self, handle: ColliderHandle) -> Option<BodyHandle> {
        self.collider_anchor(handle).map(|anchor| anchor.body())
    }

    /// Gets the anchor attaching this collider to a body or body part.
    pub fn collider_anchor(&self, handle: ColliderHandle) -> Option<&ColliderAnchor<N>> {
        self.cworld
            .collider(handle)
            .map(|co| co.anchor())
    }

    /// An iterator through all the colliders on this collision world.
    pub fn colliders(&self) -> impl Iterator<Item = &Collider<N>> {
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
    pub fn contact_events(&self) -> &ContactEvents {
        self.cworld.contact_events()
    }

    /// An iterator through all the proximity events generated during the last execution of `self.step()`.
    pub fn proximity_events(&self) -> &ProximityEvents {
        self.cworld.proximity_events()
    }
}

impl<N: RealField> Default for World<N> {
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
