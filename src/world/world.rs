use slab::Slab;

use na::{self, Real};
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
use crate::solver::{ContactModel, IntegrationParameters, MoreauJeanSolver, SignoriniCoulombPyramidModel};
use crate::world::ColliderWorld;

/// The physics world.
pub struct World<N: Real> {
    counters: Counters,
    bodies: BodySet<N>,
    active_bodies: Vec<BodyHandle>,
    // The set of colliders that have a parent.
    colliders_w_parent: Vec<ColliderHandle>,
    cworld: ColliderWorld<N>,
    solver: MoreauJeanSolver<N>,
    activation_manager: ActivationManager<N>,
    // FIXME: set those two parameters per-collider?
    prediction: N,
    gravity: Vector<N>,
    constraints: Slab<Box<JointConstraint<N>>>,
    forces: Slab<Box<ForceGenerator<N>>>,
    params: IntegrationParameters<N>,
}

impl<N: Real> World<N> {
    /// Creates a new physics world with default parameters.
    ///
    /// The ground body is automatically created and added to the world without any colliders attached.
    pub fn new() -> Self {
        let counters = Counters::new(false);
        let bv_margin = na::convert(0.01f64);
        let prediction = na::convert(0.002);
        let bodies = BodySet::new();
        let active_bodies = Vec::new();
        let colliders_w_parent = Vec::new();
        let constraints = Slab::new();
        let forces = Slab::new();
        let cworld = ColliderWorld::new(bv_margin);
        let contact_model = Box::new(SignoriniCoulombPyramidModel::new());
        let solver = MoreauJeanSolver::new(contact_model);
        let activation_manager = ActivationManager::new(na::convert(0.01f64));
        let gravity = Vector::zeros();
        let params = IntegrationParameters::default();

        World {
            counters,
            bodies,
            active_bodies,
            colliders_w_parent,
            cworld,
            solver,
            activation_manager,
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
        self.cworld.remove(handles);
        self.colliders_w_parent
            .retain(|handle| !handles.contains(handle));
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
        self.counters.update_started();

        // FIXME: objects involved in a non-linear position stabilization already
        // updated their kinematics.
        self.bodies.bodies_mut().for_each(|b| {
            b.clear_dynamics();
            b.update_kinematics();
        });

        let params = &self.params;
        let bodies = &mut self.bodies;
        self.forces.retain(|_, f| {
            f.apply(params, bodies)
        });

        let gravity = &self.gravity;
        let params = &self.params;
        self.bodies
            .bodies_mut().for_each(|b| {
            b.update_dynamics(gravity, params);
        });


        self.cworld.sync_colliders(&self.bodies);

        self.counters.update_completed();


        self.counters.collision_detection_started();
        self.cworld.clear_events();
        self.counters.broad_phase_started();
        self.cworld.perform_broad_phase();
        self.counters.broad_phase_completed();
        self.counters.narrow_phase_started();
        self.cworld.perform_narrow_phase();
        self.counters.narrow_phase_completed();
        self.counters.collision_detection_completed();

        if self.counters.enabled() {
            let count = self
                .cworld
                .contact_pairs()
                .fold((0, 0), |n, cp| (n.0 + 1, n.1 + cp.3.len()));
            self.counters.set_ncontact_pairs(count.0);
            self.counters.set_ncontacts(count.1);
        }

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

        let mut contact_manifolds = Vec::new(); // FIXME: avoid allocations.
        for (coll1, coll2, _, manifold) in self.cworld.contact_pairs() {
            // assert!(coll1.body_part() != coll2.body());

            let b1 = try_continue!(self.bodies.body(coll1.body()));
            let b2 = try_continue!(self.bodies.body(coll2.body()));

            if b1.status() != BodyStatus::Disabled && b2.status() != BodyStatus::Disabled
                && ((b1.status_dependent_ndofs() != 0 && b1.is_active())
                || (b2.status_dependent_ndofs() != 0 && b2.is_active()))
                {
                    contact_manifolds.push(ColliderContactManifold::new(coll1, coll2, manifold));
                }
        }

        self.counters.solver_started();
        // FIXME This is currently needed by the solver because otherwise
        // some kinematic bodies may end up with a companion_id (used as
        // an assembly_id) that it out of bounds of the velocity vector.
        // Note sure what the best place for this is though.
        for b in self.bodies.bodies_mut() {
            b.set_companion_id(0)
        }

        self.solver.step(
            &mut self.counters,
            &mut self.bodies,
            &mut self.constraints,
            &contact_manifolds[..],
            &self.active_bodies[..],
            &self.params,
            &self.cworld,
        );

        // FIXME: not sure what is the most pretty/efficient way of doing this.
        for mb in self.bodies.bodies_mut() {
            if mb.status() == BodyStatus::Kinematic {
                mb.integrate(&self.params)
            }
        }

        self.counters.solver_completed();
        self.counters.step_completed();
    }

    /// Remove the specified bodies.
    pub fn remove_bodies(&mut self, handles: &[BodyHandle]) {
        for handle in handles {
            self.bodies.remove_body(*handle);
        }

        self.cleanup_after_body_removal();
    }

    fn cleanup_after_body_removal(&mut self) {
        self.activate_bodies_touching_deleted_bodies();
        self.cleanup_colliders_with_deleted_parents();
        self.cleanup_constraints_with_deleted_anchors();
    }

    fn activate_bodies_touching_deleted_bodies(&mut self) {
        let bodies = &mut self.bodies;

        for (co1, co2, _, manifold) in self.cworld.contact_pairs() {
            if manifold.len() != 0 {
                let b1_exists = bodies.body(co1.body()).is_some();
                let b2_exists = bodies.body(co2.body()).is_some();

                if !b1_exists {
                    if b2_exists {
                        Self::activate_body_at(bodies, co2.body());
                    }
                } else if !b2_exists {
                    Self::activate_body_at(bodies, co1.body());
                }
            }
        }
    }

    fn cleanup_colliders_with_deleted_parents(&mut self) {
        let mut i = 0;

        while i < self.colliders_w_parent.len() {
            let cid = self.colliders_w_parent[i];
            let do_remove;

            match self
                .collider(cid)
                .expect("Internal error: collider not present")

                .anchor() {
                ColliderAnchor::OnBodyPart { body_part, .. } => {
                    do_remove = self.bodies.body(body_part.0).and_then(|b| b.part(body_part.1)).is_none()
                }
                ColliderAnchor::OnDeformableBody { body, .. } => {
                    do_remove = self.bodies.body(*body).is_none()
                }
            };

            if do_remove {
                self.cworld.remove(&[cid]);
                let _ = self.colliders_w_parent.swap_remove(i);
            } else {
                i += 1;
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

    /*
    /// Add a rigid body to the world.
    pub fn add_rigid_body(&mut self, rb: RigidBody<N>) -> &mut RigidBody<N> {
        self.bodies.add_rigid_body(rb)
    }
    */

//    /// Add a deformable collider to the world and retrieve its handle.
//    pub fn add_deformable_collider<S: Shape<N> + DeformableShape<N> + Clone>(
//        &mut self,
//        margin: N,
//        shape: S,
//        parent: BodyHandle,
//        dof_map: Option<Arc<Vec<usize>>>,
//        parts_map: Option<Arc<Vec<usize>>>,
//        material: Material<N>,
//    ) -> ColliderHandle {
//        let query = GeometricQueryType::Contacts(
//            margin + self.prediction * na::convert(0.5f64),
//            self.angular_prediction,
//        );
//        self.add_deformable_collision_object(query, margin, shape, parent, dof_map, parts_map, material)
//    }

//    /// Add a collider to the world and retrieve its handle.
//    pub fn add_collider(
//        &mut self,
//        margin: N,
//        shape: ShapeHandle<N>,
//        parent: BodyPartHandle,
//        to_parent: Isometry<N>,
//        material: Material<N>,
//    ) -> ColliderHandle {
//        let query = GeometricQueryType::Contacts(
//            margin + self.prediction * na::convert(0.5f64),
//            self.angular_prediction,
//        );
//        self.add_collision_object(query, margin, shape, parent, to_parent, material)
//    }

//    /// Add a sensor to the world and retrieve its handle.
//    pub fn add_sensor(
//        &mut self,
//        shape: ShapeHandle<N>,
//        parent: BodyPartHandle,
//        to_parent: Isometry<N>,
//    ) -> SensorHandle {
//        let query = GeometricQueryType::Proximity(self.prediction * na::convert(0.5f64));
//        self.add_collision_object(
//            query,
//            N::zero(),
//            shape,
//            parent,
//            to_parent,
//            Material::default(),
//        )
//    }
//
//    fn add_collision_object(
//        &mut self,
//        query: GeometricQueryType<N>,
//        margin: N,
//        shape: ShapeHandle<N>,
//        parent: BodyPartHandle,
//        to_parent: Isometry<N>,
//        material: Material<N>,
//    ) -> CollisionObjectHandle {
//        let (pos, ndofs) = if parent.is_ground() {
//            (to_parent, 0)
//        } else {
//            let parent_body = self.bodies.body(parent.0);
//            let parent_part = parent_body.part(parent.1);
//            (
//                parent_part.position() * to_parent,
//                parent_body.status_dependent_ndofs(),
//            )
//        };
//
//        let anchor = ColliderAnchor::OnBodyPart { body_part: parent, position_wrt_body_part: to_parent };
//        let data = ColliderData::new(margin, anchor, ndofs, material);
//        let groups = CollisionGroups::new();
//        let co = self.cworld.add(pos, shape, groups, query, data);
//
//        if !parent.is_ground() {
//            self.colliders_w_parent.push(co.handle());
//        }
//
//        co.handle()
//    }

//    fn add_deformable_collision_object<S: Shape<N> + DeformableShape<N> + Clone>(
//        &mut self,
//        query: GeometricQueryType<N>,
//        margin: N,
//        shape: S,
//        parent: BodyHandle,
//        dof_map: Option<Arc<Vec<usize>>>,
//        parts_map: Option<Arc<Vec<usize>>>,
//        material: Material<N>,
//    ) -> CollisionObjectHandle {
//        let parent_body = self.bodies.body(parent);
//        let parent_deformation_type = parent_body
//            .deformed_positions()
//            .expect("A deformable collider must be attached to a deformable body.")
//            .0;
//
//        assert_eq!(
//            parent_deformation_type,
//            shape.deformations_type(),
//            "Both the deformable shape and deformable body must support the same deformation types."
//        );
//
//        let anchor = ColliderAnchor::OnDeformableBody { body: parent, indices: dof_map, body_parts: parts_map };
//        let ndofs = parent_body.status_dependent_ndofs();
//        let data = ColliderData::new(margin, anchor, ndofs, material);
//        let groups = CollisionGroups::new();
//        let co = self.cworld.add(Isometry::identity(), ShapeHandle::new(shape), groups, query, data);
//
//        self.colliders_w_parent.push(co.handle());
//
//        co.handle()
//    }

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
        self.bodies.body(handle)?.downcast_ref::<Multibody<N>>().ok()
    }

    /// Get a mutable reference to the multibody containing the specified multibody link.
    ///
    /// Returns `None` if the handle does not correspond to a multibody link in this world.
    pub fn multibody_mut(&mut self, handle: BodyHandle) -> Option<&mut Multibody<N>> {
        self.bodies.body_mut(handle)?.downcast_mut::<Multibody<N>>().ok()
    }

    /// Get a reference to the specified rigid body.
    ///
    /// Returns `None` if the handle does not correspond to a rigid body in this world.
    pub fn rigid_body(&self, handle: BodyHandle) -> Option<&RigidBody<N>> {
        self.bodies.body(handle)?.downcast_ref::<RigidBody<N>>().ok()
    }

    /// Get a mutable reference to the specified rigid body.
    ///
    /// Returns `None` if the handle does not correspond to a rigid body in this world.
    pub fn rigid_body_mut(&mut self, handle: BodyHandle) -> Option<&mut RigidBody<N>> {
        self.bodies.body_mut(handle)?.downcast_mut::<RigidBody<N>>().ok()
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

    /// Get a mutable reference to the specified collider.
    ///
    /// Returns `None` if the handle does not correspond to a collider in this world.
    pub fn collider(&self, handle: ColliderHandle) -> Option<&Collider<N>> {
        self.cworld.collider(handle)
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

    /// An iterator through all the contact events generated during the last execution of `self.step()`.
    pub fn contact_events(&self) -> &ContactEvents {
        self.cworld.contact_events()
    }

    /// An iterator through all the proximity events generated during the last execution of `self.step()`.
    pub fn proximity_events(&self) -> &ProximityEvents {
        self.cworld.proximity_events()
    }
}

impl<N: Real> Default for World<N> {
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
