use std::f64;
use slab::Slab;

use na::{self, Real};
use ncollide;
use ncollide::world::{CollisionGroups, CollisionObjectHandle, GeometricQueryType};
use ncollide::events::{ContactEvents, ProximityEvents};
use ncollide::shape::ShapeHandle;

use counters::Counters;
use object::{Body, BodyHandle, BodyMut, BodyPart, BodySet, BodyStatus, Collider, ColliderData,
             ColliderHandle, Colliders, Multibody, MultibodyLinkMut, MultibodyLinkRef,
             MultibodyWorkspace, RigidBody, SensorHandle};
use joint::{ConstraintGenerator, ConstraintHandle, Joint};
use solver::{ContactModel, IntegrationParameters, MoreauJeanSolver, SignoriniCoulombPyramidModel};
use detection::{ActivationManager, BodyContactManifold};
use math::{Inertia, Isometry, Point, Vector};

pub type CollisionWorld<N> =
    ncollide::world::CollisionWorld<Point<N>, Isometry<N>, ColliderData<N>>;

pub struct World<N: Real> {
    counters: Counters,
    bodies: BodySet<N>,
    active_bodies: Vec<BodyHandle>,
    colliders_w_parent: Vec<ColliderHandle>, // The set of colliders that have a parent.
    cworld: CollisionWorld<N>,
    solver: MoreauJeanSolver<N>,
    activation_manager: ActivationManager<N>,
    // FIXME: set those two parameters per-collider?
    prediction: N,
    angular_prediction: N,
    gravity: Vector<N>,
    constraints: Slab<Box<ConstraintGenerator<N>>>,
    params: IntegrationParameters<N>,
    workspace: MultibodyWorkspace<N>,
}

impl<N: Real> World<N> {
    pub fn new() -> Self {
        let counters = Counters::new(false);
        let prediction = na::convert(0.02f64);
        let angular_prediction = na::convert(f64::consts::PI / 180.0 * 5.0);
        let bodies = BodySet::new();
        let active_bodies = Vec::new();
        let colliders_w_parent = Vec::new();
        let constraints = Slab::new();
        let cworld = CollisionWorld::new(prediction);
        let contact_model = Box::new(SignoriniCoulombPyramidModel::new());
        let solver = MoreauJeanSolver::new(contact_model);
        let activation_manager = ActivationManager::new(na::convert(0.01f64));
        let gravity = Vector::zeros();
        let params = IntegrationParameters::default();
        let workspace = MultibodyWorkspace::new();

        World {
            counters,
            bodies,
            active_bodies,
            colliders_w_parent,
            cworld,
            solver,
            activation_manager,
            prediction,
            angular_prediction,
            gravity,
            constraints,
            params,
            workspace,
        }
    }

    pub fn disable_performance_counters(&mut self) {
        self.counters.disable();
    }

    pub fn enable_performance_counters(&mut self) {
        self.counters.enable();
    }

    pub fn performance_counters(&self) -> &Counters {
        &self.counters
    }

    pub fn set_contact_model<C: ContactModel<N>>(&mut self, model: C) {
        self.solver.set_contact_model(Box::new(model))
    }

    pub fn integration_parameters(&self) -> &IntegrationParameters<N> {
        &self.params
    }

    pub fn set_erp(&mut self, erp: N) {
        self.params.erp = erp;
    }

    pub fn timestep(&self) -> N {
        self.params.dt
    }

    pub fn set_timestep(&mut self, dt: N) {
        self.params.dt = dt;
    }

    pub fn set_max_velocity_iterations(&mut self, niter: usize) {
        self.params.max_velocity_iterations = niter;
    }

    pub fn set_max_position_iterations(&mut self, niter: usize) {
        self.params.max_position_iterations = niter;
    }

    pub fn set_warmstart_factor(&mut self, value: N) {
        self.params.warmstart_coeff = value;
    }

    pub fn activate_body(&mut self, handle: BodyHandle) {
        Self::activate_body_at(&mut self.bodies, handle)
    }

    // NOTE: static method used to avoid borrowing issues.
    fn activate_body_at(bodies: &mut BodySet<N>, handle: BodyHandle) {
        let mut body = bodies.body_mut(handle);
        if body.status_dependent_ndofs() != 0 {
            body.activate();
        }
    }

    pub fn add_constraint<C: ConstraintGenerator<N>>(&mut self, constraint: C) -> ConstraintHandle {
        let (anchor1, anchor2) = constraint.anchors();
        self.activate_body(anchor1);
        self.activate_body(anchor2);
        self.constraints.insert(Box::new(constraint))
    }

    pub fn constraint(&self, handle: ConstraintHandle) -> &ConstraintGenerator<N> {
        &*self.constraints[handle]
    }

    pub fn constraint_mut(&mut self, handle: ConstraintHandle) -> &mut ConstraintGenerator<N> {
        let (anchor1, anchor2) = self.constraints[handle].anchors();
        self.activate_body(anchor1);
        self.activate_body(anchor2);
        &mut *self.constraints[handle]
    }

    pub fn remove_constraint(&mut self, handle: ConstraintHandle) -> Box<ConstraintGenerator<N>> {
        let constraint = self.constraints.remove(handle);
        let (anchor1, anchor2) = constraint.anchors();
        self.activate_body(anchor1);
        self.activate_body(anchor2);

        constraint
    }

    pub fn set_gravity(&mut self, gravity: Vector<N>) {
        self.gravity = gravity
    }

    pub fn step(&mut self) {
        self.counters.step_started();
        self.counters.update_started();
        // FIXME: objects involeved in a non-linear position stabilization elready
        // updated their kinematics.
        self.bodies.update_kinematics();
        self.bodies
            .update_dynamics(&self.gravity, &self.params, &mut self.workspace);
        self.counters.update_completed();

        self.counters.collision_detection_started();
        for collider_id in &self.colliders_w_parent {
            let new_pos;
            {
                // FIXME: update only if the position changed (especially for static bodies).
                let collider = self.cworld
                    .collision_object(*collider_id)
                    .expect("Internal error: collider not found.");
                let parent = self.bodies.body_part(collider.data().body());

                if !parent.is_active() {
                    continue;
                }

                let parent_pos = parent.position();
                new_pos = parent_pos * collider.data().position_wrt_parent()
            }

            self.cworld.set_position(*collider_id, new_pos);
        }

        self.cworld.clear_events();
        self.counters.broad_phase_started();
        self.cworld.perform_broad_phase();
        self.counters.broad_phase_completed();
        self.counters.narrow_phase_started();
        self.cworld.perform_narrow_phase();
        self.counters.narrow_phase_completed();
        self.counters.collision_detection_completed();

        if self.counters.enabled() {
            let npairs = self.cworld.contact_pairs().count();
            self.counters.set_ncontact_pairs(npairs);
        }

        // FIXME: for now, no island is built.
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
        for (coll1, coll2, c) in self.cworld.contact_manifolds() {
            // assert!(coll1.data().body() != coll2.data().body());

            let b1 = self.bodies.body(coll1.data().body());
            let b2 = self.bodies.body(coll2.data().body());

            if b1.status() != BodyStatus::Disabled && b2.status() != BodyStatus::Disabled
                && ((b1.status_dependent_ndofs() != 0 && b1.is_active())
                    || (b2.status_dependent_ndofs() != 0 && b2.is_active()))
            {
                contact_manifolds.push(BodyContactManifold::new(
                    coll1.data().body(),
                    coll2.data().body(),
                    coll1.data().margin(),
                    coll2.data().margin(),
                    c,
                ));
            }
        }

        self.counters.solver_started();
        self.solver.step(
            &mut self.counters,
            &mut self.bodies,
            &self.constraints,
            &contact_manifolds[..],
            &self.active_bodies[..],
            &self.params,
        );

        // FIXME: not sure what is the most pretty/efficient way of doing this.
        for rb in self.bodies.rigid_bodies_mut() {
            if rb.status() == BodyStatus::Kinematic {
                rb.integrate(&self.params)
            }
        }
        for mb in self.bodies.multibodies_mut() {
            if mb.status() == BodyStatus::Kinematic {
                mb.integrate(&self.params)
            }
        }
        self.counters.solver_completed();
        self.counters.step_completed();
    }

    /// Remove the specified bodies.
    pub fn remove_bodies(&mut self, bodies: &[BodyHandle]) {
        for body in bodies {
            self.bodies.remove_body(*body);
        }

        self.cleanup_after_body_removal();
    }

    /// Remove several links of a single multibody.
    ///
    /// Panics if not all links belong to the same multibody.
    pub fn remove_multibody_links(&mut self, links: &[BodyHandle]) {
        self.bodies.remove_multibody_links(links);
        self.cleanup_after_body_removal();
    }

    fn cleanup_after_body_removal(&mut self) {
        self.activate_bodies_touching_deleted_bodies();
        self.cleanup_colliders_with_deleted_parents();
        self.cleanup_constraints_with_deleted_anchors();
    }

    fn activate_bodies_touching_deleted_bodies(&mut self) {
        let bodies = &mut self.bodies;

        for (co1, co2, detector) in self.cworld.contact_pairs() {
            if detector.num_contacts() != 0 {
                let b1_exists = bodies.contains(co1.data().body());
                let b2_exists = bodies.contains(co2.data().body());

                if !b1_exists {
                    if b2_exists {
                        Self::activate_body_at(bodies, co2.data().body());
                    }
                } else if !b2_exists {
                    Self::activate_body_at(bodies, co1.data().body());
                }
            }
        }
    }

    fn cleanup_colliders_with_deleted_parents(&mut self) {
        let mut i = 0;

        while i < self.colliders_w_parent.len() {
            let cid = self.colliders_w_parent[i];
            let parent = self.collider(cid)
                .expect("Internal error: collider not present")
                .data()
                .body();

            if !self.bodies.contains(parent) {
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
            let b1_exists = bodies.contains(b1);
            let b2_exists = bodies.contains(b2);

            if !b1_exists {
                if b2_exists {
                    Self::activate_body_at(bodies, b2);
                }
            } else if !b2_exists {
                Self::activate_body_at(bodies, b1);
            }

            b1_exists && b2_exists
        })
    }

    pub fn add_rigid_body(
        &mut self,
        position: Isometry<N>,
        local_inertia: Inertia<N>,
    ) -> BodyHandle {
        self.bodies.add_rigid_body(position, local_inertia)
    }

    pub fn add_multibody_link<J: Joint<N>>(
        &mut self,
        parent: BodyHandle,
        joint: J,
        parent_shift: Vector<N>,
        body_shift: Vector<N>,
        inertia: Inertia<N>,
    ) -> BodyHandle {
        self.bodies
            .add_multibody_link(parent, joint, parent_shift, body_shift, inertia)
    }

    pub fn add_collider(
        &mut self,
        margin: N,
        shape: ShapeHandle<Point<N>, Isometry<N>>,
        parent: BodyHandle,
        to_parent: Isometry<N>,
    ) -> ColliderHandle {
        let query = GeometricQueryType::Contacts(
            margin + self.prediction * na::convert(0.5f64),
            self.angular_prediction,
        );
        self.add_collision_object(query, margin, shape, parent, to_parent)
    }

    pub fn add_sensor(
        &mut self,
        shape: ShapeHandle<Point<N>, Isometry<N>>,
        parent: BodyHandle,
        to_parent: Isometry<N>,
    ) -> SensorHandle {
        let query = GeometricQueryType::Proximity(self.prediction * na::convert(0.5f64));
        self.add_collision_object(query, N::zero(), shape, parent, to_parent)
    }

    fn add_collision_object(
        &mut self,
        query: GeometricQueryType<N>,
        margin: N,
        shape: ShapeHandle<Point<N>, Isometry<N>>,
        parent: BodyHandle,
        to_parent: Isometry<N>,
    ) -> CollisionObjectHandle {
        let pos = if parent.is_ground() {
            to_parent
        } else {
            self.bodies.body_part(parent).position() * to_parent
        };

        let data = ColliderData::new(margin, parent, to_parent);
        let groups = CollisionGroups::new();
        let handle = self.cworld.add(pos, shape, groups, query, data);

        if !parent.is_ground() {
            self.colliders_w_parent.push(handle);
        }

        handle
    }

    pub fn body_part(&self, handle: BodyHandle) -> BodyPart<N> {
        self.bodies.body_part(handle)
    }

    pub fn body(&self, handle: BodyHandle) -> Body<N> {
        self.bodies.body(handle)
    }

    pub fn body_mut(&mut self, handle: BodyHandle) -> BodyMut<N> {
        self.bodies.body_mut(handle)
    }

    pub fn multibody(&self, handle: BodyHandle) -> Option<&Multibody<N>> {
        self.bodies.multibody(handle)
    }

    pub fn multibody_mut(&mut self, handle: BodyHandle) -> Option<&mut Multibody<N>> {
        self.bodies.multibody_mut(handle)
    }

    pub fn multibody_link(&self, handle: BodyHandle) -> Option<MultibodyLinkRef<N>> {
        self.bodies.multibody_link(handle)
    }

    pub fn multibody_link_mut(&mut self, handle: BodyHandle) -> Option<MultibodyLinkMut<N>> {
        self.bodies.multibody_link_mut(handle)
    }

    pub fn rigid_body(&self, handle: BodyHandle) -> Option<&RigidBody<N>> {
        self.bodies.rigid_body(handle)
    }

    pub fn rigid_body_mut(&mut self, handle: BodyHandle) -> Option<&mut RigidBody<N>> {
        self.bodies.rigid_body_mut(handle)
    }

    pub fn collision_world(&self) -> &CollisionWorld<N> {
        &self.cworld
    }

    pub fn collider(&self, handle: ColliderHandle) -> Option<&Collider<N>> {
        self.cworld.collision_object(handle)
    }

    pub fn colliders(&self) -> Colliders<N> {
        self.cworld.collision_objects()
    }

    pub fn contact_events(&self) -> &ContactEvents {
        self.cworld.contact_events()
    }

    pub fn proximity_events(&self) -> &ProximityEvents {
        self.cworld.proximity_events()
    }
}
