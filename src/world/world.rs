use std::slice::Iter;
use std::iter::Map;
use std::rc::Rc;
use std::cell::RefCell;

use alga::general::Real;
use na;
use ncollide::bounding_volume::AABB;
use ncollide::utils::data::hash_map::{HashMap, Entry};
use ncollide::utils::data::hash::UintTWHash;
use ncollide::broad_phase::{DBVTBroadPhase, BroadPhasePairFilter};
use ncollide::narrow_phase::{ContactHandler, ProximityHandler, DefaultNarrowPhase,
                             DefaultContactDispatcher, DefaultProximityDispatcher,
                             ContactAlgorithm};
use ncollide::world::{CollisionWorld, CollisionObject, GeometricQueryType};
use integration::{Integrator, BodySmpEulerIntegrator, BodyForceGenerator,
                  TranslationalCCDMotionClamping};
use detection::ActivationManager;
use detection::constraint::Constraint;
use detection::joint::{JointManager, BallInSocket, Fixed};
use resolution::{Solver, AccumulatedImpulseSolver, CorrectionMode};
use object::{WorldObject, RigidBody, RigidBodyHandle, Sensor, SensorHandle};
use math::{Point, Vector, Isometry};

/// The default broad phase.
pub type WorldBroadPhase<N> = DBVTBroadPhase<Point<N>, WorldObject<N>, AABB<Point<N>>>;

/// An iterator visiting rigid bodies.
pub type RigidBodies<'a, N> = Map<Iter<'a, Entry<usize, RigidBodyHandle<N>>>, fn(&'a Entry<usize, RigidBodyHandle<N>>) -> &'a RigidBodyHandle<N>>;

/// An iterator visiting sensors.
pub type Sensors<'a, N> = Map<Iter<'a, Entry<usize, SensorHandle<N>>>, fn(&'a Entry<usize, SensorHandle<N>>) -> &'a SensorHandle<N>>;

/// Type of the collision world containing rigid bodies.
pub type RigidBodyCollisionWorld<N> = CollisionWorld<Point<N>, Isometry<N>, WorldObject<N>>;

/// Type of a collision object containing `WorldObject` body as its data.
pub type WorldCollisionObject<N> = CollisionObject<Point<N>, Isometry<N>, WorldObject<N>>;


/// The physical world.
///
/// This is the main structure of the physics engine.
pub struct World<N: Real> {
    cworld:       RigidBodyCollisionWorld<N>,
    rigid_bodies: HashMap<usize, RigidBodyHandle<N>, UintTWHash>,
    sensors:      HashMap<usize, SensorHandle<N>, UintTWHash>,
    forces:       BodyForceGenerator<N>,
    integrator:   BodySmpEulerIntegrator,
    sleep:        Rc<RefCell<ActivationManager<N>>>, // FIXME: avoid sharing (needed for the contact signal handler)
    ccd:          TranslationalCCDMotionClamping<N>,
    joints:       JointManager<N>,
    solver:       AccumulatedImpulseSolver<N>,
    prediction:   N
}

impl<N: Real> World<N> {
    /// Creates a new physics world.
    pub fn new() -> World<N> {
        /*
         * Setup the physics world
         */

        let prediction = na::convert(0.02f64); // FIXME: do not hard-code the prediction margin.

        // For the intergration
        let forces     = BodyForceGenerator::new(na::zero(), na::zero());
        let integrator = BodySmpEulerIntegrator::new();

        /*
         * For the collision detection
         */
        // Collision world
        let mut cworld = CollisionWorld::new(prediction, false);

        // Custom narrow phase.
        let disp = DefaultContactDispatcher::new();
        let prox = DefaultProximityDispatcher::new();
        let nf   = DefaultNarrowPhase::new(Box::new(disp), Box::new(prox));
        let _    = cworld.set_narrow_phase(Box::new(nf));

        // CCD handler
        let ccd = TranslationalCCDMotionClamping::new();

        // Deactivation
        let sleep = Rc::new(RefCell::new(ActivationManager::new(na::convert(0.01f64))));

        // Setup contact handler to reactivate sleeping objects that loose contact.
        let handler = ObjectActivationOnContactHandler { sleep: sleep.clone() };
        cworld.register_contact_handler("__nphysics_internal_ObjectActivationOnContactHandler", handler);

        // Setup the broad phase pair filter that will prevent sensors from colliding with their
        // parent.
        let filter      = SensorsNotCollidingTheirParentPairFilter;
        let filter_name = "__nphysics_internal_SensorsNotCollidingTheirParentPairFilter";
        cworld.register_broad_phase_pair_filter(filter_name, filter);

        // Joints
        let joints = JointManager::new();

        /*
         * For constraints resolution
         */
        let solver = AccumulatedImpulseSolver::new(
            na::convert(0.1f64),
            CorrectionMode::VelocityAndPosition(na::convert(0.2f64), na::convert(0.2f64), na::convert(0.08f64)),
            na::convert(0.4f64),
            na::convert(1.0f64),
            10,
            10);

        World {
            cworld:       cworld,
            rigid_bodies: HashMap::new(UintTWHash::new()),
            sensors:      HashMap::new(UintTWHash::new()),
            forces:       forces,
            integrator:   integrator,
            sleep:        sleep,
            ccd:          ccd,
            joints:       joints,
            solver:       solver,
            prediction:   prediction
        }
    }

    /// Updates the physics world.
    pub fn step(&mut self, dt: N) {
        for e in self.rigid_bodies.elements_mut().iter_mut() {
            let mut rb = e.value.borrow_mut();

            if rb.is_active() {
                self.forces.update(dt.clone(), &mut *rb);
                self.integrator.update(dt.clone(), &mut *rb);
                self.cworld.deferred_set_position(WorldObject::rigid_body_uid(&e.value), rb.position().clone());
            }
        }

        for e in self.sensors.elements_mut().iter_mut() {
            let sensor = e.value.borrow_mut();

            if let Some(rb) = sensor.parent() {
                if rb.borrow().is_active() {
                    self.cworld.deferred_set_position(WorldObject::sensor_uid(&e.value), sensor.position());
                }
            }
        }

        self.cworld.perform_position_update();
        self.cworld.perform_broad_phase();
        if !self.ccd.update(&mut self.cworld) {
            self.cworld.perform_narrow_phase();
        }

        self.joints.update(&mut *self.sleep.borrow_mut());
        self.sleep.borrow_mut().update(&mut self.cworld, &self.joints, &self.rigid_bodies);

        // XXX: use `self.collector` instead to avoid allocation.
        let mut collector = Vec::new();

        for (b1, b2, c) in self.cworld.contacts() {
            if let (&WorldObject::RigidBody(ref rb1), &WorldObject::RigidBody(ref rb2)) = (&b1.data, &b2.data) {
                if rb1.borrow().is_active() || rb2.borrow().is_active() {
                    let m1 = rb1.borrow().margin();
                    let m2 = rb2.borrow().margin();

                    let mut c = c.clone();
                    c.depth = c.depth + m1 + m2;

                    collector.push(Constraint::RBRB(rb1.clone(), rb2.clone(), c));
                }
            }
        }

        self.joints.constraints(&mut collector);

        self.solver.solve(dt, &collector[..]);

        collector.clear();
    }

    /// Adds a rigid body to the physics world.
    pub fn add_rigid_body(&mut self, rb: RigidBody<N>) -> RigidBodyHandle<N> {
        let position = rb.position().clone();
        let shape = rb.shape().clone();
        let groups = rb.collision_groups().as_collision_groups().clone();
        let collision_object_prediction = rb.margin() + self.prediction / na::convert(2.0f64);
        let handle = Rc::new(RefCell::new(rb));
        let uid = WorldObject::rigid_body_uid(&handle);

        self.rigid_bodies.insert(uid, handle.clone());
        self.cworld.deferred_add(uid, position, shape, groups,
                                 GeometricQueryType::Contacts(collision_object_prediction),
                                 WorldObject::RigidBody(handle.clone()));
        self.cworld.perform_additions_removals_and_broad_phase();

        handle
    }

    /// Adds a sensor to the physics world.
    pub fn add_sensor(&mut self, sensor: Sensor<N>) -> SensorHandle<N> {
        let position = sensor.position().clone();
        let shape    = sensor.shape().clone();
        let groups   = sensor.collision_groups().as_collision_groups().clone();
        let margin   = sensor.margin();
        let handle   = Rc::new(RefCell::new(sensor));
        let uid      = &*handle as *const RefCell<Sensor<N>> as usize;

        self.sensors.insert(uid, handle.clone());
        self.cworld.deferred_add(uid, position, shape, groups,
                                 GeometricQueryType::Proximity(margin),
                                 WorldObject::Sensor(handle.clone()));
        self.cworld.perform_additions_removals_and_broad_phase();

        handle
    }

    /// Remove a rigid body from the physics world.
    pub fn remove_rigid_body(&mut self, rb: &RigidBodyHandle<N>) {
        let uid = WorldObject::rigid_body_uid(rb);
        self.cworld.deferred_remove(uid);
        self.cworld.perform_additions_removals_and_broad_phase();
        self.joints.remove(rb, &mut *self.sleep.borrow_mut());
        self.ccd.remove_ccd_from(rb);
        self.rigid_bodies.remove(&uid);
        rb.borrow_mut().delete();
    }

    /// Remove a sensor from the physics world.
    pub fn remove_sensor(&mut self, sensor: &SensorHandle<N>) {
        let uid = WorldObject::sensor_uid(sensor);
        self.cworld.deferred_remove(uid);
        self.cworld.perform_additions_removals_and_broad_phase();
        self.sensors.remove(&uid);
    }

    // XXX: keep this reference mutable?
    /// Gets a mutable reference to the force generator.
    pub fn forces_generator(&mut self) -> &mut BodyForceGenerator<N> {
        &mut self.forces
    }

    // XXX: keep this reference mutable?
    /// Gets a mutable reference to the position and orientation integrator.
    pub fn integrator(&mut self) -> &mut BodySmpEulerIntegrator {
        &mut self.integrator
    }

    // XXX: keep this reference mutable?
    /// Gets a mutable reference to the CCD manager.
    pub fn ccd_manager(&mut self) -> &mut TranslationalCCDMotionClamping<N> {
        &mut self.ccd
    }

    // XXX: keep this reference mutable?
    /// Gets a mutable reference to the joint manager.
    pub fn joint_manager(&mut self) -> &mut JointManager<N> {
        &mut self.joints
    }

    // XXX: keep this reference mutable?
    /// Gets a mutable reference to the constraint solver.
    pub fn constraints_solver(&mut self) -> &mut AccumulatedImpulseSolver<N> {
        &mut self.solver
    }

    /// Gets the underlying collision world.
    pub fn collision_world(&self) -> &RigidBodyCollisionWorld<N> {
        &self.cworld
    }

    /// Sets the linear acceleration afecting every dynamic rigid body.
    pub fn set_gravity(&mut self, gravity: Vector<N>) {
        self.forces.set_lin_acc(gravity);
    }

    /*/// Sets the angular acceleration afecting every dynamic rigid body.
    pub fn set_angular_acceleration(&mut self, accel: Orientation) {
        self.forces.set_ang_acc(accel)
    }*/

    /// Gets the linear acceleration afecting every dynamic rigid body.
    pub fn gravity(&self) -> Vector<N> {
        self.forces.lin_acc()
    }

    /*/// Gets the angular acceleration afecting every dynamic rigid body.
    pub fn angular_acceleration(&self) -> Orientation {
        self.forces.ang_acc()
    }*/

    /// Adds continuous collision detection to the given rigid body.
    ///
    /// Set `trigger_sensor` to `true` if the rigid body should active the sensors that would have
    /// been missed without CCD enabled.
    pub fn add_ccd_to(&mut self, body: &RigidBodyHandle<N>, motion_thresold: N, trigger_sensors: bool) {
        self.ccd.add_ccd_to(body.clone(), motion_thresold, trigger_sensors)
    }

    /// Adds a ball-in-socket joint to the world.
    pub fn add_ball_in_socket(&mut self, joint: BallInSocket<N>) -> Rc<RefCell<BallInSocket<N>>> {
        let res = Rc::new(RefCell::new(joint));

        self.joints.add_ball_in_socket(res.clone(), &mut *self.sleep.borrow_mut());

        res
    }

    /// Removes a ball-in-socket joint from the world.
    pub fn remove_ball_in_socket(&mut self, joint: &Rc<RefCell<BallInSocket<N>>>) {
        self.joints.remove_ball_in_socket(joint, &mut *self.sleep.borrow_mut())
    }

    /// Adds a fixed joint to the world.
    pub fn add_fixed(&mut self, joint: Fixed<N>) -> Rc<RefCell<Fixed<N>>> {
        let res = Rc::new(RefCell::new(joint));

        self.joints.add_fixed(res.clone(), &mut *self.sleep.borrow_mut());

        res
    }

    /// Removes a fixed joint from the world.
    pub fn remove_fixed(&mut self, joint: &Rc<RefCell<Fixed<N>>>) {
        self.joints.remove_joint(joint, &mut *self.sleep.borrow_mut())
    }

    /// Collects every constraincts detected since the last update.
    pub fn constraints(&mut self, out: &mut Vec<Constraint<N>>) {
        // FIXME: ugly.
        for (b1, b2, c) in self.cworld.contacts() {
            if let (&WorldObject::RigidBody(ref rb1), &WorldObject::RigidBody(ref rb2)) = (&b1.data, &b2.data) {
                let m1 = rb1.borrow().margin();
                let m2 = rb2.borrow().margin();

                let mut c = c.clone();
                c.depth = c.depth + m1 + m2;

                out.push(Constraint::RBRB(rb1.clone(), rb2.clone(), c));
            }
        }

        self.joints.constraints(out);
    }

    /// An iterator visiting all rigid bodies on this world.
    pub fn rigid_bodies(&self) -> RigidBodies<N> {
        fn extract_value<N: Real>(e: &Entry<usize, RigidBodyHandle<N>>) -> &RigidBodyHandle<N> {
            &e.value
        }

        let extract_value_fn: fn(_) -> _ = extract_value;
        self.rigid_bodies.elements().iter().map(extract_value_fn)
    }

    /// An iterator visiting all sensors on this world.
    pub fn sensors(&self) -> Sensors<N> {
        fn extract_value<N: Real>(e: &Entry<usize, SensorHandle<N>>) -> &SensorHandle<N> {
            &e.value
        }

        let extract_value_fn: fn(_) -> _ = extract_value;
        self.sensors.elements().iter().map(extract_value_fn)
    }

    /// Adds a filter that tells if a potential collision pair should be ignored or not.
    ///
    /// The proximity filter returns `false` for a given pair of collision objects if they should
    /// be ignored by the narrow phase. Keep in mind that modifying the proximity filter will have
    /// a non-trivial overhead during the next update as it will force re-detection of all
    /// collision pairs.
    pub fn register_broad_phase_pair_filter<F>(&mut self, name: &str, filter: F)
        where F: BroadPhasePairFilter<Point<N>, Isometry<N>, WorldObject<N>> + 'static {
        self.cworld.register_broad_phase_pair_filter(name, filter)
    }

    /// Removes the pair filter named `name`.
    pub fn unregister_broad_phase_pair_filter(&mut self, name: &str) {
        self.cworld.unregister_broad_phase_pair_filter(name)
    }

    /// Registers a handler for contact start/stop events.
    pub fn register_contact_handler<H>(&mut self, name: &str, handler: H)
        where H: ContactHandler<Point<N>, Isometry<N>, WorldObject<N>> + 'static {
        self.cworld.register_contact_handler(name, handler)
    }

    /// Unregisters a handler for contact start/stop events.
    pub fn unregister_contact_handler(&mut self, name: &str) {
        self.cworld.unregister_contact_handler(name)
    }

    /// Registers a handler for proximity status change events.
    pub fn register_proximity_handler<H>(&mut self, name: &str, handler: H)
        where H: ProximityHandler<Point<N>, Isometry<N>, WorldObject<N>> + 'static {
        self.cworld.register_proximity_handler(name, handler);
    }

    /// Unregisters a handler for proximity status change events.
    pub fn unregister_proximity_handler(&mut self, name: &str) {
        self.cworld.unregister_proximity_handler(name);
    }
}

struct ObjectActivationOnContactHandler<N: Real> {
    sleep: Rc<RefCell<ActivationManager<N>>>
}

impl<N: Real> ContactHandler<Point<N>, Isometry<N>, WorldObject<N>>
for ObjectActivationOnContactHandler<N> {
    #[inline]
    fn handle_contact_started(&mut self,
                              _: &WorldCollisionObject<N>,
                              _: &WorldCollisionObject<N>,
                              _: &ContactAlgorithm<Point<N>, Isometry<N>>) {
        // Do nothing.
    }

    fn handle_contact_stopped(&mut self, obj1: &WorldCollisionObject<N>, obj2: &WorldCollisionObject<N>) {
        // Wake up on collision lost.

        // There is no need to wake up anything if a rigid-body intersects a sensor.
        if let (&WorldObject::RigidBody(ref rb1), &WorldObject::RigidBody(ref rb2)) = (&obj1.data, &obj2.data) {
            self.sleep.borrow_mut().deferred_activate(rb1);
            self.sleep.borrow_mut().deferred_activate(rb2);
        }
    }
}

struct SensorsNotCollidingTheirParentPairFilter;

impl<N: Real> BroadPhasePairFilter<Point<N>, Isometry<N>, WorldObject<N>>
for SensorsNotCollidingTheirParentPairFilter {
    #[inline]
    fn is_pair_valid(&self, b1: &WorldCollisionObject<N>, b2: &WorldCollisionObject<N>) -> bool {
        match (&b1.data, &b2.data) {
            (&WorldObject::RigidBody(ref rb), &WorldObject::Sensor(ref s)) |
            (&WorldObject::Sensor(ref s), &WorldObject::RigidBody(ref rb)) => {
                let bs = s.borrow();

                if let Some(parent) = bs.parent() {
                    WorldObject::rigid_body_uid(rb) != WorldObject::rigid_body_uid(parent) ||
                    bs.proximity_with_parent_enabled()
                }
                else {
                    true
                }
            },
            _ => true
        }
    }
}
