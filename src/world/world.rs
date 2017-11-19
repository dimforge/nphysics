use std::slice::Iter;
use std::iter::Map;

use alga::general::Real;
use na;
use ncollide::bounding_volume::AABB;
use ncollide::utils::data::hash_map::Entry;
use ncollide::broad_phase::{DBVTBroadPhase, BroadPhasePairFilter};
use ncollide::narrow_phase::{DefaultNarrowPhase, DefaultContactDispatcher,
                             DefaultProximityDispatcher};
use ncollide::world::{CollisionWorld, CollisionObject, GeometricQueryType};
use ncollide::ncollide_pipeline::events::ContactEvent;
use integration::{Integrator, BodySmpEulerIntegrator, BodyForceGenerator,
                  TranslationalCCDMotionClamping};
use detection::ActivationManager;
use detection::constraint::Constraint;
use detection::joint::{JointManager, BallInSocket, Fixed};
use resolution::{Solver, AccumulatedImpulseSolver, CorrectionMode};
use object::{WorldObject, RigidBody, Sensor, sensor_handle_proximity};
use math::{Point, Vector, Isometry};
use utils::{IndexVec, IndexVecIter};

/// The default broad phase.
pub type WorldBroadPhase<N> = DBVTBroadPhase<Point<N>, WorldObject, AABB<Point<N>>>;

/// An iterator visiting rigid bodies.
pub type RigidBodies<'a> = Map<Iter<'a, Entry<usize, ()>>, fn(&'a Entry<usize, ()>) -> usize>;

/// An iterator visiting sensors.
pub type Sensors<'a> = Map<Iter<'a, Entry<usize, ()>>, fn(&'a Entry<usize, ()>) -> usize>;

type FilterData<N> = SensorStorage<N>;

/// Type of the collision world containing rigid bodies.
pub type RigidBodyCollisionWorld<N> = CollisionWorld<Point<N>, Isometry<N>, WorldObject, FilterData<N>>;

/// Type of a collision object containing `WorldObject` body as its data.
pub type WorldCollisionObject<N> = CollisionObject<Point<N>, Isometry<N>, WorldObject>;

pub type RigidBodyStorage<N> = IndexVec<RigidBody<N>>;
pub type SensorStorage<N> = IndexVec<Sensor<N>>;

/// The physical world.
///
/// This is the main structure of the physics engine.
pub struct World<N: Real> {
    cworld:              RigidBodyCollisionWorld<N>,
    rigid_bodies:        IndexVec<RigidBody<N>>,
    sensors:             IndexVec<Sensor<N>>,
    forces:              BodyForceGenerator<N>,
    integrator:          BodySmpEulerIntegrator,
    sleep:               ActivationManager<N>,
    ccd:                 TranslationalCCDMotionClamping<N>,
    joints:              JointManager<N>,
    solver:              AccumulatedImpulseSolver<N>,
    prediction:          N,
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
        let activation_manager = ActivationManager::new(na::convert(0.01f64));

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
            cworld:             cworld,
            rigid_bodies:       IndexVec::new(0),
            sensors:            IndexVec::new(1<<30),
            forces:             forces,
            integrator:         integrator,
            sleep:              activation_manager,
            ccd:                ccd,
            joints:             joints,
            solver:             solver,
            prediction:         prediction,
        }
    }

    /// Updates the physics world.
    pub fn step(&mut self, dt: N) {
        for rb in self.rigid_bodies.iter_mut() {
            if rb.is_active() {
                self.forces.update(dt.clone(), rb);
                self.integrator.update(dt.clone(), rb);
                self.cworld.deferred_set_position(rb.uid(), rb.position().clone());
            }
        }

        for sensor in self.sensors.iter_mut() {
            if sensor.did_move_locally {
                sensor.did_move_locally = false;
                self.cworld.deferred_set_position(sensor.uid(), sensor.position(&self.rigid_bodies));
            }
            else if let Some(&rb) = sensor.parent() {
                if self.rigid_bodies[rb].is_active() {
                    self.cworld.deferred_set_position(sensor.uid(), sensor.position(&self.rigid_bodies));
                }
            }
        }

        self.cworld.perform_position_update();
        self.cworld.perform_broad_phase(&self.sensors);
        if !self.ccd.update(&mut self.cworld, &mut self.rigid_bodies, &mut self.sensors) {
            self.cworld.perform_narrow_phase();
        }

        // reactivate sleeping objects that loose contact.
        for contact_event in self.cworld.contact_events() {
            if let &ContactEvent::Started(obj1, obj2) = contact_event {
                for &obj in [obj1, obj2].iter() {
                    if let Some(obj) = self.cworld.collision_object(obj) {
                        if let WorldObject::RigidBody(rb) = obj.data {
                            self.sleep.deferred_activate(&self.rigid_bodies[rb]);
                        }
                    }
                }
            }
        }

        for proximity_event in self.cworld.proximity_events() {
            let o1 = &self.cworld.collision_object(proximity_event.co1).unwrap().data;
            let o2 = &self.cworld.collision_object(proximity_event.co2).unwrap().data;
            sensor_handle_proximity(
                o1, o2,
                proximity_event.prev_status,
                proximity_event.new_status,
                &mut self.sensors
            )
        }

        self.joints.update(&mut self.sleep, &self.rigid_bodies);
        self.sleep.update(&mut self.cworld, &self.joints, &mut self.rigid_bodies);

        // XXX: use `self.collector` instead to avoid allocation.
        let mut collector = Vec::new();

        for (b1, b2, c) in self.cworld.contacts() {
            if let (&WorldObject::RigidBody(rb1_uid), &WorldObject::RigidBody(rb2_uid)) = (&b1.data, &b2.data) {
                let rb1 = &self.rigid_bodies[rb1_uid];
                let rb2 = &self.rigid_bodies[rb2_uid];
                if rb1.is_active() || rb2.is_active() {
                    let m1 = rb1.margin();
                    let m2 = rb2.margin();

                    let mut c = c.clone();
                    c.depth = c.depth + m1 + m2;

                    collector.push(Constraint::RBRB(rb1_uid, rb2_uid, c));
                }
            }
        }

        self.joints.constraints(&mut collector);

        self.solver.solve(dt, &collector[..], &mut self.rigid_bodies);

        collector.clear();
    }

    /// Adds a rigid body to the physics world.
    pub fn add_rigid_body(&mut self, rb: RigidBody<N>) -> usize {
        let position = rb.position().clone();
        let shape = rb.shape().clone();
        let groups = rb.collision_groups().as_collision_groups().clone();
        let collision_object_prediction = rb.margin() + self.prediction / na::convert(2.0f64);

        let uid = self.rigid_bodies.insert_and_set_index(rb);
        self.cworld.deferred_add(uid, position, shape, groups,
                                 GeometricQueryType::Contacts(collision_object_prediction),
                                 WorldObject::RigidBody(uid));
        self.cworld.perform_additions_removals_and_broad_phase(&self.sensors);
        uid
    }

    /// Adds a sensor to the physics world.
    pub fn add_sensor(&mut self, sensor: Sensor<N>) -> usize {
        let position = sensor.position(&self.rigid_bodies).clone();
        let shape    = sensor.shape().clone();
        let groups   = sensor.collision_groups().as_collision_groups().clone();
        let margin   = sensor.margin();

        let uid = self.sensors.insert_and_set_index(sensor);
        self.cworld.deferred_add(uid, position, shape, groups,
                                 GeometricQueryType::Proximity(margin),
                                 WorldObject::Sensor(uid));
        self.cworld.perform_additions_removals_and_broad_phase(&self.sensors);
        uid
    }

    /// Remove a rigid body from the physics world.
    pub fn remove_rigid_body(&mut self, uid: usize) {
        self.cworld.deferred_remove(uid);
        self.cworld.perform_additions_removals_and_broad_phase(&self.sensors);
        self.joints.remove(uid, &mut self.sleep, &self.rigid_bodies);
        self.ccd.remove_ccd_from(uid);
        let mut rb = self.rigid_bodies.remove(uid);
        // TODO: do it ? this doesn't have much sense now.
        rb.delete();
    }

    /// Remove a sensor from the physics world.
    pub fn remove_sensor(&mut self, uid: usize) {
        self.cworld.deferred_remove(uid);
        self.cworld.perform_additions_removals_and_broad_phase(&self.sensors);
        let _ = self.sensors.remove(uid);
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
    pub fn add_ccd_to(&mut self, body: usize, motion_thresold: N, trigger_sensors: bool) {
        unimplemented!();
        // self.ccd.add_ccd_to(body.clone(), motion_thresold, trigger_sensors)
    }

    /// Adds a ball-in-socket joint to the world.
    pub fn add_ball_in_socket(&mut self, joint: BallInSocket<N>) -> usize {
        self.joints.add_ball_in_socket(joint, &mut self.sleep, &self.rigid_bodies)
    }

    /// Removes a ball-in-socket joint from the world.
    pub fn remove_ball_in_socket(&mut self, joint: usize) {
        self.joints.remove_ball_in_socket(joint, &mut self.sleep, &self.rigid_bodies)
    }

    /// Adds a fixed joint to the world.
    pub fn add_fixed(&mut self, joint: Fixed<N>) -> usize {
        self.joints.add_fixed(joint, &mut self.sleep, &self.rigid_bodies)
    }

    /// Removes a fixed joint from the world.
    pub fn remove_fixed(&mut self, joint: usize) {
        // TODO: need stronger type for joint to differentiate f from bis
        self.joints.remove_joint::<Fixed<N>, _>(joint, &mut self.sleep, &self.rigid_bodies)
    }

    /// Collects every constraincts detected since the last update.
    pub fn constraints(&mut self, out: &mut Vec<Constraint<N>>) {
        // FIXME: ugly.
        for (b1, b2, c) in self.cworld.contacts() {
            if let (&WorldObject::RigidBody(rb1), &WorldObject::RigidBody(rb2)) = (&b1.data, &b2.data) {
                let m1 = self.rigid_bodies[rb1].margin();
                let m2 = self.rigid_bodies[rb2].margin();

                let mut c = c.clone();
                c.depth = c.depth + m1 + m2;

                out.push(Constraint::RBRB(rb1, rb2, c));
            }
        }

        self.joints.constraints(out);
    }

    /// An iterator visiting all rigid bodies on this world.
    pub fn rigid_bodies(&self) -> IndexVecIter<RigidBody<N>> {
        self.rigid_bodies.iter()
    }

    /// An iterator visiting all sensors on this world.
    pub fn sensors(&self) -> IndexVecIter<Sensor<N>> {
        self.sensors.iter()
    }

    /// Adds a filter that tells if a potential collision pair should be ignored or not.
    ///
    /// The proximity filter returns `false` for a given pair of collision objects if they should
    /// be ignored by the narrow phase. Keep in mind that modifying the proximity filter will have
    /// a non-trivial overhead during the next update as it will force re-detection of all
    /// collision pairs.
    pub fn register_broad_phase_pair_filter<F>(&mut self, name: &str, filter: F)
        where F: BroadPhasePairFilter<Point<N>, Isometry<N>, WorldObject, FilterData<N>> {
        self.cworld.register_broad_phase_pair_filter(name, filter)
    }

    /// Removes the pair filter named `name`.
    pub fn unregister_broad_phase_pair_filter(&mut self, name: &str) {
        self.cworld.unregister_broad_phase_pair_filter(name)
    }

    pub fn sensor(&self, sensor: usize) -> &Sensor<N> {
        &self.sensors[sensor]
    }

    pub fn mut_sensor(&mut self, sensor: usize) -> &mut Sensor<N> {
        &mut self.sensors[sensor]
    }

    pub fn rigid_body(&self, rigid_body: usize) -> &RigidBody<N> {
        &self.rigid_bodies[rigid_body]
    }

    pub fn mut_rigid_body(&mut self, rigid_body: usize) -> &mut RigidBody<N> {
        &mut self.rigid_bodies[rigid_body]
    }
}

struct SensorsNotCollidingTheirParentPairFilter;

impl<N: Real> BroadPhasePairFilter<Point<N>, Isometry<N>, WorldObject, FilterData<N>>
for SensorsNotCollidingTheirParentPairFilter {
    #[inline]
    fn is_pair_valid(&self, b1: &WorldCollisionObject<N>, b2: &WorldCollisionObject<N>, sensors: &FilterData<N>) -> bool {
        match (&b1.data, &b2.data) {
            (&WorldObject::RigidBody(rb), &WorldObject::Sensor(s)) |
            (&WorldObject::Sensor(s), &WorldObject::RigidBody(rb)) => {
                let bs = &sensors[s];

                if let Some(&parent) = bs.parent() {
                    // TODO: parent is of type rigidbody ? Have a more stronger type here.
                    rb != parent || bs.proximity_with_parent_enabled()
                }
                else {
                    true
                }
            },
            _ => true
        }
    }
}
