use std::slice::Iter;
use std::iter::Map;
use std::rc::Rc;
use std::cell::RefCell;
use na;
use ncollide::math::Scalar;
use ncollide::bounding_volume::AABB;
use ncollide::utils::data::hash_map::{HashMap, Entry};
use ncollide::utils::data::hash::UintTWHash;
use ncollide::broad_phase::{BroadPhase, DBVTBroadPhase};
use ncollide::narrow_phase::ContactSignalHandler;
use ncollide::world::{CollisionWorld, CollisionObject};
use integration::{Integrator, BodySmpEulerIntegrator, BodyForceGenerator,
                  TranslationalCCDMotionClamping};
use detection::ActivationManager;
use detection::Detector;
use detection::constraint::Constraint;
use detection::joint::{JointManager, BallInSocket, Fixed};
use resolution::{Solver, AccumulatedImpulseSolver, CorrectionMode};
use object::{RigidBody, RigidBodyHandle};
use math::{Point, Vector, Matrix};

/// The default broad phase.
pub type WorldBroadPhase<N> = DBVTBroadPhase<Point<N>, RigidBodyHandle<N>, AABB<Point<N>>>;
/// An iterator visiting rigid bodies.
pub type RigidBodies<'a, N> = Map<Iter<'a, Entry<usize, RigidBodyHandle<N>>>, fn(&'a Entry<usize, RigidBodyHandle<N>>) -> &'a RigidBodyHandle<N>>;

/// Type of the collision world containing rigid bodies.
pub type RigidBodyCollisionWorld<N> = CollisionWorld<Point<N>, Matrix<N>, RigidBodyHandle<N>>;

/// Type of a collision object containing a rigid body as its data.
pub type RigidBodyCollisionObject<N> = CollisionObject<Point<N>, Matrix<N>, RigidBodyHandle<N>>;


/// The physical world.
///
/// This is the main structure of the physics engine.
pub struct World<N: Scalar> {
    cworld:      RigidBodyCollisionWorld<N>,
    bodies:      HashMap<usize, RigidBodyHandle<N>, UintTWHash>,
    forces:      BodyForceGenerator<N>,
    integrator:  BodySmpEulerIntegrator,
    sleep:       Rc<RefCell<ActivationManager<N>>>, // FIXME: avoid sharing (needed for the contact signal handler)
    ccd:         TranslationalCCDMotionClamping<N>,
    joints:      JointManager<N>,
    solver:      AccumulatedImpulseSolver<N>,
}

impl<N: Scalar> World<N> {
    /// Creates a new physics world.
    pub fn new() -> World<N> {
        /*
         * Setup the physics world
         */

        // For the intergration
        let forces     = BodyForceGenerator::new(na::zero(), na::zero());
        let integrator = BodySmpEulerIntegrator::new();

        /*
         * For the collision detection
         */
        // Collision world
        let mut cworld = CollisionWorld::new(na::cast(0.10f64), na::cast(0.10f64), false);

        // CCD handler
        let ccd = TranslationalCCDMotionClamping::new();

        // Deactivation
        let sleep = Rc::new(RefCell::new(ActivationManager::new(na::cast(0.01f64))));

        // Setup contact handler to reactivate sleeping objects that loose contact.
        let handler = ObjectActivationOnContactHandler::new(sleep.clone());
        cworld.register_contact_signal_handler("__nphysics_internal_ObjectActivationOnContactHandler", handler);

        // Joints
        let joints = JointManager::new();

        /*
         * For constraints resolution
         */
        let solver = AccumulatedImpulseSolver::new(
            na::cast(0.1f64),
            CorrectionMode::VelocityAndPosition(na::cast(0.2f64), na::cast(0.2f64), na::cast(0.08f64)),
            na::cast(0.4f64),
            na::cast(1.0f64),
            10,
            10);

        World {
            cworld:     cworld,
            bodies:     HashMap::new(UintTWHash::new()),
            forces:     forces,
            integrator: integrator,
            sleep:      sleep,
            ccd:        ccd,
            joints:     joints,
            solver:     solver,
        }
    }

    /// Updates the physics world.
    pub fn step(&mut self, dt: N) {
        for e in self.bodies.elements_mut().iter_mut() {
            let mut rb = e.value.borrow_mut();

            if rb.is_active() {
                self.forces.update(dt.clone(), &mut *rb);
                self.integrator.update(dt.clone(), &mut *rb);
                self.cworld.deferred_set_position(&*e.value as *const RefCell<RigidBody<N>> as usize,
                                                  rb.position().clone());
            }
        }

        self.cworld.perform_position_update();
        self.cworld.perform_broad_phase();
        self.ccd.update(&mut self.cworld);
        self.cworld.perform_narrow_phase();

        self.joints.update(&mut *self.sleep.borrow_mut());
        self.sleep.borrow_mut().update(&mut self.cworld, &self.joints, &self.bodies);

        // XXX: use `self.collector` instead to avoid allocation.
        let mut collector = Vec::new();

        for (b1, b2, c) in self.cworld.contacts() {
            if b1.data.borrow().is_active() || b2.data.borrow().is_active() {
                let m1 = b1.data.borrow().margin();
                let m2 = b2.data.borrow().margin();

                let mut c = c.clone();
                c.depth = c.depth + m1 + m2;

                collector.push(Constraint::RBRB(b1.data.clone(), b2.data.clone(), c));
            }
        }

        self.joints.interferences(&mut collector);

        self.solver.solve(dt, &collector[..]);

        collector.clear();
    }

    /// Adds a rigid body to the physics world.
    pub fn add_body(&mut self, rb: RigidBody<N>) -> RigidBodyHandle<N> {
        let position = rb.position().clone();
        let shape = rb.shape().clone();
        let groups = rb.collision_groups().as_collision_groups().clone();

        let handle = Rc::new(RefCell::new(rb));
        let uid = &*handle as *const RefCell<RigidBody<N>> as usize;

        self.bodies.insert(uid, handle.clone());
        self.cworld.add(uid, position, shape, groups, handle.clone());

        handle
    }

    /// Remove a rigid body from the physics world.
    pub fn remove_body(&mut self, b: &RigidBodyHandle<N>) {
        let uid = &**b as *const RefCell<RigidBody<N>> as usize;
        self.cworld.deferred_remove(uid);
        self.cworld.perform_removals_and_broad_phase();
        self.joints.remove(b, &mut *self.sleep.borrow_mut());
        self.ccd.remove_ccd_from(b);
        self.bodies.remove(&uid);
        b.borrow_mut().delete();
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
    pub fn add_ccd_to(&mut self, body: &RigidBodyHandle<N>, motion_thresold: N) {
        self.ccd.add_ccd_to(body.clone(), motion_thresold)
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

    /// Collects every interferences detected since the last update.
    pub fn interferences(&mut self, out: &mut Vec<Constraint<N>>) {
        // FIXME: ugly.
        for (b1, b2, c) in self.cworld.contacts() {
            let m1 = b1.data.borrow().margin();
            let m2 = b2.data.borrow().margin();

            let mut c = c.clone();
            c.depth = c.depth + m1 + m2;

            out.push(Constraint::RBRB(b1.data.clone(), b2.data.clone(), c));
        }

        self.joints.interferences(out);
    }

    /// An iterator visiting all rigid bodies on this world.
    pub fn bodies(&self) -> RigidBodies<N> {
        fn extract_value<N: Scalar>(e: &Entry<usize, RigidBodyHandle<N>>) -> &RigidBodyHandle<N> {
            &e.value
        }

        let extract_value_fn: fn(_) -> _ = extract_value;
        self.bodies.elements().iter().map(extract_value_fn)
    }

    /* FIXME
    /// Registers a handler for proximity start/stop events.
    pub fn register_proximity_signal_handler<H>(&mut self, name: &str, handler: H)
        where H: ProximitySignalHandler<RigidBodyHandle<N>> + 'static {
        self.cworld.register_proximity_signal_handler(name, handler)
    }

    /// Unregisters a handler for proximity start/stop events.
    pub fn unregister_proximity_signal_handler(&mut self, name: &str) {
        self.cworld.unregister_proximity_signal_handler(name)
    }
    */

    /// Registers a handler for contact start/stop events.
    pub fn register_contact_signal_handler<H>(&mut self, name: &str, handler: H)
        where H: ContactSignalHandler<RigidBodyHandle<N>> + 'static {
        self.cworld.register_contact_signal_handler(name, handler)
    }

    /// Unregisters a handler for contact start/stop events.
    pub fn unregister_contact_signal_handler(&mut self, name: &str) {
        self.cworld.unregister_contact_signal_handler(name)
    }
}

struct ObjectActivationOnContactHandler<N: Scalar> {
    sleep: Rc<RefCell<ActivationManager<N>>>
}

impl<N: Scalar> ObjectActivationOnContactHandler<N> {
    pub fn new(sleep: Rc<RefCell<ActivationManager<N>>>) -> ObjectActivationOnContactHandler<N> {
        ObjectActivationOnContactHandler {
            sleep: sleep
        }
    }
}

impl<N: Scalar> ContactSignalHandler<RigidBodyHandle<N>> for ObjectActivationOnContactHandler<N> {
    fn handle_contact(&mut self, b1: &RigidBodyHandle<N>, b2: &RigidBodyHandle<N>, started: bool) {
        // Wake on collision lost.
        if !started {
            self.sleep.borrow_mut().deferred_activate(b1);
            self.sleep.borrow_mut().deferred_activate(b2);
        }
    }
}
