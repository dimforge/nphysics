use std::rc::Rc;
use std::cell::RefCell;
use std::iter::Map;
use std::slice::Items;
use nalgebra::na;
use ncollide::bounding_volume::AABB;
use ncollide::broad::{BroadPhase, DBVTBroadPhase};
use ncollide::ray::Ray;
use ncollide::narrow::{GeomGeomDispatcher, GeomGeomCollisionDetector};
use ncollide::math::{Scalar, Vect, Orientation};
use ncollide::data::hash_map::{HashMap, Entry};
use ncollide::data::hash::UintTWHash;
use integration::{Integrator, BodySmpEulerIntegrator, BodyForceGenerator};
use detection::{BodiesBodies, BodyBodyDispatcher, ActivationManager};
use detection::Detector;
use detection::constraint::Constraint;
use detection::joint::{JointManager, BallInSocket, Fixed};
use resolution::{Solver, AccumulatedImpulseSolver, VelocityAndPosition};
use object::RigidBody;

/// The default broad phase.
pub type WorldBroadPhase = DBVTBroadPhase<Rc<RefCell<RigidBody>>, AABB, BodyBodyDispatcher, Box<GeomGeomCollisionDetector + Send>>;
/// An iterator visiting rigid bodies.
pub type RigidBodies<'a> = Map<'a, &'a Entry<uint, Rc<RefCell<RigidBody>>>, &'a Rc<RefCell<RigidBody>>, Items<'a, Entry<uint, Rc<RefCell<RigidBody>>>>>;

/// The physics world.
///
/// This is the main structure of the physics engine.
pub struct World {
    bodies:      HashMap<uint, Rc<RefCell<RigidBody>>, UintTWHash>,
    forces:      BodyForceGenerator,
    broad_phase: WorldBroadPhase,
    integrator:  BodySmpEulerIntegrator,
    detector:    BodiesBodies<WorldBroadPhase>,
    sleep:       ActivationManager,
    // ccd:        SweptBallMotionClamping<WorldBroadPhase>,
    joints:      JointManager,
    solver:      AccumulatedImpulseSolver,
    collector:   Vec<Constraint>
}

impl World {
    /// Creates a new physics world.
    pub fn new() -> World {
        /*
         * Setup the physics world
         */

        // For the intergration
        let forces     = BodyForceGenerator::new(na::zero(), na::zero());
        let integrator = BodySmpEulerIntegrator::new();

        /*
         * For the collision detection
         */
        // Collision Dispatcher
        let geom_dispatcher = Rc::new(GeomGeomDispatcher::new());
        let dispatcher = BodyBodyDispatcher::new(geom_dispatcher.clone());

        // Broad phase
        let broad_phase = DBVTBroadPhase::new(dispatcher, na::cast(0.08f64));

        // CCD handler
        // XXX let ccd = SweptBallMotionClamping::new(broad_phase, true);

        // Collision detector
        let detector = BodiesBodies::new(geom_dispatcher);

        // Deactivation
        let sleep = ActivationManager::new(na::cast(0.01f64));

        // Joints
        let joints = JointManager::new();

        /*
         * For constraints resolution
         */
        let solver = AccumulatedImpulseSolver::new(
            na::cast(0.1f64),
            VelocityAndPosition(na::cast(0.2f64), na::cast(0.2f64), na::cast(0.08f64)),
            na::cast(0.4f64),
            na::cast(1.0f64),
            10,
            10);

        World {
            bodies:      HashMap::new(UintTWHash::new()),
            broad_phase: broad_phase,
            forces:      forces,
            integrator:  integrator,
            detector:    detector,
            sleep:       sleep,
            // ccd:      ccd,
            joints:      joints,
            solver:      solver,
            collector:   Vec::new()
        }
    }

    /// Updates the physics world.
    pub fn step(&mut self, dt: Scalar) {
        for e in self.bodies.elements_mut().mut_iter() {
            let mut rb = e.value.borrow_mut();

            self.forces.update(dt.clone(), rb.deref_mut());
            self.integrator.update(dt.clone(), rb.deref_mut());
        }

        self.broad_phase.update();

        self.detector.update(&mut self.broad_phase, &mut self.sleep);
        self.joints.update(&mut self.broad_phase, &mut self.sleep);
        self.sleep.update(&mut self.broad_phase, &self.joints, &mut self.bodies);

        self.detector.interferences(&mut self.collector, &mut self.broad_phase);
        self.joints.interferences(&mut self.collector, &mut self.broad_phase);

        self.solver.solve(dt, self.collector.as_slice());

        self.collector.clear();
    }

    /// Adds a rigid body to the physics world.
    pub fn add_body(&mut self, b: Rc<RefCell<RigidBody>>) {
        self.bodies.insert(b.deref() as *const RefCell<RigidBody> as uint, b.clone());
        self.broad_phase.add(b);
    }

    /// Remove a rigid body from the physics world.
    pub fn remove_body(&mut self, b: &Rc<RefCell<RigidBody>>) {
        self.bodies.remove(&(b.deref() as *const RefCell<RigidBody> as uint));
        self.broad_phase.remove(b);
        self.detector.remove(b, &mut self.broad_phase, &mut self.sleep);
        self.joints.remove(b, &mut self.sleep);
        b.borrow_mut().delete();
    }

    /// Gets a mutable reference to the force generator.
    pub fn forces_generator<'a>(&'a mut self) -> &'a mut BodyForceGenerator {
        &mut self.forces
    }

    /// Gets a mutable reference to the position and orientation integrator.
    pub fn integrator<'a>(&'a mut self) -> &'a mut BodySmpEulerIntegrator {
        &mut self.integrator
    }

    /// Gets a mutable reference to the collision detector.
    pub fn collision_detector<'a>(&'a mut self) -> &'a mut BodiesBodies<WorldBroadPhase> {
        &mut self.detector
    }

    /// Gets a mutable reference to the broad phase.
    pub fn broad_phase<'a>(&'a mut self) -> &'a mut WorldBroadPhase {
        &mut self.broad_phase
    }

    // pub fn ccd_manager<'a>(&'a mut self) -> &'a mut SweptBallMotionClamping<WorldBroadPhase> {
    //     self.ccd
    // }

    /// Gets a mutable reference to the joint manager.
    pub fn joint_manager<'a>(&'a mut self) -> &'a mut JointManager {
        &mut self.joints
    }

    /// Gets a mutable reference to the constraint solver.
    pub fn constraints_solver<'a>(&'a mut self) -> &'a mut AccumulatedImpulseSolver {
        &mut self.solver
    }

    /// Sets the linear acceleration afecting every dynamic rigid body.
    pub fn set_gravity(&mut self, gravity: Vect) {
        self.forces.set_lin_acc(gravity)
    }

    /// Sets the angular acceleration afecting every dynamic rigid body.
    pub fn set_angular_acceleration(&mut self, accel: Orientation) {
        self.forces.set_ang_acc(accel)
    }

    /// Gets the linear acceleration afecting every dynamic rigid body.
    pub fn gravity(&self) -> Vect {
        self.forces.lin_acc()
    }

    /// Gets the angular acceleration afecting every dynamic rigid body.
    pub fn angular_acceleration(&self) -> Orientation {
        self.forces.ang_acc()
    }

    /// Gets every body intersected by a given ray.
    pub fn cast_ray(&mut self, ray: &Ray, out: &mut Vec<(Rc<RefCell<RigidBody>>, Scalar)>) {
        self.detector.interferences_with_ray(ray, &mut self.broad_phase, out)
    }

    /*
    pub fn add_ccd_to(&mut self,
                      body:                Rc<RefCell<RigidBody>>,
                      swept_sphere_radius: Scalar,
                      motion_thresold:     Scalar) {
        self.ccd.add_ccd_to(body, swept_sphere_radius, motion_thresold)
    }
    */

    /// Adds a ball-in-socket joint to the world.
    pub fn add_ball_in_socket(&mut self, joint: Rc<RefCell<BallInSocket>>) {
        self.joints.add_ball_in_socket(joint, &mut self.sleep)
    }

    /// Removes a ball-in-socket joint from the world.
    pub fn remove_ball_in_socket(&mut self, joint: &Rc<RefCell<BallInSocket>>) {
        self.joints.remove_ball_in_socket(joint, &mut self.sleep)
    }

    /// Adds a fixed joint to the world.
    pub fn add_fixed(&mut self, joint: Rc<RefCell<Fixed>>) {
        self.joints.add_fixed(joint, &mut self.sleep)
    }

    /// Removes a fixed joint from the world.
    pub fn remove_fixed(&mut self, joint: &Rc<RefCell<Fixed>>) {
        self.joints.remove_joint(joint, &mut self.sleep)
    }

    /// Collects every interferences detected since the last update.
    pub fn interferences(&mut self, out: &mut Vec<Constraint>) {
        self.detector.interferences(out, &mut self.broad_phase);
        self.joints.interferences(out, &mut self.broad_phase);
    }

    /// An iterator visiting all rigid bodies on this world.
    pub fn bodies<'a>(&'a self) -> RigidBodies<'a> {
        self.bodies.elements().iter().map(|e| &e.value)
    }
}
