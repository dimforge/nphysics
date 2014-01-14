use std::rc::Rc;
use std::cell::RefCell;
use std::borrow;
use nalgebra::na;
use ncollide::bounding_volume::AABB;
use ncollide::broad::{BroadPhase, DBVTBroadPhase};
use ncollide::ray::Ray;
use ncollide::narrow::{GeomGeomDispatcher, GeomGeomCollisionDetector};
use ncollide::math::{N, LV, AV};
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use integration::{Integrator, BodySmpEulerIntegrator, BodyForceGenerator};
use detection::{BodiesBodies, BodyBodyDispatcher, ActivationManager};
use detection::Detector;
use detection::constraint::Constraint;
use detection::joint::{JointManager, BallInSocket, Fixed};
use resolution::{Solver, AccumulatedImpulseSolver, VelocityAndPosition};
use object::RigidBody;

type BF = DBVTBroadPhase<Rc<RefCell<RigidBody>>, AABB, BodyBodyDispatcher, ~GeomGeomCollisionDetector>;

/// The physics world.
///
/// This is the main structure of the physics engine.
pub struct World {
    priv bodies:      HashMap<uint, Rc<RefCell<RigidBody>>, UintTWHash>,
    priv forces:      BodyForceGenerator,
    priv broad_phase: BF,
    priv integrator:  BodySmpEulerIntegrator,
    priv detector:    BodiesBodies<BF>,
    priv sleep:       ActivationManager,
    // ccd:        SweptBallMotionClamping<BF>,
    priv joints:      JointManager,
    priv solver:      AccumulatedImpulseSolver,
    priv collector:   ~[Constraint]
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
        let broad_phase = DBVTBroadPhase::new(dispatcher, na::cast(0.08));

        // CCD handler
        // XXX let ccd = SweptBallMotionClamping::new(broad_phase, true);

        // Collision detector
        let detector = BodiesBodies::new(geom_dispatcher);

        // Deactivation
        let sleep = ActivationManager::new(na::cast(1.0), na::cast(0.01));

        // Joints
        let joints = JointManager::new();

        /*
         * For constraints resolution
         */
        let solver = AccumulatedImpulseSolver::new(
            na::cast(0.1),
            VelocityAndPosition(na::cast(0.2), na::cast(0.2), na::cast(0.08)),
            na::cast(0.4),
            na::cast(1.0),
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
            collector:   ~[]
        }
    }

    /// Updates the physics world.
    pub fn step(&mut self, dt: N) {
        for e in self.bodies.elements_mut().mut_iter() {
            let mut rb = e.value.borrow().borrow_mut();

            self.forces.update(dt.clone(), rb.get());
            self.integrator.update(dt.clone(), rb.get());
        }

        self.broad_phase.update();

        self.detector.update(&mut self.broad_phase, &mut self.sleep);
        self.joints.update(&mut self.broad_phase, &mut self.sleep);
        self.sleep.update(&mut self.broad_phase, &mut self.bodies);

        self.detector.interferences(&mut self.collector, &mut self.broad_phase);
        self.joints.interferences(&mut self.collector, &mut self.broad_phase);

        self.solver.solve(dt, self.collector);

        self.collector.clear();
    }

    /// Adds a rigid body to the physics world.
    pub fn add_body(&mut self, b: Rc<RefCell<RigidBody>>) {
        self.bodies.insert(borrow::to_uint(b.borrow()), b.clone());
        self.broad_phase.add(b);
    }

    /// Remove a rigid body from the physics world.
    pub fn remove_body(&mut self, b: &Rc<RefCell<RigidBody>>) {
        self.bodies.remove(&borrow::to_uint(b.borrow()));
        self.broad_phase.remove(b);
        self.detector.remove(b, &mut self.broad_phase, &mut self.sleep);
        self.joints.remove(b, &mut self.sleep);
        b.borrow().with_mut(|b| b.delete());
    }

    /// Gets a mutable reference to the force generator.
    pub fn forces_generator<'a>(&'a mut self) -> &'a mut BodyForceGenerator {
        &'a mut self.forces
    }

    /// Gets a mutable reference to the position and orientation integrator.
    pub fn integrator<'a>(&'a mut self) -> &'a mut BodySmpEulerIntegrator {
        &'a mut self.integrator
    }

    /// Gets a mutable reference to the collision detector.
    pub fn collision_detector<'a>(&'a mut self) -> &'a mut BodiesBodies<BF> {
        &'a mut self.detector
    }

    /// Gets a mutable reference to the broad phase.
    pub fn broad_phase<'a>(&'a mut self) -> &'a mut BF {
        &'a mut self.broad_phase
    }

    // pub fn ccd_manager<'a>(&'a mut self) -> &'a mut SweptBallMotionClamping<BF> {
    //     self.ccd
    // }

    /// Gets a mutable reference to the joint manager.
    pub fn joint_manager<'a>(&'a mut self) -> &'a mut JointManager {
        &'a mut self.joints
    }

    /// Gets a mutable reference to the constraint solver.
    pub fn constraints_solver<'a>(&'a mut self) -> &'a mut AccumulatedImpulseSolver {
        &'a mut self.solver
    }

    /// Sets the linear acceleration afecting every dynamic rigid body.
    pub fn set_gravity(&mut self, gravity: LV) {
        self.forces.set_lin_acc(gravity)
    }

    /// Sets the angular acceleration afecting every dynamic rigid body.
    pub fn set_angular_acceleration(&mut self, accel: AV) {
        self.forces.set_ang_acc(accel)
    }

    /// Gets the linear acceleration afecting every dynamic rigid body.
    pub fn gravity(&self) -> LV {
        self.forces.lin_acc()
    }

    /// Gets the angular acceleration afecting every dynamic rigid body.
    pub fn angular_acceleration(&self) -> AV {
        self.forces.ang_acc()
    }

    /// Gets every body intersected by a given ray.
    pub fn cast_ray(&mut self, ray: &Ray, out: &mut ~[(Rc<RefCell<RigidBody>>, N)]) {
        self.detector.interferences_with_ray(ray, &mut self.broad_phase, out)
    }

    /*
    pub fn add_ccd_to(&mut self,
                      body:                Rc<RefCell<RigidBody>>,
                      swept_sphere_radius: N,
                      motion_thresold:     N) {
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
        self.joints.remove_fixed(joint, &mut self.sleep)
    }

    /// Collects every interferences detected since the last update.
    pub fn interferences(&mut self, out: &mut ~[Constraint]) {
        self.detector.interferences(out, &mut self.broad_phase);
        self.joints.interferences(out, &mut self.broad_phase);
    }
}
