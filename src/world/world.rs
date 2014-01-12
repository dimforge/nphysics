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
use detection::detector::Detector;
use detection::constraint::Constraint;
use detection::joint::joint_manager::JointManager;
use detection::joint::ball_in_socket::BallInSocket;
use detection::joint::fixed::Fixed;
use resolution::{AccumulatedImpulseSolver, VelocityAndPosition};
use resolution::solver::Solver;
use object::RigidBody;

type BF = DBVTBroadPhase<Rc<RefCell<RigidBody>>, AABB, BodyBodyDispatcher, ~GeomGeomCollisionDetector>;

pub struct World {
    bodies:      HashMap<uint, Rc<RefCell<RigidBody>>, UintTWHash>,
    forces:      BodyForceGenerator,
    broad_phase: BF,
    integrator:  BodySmpEulerIntegrator,
    detector:    BodiesBodies<BF>,
    sleep:       ActivationManager,
    // ccd:        SweptBallMotionClamping<BF>,
    joints:      JointManager,
    solver:      AccumulatedImpulseSolver,
    collector:   ~[Constraint]
}

impl World {
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

    pub fn add_body(&mut self, b: Rc<RefCell<RigidBody>>) {
        self.bodies.insert(borrow::to_uint(b.borrow()), b.clone());
        self.broad_phase.add(b);
    }

    pub fn remove_body(&mut self, b: &Rc<RefCell<RigidBody>>) {
        self.bodies.remove(&borrow::to_uint(b.borrow()));
        self.broad_phase.remove(b);
        self.detector.remove(b, &mut self.broad_phase, &mut self.sleep);
        self.joints.remove(b, &mut self.sleep);
        b.borrow().with_mut(|b| b.delete());
    }

    pub fn forces_generator<'a>(&'a mut self) -> &'a mut BodyForceGenerator {
        &'a mut self.forces
    }

    pub fn integrator<'a>(&'a mut self) -> &'a mut BodySmpEulerIntegrator {
        &'a mut self.integrator
    }

    pub fn collision_detector<'a>(&'a mut self) -> &'a mut BodiesBodies<BF> {
        &'a mut self.detector
    }

    pub fn broad_phase<'a>(&'a mut self) -> &'a mut BF {
        &'a mut self.broad_phase
    }

    // pub fn ccd_manager<'a>(&'a mut self) -> &'a mut SweptBallMotionClamping<BF> {
    //     self.ccd
    // }

    pub fn joint_manager<'a>(&'a mut self) -> &'a mut JointManager {
        &'a mut self.joints
    }

    pub fn constraints_solver<'a>(&'a mut self) -> &'a mut AccumulatedImpulseSolver {
        &'a mut self.solver
    }

    pub fn set_gravity(&mut self, gravity: LV) {
        self.forces.set_lin_acc(gravity)
    }

    pub fn set_angular_acceleration(&mut self, accel: AV) {
        self.forces.set_ang_acc(accel)
    }

    pub fn gravity(&self) -> LV {
        self.forces.lin_acc()
    }

    pub fn angular_acceleration(&self) -> AV {
        self.forces.ang_acc()
    }

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

    pub fn add_ball_in_socket(&mut self, joint: Rc<RefCell<BallInSocket>>) {
        self.joints.add_ball_in_socket(joint, &mut self.sleep)
    }

    pub fn remove_ball_in_socket(&mut self, joint: &Rc<RefCell<BallInSocket>>) {
        self.joints.remove_ball_in_socket(joint, &mut self.sleep)
    }

    pub fn add_fixed(&mut self, joint: Rc<RefCell<Fixed>>) {
        self.joints.add_fixed(joint, &mut self.sleep)
    }

    pub fn remove_fixed(&mut self, joint: &Rc<RefCell<Fixed>>) {
        self.joints.remove_fixed(joint, &mut self.sleep)
    }

    pub fn interferences(&mut self, out: &mut ~[Constraint]) {
        self.detector.interferences(out, &mut self.broad_phase);
        self.joints.interferences(out, &mut self.broad_phase);
    }
}
