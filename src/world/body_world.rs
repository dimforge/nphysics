use std::rc::Rc;
use nalgebra::na;
use ncollide::bounding_volume::AABB;
use ncollide::broad::DBVTBroadPhase;
use ncollide::ray::Ray;
use ncollide::narrow::{GeomGeomDispatcher, GeomGeomCollisionDetector};
use ncollide::math::{N, LV, AV};
use integration::{Integrator, BodyForceGenerator, BodySmpEulerIntegrator, SweptBallMotionClamping};
use detection::{BodiesBodies, BodyBodyDispatcher};
use detection::detector::Detector;
use detection::constraint::Constraint;
use detection::joint::joint_manager::JointManager;
use detection::joint::ball_in_socket::BallInSocket;
use detection::joint::fixed::Fixed;
use detection::IslandActivationManager;
use resolution::{AccumulatedImpulseSolver, VelocityAndPosition};
use resolution::solver::Solver;
use world::World;
use object::Body;
use signal::signal::SignalEmiter;

type BF = DBVTBroadPhase<Body, AABB, BodyBodyDispatcher, ~GeomGeomCollisionDetector>;

pub struct BodyWorld {
    world:      World<Body, Constraint>,
    forces:     @mut BodyForceGenerator,
    integrator: @mut BodySmpEulerIntegrator,
    detector:   @mut BodiesBodies<BF>,
    sleep:      @mut IslandActivationManager,
    ccd:        @mut SweptBallMotionClamping<BF>,
    joints:     @mut JointManager,
    solver:     @mut AccumulatedImpulseSolver
}

impl BodyWorld {
    pub fn new() -> BodyWorld {
        /*
         * Setup the physics world
         */
        let mut world = World::new();

        // events handler
        let events = @mut SignalEmiter::new();

        // For the intergration
        let forces     = BodyForceGenerator::new(events, na::zero(), na::zero());
        let integrator = BodySmpEulerIntegrator::new(events);

        /*
         * For the collision detection
         */
        // Collision Dispatcher
        let geom_dispatcher = Rc::from_send(GeomGeomDispatcher::new());
        let dispatcher = BodyBodyDispatcher::new(geom_dispatcher.clone());
        // Broad phase
        let broad_phase = @mut DBVTBroadPhase::new(dispatcher, na::cast(0.08));
        // CCD handler
        let ccd = SweptBallMotionClamping::new(events, broad_phase, true);
        // Collision detector
        let detector = BodiesBodies::new(events, broad_phase, geom_dispatcher, false);
        // Deactivation
        let sleep = IslandActivationManager::new(events, na::cast(1.0), na::cast(0.01));
        // Joints
        let joints = JointManager::new(events);

        /*
         * For constraints resolution
         */
        let solver = @mut AccumulatedImpulseSolver::new(
            na::cast(0.1),
            VelocityAndPosition(na::cast(0.2), na::cast(0.2), na::cast(0.08)),
            na::cast(0.4),
            na::cast(1.0),
            10,
            10);

        /*
         * Add everything to the world
         */
        world.add_integrator(forces);
        world.add_integrator(integrator);
        world.add_integrator(ccd);
        world.add_detector(detector);
        world.add_detector(joints);
        // world.add_detector(sleep);
        world.add_solver(solver);

        BodyWorld {
            world:      world,
            forces:     forces,
            integrator: integrator,
            detector:   detector,
            sleep:      sleep,
            ccd:        ccd,
            joints:     joints,
            solver:     solver
        }
    }

    pub fn step(&mut self, dt: N) {
        self.world.step(dt)
    }

    pub fn add_body(&mut self, b: @mut Body) {
        self.world.add_object(b)
    }

    pub fn remove_body(&mut self, b: @mut Body) {
        self.world.remove_object(b)
    }

    pub fn world<'r>(&'r self) -> &'r World<Body, Constraint> {
        &'r self.world
    }

    pub fn world_mut<'r>(&'r mut self) -> &'r mut World<Body, Constraint> {
        &'r mut self.world
    }

    pub fn forces_generator(&self) -> @mut BodyForceGenerator {
        self.forces
    }

    pub fn integrator(&self) -> @mut BodySmpEulerIntegrator {
        self.integrator
    }

    pub fn collison_detector(&self) -> @mut BodiesBodies<BF> {
        self.detector
    }

    pub fn sleep_manager(&self) -> @mut IslandActivationManager {
        self.sleep
    }

    pub fn ccd_manager(&self) -> @mut SweptBallMotionClamping<BF> {
        self.ccd
    }

    pub fn joints_manager(&self) -> @mut JointManager {
        self.joints
    }

    pub fn constraints_solver(&self) -> @mut AccumulatedImpulseSolver {
        self.solver
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

    pub fn cast_ray(&self, ray: &Ray, out: &mut ~[(@mut Body, N)]) {
        self.detector.interferences_with_ray(ray, out)
    }

    pub fn add_ccd_to(&mut self,
                      body:                @mut Body,
                      swept_sphere_radius: N,
                      motion_thresold:     N) {
        self.ccd.add_ccd_to(body, swept_sphere_radius, motion_thresold)
    }

    pub fn add_ball_in_socket(&mut self, joint: @mut BallInSocket) {
        self.joints.add_ball_in_socket(joint)
    }

    pub fn remove_ball_in_socket(&mut self, joint: @mut BallInSocket) {
        self.joints.remove_ball_in_socket(joint)
    }

    pub fn add_fixed(&mut self, joint: @mut Fixed) {
        self.joints.add_fixed(joint)
    }

    pub fn remove_fixed(&mut self, joint: @mut Fixed) {
        self.joints.remove_fixed(joint)
    }

    pub fn add_detector<D: 'static + Detector<Body, Constraint>>(
                        &mut self,
                        d: @mut D) {
        self.world.add_detector(d)
    }

    pub fn add_integrator<I: 'static + Integrator<Body>>(&mut self, i: @mut I) {
        self.world.add_integrator(i)
    }

    pub fn add_solver<S: 'static + Solver<Constraint>>(&mut self, s: @mut S) {
        self.world.add_solver(s)
    }
}
