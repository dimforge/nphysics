use nalgebra::na::{CrossMatrix, Row};
use nalgebra::na;
use ncollide::bounding_volume::AABB;
use ncollide::broad::DBVTBroadPhase;
use ncollide::ray::Ray;
use integration::{Integrator, BodyForceGenerator, BodySmpEulerIntegrator, SweptBallMotionClamping};
use detection::collision::bodies_bodies::PairwiseDetector;
use detection::{BodiesBodies, BodiesBodiesDispatcher};
use detection::detector::Detector;
use detection::constraint::Constraint;
use detection::joint::joint_manager::JointManager;
use detection::joint::ball_in_socket::BallInSocket;
use detection::joint::fixed::Fixed;
use detection::IslandActivationManager;
use resolution::{AccumulatedImpulseSolver, Velocity};
use resolution::solver::Solver;
use world::World;
use aliases::traits::{NPhysicsScalar, NPhysicsDirection, NPhysicsOrientation, NPhysicsTransform, NPhysicsInertia};
use object::Body;
use signal::signal::SignalEmiter;

type BF<N, LV, AV, M, II> =
    DBVTBroadPhase<
        N,
        LV,
        Body<N, LV, AV, M, II>,
        AABB<N, LV>,
        BodiesBodiesDispatcher<N, LV, AV, M, II>,
        PairwiseDetector<N, LV, AV, M>
    >;

pub struct BodyWorld<N, LV, AV, M, II, CM> {
    world:      World<N, Body<N, LV, AV, M, II>, Constraint<N, LV, AV, M, II>>,
    forces:     @mut BodyForceGenerator<N, LV, AV, M, II>,
    integrator: @mut BodySmpEulerIntegrator<N, LV, AV, M, II>,
    detector:   @mut BodiesBodies<N, LV, AV, M, II, BF<N, LV, AV, M, II>>,
    sleep:      @mut IslandActivationManager<N, LV, AV, M, II>,
    ccd:        @mut SweptBallMotionClamping<N, LV, AV, M, II, BF<N, LV, AV, M, II>>,
    joints:     @mut JointManager<N, LV, AV, M, II>,
    solver:     @mut AccumulatedImpulseSolver<N, LV, AV, M, II, CM>
}

impl<N:  'static + NPhysicsScalar,
     LV: 'static + Clone + CrossMatrix<CM> + NPhysicsDirection<N, AV>,
     AV: 'static + Clone + NPhysicsOrientation<N>,
     M:  'static + Clone + NPhysicsTransform<LV, AV>,
     II: 'static + Clone + NPhysicsInertia<N, LV, AV, M>,
     CM: Row<AV>>
BodyWorld<N, LV, AV, M, II, CM> {
    pub fn new() -> BodyWorld<N, LV, AV, M, II, CM> {
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
        let dispatcher = BodiesBodiesDispatcher::new();
        // Broad phase
        let broad_phase = @mut DBVTBroadPhase::new(dispatcher, na::cast(0.08));
        // CCD handler
        let ccd = SweptBallMotionClamping::new(events, broad_phase, true);
        // Collision detector
        let detector = BodiesBodies::new(events, broad_phase, false);
        // Deactivation
        let sleep = IslandActivationManager::new(events, na::cast(1.0), na::cast(0.01));
        // Joints
        let joints = JointManager::new(events);

        /*
         * For constraints resolution
         */
        let solver = @mut AccumulatedImpulseSolver::new(
            na::cast(0.1),
            Velocity(na::cast(0.2)),
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
        world.add_detector(sleep);
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

    pub fn add_body(&mut self, b: @mut Body<N, LV, AV, M, II>) {
        self.world.add_object(b)
    }

    pub fn remove_body(&mut self, b: @mut Body<N, LV, AV, M, II>) {
        self.world.remove_object(b)
    }

    pub fn world<'r>(&'r self) -> &'r World<N, Body<N, LV, AV, M, II>, Constraint<N, LV, AV, M, II>> {
        &'r self.world
    }

    pub fn world_mut<'r>(&'r mut self) -> &'r mut World<N, Body<N, LV, AV, M, II>, Constraint<N, LV, AV, M, II>> {
        &'r mut self.world
    }

    pub fn forces_generator(&self) -> @mut BodyForceGenerator<N, LV, AV, M, II> {
        self.forces
    }

    pub fn integrator(&self) -> @mut BodySmpEulerIntegrator<N, LV, AV, M, II> {
        self.integrator
    }

    pub fn collison_detector(&self) -> @mut BodiesBodies<N, LV, AV, M, II, BF<N, LV, AV, M, II>> {
        self.detector
    }

    pub fn sleep_manager(&self) -> @mut IslandActivationManager<N, LV, AV, M, II> {
        self.sleep
    }

    pub fn ccd_manager(&self) -> @mut SweptBallMotionClamping<N, LV, AV, M, II, BF<N, LV, AV, M, II>> {
        self.ccd
    }

    pub fn joints_manager(&self) -> @mut JointManager<N, LV, AV, M, II> {
        self.joints
    }

    pub fn constraints_solver(&self) -> @mut AccumulatedImpulseSolver<N, LV, AV, M, II, CM> {
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

    pub fn cast_ray(&self, ray: &Ray<LV>, out: &mut ~[(@mut Body<N, LV, AV, M, II>, N)]) {
        self.detector.interferences_with_ray(ray, out)
    }

    pub fn add_ccd_to(&mut self,
                      body:                @mut Body<N, LV, AV, M, II>,
                      swept_sphere_radius: N,
                      motion_thresold:     N) {
        self.ccd.add_ccd_to(body, swept_sphere_radius, motion_thresold)
    }

    pub fn add_ball_in_socket(&mut self, joint: @mut BallInSocket<N, LV, AV, M, II>) {
        self.joints.add_ball_in_socket(joint)
    }

    pub fn remove_ball_in_socket(&mut self, joint: @mut BallInSocket<N, LV, AV, M, II>) {
        self.joints.remove_ball_in_socket(joint)
    }

    pub fn add_fixed(&mut self, joint: @mut Fixed<N, LV, AV, M, II>) {
        self.joints.add_fixed(joint)
    }

    pub fn remove_fixed(&mut self, joint: @mut Fixed<N, LV, AV, M, II>) {
        self.joints.remove_fixed(joint)
    }

    pub fn add_detector<D: 'static + Detector<N, Body<N, LV, AV, M, II>, Constraint<N, LV, AV, M, II>>>(
                        &mut self,
                        d: @mut D) {
        self.world.add_detector(d)
    }

    pub fn add_integrator<I: 'static + Integrator<N, Body<N, LV, AV, M, II>>>(&mut self, i: @mut I) {
        self.world.add_integrator(i)
    }

    pub fn add_solver<S: 'static + Solver<N, Constraint<N, LV, AV, M, II>>>(&mut self, s: @mut S) {
        self.world.add_solver(s)
    }
}
