#[link(name     = "pyramid3d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "9ec127f7-d531-4fbd-b405-9af404cb7aaa")];
#[crate_type = "bin"];
#[warn(non_camel_case_types)]

extern mod std;
extern mod extra;
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;
extern mod graphics3d;

use std::num::One;

use nalgebra::mat::Translation;
use nalgebra::vec::{Vec3, AlgebraicVec};

use ncollide::geom::{Box, Plane};
use ncollide::broad::DBVTBroadPhase;

use nphysics::world::World;
use nphysics::aliases::dim3;
use nphysics::integration::{BodyForceGenerator, RigidBodySmpEulerIntegrator, SweptBallMotionClamping};
use nphysics::detection::{BodiesBodies, BodiesBodiesDispatcher, IslandActivationManager, JointManager};
use nphysics::resolution::{AccumulatedImpulseSolver, VelocityAndPosition};
use nphysics::object::{RigidBody, Static, Dynamic, DefaultGeom, RB};
use nphysics::signal::signal::SignalEmiter;

use graphics3d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8, crate_map: *u8) -> int {
    std::rt::start_on_main_thread(argc, argv, crate_map, main)
}

fn main() {
    GraphicsManager::simulate(boxes_vee_3d)
}

pub fn boxes_vee_3d(graphics: &mut GraphicsManager)
                -> (dim3::World3d<f64>,
                    @mut dim3::DBVTCollisionDetector3d<f64>,
                    @mut dim3::DBVTSweptBallMotionClamping3d<f64>,
                    @mut dim3::JointManager3d<f64>) {
    /*
     * Setup the physics world
     */
    let mut world = World::new();

    // events handler
    let events = @mut SignalEmiter::new();

    // For the intergration
    let gravity = Vec3::new(0.0f64, -9.81, 0.0);
    let tornado = Vec3::new(0.0f64, 0.0, 0.0);

    let forces = BodyForceGenerator::new(events, gravity, tornado);
    let integrator = RigidBodySmpEulerIntegrator::new(events);

    /*
     * For the collision detection
     */
    // Collision Dispatcher
    let dispatcher = BodiesBodiesDispatcher::new();
    // Broad phase
    let broad_phase = @mut DBVTBroadPhase::new(dispatcher, 0.08f64);
    // CCD handler
    let ccd = SweptBallMotionClamping::new(events, broad_phase, true);
    // Collision detector
    let detector = BodiesBodies::new(events, broad_phase, false);
    // Deactivation
    let sleep = IslandActivationManager::new(events, 1.0, 0.01);
    // Joints
    let joints = JointManager::new(events);

    /*
     * For constraints resolution
     */
    let solver: @mut dim3::ContactSolver3d<f64> =
        @mut AccumulatedImpulseSolver::new(0.1f64, VelocityAndPosition(0.2, 0.2, 0.08), 1.0, 10, 10);

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

    let normals = [
        Vec3::new(0.0f64, 1.0, 0.0 ).normalized(),
        // Vec3::new(-1.0f64, 1.0, -1.0 ).normalized(),
        // Vec3::new(1.0f64, 1.0, -1.0 ).normalized(),
        // Vec3::new(-1.0f64, 1.0, 1.0 ).normalized(),
        // Vec3::new(1.0f64, 1.0, 1.0 ).normalized()
    ];

    /*
     * Planes
     */
    for n in normals.iter() {
        let geom = Plane::new(*n);
        let body = @mut RigidBody::new(DefaultGeom::new_plane(geom), 0.0f64, Static, 0.3, 0.6);

        world.add_object(@mut RB(body));
        graphics.add_plane(body, &geom);
    }

    /*
     * Create the boxes
     */
    let num     = 30;
    let rad     = 0.5;
    let shift   = rad * 2.0;
    let centerx = shift * (num as f64) / 2.0;
    let centery = shift / 2.0;

    for i in range(0u, num) {
        for j in range(i, num) {
            let fi = i as f64;
            let fj = (j - i) as f64;
            let x = (fi * shift / 2.0) + fj * shift - centerx;
            let y = fi * shift + centery;

            let box  = Box::new(Vec3::new(rad, rad, rad));
            let geom = DefaultGeom::new_box(box);
            let body = @mut RigidBody::new(geom, 1.0f64, Dynamic, 0.3, 0.6);

            body.translate_by(&Vec3::new(x, y, 0.0));

            world.add_object(@mut RB(body));
            graphics.add_cube(body, One::one(), &box);
        }
    }

    /*
     * Set up the camera and that is it!
     */
    graphics.look_at(Vec3::new(0.0, 60.0, -60.0), Vec3::new(0.0, 0.0, 0.0));

    (world, detector, ccd, joints)
}
