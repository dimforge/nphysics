#[link(name     = "wall2d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "dea0027e-7f5c-4fa0-9d04-3469f6836b20")];
#[crate_type = "bin"];
#[warn(non_camel_case_types)]

extern mod std;
extern mod extra;
extern mod rsfml;
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;
extern mod graphics2d;

use std::num::One;
use nalgebra::vec::{Vec2, Vec1};
use nalgebra::traits::vector::AlgebraicVec;
use nalgebra::traits::translation::Translation;
use ncollide::geom::plane::Plane;
use ncollide::geom::box::Box;
use ncollide::broad::dbvt_broad_phase::DBVTBroadPhase;
use nphysics::aliases;
use nphysics::object::body;
use nphysics::object::rigid_body::{RigidBody, Static, Dynamic};
use nphysics::object::implicit_geom::DefaultGeom;
use nphysics::world::world::World;
use nphysics::aliases::dim2;
use nphysics::integration::body_force_generator::BodyForceGenerator;
use nphysics::integration::rigid_body_integrator::RigidBodySmpEulerIntegrator;
use nphysics::integration::swept_ball_motion_clamping::SweptBallMotionClamping;
use nphysics::detection::collision::bodies_bodies::{BodiesBodies, Dispatcher};
use nphysics::detection::island_activation_manager::IslandActivationManager;
use nphysics::resolution::constraint::accumulated_impulse_solver::AccumulatedImpulseSolver;
use nphysics::resolution::constraint::contact_equation::VelocityAndPosition;
use nphysics::signal::signal::SignalEmiter;
use graphics2d::engine::GraphicsManager;


fn main() {
    GraphicsManager::simulate(wall_2d)
}


pub fn wall_2d(graphics: &mut GraphicsManager) -> aliases::dim2::World2d<f64> {
    /*
     * Setup the physics world
     */
    let mut world = World::new();

    // events handler
    let events = @mut SignalEmiter::new();

    // For the intergration
    let gravity = Vec2::new(0.0f64, 9.81);
    let tornado = Vec1::new(0.0f64);

    let forces: @mut dim2::ForceGenerator2d<f64> = BodyForceGenerator::new(events, gravity, tornado);
    let integrator: @mut dim2::RigidBodyIntegrator2d<f64> = RigidBodySmpEulerIntegrator::new(events);

    /*
     * For the collision detection
     */
    // Collision Dispatcher
    let dispatcher: dim2::Dispatcher2d<f64>  = Dispatcher::new();
    // Broad phase
    let broad_phase = @mut DBVTBroadPhase::new(dispatcher, 0.08f64);
    // CCD handler
    let ccd = SweptBallMotionClamping::new(events, broad_phase, true);
    // Collision detector
    let detector = BodiesBodies::new(events, broad_phase, false);
    // Deactivation
    let sleep = IslandActivationManager::new(events, 1.0, 0.01);

    /*
     * For constraints resolution
     */
    let solver: @mut dim2::ContactSolver2d<f64> =
        @mut AccumulatedImpulseSolver::new(0.1f64, VelocityAndPosition(0.2, 0.2, 0.08), 1.0, 10, 10);

    /*
     * Add everything to the world
     */
    world.add_integrator(forces);
    world.add_integrator(integrator);
    world.add_integrator(ccd);
    world.add_detector(detector);
    world.add_detector(sleep);
    world.add_solver(solver);

    /*
     * First plane
     */
    let geom = Plane::new(Vec2::new(0.0f64, -1.0).normalized());
    let body = @mut RigidBody::new(DefaultGeom::new_plane(geom), 0.0f64, Static, 0.3, 0.6);

    world.add_object(@mut body::RigidBody(body));
    graphics.add_plane(body, &geom);

    /*
     * Create the boxes
     */
    let width   = 100;
    let height  = 20;
    let rad     = 0.5;
    let shift   = 2.0 * rad;
    let centerx = shift * (width as f64) / 2.0;

    for i in range(0u, height) {
        for j in range(0u, width) {
            let fj = j as f64;
            let fi = i as f64;
            let x = fj * 2.0 * rad - centerx;
            let y = -fi * 2.0 * rad;

            let box  = Box::new(Vec2::new(rad, rad));
            let geom = DefaultGeom::new_box(box);
            let body = @mut RigidBody::new(geom, 1.0f64, Dynamic, 0.3, 0.6);

            body.translate_by(&Vec2::new(x, y));

            world.add_object(@mut body::RigidBody(body));
            graphics.add_cube(body, One::one(), &box);
        }
    }

    /*
     * The end.
     */
    world
}
