#[link(name     = "balls_vee2d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "d8d15542-6eda-4969-8bcf-ca3f666d1d58")];
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
use ncollide::geom::ball::Ball;
use ncollide::broad::dbvt_broad_phase::DBVTBroadPhase;
use nphysics::aliases;
use nphysics::object::body;
use nphysics::object::rigid_body::{RigidBody, Static, Dynamic};
use nphysics::object::implicit_geom::DefaultGeom;
use nphysics::world::world::World;
use nphysics::aliases::dim2;
use nphysics::integration::body_force_generator::BodyForceGenerator;
use nphysics::integration::rigid_body_integrator::RigidBodySmpEulerIntegrator;
use nphysics::detection::collision::bodies_bodies::{BodiesBodies, Dispatcher};
use nphysics::detection::island_activation_manager::IslandActivationManager;
use nphysics::resolution::constraint::accumulated_impulse_solver::AccumulatedImpulseSolver;
use nphysics::resolution::constraint::contact_equation::VelocityAndPosition;
use graphics2d::engine::GraphicsManager;


fn main() {
    GraphicsManager::simulate(balls_vee_2d)
}


pub fn balls_vee_2d(graphics: &mut GraphicsManager) -> aliases::dim2::World2d<f64> {
    /*
     * Setup the physics world
     */
    let mut world = World::new();

    // For the intergration
    let gravity = Vec2::new(0.0, 9.81);
    let tornado = Vec1::new(0.0);

    let forces: @mut dim2::ForceGenerator2d<f64> = @mut BodyForceGenerator::new(gravity, tornado);
    let integrator: @mut dim2::RigidBodyIntegrator2d<f64> = @mut RigidBodySmpEulerIntegrator::new();

    /*
     * For the collision detection
     */
    // Collision Dispatcher
    let dispatcher: dim2::Dispatcher2d<f64> = Dispatcher::new();
    // Broad phase
    let broad_phase = @mut DBVTBroadPhase::new(dispatcher, 0.08f64);
    // Collision detector
    let detector = @mut BodiesBodies::new(broad_phase, true);

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
    world.add_detector(detector);
    world.add_solver(solver);

    // NOTE: this must be done _after_ the addition of every other controllers
    let sleep: @mut dim2::IslandActivationManager2d<f64> =
        @mut IslandActivationManager::new(1.0, 0.01, &mut world);
    world.add_detector(sleep);

    /*
     * First plane
     */
    let geom = Plane::new(Vec2::new(-1.0f64, -1.0).normalized());
    let body = @mut RigidBody::new(DefaultGeom::new_plane(geom), 0.0f64, Static, 0.3, 0.6);

    body.translate_by(&Vec2::new(0.0, 10.0));

    world.add_object(@mut body::RigidBody(body));
    graphics.add_plane(body, &geom);

    /*
     * Second plane
     */
    let geom = Plane::new(Vec2::new(1.0f64, -1.0).normalized());
    let body = @mut RigidBody::new(DefaultGeom::new_plane(geom), 0.0f64, Static, 0.3, 0.6);

    body.translate_by(&Vec2::new(0.0, 10.0));

    world.add_object(@mut body::RigidBody(body));
    graphics.add_plane(body, &geom);

    /*
     * Create the balls
     */
    let num     = (4000.0f64.sqrt()) as uint;
    let rad     = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f64) / 2.0;
    let centery = shift * (num as f64) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            let x = i as f64 * 2.5 * rad - centerx;
            let y = j as f64 * 2.5 * rad - centery * 2.0 - 20.0;

            let ball = Ball::new(rad);
            let geom = DefaultGeom::new_ball(ball);
            let body = @mut RigidBody::new(geom, 1.0f64, Dynamic, 0.3, 0.6);

            body.translate_by(&Vec2::new(x, y));

            world.add_object(@mut body::RigidBody(body));
            graphics.add_ball(body, One::one(), &ball);
        }
    }

    /*
     * The end.
     */
    world
}
