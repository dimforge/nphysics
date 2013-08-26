#[link(name     = "compound2d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "f31fbf6a-b82c-4a8a-857a-12b2387f20ae")];
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
use ncollide::geom::compound::CompoundAABB;
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
    GraphicsManager::simulate(compound_2d)
}


pub fn compound_2d(graphics: &mut GraphicsManager) -> aliases::dim2::World2d<f64> {
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
     * Cross shaped geometry
     */
    let box1 = Box::new(Vec2::new(5.0f64, 0.75));
    let box2 = Box::new(Vec2::new(0.75f64, 5.0));

    let delta1 = One::one::<dim2::Transform2d<f64>>().translated(&Vec2::new(0.0, -5.0));
    let delta2 = One::one::<dim2::Transform2d<f64>>().translated(&Vec2::new(-5.0, 0.0));
    let delta3 = One::one::<dim2::Transform2d<f64>>().translated(&Vec2::new(5.0,  0.0));

    let mut cross_geoms = ~[];
    cross_geoms.push((delta1, DefaultGeom::new_box(box1)));
    cross_geoms.push((delta2, DefaultGeom::new_box(box2)));
    cross_geoms.push((delta3, DefaultGeom::new_box(box2)));

    let cross = @CompoundAABB::new(cross_geoms);

    /*
     * Create the boxes
     */
    let num     = (750.0f64.sqrt()) as uint;
    let rad     = 5.0;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f64) / 2.0;
    let centery = shift * (num as f64) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            let x = i as f64 * 2.5 * rad - centerx;
            let y = j as f64 * 2.5 * rad - centery * 2.0 - 250.0;

            let geom = DefaultGeom::new_compound(cross);
            let body = @mut RigidBody::new(geom, 1.0f64, Dynamic, 0.3, 0.6);

            body.translate_by(&Vec2::new(x, y));

            world.add_object(@mut body::RigidBody(body));
            graphics.add_cube(body, delta1, &box1);
            graphics.add_cube(body, delta2, &box2);
            graphics.add_cube(body, delta3, &box2);
        }
    }

    /*
     * The end.
     */
    world
}
