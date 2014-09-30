extern crate native;
extern crate "nalgebra" as na;
extern crate ncollide;
extern crate "nphysics2df32" as nphysics;
extern crate nphysics_testbed2d;

use na::{Vec2, Translation};
use ncollide::geom::{Plane, Cuboid};
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics_testbed2d::Testbed;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec2::new(0.0f32, 9.81));

    /*
     * First plane
     */
    let rb = RigidBody::new_static(Plane::new(Vec2::new(0.0f32, -1.0)), 0.3, 0.6);

    world.add_body(rb);

    /*
     * Create the boxes
     */
    let width   = 100;
    let height  = 20;
    let rad     = 0.5;
    let shift   = 2.0 * rad;
    let centerx = shift * (width as f32) / 2.0;

    for i in range(0u, height) {
        for j in range(0u, width) {
            let fj = j as f32;
            let fi = i as f32;
            let x  = fj * 2.0 * rad - centerx;
            let y  = -fi * 2.0 * rad - 0.04 - rad;

            let mut rb = RigidBody::new_dynamic(Cuboid::new(Vec2::new(rad - 0.04, rad - 0.04)), 1.0f32, 0.3, 0.6);

            rb.append_translation(&Vec2::new(x, y));

            world.add_body(rb);
        }
    }

    /*
     * Run the simulation.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}
