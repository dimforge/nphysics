extern crate native;
extern crate nalgebra;
extern crate "ncollide2df32" as ncollide;
extern crate "nphysics2df32" as nphysics;
extern crate nphysics_testbed2d;

use nalgebra::na::{Vec2, Translation};
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
    let num = 25;
    let rad = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;

    for i in range(0u, num) {
        for j in range(i, num) {
            let fj = j as f32;
            let fi = i as f32;
            let x = (fi * shift / 2.0) + (fj - fi) * 2.5 * rad - centerx;
            let y = -fi * 2.5 * rad - 0.04 - rad;

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
