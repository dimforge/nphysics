extern crate native;
extern crate nalgebra;
extern crate ncollide = "ncollide2df32";
extern crate nphysics = "nphysics2df32";
extern crate nphysics_testbed2d;

use std::rc::Rc;
use std::cell::RefCell;
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
    let mut rb = RigidBody::new_static(Plane::new(Vec2::new(-1.0f32, -1.0)), 0.3, 0.6);

    rb.append_translation(&Vec2::new(0.0, 10.0));

    let body = Rc::new(RefCell::new(rb));

    world.add_body(body.clone());

    /*
     * Second plane
     */
    let mut rb = RigidBody::new_static(Plane::new(Vec2::new(1.0f32, -1.0)), 0.3, 0.6);

    rb.append_translation(&Vec2::new(0.0, 10.0));

    let body = Rc::new(RefCell::new(rb));

    world.add_body(body.clone());

    /*
     * Create the boxes
     */
    let num = (1000.0f32.sqrt()) as uint;
    let rad = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            let x = i as f32 * 2.5 * rad - centerx;
            let y = j as f32 * 2.5 * rad - centery * 2.0 - 10.0;

            let geom   = Cuboid::new(Vec2::new(rad, rad));
            let mut rb = RigidBody::new_dynamic(geom, 1.0f32, 0.3, 0.6);

            rb.append_translation(&Vec2::new(x, y));

            let body = Rc::new(RefCell::new(rb));

            world.add_body(body.clone());
        }
    }

    /*
     * Run the simulation.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}
