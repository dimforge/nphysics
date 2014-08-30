extern crate native;
extern crate nalgebra;
extern crate ncollide = "ncollide3df32";
extern crate nphysics = "nphysics3df32";
extern crate nphysics_testbed3d;

use std::rc::Rc;
use std::cell::RefCell;
use nalgebra::na::{Vec3, Translation};
use ncollide::geom::{Plane, Cuboid};
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics_testbed3d::Testbed;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    /*
     * Planes
     */
    let rb   = RigidBody::new_static(Plane::new(Vec3::new(0.0f32, 1.0, 0.0)), 0.3, 0.6);
    let body = Rc::new(RefCell::new(rb));

    world.add_body(body.clone());

    /*
     * Create the boxes
     */
    let width   = 50;
    let height  = 10;
    let rad     = 1.0;
    let shift   = rad * 2.0;
    let centerx = shift * (width as f32) / 2.0;
    let centery = shift / 2.0;

    for i in range(0u, width) {
        for j in range(0u, height) {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery;

            let mut rb = RigidBody::new_dynamic(Cuboid::new(Vec3::new(rad, rad, rad)), 1.0f32, 0.3, 0.5);

            rb.append_translation(&Vec3::new(x, y, 0.0));

            let body = Rc::new(RefCell::new(rb));

            world.add_body(body.clone());
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Vec3::new(-30.0, 30.0, -30.0), Vec3::new(0.0, 0.0, 0.0));
    testbed.run();
}
