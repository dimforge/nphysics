extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::Vector2;
use ncollide::shape::{Plane, Cuboid};
use nphysics2d::world::World;
use nphysics2d::object::RigidBody;
use nphysics_testbed2d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, 9.81));

    /*
     * First plane
     */
    let rb = RigidBody::new_static(Plane::new(Vector2::new(0.0, -1.0)), 0.3, 0.6);

    world.add_rigid_body(rb);

    /*
     * Create the boxes
     */
    let width   = 100;
    let height  = 20;
    let rad     = 0.5;
    let shift   = 2.0 * rad;
    let centerx = shift * (width as f32) / 2.0;

    for i in 0usize .. height {
        for j in 0usize .. width {
            let fj = j as f32;
            let fi = i as f32;
            let x  = fj * 2.0 * rad - centerx;
            let y  = -fi * 2.0 * rad - 0.04 - rad;

            let mut rb = RigidBody::new_dynamic(Cuboid::new(Vector2::new(rad - 0.04, rad - 0.04)), 1.0, 0.3, 0.6);

            rb.append_translation(&Vector2::new(x, y));

            world.add_rigid_body(rb);
        }
    }

    /*
     * Run the simulation.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}
