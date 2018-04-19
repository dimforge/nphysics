extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Vector2, Translation2};
use ncollide2d::shape::{Plane, Cuboid};
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
    let num = 25;
    let rad = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;

    for i in 0usize .. num {
        for j in i .. num {
            let fj = j as f32;
            let fi = i as f32;
            let x = (fi * shift / 2.0) + (fj - fi) * 2.5 * rad - centerx;
            let y = -fi * 2.5 * rad - 0.04 - rad;

            let mut rb = RigidBody::new_dynamic(Cuboid::new(Vector2::new(rad - 0.04, rad - 0.04)), 1.0, 0.3, 0.6);

            rb.append_translation(&Translation2::new(x, y));

            world.add_rigid_body(rb);
        }
    }

    /*
     * Run the simulation.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}
