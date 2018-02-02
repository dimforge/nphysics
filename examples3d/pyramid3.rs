extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Point3, Vector3, Translation3};
use ncollide::shape::{Plane, Cuboid};
use nphysics3d::world::World;
use nphysics3d::object::RigidBody;
use nphysics_testbed3d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Planes
     */
    let rb = RigidBody::new_static(Plane::new(Vector3::new(0.0, 1.0, 0.0)), 0.3, 0.6);

    world.add_rigid_body(rb);

    /*
     * Create the boxes
     */
    let num     = 30;
    let rad     = 0.5;
    let shift   = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0 + 0.04;

    for i in 0usize .. num {
        for j in i .. num {
            let fi = i as f32;
            let fj = (j - i) as f32;
            let x = (fi * shift / 2.0) + fj * shift - centerx;
            let y = fi * shift + centery;

            let mut rb = RigidBody::new_dynamic(Cuboid::new(Vector3::new(rad - 0.01, rad - 0.01, rad - 0.01)), 1.0, 0.3, 0.6);

            rb.append_translation(&Translation3::new(x, y, 0.0));

            world.add_rigid_body(rb);
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(-30.0, 30.0, -30.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
