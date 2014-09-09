extern crate native;
extern crate nalgebra;
extern crate "ncollide3df32" as ncollide;
extern crate "nphysics3df32" as nphysics;
extern crate nphysics_testbed3d;

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
    let rb = RigidBody::new_static(Plane::new(Vec3::new(0.0f32, 1.0, 0.0)), 0.3, 0.6);

    world.add_body(rb);

    /*
     * Create the boxes
     */
    let num     = 30;
    let rad     = 0.5;
    let shift   = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0 + 0.04;

    for i in range(0u, num) {
        for j in range(i, num) {
            let fi = i as f32;
            let fj = (j - i) as f32;
            let x = (fi * shift / 2.0) + fj * shift - centerx;
            let y = fi * shift + centery;

            let mut rb = RigidBody::new_dynamic(Cuboid::new(Vec3::new(rad - 0.04, rad - 0.04, rad - 0.04)), 1.0f32, 0.3, 0.6);

            rb.append_translation(&Vec3::new(x, y, 0.0));

            world.add_body(rb);
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Vec3::new(-30.0, 30.0, -30.0), Vec3::new(0.0, 0.0, 0.0));
    testbed.run();
}
