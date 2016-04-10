extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Pnt3, Vec3, Translation};
use ncollide::shape::{Plane, Cuboid};
use nphysics3d::world::World;
use nphysics3d::object::RigidBody;
use nphysics_testbed3d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    /*
     * Plane
     */
    let geom = Plane::new(Vec3::new(0.0, 1.0, 0.0));

    world.add_rigid_body(RigidBody::new_static(geom, 0.3, 0.6));

    /*
     * Create the boxes
     */
    let num     = 8;
    let rad     = 1.0;
    let shift   = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0 + 0.04;
    let centerz = shift * (num as f32) / 2.0;

    for i in 0usize .. num {
        for j in 0usize .. num {
            for k in 0usize .. num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                let geom   = Cuboid::new(Vec3::new(rad - 0.04, rad - 0.04, rad - 0.04));
                let mut rb = RigidBody::new_dynamic(geom, 1.0, 0.3, 0.5);

                rb.append_translation(&Vec3::new(x, y, z));

                world.add_rigid_body(rb);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Pnt3::new(-30.0, 30.0, -30.0), Pnt3::new(0.0, 0.0, 0.0));
    testbed.run();
}
