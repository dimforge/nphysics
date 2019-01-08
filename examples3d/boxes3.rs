extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::object::{BodyPartHandle, Material, BodyPart, RigidBody, RigidBodyDesc, ColliderDesc};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::{World, ColliderWorld};
use nphysics_testbed3d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Ground.
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size)));

    ColliderDesc::new(ground_shape)
        .with_translation(Vector3::y() * -ground_size)
        .build(&mut world);

    /*
     * Create the boxes
     */
    let num = 10;
    let rad = 0.1;
    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;
    let height = 3.0;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    let mut collider_desc = ColliderDesc::new(cuboid)
        .with_density(Some(1.0));

    let mut rb_desc = RigidBodyDesc::default()
        .with_collider(&collider_desc);

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * (shift + collider_desc.margin()) - centerx;
                let y = j as f32 * (shift + collider_desc.margin()) + centery + height;
                let z = k as f32 * (shift + collider_desc.margin()) - centerz;

                // Build the rigid body and its collider.
                let rb = rb_desc
                    .set_translation(Vector3::new(x, y, z))
                    .build(&mut world);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    // testbed.hide_performance_counters();
    testbed.look_at(Point3::new(-4.0, 1.0, -4.0), Point3::new(0.0, 1.0, 0.0));
    testbed.run();
}
