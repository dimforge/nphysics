extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::world::World;
use nphysics3d::object::{ColliderDesc, RigidBodyDesc};
use nphysics_testbed3d::Testbed;


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Planes
     */
    let ground_size = 15.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size)));

    ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_size)
        .build(&mut world);

    /*
     * Create the boxes
     */
    let width = 50;
    let height = 10;
    let rad = 0.1;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    let collider_desc = ColliderDesc::new(cuboid)
        .density(1.0);

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    let shift = (rad + collider_desc.get_margin()) * 2.0;
    let centerx = shift * (width as f32) / 2.0;
    let centery = shift / 2.0;

    for i in 0usize..width {
        for j in 0usize..height {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery;

            // Build the rigid body and its collider.
            rb_desc
                .set_translation(Vector3::new(x, y, 0.0))
                .build(&mut world);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point3::new(-5.0, 5.0, -5.0), Point3::new(0.0, 0.0, 0.0));
}

fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}
