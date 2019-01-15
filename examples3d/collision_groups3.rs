extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use ncollide3d::world::CollisionGroups;
use nphysics3d::object::{ColliderDesc, RigidBodyDesc};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;


fn main() {
    let mut testbed = Testbed::new_empty();

    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Setup groups.
     */
    const GREEN_GROUP_ID: usize = 0;
    let mut green_group = CollisionGroups::new();
    green_group.set_membership(&[GREEN_GROUP_ID]);
    green_group.set_whitelist(&[GREEN_GROUP_ID]);

    const BLUE_GROUP_ID: usize = 1;
    let mut blue_group = CollisionGroups::new();
    blue_group.set_membership(&[BLUE_GROUP_ID]);
    blue_group.set_whitelist(&[BLUE_GROUP_ID]);

    /*
     * A floor that will collide with everything (default behaviour).
     */
    let ground_size = 5.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size)));

    ColliderDesc::new(ground_shape)
        .with_translation(Vector3::y() * -ground_size)
        .build(&mut world);

    /*
     * A green floor that will collide with the GREEN group only.
     */
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(1.0, 0.1, 1.0)));
    let collider_handle = ColliderDesc::new(ground_shape.clone())
        .with_translation(Vector3::y())
        .with_collision_groups(green_group)
        .build(&mut world)
        .handle();

    testbed.set_collider_color(collider_handle, Point3::new(0.0, 1.0, 0.0));

    /*
     * A blue floor that will collide with the BLUE group only.
     */
    let collider_handle = ColliderDesc::new(ground_shape)
        .with_translation(Vector3::y() * 2.0)
        .with_collision_groups(blue_group)
        .build(&mut world)
        .handle();

    testbed.set_collider_color(collider_handle, Point3::new(0.0, 0.0, 1.0));

    /*
     * Create the boxes
     */
    let num = 8;
    let rad = 0.1;
    let shift = (rad + ColliderDesc::<f32>::default_margin()) * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = 2.5;
    let centerz = shift * (num as f32) / 2.0;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));

    for k in 0usize..4 {
        for i in 0usize..num {
            for j in 0usize..num {
                let x = i as f32 * shift - centerx;
                let z = j as f32 * shift - centerz;
                let y = k as f32 * shift + centery;

                // Alternate between the GREEN and BLUE groups.
                let (group, color) = if k % 2 == 0 {
                    (green_group, Point3::new(0.0, 1.0, 0.0))
                } else {
                    (blue_group, Point3::new(0.0, 0.0, 1.0))
                };

                // Build the rigid body and its collider.
                let collider_desc = ColliderDesc::new(cuboid.clone())
                    .with_density(1.0)
                    .with_collision_groups(group);

                let body_handle = RigidBodyDesc::new()
                    .with_collider(&collider_desc)
                    .with_translation(Vector3::new(x, y, z))
                    .build(&mut world)
                    .handle();

                testbed.set_body_color(body_handle, color);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point3::new(-4.0, 1.0, -4.0), Point3::new(0.0, 1.0, 0.0));
    testbed.run();
}
