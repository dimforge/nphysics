extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Point2, Point3, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use ncollide2d::world::CollisionGroups;
use nphysics2d::object::{ColliderDesc, RigidBodyDesc};
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

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
    let ground_radx = 5.0;
    let ground_rady = 1.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        ground_radx,
        ground_rady,
    )));

    ColliderDesc::new(ground_shape)
        .translation(-Vector2::y() * ground_rady)
        .build(&mut world);

    /*
     * A green floor that will collide with the GREEN group only.
     */
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(1.0, 0.1)));

    let collider_handle = ColliderDesc::new(ground_shape.clone())
        .translation(Vector2::y())
        .collision_groups(green_group)
        .build(&mut world)
        .handle();

    testbed.set_collider_color(collider_handle, Point3::new(0.0, 1.0, 0.0));

    /*
     * A blue floor that will collide with the BLUE group only.
     */
    let collider_handle = ColliderDesc::new(ground_shape)
        .translation(Vector2::y() * 2.0)
        .collision_groups(blue_group)
        .build(&mut world)
        .handle();

    testbed.set_collider_color(collider_handle, Point3::new(0.0, 0.0, 1.0));

    /*
     * Create the boxes
     */
    let num = 8;
    let rad = 0.1;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let shift = (rad + ColliderDesc::<f32>::default_margin()) * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = 2.5;

    for k in 0usize..4 {
        for i in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = k as f32 * shift + centery;

            // Alternate between the GREEN and BLUE groups.
            let (group, color) = if k % 2 == 0 {
                (green_group, Point3::new(0.0, 1.0, 0.0))
            } else {
                (blue_group, Point3::new(0.0, 0.0, 1.0))
            };

            // Build the rigid body and its collider.
            let collider_desc = ColliderDesc::new(cuboid.clone())
                .density(1.0)
                .collision_groups(group);

            let body_handle = RigidBodyDesc::new()
                .collider(&collider_desc)
                .translation(Vector2::new(x, y))
                .build(&mut world)
                .handle();

            testbed.set_body_color(body_handle, color);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point2::new(0.0, -1.0), 100.0);
}


fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}