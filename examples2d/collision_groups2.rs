extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Point3, Vector2};
use ncollide2d::world::CollisionGroups;
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::world::World;
use nphysics2d::object::{BodyHandle, Material};
use nphysics2d::volumetric::Volumetric;
use nphysics_testbed2d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    let mut testbed = Testbed::new_empty();

    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, 9.81));

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
        ground_radx - COLLIDER_MARGIN,
        ground_rady - COLLIDER_MARGIN,
    )));

    let ground_pos = Isometry2::new(Vector2::y() * ground_rady, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * A green floor that will collide with the GREEN group only.
     */
    let geom = ShapeHandle::new(Cuboid::new(Vector2::new(1.0, 0.1)));
    let handle = world.add_collider(
        COLLIDER_MARGIN,
        geom,
        BodyHandle::ground(),
        Isometry2::new(Vector2::y() * -1.0, na::zero()),
        Material::default(),
    );

    world
        .collision_world_mut()
        .set_collision_groups(handle, green_group);

    testbed.set_collider_color(handle, Point3::new(0.0, 1.0, 0.0));

    /*
     * A blue floor that will collide with the BLUE group only.
     */
    let geom = ShapeHandle::new(Cuboid::new(Vector2::new(1.0, 0.1)));
    let handle = world.add_collider(
        COLLIDER_MARGIN,
        geom,
        BodyHandle::ground(),
        Isometry2::new(Vector2::y() * -2.0, na::zero()),
        Material::default(),
    );

    world
        .collision_world_mut()
        .set_collision_groups(handle, blue_group);

    testbed.set_collider_color(handle, Point3::new(0.0, 0.0, 1.0));

    /*
     * Create the boxes
     */
    let num = 8;
    let rad = 0.1;
    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = 2.5;

    let geom = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad - COLLIDER_MARGIN)));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for k in 0usize..4 {
        for i in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = k as f32 * shift + centery;

            /*
             * Create the rigid body.
             */
            let pos = Isometry2::new(Vector2::new(x, -y), na::zero());
            let body_handle = world.add_rigid_body(pos, inertia, center_of_mass);

            /*
             * Create the collider.
             */
            let collider_handle = world.add_collider(
                COLLIDER_MARGIN,
                geom.clone(),
                body_handle,
                Isometry2::identity(),
                Material::default(),
            );

            // Alternate between the GREEN and BLUE groups.
            if k % 2 == 0 {
                world
                    .collision_world_mut()
                    .set_collision_groups(collider_handle, green_group);
                testbed.set_body_color(&world, body_handle, Point3::new(0.0, 1.0, 0.0));
            } else {
                world
                    .collision_world_mut()
                    .set_collision_groups(collider_handle, blue_group);
                testbed.set_body_color(&world, body_handle, Point3::new(0.0, 0.0, 1.0));
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.run();
}
