extern crate env_logger;
extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use ncollide3d::world::CollisionGroups;
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    env_logger::init();
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
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry3::new(Vector3::y() * -ground_size, na::zero());

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
    let geom = ShapeHandle::new(Cuboid::new(Vector3::new(1.0, 0.1, 1.0)));
    let handle = world.add_collider(
        COLLIDER_MARGIN,
        geom,
        BodyHandle::ground(),
        Isometry3::new(Vector3::y() * 1.0, na::zero()),
        Material::default(),
    );

    world
        .collision_world_mut()
        .set_collision_groups(handle, green_group);

    testbed.set_collider_color(handle, Point3::new(0.0, 1.0, 0.0));

    /*
     * A blue floor that will collide with the BLUE group only.
     */
    let geom = ShapeHandle::new(Cuboid::new(Vector3::new(1.0, 0.1, 1.0)));
    let handle = world.add_collider(
        COLLIDER_MARGIN,
        geom,
        BodyHandle::ground(),
        Isometry3::new(Vector3::y() * 2.0, na::zero()),
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
    let centerz = shift * (num as f32) / 2.0;

    let geom = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad - COLLIDER_MARGIN)));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for k in 0usize..4 {
        for i in 0usize..num {
            for j in 0usize..num {
                let x = i as f32 * shift - centerx;
                let z = j as f32 * shift - centerz;
                let y = k as f32 * shift + centery;

                /*
                 * Create the rigid body.
                 */
                let pos = Isometry3::new(Vector3::new(x, y, z), na::zero());
                let body_handle = world.add_rigid_body(pos, inertia, center_of_mass);

                /*
                 * Create the collider.
                 */
                let collider_handle = world.add_collider(
                    COLLIDER_MARGIN,
                    geom.clone(),
                    body_handle,
                    Isometry3::identity(),
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
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point3::new(-4.0, 1.0, -4.0), Point3::new(0.0, 1.0, 0.0));
    testbed.run();
}
