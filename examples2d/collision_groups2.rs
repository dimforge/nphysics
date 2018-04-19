extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Point3, Vector2, Translation2};
use ncollide2d::shape::{Plane, Cuboid};
use nphysics2d::world::World;
use nphysics2d::object::{RigidBody, RigidBodyCollisionGroups};
use nphysics_testbed2d::Testbed;

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
    let mut green_static_group  = RigidBodyCollisionGroups::new_static();
    green_static_group.set_membership(&[ GREEN_GROUP_ID ]);
    green_static_group.set_whitelist(&[ GREEN_GROUP_ID ]);

    let mut green_dynamic_group = RigidBodyCollisionGroups::new_dynamic();
    green_dynamic_group.set_membership(&[ GREEN_GROUP_ID ]);
    green_dynamic_group.set_whitelist(&[ GREEN_GROUP_ID ]);


    const BLUE_GROUP_ID: usize = 1;
    let mut blue_static_group = RigidBodyCollisionGroups::new_static();
    blue_static_group.set_membership(&[ BLUE_GROUP_ID ]);
    blue_static_group.set_whitelist(&[ BLUE_GROUP_ID ]);

    let mut blue_dynamic_group = RigidBodyCollisionGroups::new_dynamic();
    blue_dynamic_group.set_membership(&[ BLUE_GROUP_ID ]);
    blue_dynamic_group.set_whitelist(&[ BLUE_GROUP_ID ]);

    /*
     * A floor that will collide with everything (default behaviour).
     */
    let geom = Plane::new(Vector2::new(0.0, -1.0));
    world.add_rigid_body(RigidBody::new_static(geom, 0.3, 0.6));

    /*
     * A green floor that will collide with the GREEN group only.
     */
    let geom   = Cuboid::new(Vector2::new(10.0, 1.0));
    let mut rb = RigidBody::new_static(geom, 0.3, 0.6);

    rb.set_collision_groups(green_static_group);
    rb.append_translation(&Translation2::new(0.0, -5.0));

    let handle = world.add_rigid_body(rb);
    testbed.set_rigid_body_color(&handle, Point3::new(0.0, 1.0, 0.0));

    /*
     * A blue floor that will collide with the BLUE group only.
     */
    let geom   = Cuboid::new(Vector2::new(10.0, 1.0));
    let mut rb = RigidBody::new_static(geom, 0.3, 0.6);

    rb.set_collision_groups(blue_static_group);
    rb.append_translation(&Translation2::new(0.0, -10.0));

    let handle = world.add_rigid_body(rb);
    testbed.set_rigid_body_color(&handle, Point3::new(0.0, 0.0, 1.0));

    /*
     * Create the boxes
     */
    let num     = 8;
    let rad     = 1.0;
    let shift   = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = 25.0;

    for k in 0usize .. 4 {
        for i in 0usize .. num {
            let x = i as f32 * shift - centerx;
            let y = k as f32 * shift + centery;

            let geom   = Cuboid::new(Vector2::new(rad - 0.04, rad - 0.04));
            let mut rb = RigidBody::new_dynamic(geom, 1.0, 0.3, 0.5);

            rb.append_translation(&Translation2::new(x, -y));

            // Alternate between the GREEN and BLUE groups.
            if k % 2 == 0 {
                rb.set_collision_groups(green_dynamic_group);
                let handle = world.add_rigid_body(rb);
                testbed.set_rigid_body_color(&handle, Point3::new(0.0, 1.0, 0.0));
            }
            else {
                rb.set_collision_groups(blue_dynamic_group);
                let handle = world.add_rigid_body(rb);
                testbed.set_rigid_body_color(&handle, Point3::new(0.0, 0.0, 1.0));
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.run();
}
