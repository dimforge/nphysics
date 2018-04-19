extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::f32::consts::PI;
use na::{Isometry3, Point3, Translation3, Vector3};
use ncollide3d::query::Proximity;
use ncollide3d::shape::{Ball, Cuboid, Plane, ShapeHandle};
use nphysics3d::world::World;
use nphysics3d::object::BodyHandle;
use nphysics3d::joint::FreeJoint;
use nphysics3d::volumetric::Volumetric;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    let mut testbed = Testbed::new_empty();

    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Ground.
     */
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(50.0, 50.0, 50.0)));
    let ground_pos = Isometry3::new(Vector3::y() * -50.0, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
    );

    /*
     * Create some boxes.
     */
    let num = 10;
    let rad = 0.2;
    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    let geom = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad - COLLIDER_MARGIN)));
    let inertia = geom.inertia(1.0);

    for i in 0usize..num {
        for k in 0usize..num {
            let x = i as f32 * shift - centerx;
            let z = k as f32 * shift - centerz;

            /*
             * Create the rigid body.
             */
            let pos = Isometry3::new(Vector3::new(x, 3.0, z), na::zero());
            let handle = world.add_rigid_body(pos, inertia);

            /*
             * Create the collider.
             */
            world.add_collider(COLLIDER_MARGIN, geom.clone(), handle, Isometry3::identity());
            testbed.set_body_color(&world, handle, Point3::new(0.5, 0.5, 1.0));
        }
    }

    /*
     * Create a box that will have a sensor attached.
     */
    let pos = Isometry3::new(Vector3::new(0.0, 10.0, 0.0), na::zero());
    let sensor_body = world.add_rigid_body(pos, inertia);
    let sensor_geom = ShapeHandle::new(Ball::new(rad * 5.0));

    world.add_collider(COLLIDER_MARGIN, geom, sensor_body, Isometry3::identity());
    world.add_sensor(sensor_geom, sensor_body, Isometry3::identity());
    testbed.set_body_color(&world, sensor_body, Point3::new(0.5, 1.0, 1.0));

    // Callback that will be executed on the main loop to handle proximities.
    let graphics = testbed.graphics();
    testbed.add_callback(move |world, _| {
        for prox in world.proximity_events() {
            let color = match prox.new_status {
                Proximity::WithinMargin | Proximity::Intersecting => Point3::new(1.0, 1.0, 0.0),
                Proximity::Disjoint => Point3::new(0.5, 0.5, 1.0),
            };

            let body1 = world.collider(prox.collider1).unwrap().data().body();
            let body2 = world.collider(prox.collider2).unwrap().data().body();

            if !body1.is_ground() && body1 != sensor_body {
                graphics.borrow_mut().set_body_color(world, body1, color);
            }
            if !body2.is_ground() && body2 != sensor_body {
                graphics.borrow_mut().set_body_color(world, body2, color);
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point3::new(-8.0, 8.0, -8.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
