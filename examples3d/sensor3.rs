extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Point3, Vector3};
use ncollide3d::query::Proximity;
use ncollide3d::shape::{Ball, Cuboid, ShapeHandle};
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
     * Ground.
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size)));

    ColliderDesc::new(ground_shape)
        .with_translation(Vector3::y() * -ground_size)
        .build(&mut world);

    /*
     * Create some boxes.
     */
    let num = 10;
    let rad = 0.2;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    let collider_desc = ColliderDesc::new(cuboid)
        .with_density(1.0);

    let mut rb_desc = RigidBodyDesc::new()
        .with_collider(&collider_desc);

    let shift = (rad + collider_desc.margin()) * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    for i in 0usize..num {
        for k in 0usize..num {
            let x = i as f32 * shift - centerx;
            let z = k as f32 * shift - centerz;

            // Build the rigid body and its collider.
            let handle = rb_desc
                .set_translation(Vector3::new(x, 3.0, z))
                .build(&mut world)
                .handle();
            testbed.set_body_color(handle, Point3::new(0.5, 0.5, 1.0));
        }
    }

    /*
     * Create a ball that will have a ball-shaped sensor attached.
     */
    let sensor_geom = ShapeHandle::new(Ball::new(rad * 5.0));
    // We create a collider desc without density because we don't
    // want it to contribute to the rigid body mass.
    let sensor_collider = ColliderDesc::new(sensor_geom)
        .as_sensor(true);

    let sensor_handle = RigidBodyDesc::new()
        .with_collider(&collider_desc)
        .with_collider(&sensor_collider)
        .with_translation(Vector3::new(0.0, 10.0, 0.0))
        .build(&mut world)
        .handle();

    testbed.set_body_color(sensor_handle, Point3::new(0.5, 1.0, 1.0));


    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |world, graphics, _| {
        for prox in world.proximity_events() {
            let color = match prox.new_status {
                Proximity::WithinMargin | Proximity::Intersecting => Point3::new(1.0, 1.0, 0.0),
                Proximity::Disjoint => Point3::new(0.5, 0.5, 1.0),
            };

            let body_handle1 = world.collider(prox.collider1).unwrap().body();
            let body_handle2 = world.collider(prox.collider2).unwrap().body();

            if !body_handle1.is_ground() && body_handle1 != sensor_handle {
                graphics.set_body_color(body_handle1, color);
            }
            if !body_handle2.is_ground() && body_handle2 != sensor_handle {
                graphics.set_body_color(body_handle2, color);
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point3::new(-6.0, 4.0, -6.0), Point3::new(0.0, 1.0, 0.0));
    testbed.run();
}
