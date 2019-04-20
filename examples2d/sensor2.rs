extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Point2, Point3, Vector2};
use ncollide2d::query::Proximity;
use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};
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
     * Ground.
     */
    let ground_size = 10.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 1.0)));

    ColliderDesc::new(ground_shape)
        .translation(-Vector2::y())
        .build(&mut world);

    /*
     * Create some boxes.
     */
    let num = 15;
    let rad = 0.2;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let collider_desc = ColliderDesc::new(cuboid)
        .density(1.0);

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    let shift = (rad + collider_desc.get_margin()) * 2.0;
    let centerx = shift * (num as f32) / 2.0;

    for i in 0usize..num {
        let x = i as f32 * shift - centerx;

        // Build the rigid body and its collider.
        let handle = rb_desc
            .set_translation(Vector2::new(x, 2.0))
            .build(&mut world)
            .handle();

        testbed.set_body_color(handle, Point3::new(0.5, 0.5, 1.0));
    }

    /*
     * Create a box that will have a ball-shaped sensor attached.
     */
    let sensor_geom = ShapeHandle::new(Ball::new(rad * 5.0));
    // We create a collider desc without density because we don't
    // want it to contribute to the rigid body mass.
    let sensor_collider = ColliderDesc::new(sensor_geom)
        .sensor(true);


    let sensor_body = RigidBodyDesc::new()
        .collider(&collider_desc)
        .collider(&sensor_collider)
        .translation(Vector2::new(0.0, 4.0))
        .build(&mut world)
        .handle();

    testbed.set_body_color(sensor_body, Point3::new(0.5, 1.0, 1.0));

    // Callback that will be executed on the main loop to handle proximities.
    testbed.add_callback(move |world, graphics, _| {
        let world = world.get();
        for prox in world.proximity_events() {
            let color = match prox.new_status {
                Proximity::WithinMargin | Proximity::Intersecting => Point3::new(1.0, 1.0, 0.0),
                Proximity::Disjoint => Point3::new(0.5, 0.5, 1.0),
            };

            let body1 = world.collider(prox.collider1).unwrap().body();
            let body2 = world.collider(prox.collider2).unwrap().body();

            if !body1.is_ground() && body1 != sensor_body {
                graphics.set_body_color(body1, color);
            }

            if !body2.is_ground() && body2 != sensor_body {
                graphics.set_body_color(body2, color);
            }
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point2::origin(), 75.0);
}


fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}