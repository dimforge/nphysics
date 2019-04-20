extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Point3, Vector2};
use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics2d::force_generator::ConstantAcceleration;
use nphysics2d::object::{RigidBodyDesc, ColliderDesc};
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = World::new();

    // We setup two force generators that will replace the gravity.
    let mut up_gravity = ConstantAcceleration::new(Vector2::y() * -9.81, 0.0);
    let mut down_gravity = ConstantAcceleration::new(Vector2::y() * 9.81, 0.0);

    /*
     * Grouds
     */
    let ground_radx = 25.0;
    let ground_rady = 1.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        ground_radx,
        ground_rady,
    )));

    let mut ground_desc = ColliderDesc::new(ground_shape);

    ground_desc
        .set_translation(-Vector2::y() * 2.0)
        .build(&mut world);

    ground_desc
        .set_translation(Vector2::y() * 3.0)
        .build(&mut world);

    /*
     * Create the balls
     */
    let num = 100f64 as usize;
    let rad = 0.2;
    let shift = 2.0 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = rad * 4.0;

    let geom = ShapeHandle::new(Ball::new(rad));
    let collider_desc = ColliderDesc::new(geom)
        .density(1.0);
    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    for i in 0usize..num {
        for j in 0usize..2 {
            let x = i as f32 * 2.5 * rad - centerx;
            let y = j as f32 * 2.5 * -rad + centery;

            // Crate the rigid body and its collider.
            let rb_handle = rb_desc
                .set_translation(Vector2::new(x, y))
                .build(&mut world)
                .part_handle();

            /*
             * Set artifical gravity.
             */
            let color;

            if j == 1 {
                up_gravity.add_body_part(rb_handle);
                color = Point3::new(0.0, 0.0, 1.0);
            } else {
                down_gravity.add_body_part(rb_handle);
                color = Point3::new(0.0, 1.0, 0.0);
            }

            testbed.set_body_color(rb_handle.0, color);
        }
    }

    world.add_force_generator(up_gravity);
    world.add_force_generator(down_gravity);

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
}


fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}