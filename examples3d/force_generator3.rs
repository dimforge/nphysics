extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Point3, Vector3};
use ncollide3d::shape::{Ball, Plane, ShapeHandle};
use nphysics3d::world::World;
use nphysics3d::force_generator::ConstantAcceleration;
use nphysics3d::object::{ColliderDesc, RigidBodyDesc};
use nphysics_testbed3d::Testbed;


fn main() {
    let mut testbed = Testbed::new_empty();

    /*
     * World
     */
    let mut world = World::new();

    // We setup two force generators that will replace the gravity.
    let mut up_gravity = ConstantAcceleration::new(Vector3::y() * 9.81, Vector3::zeros());
    let mut down_gravity = ConstantAcceleration::new(Vector3::y() * -9.81, Vector3::zeros());

    /*
     * Planes
     */
    let plane = ShapeHandle::new(Plane::new(Vector3::y_axis()));

    ColliderDesc::new(plane)
        .build(&mut world);

    let plane = ShapeHandle::new(Plane::new(-Vector3::y_axis()));

    ColliderDesc::new(plane)
        .translation(Vector3::y() * 20.0)
        .build(&mut world);

    /*
     * Create the balls
     */
    let num = 1000f64.sqrt() as usize;
    let rad = 0.1;
    let shift = 0.25 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = 0.5;

    let ball = ShapeHandle::new(Ball::new(rad));
    let collider_desc = ColliderDesc::new(ball)
        .density(1.0);

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    for i in 0usize..num {
        for j in 0usize..2 {
            for k in 0usize..num {
                let x = i as f32 * 2.5 * rad - centerx;
                let y = 1.0 + j as f32 * 2.5 * rad + centery;
                let z = k as f32 * 2.5 * rad - centerx;

                // Build the rigid body and its collider.
                let rb_handle = rb_desc
                    .set_translation(Vector3::new(x, y, z))
                    .build(&mut world);
                
                let rb_handle = world.rigid_body(rb_handle).unwrap().part_handle();

                /*
                 * Set artificial gravity.
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
    }

    /*
     * Add the force generators to the world.
     */
    world.add_force_generator(up_gravity);
    world.add_force_generator(down_gravity);

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point3::new(-1.0, 5.0, -1.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
