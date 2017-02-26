extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Point3, Vector3, Translation3};
use ncollide::shape::{Ball, Plane};
use nphysics3d::world::World;
use nphysics3d::object::RigidBody;
use nphysics_testbed3d::Testbed;

fn main() {
    let mut testbed = Testbed::new_empty();

    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Planes
     */
    let geom = Plane::new(Vector3::new(0.0, 1.0, 0.0));

    world.add_rigid_body(RigidBody::new_static(geom, 0.3, 0.6));

    let geom   = Plane::new(Vector3::new(0.0, -1.0, 0.0));
    let mut rb = RigidBody::new_static(geom, 0.3, 0.6);

    rb.append_translation(&Translation3::new(0.0, 50.0, 0.0));

    world.add_rigid_body(rb);

    /*
     * Create the balls
     */

    let num     = 1000f64.sqrt() as usize;
    let rad     = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = 5.0;

    for i in 0usize .. num {
        for j in 0usize .. 2 {
            for k in 0usize .. num {
                let x = i as f32 * 2.5 * rad - centerx;
                let y = 10.0 + j as f32 * 2.5 * rad + centery;
                let z = k as f32 * 2.5 * rad - centerx;

                let mut rb = RigidBody::new_dynamic(Ball::new(rad), 1.0, 0.3, 0.6);

                rb.append_translation(&Translation3::new(x, y, z));

                let color;

                if j == 1 {
                    // Invert the gravity for the blue balls.
                    rb.set_lin_acc_scale(Vector3::new(0.0, -1.0, 0.0));
                    color = Point3::new(0.0, 0.0, 1.0);
                }
                else {
                    // Double the gravity for the green balls.
                    rb.set_lin_acc_scale(Vector3::new(0.0, 2.0, 0.0));
                    color = Point3::new(0.0, 1.0, 0.0);
                }

                let rb_handle = world.add_rigid_body(rb);
                testbed.set_rigid_body_color(&rb_handle, color);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point3::new(-10.0, 50.0, -10.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
