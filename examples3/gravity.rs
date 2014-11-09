extern crate native;
extern crate "nalgebra" as na;
extern crate ncollide;
extern crate nphysics;
extern crate nphysics_testbed3d;

use na::{Pnt3, Vec3, Translation};
use ncollide::shape::{Ball, Plane};
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics_testbed3d::Testbed;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    let mut testbed = Testbed::new_empty();

    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    /*
     * Planes
     */
    let geom = Plane::new(Vec3::new(0.0, 1.0, 0.0));

    world.add_body(RigidBody::new_static(geom, 0.3, 0.6));

    let geom   = Plane::new(Vec3::new(0.0, -1.0, 0.0));
    let mut rb = RigidBody::new_static(geom, 0.3, 0.6);

    rb.append_translation(&Vec3::new(0.0, 50.0, 0.0));

    world.add_body(rb);

    /*
     * Create the balls
     */

    let num     = 1000f64.sqrt() as uint;
    let rad     = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = 5.0;

    for i in range(0u, num) {
        for j in range(0u, 2) {
            for k in range(0u, num) {
                let x = i as f32 * 2.5 * rad - centerx;
                let y = 10.0 + j as f32 * 2.5 * rad + centery;
                let z = k as f32 * 2.5 * rad - centerx;

                let mut rb = RigidBody::new_dynamic(Ball::new(rad), 1.0, 0.3, 0.6);

                rb.append_translation(&Vec3::new(x, y, z));

                let color;

                if j == 1 {
                    // invert the gravity for the blue balls.
                    rb.set_lin_acc_scale(Vec3::new(0.0, -1.0, 0.0));
                    color = Pnt3::new(0.0, 0.0, 1.0);
                }
                else {
                    // double the gravity for the green balls.
                    rb.set_lin_acc_scale(Vec3::new(0.0, 2.0, 0.0));
                    color = Pnt3::new(0.0, 1.0, 0.0);
                }

                let body = world.add_body(rb);
                testbed.set_color(&body, color);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Pnt3::new(-10.0, 50.0, -10.0), Pnt3::new(0.0, 0.0, 0.0));
    testbed.run();
}
