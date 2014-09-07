extern crate native;
extern crate nalgebra;
extern crate "ncollide2df32" as ncollide;
extern crate "nphysics2df32" as nphysics;
extern crate nphysics_testbed2d;

use nalgebra::na::{Vec2, Vec3, Translation};
use ncollide::geom::{Ball, Plane};
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics_testbed2d::Testbed;

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
    world.set_gravity(Vec2::new(0.0f32, 9.81));

    /*
     * First plane
     */
    let mut rb = RigidBody::new_static(Plane::new(Vec2::new(0.0f32, 1.0)), 0.3, 0.6);

    rb.append_translation(&Vec2::new(0.0, -10.0));

    world.add_body(rb);

    /*
     * Second plane
     */
    let mut rb = RigidBody::new_static(Plane::new(Vec2::new(0.0f32, -1.0)), 0.3, 0.6);

    rb.append_translation(&Vec2::new(0.0, 10.0));

    world.add_body(rb);

    /*
     * Create the balls
     */
    let num     = 1000u;
    let rad     = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = 2.0;

    for i in range(0u, num) {
        for j in range(0u, 2) {
            let x = i as f32 * 2.5 * rad - centerx;
            let y = j as f32 * 2.5 * rad - centery * 2.0;

            let mut rb = RigidBody::new_dynamic(Ball::new(rad), 1.0f32, 0.3, 0.6);

            rb.append_translation(&Vec2::new(x, y));

            let color;

            if j == 0 {
                // invert the gravity for the blue balls.
                rb.set_lin_acc_scale(Vec2::new(0.0, -1.0));
                color = Vec3::new(0.0, 0.0, 1.0);
            }
            else {
                // double the gravity for the green balls.
                rb.set_lin_acc_scale(Vec2::new(0.0, 2.0));
                color = Vec3::new(0.0, 1.0, 0.0);
            }

            let body = world.add_body(rb);
            testbed.set_color(&body, color);
        }
    }

    /*
     * Run the simulation.
     */
    testbed.set_world(world);
    testbed.run();
}
