#![crate_type = "bin"]
#![warn(non_camel_case_types)]

extern crate native;
extern crate kiss3d;
extern crate nphysics_testbed3d;
extern crate nphysics = "nphysics3df32";
extern crate ncollide = "ncollide3df32";
extern crate nalgebra;

use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::window::Window;
use nalgebra::na::{Vec3, Translation};
use ncollide::geom::{Ball, Plane};
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics_testbed3d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(balls_vee_3d)
}

pub fn balls_vee_3d(window: &mut Window, graphics: &mut GraphicsManager) -> World {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    /*
     * Planes
     */
    let geom = Plane::new(Vec3::new(0.0f32, 1.0, 0.0));
    let body = Rc::new(RefCell::new(RigidBody::new_static(geom, 0.3, 0.6)));

    world.add_body(body.clone());
    graphics.add(window, body);

    let geom   = Plane::new(Vec3::new(0.0f32, -1.0, 0.0));
    let mut rb = RigidBody::new_static(geom, 0.3, 0.6);
    rb.append_translation(&Vec3::new(0.0, 50.0, 0.0));
    let body = Rc::new(RefCell::new(rb));

    world.add_body(body.clone());
    graphics.add(window, body);

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

                let mut rb = RigidBody::new_dynamic(Ball::new(rad), 1.0f32, 0.3, 0.6);

                rb.append_translation(&Vec3::new(x, y, z));

                let color;

                if j == 1 {
                    // invert the gravity for the blue balls.
                    rb.set_lin_acc_scale(Vec3::new(0.0, -1.0, 0.0));
                    color = Vec3::new(0.0, 0.0, 1.0);
                }
                else {
                    // double the gravity for the green balls.
                    rb.set_lin_acc_scale(Vec3::new(0.0, 2.0, 0.0));
                    color = Vec3::new(0.0, 1.0, 0.0);
                }

                let body = Rc::new(RefCell::new(rb));

                world.add_body(body.clone());
                graphics.add_with_color(window, body, color);
            }
        }
    }

    /*
     * Set up the camera and that is it!
     */
    graphics.look_at(Vec3::new(-10.0, 50.0, -10.0), Vec3::new(0.0, 0.0, 0.0));

    world
}
