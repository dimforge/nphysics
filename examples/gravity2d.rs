#![crate_type = "bin"]
#![warn(non_camel_case_types)]

extern crate native;
extern crate rsfml;
extern crate nphysics = "nphysics2df32";
extern crate nalgebra;
extern crate ncollide = "ncollide2df32";
extern crate graphics2d;

use std::rc::Rc;
use std::cell::RefCell;
use nalgebra::na::{Vec2, Vec3, Translation};
use ncollide::geom::{Ball, Plane};
use nphysics::world::World;
use nphysics::object::RigidBody;
use graphics2d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(balls_vee_2d)
}

pub fn balls_vee_2d(graphics: &mut GraphicsManager) -> World {
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

    let body = Rc::new(RefCell::new(rb));

    world.add_body(body.clone());
    graphics.add(body);

    /*
     * Second plane
     */
    let mut rb = RigidBody::new_static(Plane::new(Vec2::new(0.0f32, -1.0)), 0.3, 0.6);

    rb.append_translation(&Vec2::new(0.0, 10.0));

    let body = Rc::new(RefCell::new(rb));

    world.add_body(body.clone());
    graphics.add(body);

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

            let body = Rc::new(RefCell::new(rb));

            world.add_body(body.clone());
            graphics.add_with_color(body, color);
        }
    }

    /*
     * The end.
     */
    world
}
