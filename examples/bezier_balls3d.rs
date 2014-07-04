#![crate_type = "bin"]
#![warn(non_camel_case_types)]

extern crate std;
extern crate native;
extern crate kiss3d;
extern crate graphics3d;
extern crate nphysics = "nphysics3df32";
extern crate ncollide = "ncollide3df32";
extern crate nalgebra;

use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::window::Window;
use nalgebra::na::{Vec3, Translation};
use ncollide::geom::{Ball, BezierSurface};
use nphysics::world::World;
use nphysics::object::RigidBody;
use graphics3d::engine::GraphicsManager;

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
     * Ground
     */
    let control_points = vec!(
        Vec3::new(-20.0, -00.0, 020.0), Vec3::new(-10.0, -00.0, 020.0), Vec3::new(00.0, -00.0, 020.0), Vec3::new(10.0, -00.0, 020.0), Vec3::new(20.0, -00.0, 020.0),
        Vec3::new(-20.0, -00.0, 010.0), Vec3::new(-10.0, -40.0, 010.0), Vec3::new(00.0, 040.0, 010.0), Vec3::new(10.0, -40.0, 010.0), Vec3::new(20.0, -00.0, 010.0),
        Vec3::new(-20.0, -00.0, 000.0), Vec3::new(-10.0, -40.0, 000.0), Vec3::new(00.0, 040.0, 000.0), Vec3::new(10.0, -40.0, 000.0), Vec3::new(20.0, -00.0, 000.0),
        Vec3::new(-20.0, -00.0, -10.0), Vec3::new(-10.0, -40.0, -10.0), Vec3::new(00.0, 040.0, -10.0), Vec3::new(10.0, -40.0, -10.0), Vec3::new(20.0, -00.0, -10.0),
        Vec3::new(-20.0, -00.0, -20.0), Vec3::new(-10.0, -00.0, -20.0), Vec3::new(00.0, -00.0, -20.0), Vec3::new(10.0, -00.0, -20.0), Vec3::new(20.0, -00.0, -20.0),
    );
    /*
    let control_points = vec!(
        Vec3::new(-20.0, -00.0, 020.0), Vec3::new(-10.0, -00.0, 020.0), Vec3::new(00.0, -00.0, 020.0), Vec3::new(10.0, -00.0, 020.0), Vec3::new(20.0, -00.0, 020.0),
        Vec3::new(-20.0, -00.0, 010.0), Vec3::new(-10.0, -00.0, 010.0), Vec3::new(00.0, 000.0, 010.0), Vec3::new(10.0, -00.0, 010.0), Vec3::new(20.0, -00.0, 010.0),
        Vec3::new(-20.0, -00.0, 000.0), Vec3::new(-10.0, -00.0, 000.0), Vec3::new(00.0, 000.0, 000.0), Vec3::new(10.0, -00.0, 000.0), Vec3::new(20.0, -00.0, 000.0),
        Vec3::new(-20.0, -00.0, -10.0), Vec3::new(-10.0, -00.0, -10.0), Vec3::new(00.0, 000.0, -10.0), Vec3::new(10.0, -00.0, -10.0), Vec3::new(20.0, -00.0, -10.0),
        Vec3::new(-20.0, -00.0, -20.0), Vec3::new(-10.0, -00.0, -20.0), Vec3::new(00.0, -00.0, -20.0), Vec3::new(10.0, -00.0, -20.0), Vec3::new(20.0, -00.0, -20.0),
    );
    */

    let rb   = RigidBody::new_static(BezierSurface::new(control_points, 5, 5), 0.3, 0.6);
    let body = Rc::new(RefCell::new(rb));

    world.add_body(body.clone());
    graphics.add(window, body);

    /*
     * Create the balls
     */
    let num     = 150.0f32.powf(1.0f32 / 3.0) as uint;
    let rad     = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            for k in range(0u, num) {
                let x = i as f32 * 2.5 * rad - centerx;
                let y = j as f32 * 2.5 * rad + centery * 2.0;
                let z = k as f32 * 2.5 * rad - centerx;

                let mut rb = RigidBody::new_dynamic(Ball::new(rad), 1.0f32, 0.3, 0.6);

                rb.append_translation(&Vec3::new(x, y, z));

                let body = Rc::new(RefCell::new(rb));

                world.add_body(body.clone());
                graphics.add(window, body);
            }
        }
    }

    /*
     * Set up the camera and that is it!
     */
    graphics.look_at(Vec3::new(-10.0, 50.0, -10.0), Vec3::new(0.0, 0.0, 0.0));

    world
}
