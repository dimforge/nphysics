#[crate_type = "bin"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

extern crate std;
extern crate native;
extern crate kiss3d;
extern crate graphics3d;
extern crate nphysics = "nphysics3df32";
extern crate ncollide = "ncollide3df32";
extern crate nalgebra;

use std::rc::Rc;
use std::cell::RefCell;
use nalgebra::na::{Vec3, Translation};
use kiss3d::window::Window;
use ncollide::geom::{Plane, Box};
use nphysics::world::World;
use nphysics::object::{RigidBody, Static, Dynamic};
use graphics3d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(boxes_vee_3d)
}

pub fn boxes_vee_3d(window: &mut Window, graphics: &mut GraphicsManager) -> World {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    /*
     * Plane
     */
    let geom = Plane::new(Vec3::new(0.0f32, 1.0, 0.0));
    let body = Rc::new(RefCell::new(RigidBody::new(geom, 0.0f32, Static, 0.3, 0.6)));

    world.add_body(body.clone());
    graphics.add(window, body);

    /*
     * Create the boxes
     */
    let num     = 8;
    let rad     = 1.0;
    let shift   = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            for k in range(0u, num) {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                let geom   = Box::new(Vec3::new(rad, rad, rad));
                let mut rb = RigidBody::new(geom, 1.0f32, Dynamic, 0.3, 0.5);

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
    graphics.look_at(Vec3::new(-30.0, 30.0, -30.0), Vec3::new(0.0, 0.0, 0.0));

    world
}
