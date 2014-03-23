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
use kiss3d::window::Window;
use nalgebra::na::{Vec3, Translation};
use nalgebra::na;
use ncollide::geom::{Plane, Box, Compound, Geom};
use nphysics::world::World;
use nphysics::object::{RigidBody, Static, Dynamic};
use graphics3d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(cross3d)
}

pub fn cross3d(window: &mut Window, graphics: &mut GraphicsManager) -> World {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    /*
     * Planes
     */
    let rb   = RigidBody::new(Plane::new(Vec3::new(0.0f32, 1.0, 0.0)), 0.0, Static, 0.3, 0.6);
    let body = Rc::new(RefCell::new(rb));

    world.add_body(body.clone());
    graphics.add(window, body);

    /*
     * Cross shaped geometry
     */
    let mut cross_geoms = Vec::new();
    cross_geoms.push((na::one(), ~Box::new(Vec3::new(5.0f32, 0.25, 0.25)) as ~Geom));
    cross_geoms.push((na::one(), ~Box::new(Vec3::new(0.25f32, 5.0, 0.25)) as ~Geom));
    cross_geoms.push((na::one(), ~Box::new(Vec3::new(0.25f32, 0.25, 5.0)) as ~Geom));

    let cross = Rc::new(~Compound::new(cross_geoms) as ~Geom);

    /*
     * Create the crosses 
     */
    let num     = 6;
    let rad     = 5.0;
    let shift   = (rad + 0.08) * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = 30.0 + shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            for k in range(0u, num) {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                let mut rb = RigidBody::new_with_shared_geom(cross.clone(), 1.0f32, Dynamic, 0.3, 0.5);

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
