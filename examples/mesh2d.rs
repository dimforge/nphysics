#![crate_type = "bin"]
#![warn(non_camel_case_types)]

extern crate std;
extern crate rand;
extern crate native;
extern crate sync;
extern crate rsfml;
extern crate nphysics = "nphysics2df32";
extern crate nalgebra;
extern crate ncollide = "ncollide2df32";
extern crate graphics2d;

use std::rc::Rc;
use std::cell::RefCell;
use rand::{StdRng, SeedableRng, Rng};
use sync::Arc;
use nalgebra::na::{Vec2, Translation};
use ncollide::geom::{Box, Mesh};
use nphysics::world::World;
use nphysics::object::{RigidBody, Static, Dynamic};
use graphics2d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(mesh2d)
}

pub fn mesh2d(graphics: &mut GraphicsManager) -> World {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec2::new(0.0f32, 9.81));

    /*
     * First plane
     */
    let num_split = 5;
    let begin     = -75.0f32;
    let max_h     = 15.0;
    let begin_h   = 15.0;
    let step      = (begin.abs() * 2.0) / (num_split as f32);
    let mut vertices = Vec::from_fn(num_split + 2, |i| Vec2::new(begin + (i as f32) * step, 0.0));
    let mut indices  = Vec::new();
    let mut rng: StdRng = SeedableRng::from_seed(&[1, 2, 3, 4]);

    for i in range(0u, num_split) {
        let h: f32 = rng.gen();
        vertices.get_mut(i + 1).y = begin_h - h * max_h;

        indices.push(i);
        indices.push(i + 1);
    }
    indices.push(num_split);
    indices.push(num_split + 1);

    let mesh = Mesh::new(Arc::new(vertices), Arc::new(indices), None, None);
    let rb   = RigidBody::new(mesh, 0.0f32, Static, 0.3, 0.6);
    let body = Rc::new(RefCell::new(rb));

    world.add_body(body.clone());
    graphics.add(body);

    /*
     * Create the boxes
     */
    let width   = 100;
    let height  = 20;
    let rad     = 0.5;
    let shift   = 2.0 * rad;
    let centerx = shift * (width as f32) / 2.0;

    for i in range(0u, height) {
        for j in range(0u, width) {
            let fj = j as f32;
            let fi = i as f32;
            let x = fj * 2.0 * rad - centerx;
            let y = -fi * 2.0 * rad;

            let mut rb = RigidBody::new(Box::new(Vec2::new(rad, rad)), 1.0f32, Dynamic, 0.3, 0.6);

            rb.append_translation(&Vec2::new(x, y));

            let body = Rc::new(RefCell::new(rb));

            world.add_body(body.clone());
            graphics.add(body);
        }
    }

    /*
     * The end.
     */
    world
}
