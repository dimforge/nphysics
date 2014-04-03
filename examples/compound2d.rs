#![crate_type = "bin"]
#![warn(non_camel_case_types)]
#![feature(managed_boxes)]

extern crate std;
extern crate native;
extern crate rsfml;
extern crate nphysics = "nphysics2df32";
extern crate nalgebra;
extern crate ncollide = "ncollide2df32";
extern crate graphics2d;

use std::rc::Rc;
use std::cell::RefCell;
use nalgebra::na::{Vec2, Iso2, Translation};
use nalgebra::na;
use ncollide::geom::{Plane, Box, Compound, Geom};
use nphysics::world::World;
use nphysics::object::{RigidBody, Static, Dynamic};
use graphics2d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(compound_2d)
}

pub fn compound_2d(graphics: &mut GraphicsManager) -> World {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec2::new(0.0f32, 9.81));

    /*
     * First plane
     */
    let mut rb = RigidBody::new(Plane::new(Vec2::new(-1.0f32, -1.0)), 0.0f32, Static, 0.3, 0.6);

    rb.append_translation(&Vec2::new(0.0, 10.0));

    let body = Rc::new(RefCell::new(rb));

    world.add_body(body.clone());
    graphics.add(body);

    /*
     * Second plane
     */
    let mut rb = RigidBody::new(Plane::new(Vec2::new(1.0f32, -1.0)), 0.0f32, Static, 0.3, 0.6);

    rb.append_translation(&Vec2::new(0.0, 10.0));

    let body = Rc::new(RefCell::new(rb));

    world.add_body(body.clone());
    graphics.add(body);

    /*
     * Cross shaped geometry
     */
    let delta1 = Iso2::new(Vec2::new(0.0f32, -5.0), na::zero());
    let delta2 = Iso2::new(Vec2::new(-5.0f32, 0.0), na::zero());
    let delta3 = Iso2::new(Vec2::new(5.0f32,  0.0), na::zero());

    let mut cross_geoms = Vec::new();
    cross_geoms.push((delta1, ~Box::new(Vec2::new(5.0f32, 0.75)) as ~Geom:Send));
    cross_geoms.push((delta2, ~Box::new(Vec2::new(0.75f32, 5.0)) as ~Geom:Send));
    cross_geoms.push((delta3, ~Box::new(Vec2::new(0.75f32, 5.0)) as ~Geom:Send));

    let cross = Rc::new(~Compound::new(cross_geoms) as ~Geom:'static);

    /*
     * Create the boxes
     */
    let num     = (750.0f32.sqrt()) as uint;
    let rad     = 5.0;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift * (num as f32) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            let x = i as f32 * 2.5 * rad - centerx;
            let y = j as f32 * 2.5 * rad - centery * 2.0 - 250.0;

            let mut rb = RigidBody::new_with_shared_geom(cross.clone(), 1.0f32, Dynamic, 0.3, 0.6);

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
