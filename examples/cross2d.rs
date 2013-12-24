#[crate_type = "bin"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

extern mod std;
extern mod extra;
extern mod rsfml;
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;
extern mod graphics2d;

use std::rc::Rc;
use nalgebra::na::{Vec2, Translation};
use nalgebra::na;
use ncollide::geom::{Plane, Box, Compound};
use nphysics::world::BodyWorld;
use nphysics::aliases::dim2;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics2d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8) -> int {
    std::rt::start_on_main_thread(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(cross_2d)
}

pub fn cross_2d(graphics: &mut GraphicsManager) -> dim2::BodyWorld2d<f32> {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(Vec2::new(0.0f32, 9.81));

    /*
     * First plane
     */
    let mut rb = RigidBody::new(Plane::new(Vec2::new(-1.0f32, -1.0)), 0.0f32, Static, 0.3, 0.6);

    rb.append_translation(&Vec2::new(0.0, 10.0));

    let body = @mut RB(rb);

    world.add_body(body);
    graphics.add(body);

    /*
     * Second plane
     */
    let mut rb = RigidBody::new(Plane::new(Vec2::new(1.0f32, -1.0)), 0.0f32, Static, 0.3, 0.6);

    rb.append_translation(&Vec2::new(0.0, 10.0));

    let body = @mut RB(rb);

    world.add_body(body);
    graphics.add(body);

    /*
     * Cross shaped geometry
     */
    let mut cross_geoms = ~[];
    cross_geoms.push((na::one(), ~Box::new(Vec2::new(5.0f32, 0.25)) as dim2::Geom2d<f32>));
    cross_geoms.push((na::one(), ~Box::new(Vec2::new(0.25f32, 5.0)) as dim2::Geom2d<f32>));

    let cross = Rc::from_send(~Compound::new(cross_geoms) as dim2::Geom2d<f32>);

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

            let body = @mut RB(rb);

            world.add_body(body);
            graphics.add(body);
        }
    }

    /*
     * The end.
     */
    world
}
