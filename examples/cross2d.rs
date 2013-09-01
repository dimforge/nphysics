#[link(name     = "cross2d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "f31fbf6a-b82c-4a8a-857a-12b2387f20ae")];
#[crate_type = "bin"];
#[warn(non_camel_case_types)]

extern mod std;
extern mod extra;
extern mod rsfml;
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;
extern mod graphics2d;

use std::num::One;
use nalgebra::mat::Translation;
use nalgebra::vec::Vec2;
use ncollide::geom::{Geom, Box, Plane, CompoundAABB};
use nphysics::world::BodyWorld;
use nphysics::aliases::dim2;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics2d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8, crate_map: *u8) -> int {
    std::rt::start_on_main_thread(argc, argv, crate_map, main)
}

fn main() {
    GraphicsManager::simulate(cross_2d)
}

pub fn cross_2d(graphics: &mut GraphicsManager) -> dim2::BodyWorld2d<f64> {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(Vec2::new(0.0f64, 9.81));

    /*
     * First plane
     */
    let geom   = Plane::new(Vec2::new(-1.0f64, -1.0));
    let mut rb = RigidBody::new(Geom::new_plane(geom), 0.0f64, Static, 0.3, 0.6);

    rb.translate_by(&Vec2::new(0.0, 10.0));

    let body = @mut RB(rb);

    world.add_body(body);
    graphics.add(body);

    /*
     * Second plane
     */
    let geom   = Plane::new(Vec2::new(1.0f64, -1.0));
    let mut rb = RigidBody::new(Geom::new_plane(geom), 0.0f64, Static, 0.3, 0.6);

    rb.translate_by(&Vec2::new(0.0, 10.0));

    let body = @mut RB(rb);

    world.add_body(body);
    graphics.add(body);

    /*
     * Cross shaped geometry
     */
    let box1 = Box::new(Vec2::new(5.0f64, 0.25));
    let box2 = Box::new(Vec2::new(0.25f64, 5.0));

    let mut cross_geoms = ~[];
    cross_geoms.push((One::one(), Geom::new_box(box1)));
    cross_geoms.push((One::one(), Geom::new_box(box2)));

    let cross = @CompoundAABB::new(cross_geoms);

    /*
     * Create the boxes
     */
    let num     = (750.0f64.sqrt()) as uint;
    let rad     = 5.0;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f64) / 2.0;
    let centery = shift * (num as f64) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            let x = i as f64 * 2.5 * rad - centerx;
            let y = j as f64 * 2.5 * rad - centery * 2.0 - 250.0;

            let geom   = Geom::new_compound(cross);
            let mut rb = RigidBody::new(geom, 1.0f64, Dynamic, 0.3, 0.6);

            rb.translate_by(&Vec2::new(x, y));

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
