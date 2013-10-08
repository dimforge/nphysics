#[link(name     = "wall2d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "dea0027e-7f5c-4fa0-9d04-3469f6836b20")];
#[crate_type = "bin"];
#[warn(non_camel_case_types)]

extern mod std;
extern mod extra;
extern mod rsfml;
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;
extern mod graphics2d;

use nalgebra::na;
use ncollide::geom::{Geom, Box, Plane};
use nphysics::world::BodyWorld;
use nphysics::aliases::dim2;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics2d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8) -> int {
    std::rt::start_on_main_thread(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(wall_2d)
}

pub fn wall_2d(graphics: &mut GraphicsManager) -> dim2::BodyWorld2d<f64> {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(na::vec2(0.0f64, 9.81));

    /*
     * First plane
     */
    let geom = Plane::new(na::vec2(0.0f64, -1.0));
    let rb   = RigidBody::new(Geom::new_plane(geom), 0.0f64, Static, 0.3, 0.6);
    let body = @mut RB(rb);

    world.add_body(body);
    graphics.add(body);

    /*
     * Create the boxes
     */
    let width   = 100;
    let height  = 20;
    let rad     = 0.5;
    let shift   = 2.0 * rad;
    let centerx = shift * (width as f64) / 2.0;

    for i in range(0u, height) {
        for j in range(0u, width) {
            let fj = j as f64;
            let fi = i as f64;
            let x = fj * 2.0 * rad - centerx;
            let y = -fi * 2.0 * rad;

            let box    = Box::new(na::vec2(rad, rad));
            let mut rb = RigidBody::new(Geom::new_box(box), 1.0f64, Dynamic, 0.3, 0.6);

            na::translate_by(&mut rb, &na::vec2(x, y));

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
