#[link(name     = "boxes_vee2d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "8c42b8fb-91ec-4650-9c64-a160bf4d0808")];
#[crate_type = "bin"];
#[warn(non_camel_case_types)]

extern mod std;
extern mod extra;
extern mod rsfml;
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;
extern mod graphics2d;

use nalgebra::mat::Translation;
use nalgebra::vec::Vec2;
use ncollide::geom::{Geom, Box, Plane};
use nphysics::world::BodyWorld;
use nphysics::aliases::dim2;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics2d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8, crate_map: *u8) -> int {
    std::rt::start_on_main_thread(argc, argv, crate_map, main)
}

fn main() {
    GraphicsManager::simulate(boxes_vee_2d)
}

pub fn boxes_vee_2d(graphics: &mut GraphicsManager) -> dim2::BodyWorld2d<f64> {
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
     * Create the boxes
     */
    let num = (1000.0f64.sqrt()) as uint;
    let rad = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f64) / 2.0;
    let centery = shift * (num as f64) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            let x = i as f64 * 2.5 * rad - centerx;
            let y = j as f64 * 2.5 * rad - centery * 2.0 - 10.0;

            let box    = Box::new(Vec2::new(rad, rad));
            let geom   = Geom::new_box(box);
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
