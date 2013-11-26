#[link(name     = "pyramid2d"
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

use nalgebra::na::{Vec2, Translation};
use ncollide::geom::Geom;
use nphysics::world::BodyWorld;
use nphysics::aliases::dim2;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics2d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8) -> int {
    std::rt::start_on_main_thread(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(pyramid_2d)
}

pub fn pyramid_2d(graphics: &mut GraphicsManager) -> dim2::BodyWorld2d<f32> {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(Vec2::new(0.0f32, 9.81));

    /*
     * First plane
     */
    let rb   = RigidBody::new(Geom::new_plane(Vec2::new(0.0f32, -1.0)), 0.0, Static, 0.3, 0.6);
    let body = @mut RB(rb);

    world.add_body(body);
    graphics.add(body);

    /*
     * Create the boxes
     */
    let num = 25;
    let rad = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f32) / 2.0;

    for i in range(0u, num) {
        for j in range(i, num) {
            let fj = j as f32;
            let fi = i as f32;
            let x = (fi * shift / 2.0) + (fj - fi) * 2.5 * rad - centerx;
            let y = -fi * 2.5 * rad;

            let mut rb = RigidBody::new(Geom::new_box(Vec2::new(rad, rad)), 1.0f32, Dynamic, 0.3, 0.6);

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
