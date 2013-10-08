#[link(name     = "balls_vee2d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "d8d15542-6eda-4969-8bcf-ca3f666d1d58")];
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
use ncollide::geom::{Geom, Ball, Plane};
use nphysics::world::BodyWorld;
use nphysics::aliases::dim2;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics2d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8) -> int {
    std::rt::start_on_main_thread(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(balls_vee_2d)
}

pub fn balls_vee_2d(graphics: &mut GraphicsManager) -> dim2::BodyWorld2d<f64> {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(na::vec2(0.0f64, 9.81));

    /*
     * First plane
     */
    let geom   = Plane::new(na::vec2(-1.0f64, -1.0));
    let mut rb = RigidBody::new(Geom::new_plane(geom), 0.0f64, Static, 0.3, 0.6);

    na::translate_by(&mut rb, &na::vec2(0.0, 10.0));

    let body = @mut RB(rb);

    world.add_body(body);
    graphics.add(body);

    /*
     * Second plane
     */
    let geom   = Plane::new(na::vec2(1.0f64, -1.0));
    let mut rb = RigidBody::new(Geom::new_plane(geom), 0.0f64, Static, 0.3, 0.6);

    na::translate_by(&mut rb, &na::vec2(0.0, 10.0));

    let body = @mut RB(rb);

    world.add_body(body);
    graphics.add(body);

    /*
     * Create the balls
     */
    let num     = (4000.0f64.sqrt()) as uint;
    let rad     = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f64) / 2.0;
    let centery = shift * (num as f64) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            let x = i as f64 * 2.5 * rad - centerx;
            let y = j as f64 * 2.5 * rad - centery * 2.0 - 20.0;

            let ball   = Ball::new(rad);
            let mut rb = RigidBody::new(Geom::new_ball(ball), 1.0f64, Dynamic, 0.3, 0.6);

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
