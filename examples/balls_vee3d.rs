#[link(name     = "balls_vee3d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "76b38155-08c2-403d-b3f2-a5606402c0cb")];
#[crate_type = "bin"];
#[warn(non_camel_case_types)]

extern mod std;
extern mod extra;
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;
extern mod graphics3d;

use std::num::One;
use nalgebra::mat::Translation;
use nalgebra::vec::Vec3;
use ncollide::geom::{Geom, Ball, Plane};
use nphysics::world::BodyWorld;
use nphysics::aliases::dim3;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics3d::engine::GraphicsManager;


#[start]
fn start(argc: int, argv: **u8, crate_map: *u8) -> int {
    std::rt::start_on_main_thread(argc, argv, crate_map, main)
}

fn main() {
    GraphicsManager::simulate(balls_vee_3d)
}

pub fn balls_vee_3d(graphics: &mut GraphicsManager) -> dim3::BodyWorld3d<f64> {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(Vec3::new(0.0f64, -9.81, 0.0));

    /*
     * Planes
     */
    let normals = [
        Vec3::new(-1.0f64, 1.0, -1.0 ),
        Vec3::new(1.0f64, 1.0, -1.0 ),
        Vec3::new(-1.0f64, 1.0, 1.0 ),
        Vec3::new(1.0f64, 1.0, 1.0 )
    ];
    for n in normals.iter() {
        let geom = Plane::new(*n);
        let body = @mut RigidBody::new(Geom::new_plane(geom), 0.0f64, Static, 0.3, 0.6);

        world.add_body(@mut RB(body));
        graphics.add_plane(body, &geom);
    }

    /*
     * Create the balls
     */
    let num     = (1500.0f64.pow(&(1.0f64 / 3.0))) as uint;
    let rad     = 0.5;
    let shift   = 2.5 * rad;
    let centerx = shift * (num as f64) / 2.0;
    let centery = shift * (num as f64) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            for k in range(0u, num) {
                let x = i as f64 * 2.5 * rad - centerx;
                let y = 10.0 + j as f64 * 2.5 * rad + centery * 2.0;
                let z = k as f64 * 2.5 * rad - centerx;

                let ball = Ball::new(rad);
                let geom = Geom::new_ball(ball);
                let body = @mut RigidBody::new(geom, 1.0f64, Dynamic, 0.3, 0.6);

                body.translate_by(&Vec3::new(x, y, z));

                world.add_body(@mut RB(body));
                graphics.add_ball(body, One::one(), &ball);
            }
        }
    }

    /*
     * Set up the camera and that is it!
     */
    graphics.look_at(Vec3::new(-10.0, 50.0, -10.0), Vec3::new(0.0, 0.0, 0.0));

    world
}
