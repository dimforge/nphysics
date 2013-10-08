#[link(name     = "balls_vee3d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "76b38155-08c2-403d-b3f2-a5606402c0cb")];
#[crate_type = "bin"];
#[warn(non_camel_case_types)]

extern mod std;
extern mod extra;
extern mod kiss3d;
extern mod graphics3d;
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;

use kiss3d::window::Window;
use nalgebra::na;
use ncollide::geom::{Geom, Ball, Plane};
use nphysics::world::BodyWorld;
use nphysics::aliases::dim3;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics3d::engine::GraphicsManager;


#[start]
fn start(argc: int, argv: **u8) -> int {
    std::rt::start_on_main_thread(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(balls_vee_3d)
}

pub fn balls_vee_3d(window: &mut Window, graphics: &mut GraphicsManager) -> dim3::BodyWorld3d<f64> {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(na::vec3(0.0f64, -9.81, 0.0));

    /*
     * Planes
     */
    let normals = [
        na::vec3(-1.0f64, 1.0, -1.0 ),
        na::vec3(1.0f64, 1.0, -1.0 ),
        na::vec3(-1.0f64, 1.0, 1.0 ),
        na::vec3(1.0f64, 1.0, 1.0 )
    ];
    for n in normals.iter() {
        let rb   = RigidBody::new(Geom::new_plane(Plane::new(*n)), 0.0f64, Static, 0.3, 0.6);
        let body = @mut RB(rb);

        world.add_body(body);
        graphics.add(window, body);
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

                let mut rb = RigidBody::new(Geom::new_ball(Ball::new(rad)), 1.0f64, Dynamic, 0.3, 0.6);

                na::translate_by(&mut rb, &na::vec3(x, y, z));

                let body = @mut RB(rb);

                world.add_body(body);
                graphics.add(window, body);
            }
        }
    }

    /*
     * Set up the camera and that is it!
     */
    graphics.look_at(na::vec3(-10.0, 50.0, -10.0), na::vec3(0.0, 0.0, 0.0));

    world
}
