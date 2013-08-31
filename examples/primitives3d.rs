#[link(name     = "primitives3d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "34c528db-74e1-44a1-8b43-4cf69a170480")];
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
use ncollide::geom::{Geom, Ball, Box, Cone, Cylinder, Plane};
use nphysics::world::BodyWorld;
use nphysics::aliases::dim3;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics3d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8, crate_map: *u8) -> int {
    std::rt::start_on_main_thread(argc, argv, crate_map, main)
}

fn main() {
    GraphicsManager::simulate(primitives_3d)
}


pub fn primitives_3d(graphics: &mut GraphicsManager) -> dim3::BodyWorld3d<f64> {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(Vec3::new(0.0f64, -9.81, 0.0));

    /*
     * Planes
     */
    let geom = Plane::new(Vec3::y());
    let body = @mut RigidBody::new(Geom::new_plane(geom), 0.0f64, Static, 0.3, 0.6);

    world.add_body(@mut RB(body));
    graphics.add_plane(body, &geom);

    /*
     * Create the boxes
     */
    let num     = 8;
    let rad     = 1.0;
    let shift   = (rad + 0.08) * 2.0;
    let centerx = shift * (num as f64) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f64) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            for k in range(0u, num) {
                let x = i as f64 * shift - centerx;
                let y = j as f64 * shift + centery;
                let z = k as f64 * shift - centerz;
                let body;


                if j % 4 == 0 {
                    let box  = Box::new(Vec3::new(rad, rad, rad));
                    let geom = Geom::new_box(box);
                    body = @mut RigidBody::new(geom, 1.0f64, Dynamic, 0.3, 0.5);
                    graphics.add_cube(body, One::one(), &box);
                }
                else if j % 3 == 0 {
                    let ball = Ball::new(rad);
                    let geom = Geom::new_ball(ball);
                    body     = @mut RigidBody::new(geom, 1.0f64, Dynamic, 0.3, 0.5);
                    graphics.add_ball(body, One::one(), &ball);
                }
                else if j % 2 == 0 {
                    let cylinder = Cylinder::new(rad, rad);
                    let geom     = Geom::new_cylinder(cylinder);
                    body         = @mut RigidBody::new(geom, 1.0f64, Dynamic, 0.3, 0.5);
                    graphics.add_cylinder(body, One::one(), &cylinder);
                }
                else {
                 let cone = Cone::new(rad, rad);
                 let geom = Geom::new_cone(cone);
                 body     = @mut RigidBody::new(geom, 1.0f64, Dynamic, 0.3, 0.5);
                 graphics.add_cone(body, One::one(), &cone);
                }

                body.translate_by(&Vec3::new(x, y, z));
                world.add_body(@mut RB(body));
            }
        }
    }

    /*
     * Set up the camera and that is it!
     */
    graphics.look_at(Vec3::new(-30.0, 30.0, -30.0), Vec3::new(0.0, 0.0, 0.0));

    world
}
