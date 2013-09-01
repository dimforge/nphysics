#[link(name     = "boxes_vee3d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "4f1fb8c5-5f02-4d45-89f3-64acfd1718fa")];
#[crate_type = "bin"];
#[warn(non_camel_case_types)]

extern mod std;
extern mod extra;
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;
extern mod graphics3d;

use nalgebra::mat::Translation;
use nalgebra::vec::Vec3;
use ncollide::geom::{Geom, Box, Plane};
use nphysics::world::BodyWorld;
use nphysics::aliases::dim3;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics3d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8, crate_map: *u8) -> int {
    std::rt::start_on_main_thread(argc, argv, crate_map, main)
}

fn main() {
    GraphicsManager::simulate(boxes_vee_3d)
}

pub fn boxes_vee_3d(graphics: &mut GraphicsManager) -> dim3::BodyWorld3d<f64> {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(Vec3::new(0.0f64, -9.81, 0.0));

    /*
     * Plane
     */
    let geom = Geom::new_plane(Plane::new(Vec3::y()));
    let body = @mut RB(RigidBody::new(geom, 0.0f64, Static, 0.3, 0.6));

    world.add_body(body);
    graphics.add(body);

    /*
     * Create the boxes
     */
    let num     = 8;
    let rad     = 1.0;
    let shift   = rad * 2.0;
    let centerx = shift * (num as f64) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f64) / 2.0;

    for i in range(0u, num) {
        for j in range(0u, num) {
            for k in range(0u, num) {
                let x = i as f64 * shift - centerx;
                let y = j as f64 * shift + centery;
                let z = k as f64 * shift - centerz;

                let geom   = Geom::new_box(Box::new(Vec3::new(rad, rad, rad)));
                let mut rb = RigidBody::new(geom, 1.0f64, Dynamic, 0.3, 0.5);

                rb.translate_by(&Vec3::new(x, y, z));

                let body = @mut RB(rb);

                world.add_body(body);
                graphics.add(body);
            }
        }
    }

    /*
     * Set up the camera and that is it!
     */
    graphics.look_at(Vec3::new(-30.0, 30.0, -30.0), Vec3::new(0.0, 0.0, 0.0));

    world
}
