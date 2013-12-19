#[link(name     = "pyramid3d"
       , vers   = "0.0"
       , author = "Sébastien Crozet"
       , uuid   = "9ec127f7-d531-4fbd-b405-9af404cb7aaa")];
#[crate_type = "bin"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

extern mod std;
extern mod extra;
extern mod kiss3d;
extern mod graphics3d;
extern mod nphysics;
extern mod nalgebra;
extern mod ncollide;

use kiss3d::window::Window;
use nalgebra::na::{Vec3, Translation};
use ncollide::geom::Geom;
use nphysics::world::BodyWorld;
use nphysics::aliases::dim3;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics3d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8) -> int {
    std::rt::start_on_main_thread(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(pyramid3d)
}

pub fn pyramid3d(window: &mut Window, graphics: &mut GraphicsManager) -> dim3::BodyWorld3d<f32> {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    /*
     * Planes
     */
    let rb   = RigidBody::new(Geom::new_plane(Vec3::new(0.0f32, 1.0, 0.0)), 0.0, Static, 0.3, 0.6);
    let body = @mut RB(rb);


    world.add_body(body);
    graphics.add(window, body);

    /*
     * Create the boxes
     */
    let num     = 30;
    let rad     = 0.5;
    let shift   = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;

    for i in range(0u, num) {
        for j in range(i, num) {
            let fi = i as f32;
            let fj = (j - i) as f32;
            let x = (fi * shift / 2.0) + fj * shift - centerx;
            let y = fi * shift + centery;

            let mut rb = RigidBody::new(Geom::new_box(Vec3::new(rad, rad, rad)), 1.0f32, Dynamic, 0.3, 0.6);

            rb.append_translation(&Vec3::new(x, y, 0.0));

            let body = @mut RB(rb);

            world.add_body(body);
            graphics.add(window, body);
        }
    }

    /*
     * Set up the camera and that is it!
     */
    graphics.look_at(Vec3::new(0.0, 60.0, -60.0), Vec3::new(0.0, 0.0, 0.0));

    world
}
