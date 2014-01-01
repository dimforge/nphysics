#[crate_type = "bin"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

extern mod std;
extern mod extra;
extern mod kiss3d;
extern mod graphics3d;
extern mod nphysics = "nphysics3df32";
extern mod ncollide = "ncollide3df32";
extern mod nalgebra;

use kiss3d::window::Window;
use nalgebra::na::{Vec3, Translation};
use ncollide::geom::{Plane, Box};
use nphysics::world::BodyWorld;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics3d::engine::GraphicsManager;

fn main() {
    GraphicsManager::simulate(wall_3d)
}


pub fn wall_3d(window: &mut Window, graphics: &mut GraphicsManager) -> BodyWorld {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    /*
     * Planes
     */
    let rb   = RigidBody::new(Plane::new(Vec3::new(0.0f32, 1.0, 0.0)), 0.0, Static, 0.3, 0.6);
    let body = @mut RB(rb);

    world.add_body(body);
    graphics.add(window, body);

    /*
     * Create the boxes
     */
    let width   = 50;
    let height  = 10;
    let rad     = 1.0;
    let shift   = rad * 2.0;
    let centerx = shift * (width as f32) / 2.0;
    let centery = shift / 2.0;

    for i in range(0u, width) {
        for j in range(0u, height) {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery;

            let mut rb = RigidBody::new(Box::new(Vec3::new(rad, rad, rad)), 1.0f32, Dynamic, 0.3, 0.5);

            rb.append_translation(&Vec3::new(x, y, 0.0));

            let body = @mut RB(rb);

            world.add_body(body);
            graphics.add(window, body);
        }
    }

    /*
     * Set up the camera and that is it!
     */
    graphics.look_at(Vec3::new(-30.0, 30.0, -30.0), Vec3::new(0.0, 0.0, 0.0));

    world
}
