#[crate_type = "bin"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

extern mod std;
extern mod extra;
extern mod rsfml;
extern mod nphysics = "nphysics2df32";
extern mod nalgebra;
extern mod ncollide = "ncollide2df32";
extern mod graphics2d;

use nalgebra::na::{Vec2, Translation};
use ncollide::geom::{Plane, Box};
use nphysics::world::BodyWorld;
use nphysics::object::{RigidBody, Static, Dynamic, RB};
use graphics2d::engine::GraphicsManager;

fn main() {
    GraphicsManager::simulate(wall_2d)
}

pub fn wall_2d(graphics: &mut GraphicsManager) -> BodyWorld {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(Vec2::new(0.0f32, 9.81));

    /*
     * First plane
     */
    let rb   = RigidBody::new(Plane::new(Vec2::new(0.0f32, -1.0)), 0.0f32, Static, 0.3, 0.6);
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
    let centerx = shift * (width as f32) / 2.0;

    for i in range(0u, height) {
        for j in range(0u, width) {
            let fj = j as f32;
            let fi = i as f32;
            let x = fj * 2.0 * rad - centerx;
            let y = -fi * 2.0 * rad;

            let mut rb = RigidBody::new(Box::new(Vec2::new(rad, rad)), 1.0f32, Dynamic, 0.3, 0.6);

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
