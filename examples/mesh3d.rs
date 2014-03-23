#[crate_type = "bin"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

extern crate std;
extern crate native;
extern crate sync;
extern crate kiss3d;
extern crate graphics3d;
extern crate nphysics = "nphysics3df32";
extern crate ncollide = "ncollide3df32";
extern crate nalgebra;

use std::rc::Rc;
use std::cell::RefCell;
use sync::Arc;
use nalgebra::na::Vec3;
use kiss3d::window::Window;
use ncollide::geom::Mesh;
use nphysics::world::World;
use nphysics::object::{RigidBody, Static};
use graphics3d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(mesh3d)
}

pub fn mesh3d(window: &mut Window, graphics: &mut GraphicsManager) -> World {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    let meshes = graphics.load_mesh("media/great_hall.obj");

    for (vertices, indices) in meshes.move_iter() {
        let vertices = vertices.iter().map(|v| v * 3.0f32).collect();
        let mesh     = Mesh::new(Arc::new(vertices), Arc::new(indices), None, None);
        let body     = Rc::new(RefCell::new(RigidBody::new(mesh, 0.0f32, Static, 0.3, 0.6)));

        world.add_body(body.clone());
        graphics.add(window, body);
    }

    world
}
