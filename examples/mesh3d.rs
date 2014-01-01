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

use extra::arc::Arc;
use nalgebra::na::Vec3;
use kiss3d::window::Window;
use ncollide::geom::Mesh;
use nphysics::world::BodyWorld;
use nphysics::object::{RigidBody, Static, RB};
use graphics3d::engine::GraphicsManager;

fn main() {
    GraphicsManager::simulate(mesh3d)
}

pub fn mesh3d(window: &mut Window, graphics: &mut GraphicsManager) -> BodyWorld {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    let meshes = graphics.load_mesh("media/great_hall.obj");

    for (vertices, indices) in meshes.move_iter() {
        let vertices = vertices.map(|v| v * 3.0f32);
        let mesh     = Mesh::new(Arc::new(vertices), Arc::new(indices), None, None);
        let body     = @mut RB(RigidBody::new(mesh, 0.0f32, Static, 0.3, 0.6));

        world.add_body(body);
        graphics.add(window, body);
    }

    world
}
