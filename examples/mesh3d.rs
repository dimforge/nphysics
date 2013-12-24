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

use extra::arc::Arc;
use nalgebra::na::Vec3;
use kiss3d::window::Window;
use ncollide::geom::Mesh;
use nphysics::world::BodyWorld;
use nphysics::aliases::dim3;
use nphysics::object::{RigidBody, Static, RB};
use graphics3d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8) -> int {
    std::rt::start_on_main_thread(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(mesh3d)
}

pub fn mesh3d(window: &mut Window, graphics: &mut GraphicsManager) -> dim3::BodyWorld3d<f32> {
    /*
     * World
     */
    let mut world = BodyWorld::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    let (vertices, indices) = graphics.load_mesh("media/great_hall.obj");
    let vertices = vertices.map(|v| v * 3.0f32);
    let mesh: dim3::TriangleMesh3d<f32> = Mesh::new(Arc::new(vertices), Arc::new(indices), None);
    let body = @mut RB(RigidBody::new(mesh, 0.0f32, Static, 0.3, 0.6));

    world.add_body(body);
    graphics.add(window, body);

    world
}
