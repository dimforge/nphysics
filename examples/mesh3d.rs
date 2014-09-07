extern crate native;
extern crate sync;
extern crate nalgebra;
extern crate "ncollide3df32" as ncollide;
extern crate "nphysics3df32" as nphysics;
extern crate nphysics_testbed3d;

use sync::Arc;
use nalgebra::na::Vec3;
use ncollide::geom::Mesh;
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics_testbed3d::Testbed;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    let meshes = Testbed::load_obj("media/great_hall.obj");

    for (vertices, indices) in meshes.move_iter() {
        let vertices = vertices.iter().map(|v| v * 3.0f32).collect();
        let mesh     = Mesh::new(Arc::new(vertices), Arc::new(indices), None, None);

        world.add_body(RigidBody::new_static(mesh, 0.3, 0.6));
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}
