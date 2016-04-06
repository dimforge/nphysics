extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::sync::Arc;
use na::{Vec3, Pnt3};
use ncollide::shape::{TriMesh, TriMesh3};
use nphysics3d::world::World;
use nphysics3d::object::RigidBody;
use nphysics_testbed3d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    let meshes = Testbed::load_obj("media/great_hall.obj");

    for (vertices, indices) in meshes.into_iter() {
        let vertices = vertices.iter().map(|v| *v * 3.0).collect();
        let indices  = indices.chunks(3).map(|is| Pnt3::new(is[0], is[1], is[2])).collect();

        let mesh: TriMesh3<f32> = TriMesh::new(Arc::new(vertices), Arc::new(indices), None, None);

        world.add_body(RigidBody::new_static(mesh, 0.3, 0.6, None));
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}
