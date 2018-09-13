extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::sync::Arc;
use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, TriMesh, ShapeHandle};
use nphysics3d::object::{BodyPartHandle, Material, DeformableVolume};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Ground.
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry3::new(Vector3::y() * -ground_size, na::zero());

//    world.add_collider(
//        COLLIDER_MARGIN,
//        ground_shape,
//        BodyPartHandle::ground(),
//        ground_pos,
//        Material::default(),
//    );

    /*
     * Create the deformable body and a collider for its contour.
     */
    let volume = DeformableVolume::cube(
        Vector3::new(1.0, 0.05, 0.1),
        4, 4, 4,
        1.0, 1.0e4, 0.3,
        (0.4, 0.0));
    let (mesh, ids_map) = volume.boundary_mesh();

    let handle = world.add_body(Box::new(volume));
    world.add_deformable_collider(
        COLLIDER_MARGIN,
        mesh,
        handle,
        Arc::new(ids_map),
        Material::default(),
    );


    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    // testbed.hide_performance_counters();
    testbed.look_at(Point3::new(-4.0, 1.0, -4.0), Point3::new(0.0, 1.0, 0.0));
    testbed.run();
}
