extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::sync::Arc;
use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle, TriMesh};
use nphysics3d::object::{BodyPartHandle, Material, DeformableVolume};
use nphysics3d::world::World;
use nphysics3d::volumetric::Volumetric;
use ncollide3d::transformation::ToTriMesh;
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
    let ground_pos = Isometry3::new(Vector3::y() * (-ground_size - 1.0), na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape.clone(),
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );


    let ground_size = 3.0;
    let mesh: TriMesh<f32> = Cuboid::new(Vector3::new(0.02, 0.02, ground_size - COLLIDER_MARGIN)).to_trimesh(()).into();
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::new(0.02, 0.02, ground_size - COLLIDER_MARGIN))); // mesh);
    let ground_pos = Isometry3::new(Vector3::new(0.4, -0.01, 0.0), na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape.clone(),
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );

    let ground_pos = Isometry3::new(Vector3::new(-0.4, -0.01, 0.0), na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape.clone(),
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Create the deformable body and a collider for its contour.
     */
    let volume = DeformableVolumeDesc::cube(50, 1, 1)
        .with_scaling(1.0, 0.1, 0.1)
        .with_translation(Vecotr3::y() * 0.1)
        .with_young_modulus(1.0e3)
        .with_sleeping_threshold(None)
        .with_damping(0.2, 0.0);

    volume.build(&mut world, Some(&mut cworld));




//    let mut volume = DeformableVolume::cube(
//        &Isometry3::new(Vector3::y() * 0.1, na::zero()), // Vector3::z() * 1.0),
//        &Vector3::new(1.1, 0.1, 0.1),
//        50, 1, 1,
//        1.0, 1.0e3, 0.0,
//        (0.2, 0.0));
//    let (mesh, ids_map, parts_map) = volume.boundary_mesh();
//    volume.renumber_dofs(&ids_map);
//
//
//    let handle = world.add_body(Box::new(volume));
//    world.add_deformable_collider(
//        COLLIDER_MARGIN,
//        mesh,
//        handle,
//        None,
//        Some(Arc::new(parts_map)),
//        Material::default(),
//    );
//    world.body_mut(handle).set_deactivation_threshold(None);




    let geom = ShapeHandle::new(Cuboid::new(Vector3::new(0.05 - COLLIDER_MARGIN, 0.05 - COLLIDER_MARGIN, 0.05 - COLLIDER_MARGIN)));
    let inertia = geom.inertia(10.1);
    let center_of_mass = geom.center_of_mass();
    let pos = Isometry3::new(Vector3::y() * 0.5, na::zero());
    let handle = world.add_rigid_body(pos, inertia, center_of_mass);

    world.add_collider(
        COLLIDER_MARGIN,
        geom.clone(),
        handle,
        Isometry3::identity(),
        Material::default(),
    );

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    // testbed.hide_performance_counters();
    testbed.look_at(Point3::new(0.0, 0.0, 2.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
