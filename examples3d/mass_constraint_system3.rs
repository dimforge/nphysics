extern crate nalgebra as na;
extern crate kiss3d;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::f32;
use std::path::Path;
use na::{Isometry3, Point3, Vector3};
use kiss3d::loader::obj;
use ncollide3d::shape::{Cuboid, ShapeHandle};
use ncollide3d::procedural;
use nphysics3d::object::{ColliderDesc, MassConstraintSystemDesc};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;


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
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size)));

    ColliderDesc::new(ground_shape)
        .with_translation(Vector3::y() * -ground_size)
        .build(&mut world);

    /*
     * Create the deformable body and a collider for its contour.
     */

    let obj_path = "media/models/rust_logo_simplified.obj";
    let obj = obj::parse_file(&Path::new(&obj_path), &Path::new(""), "");

    if let Ok(model) = obj {
        let mut meshes: Vec<procedural::TriMesh<f32>> = model
            .into_iter()
            .map(|mesh| mesh.1.to_trimesh().unwrap())
            .collect();
        meshes[0].split_index_buffer(true);

        let trimesh = meshes[0].clone().into();

        let rot = Vector3::x() * f32::consts::FRAC_PI_2;
        let mut desc = MassConstraintSystemDesc::from_trimesh(&trimesh)
            .with_scale(Vector3::repeat(0.5))
            .with_position(Isometry3::new(Vector3::y() * 5.0, rot))
            .with_stiffness(Some(0.1))
            .with_collider_enabled(true);

        let deformable1 = desc.build(&mut world);
        deformable1.generate_neighbor_constraints(Some(0.1));
        deformable1.generate_neighbor_constraints(Some(0.1));


        let deformable2 = desc
            .set_position(Isometry3::new(Vector3::y() * 9.5, rot))
            .set_stiffness(Some(100.0))
            .build(&mut world);
        deformable2.generate_neighbor_constraints(Some(100.0));
        deformable2.generate_neighbor_constraints(Some(100.0));
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.look_at(Point3::new(0.0, 0.0, 2.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
