extern crate nalgebra as na;
extern crate kiss3d;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::sync::Arc;
use std::f32;
use std::path::Path;
use na::{Isometry3, Point3, Vector3, Vector2, Translation3};
use kiss3d::loader::obj;
use ncollide3d::shape::{Cuboid, ShapeHandle, Ball, TriMesh};
use ncollide3d::procedural;
use ncollide3d::bounding_volume::{self, AABB, BoundingVolume};
use nphysics3d::object::{BodyPartHandle, Material, MassSpringSurface, MassConstraintSurface};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;


const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));
    world.integration_parameters_mut().max_position_iterations = 0;
//    world.integration_parameters_mut().max_velocity_iterations = 1000;

    /*
     * Ground.
     */
    let ground_size = 50.0;
//    let ground_size = 0.2;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size - COLLIDER_MARGIN)));
//    let ground_shape =
//        ShapeHandle::new(Ball::new(ground_size - COLLIDER_MARGIN));
    let ground_pos = Isometry3::new(Vector3::y() * -ground_size, na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape.clone(),
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
    let ground_size = 3.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::new(0.02, 0.02, ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry3::new(Vector3::new(0.5, -0.01, 0.0), na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape.clone(),
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );

    let ground_pos = Isometry3::new(Vector3::new(-0.5, -0.01, 0.0), na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape.clone(),
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );


    let ground_pos = Isometry3::new(Vector3::new(0.0, -0.2, 0.0), na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape.clone(),
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );*/

    /*
     * Create the deformable body and a collider for its contour.
     */
    /*
    //    let mut volume = MassSpringSurface::quad(
    //        &Isometry3::new(Vector3::y() * 0.5, Vector3::x() * -f32::consts::FRAC_PI_2),
    //        &Vector2::new(1.0, 1.0),
    //        10, 10,
    //        1.0, 100000.0, 1.0);
        let volume = MassConstraintSurface::quad(
            &Isometry3::new(Vector3::y() * 0.5, Vector3::x() * -f32::consts::FRAC_PI_2),
            &Vector2::new(1.0, 1.0),
            10, 10,
            1.0, Some((10.0, 1.0)));
        let mesh = volume.mesh();
    //    volume.generate_neighbor_springs(100000.0, 0.0);
    //    volume.generate_neighbor_springs(100.0, 0.0);
    //    volume.generate_neighbor_springs(100.0, 0.0);

        let handle = world.add_body(Box::new(volume));
        world.add_deformable_collider(
            COLLIDER_MARGIN,
            mesh,
            handle,
            None,
            None,
            Material::default(),
        );*/

    let obj_path = "media/models/rust_logo_simplified.obj";
    let obj = obj::parse_file(&Path::new(&obj_path), &Path::new(""), "");

    if let Ok(model) = obj {
        let mut meshes: Vec<procedural::TriMesh<f32>> = model
            .into_iter()
            .map(|mesh| mesh.1.to_trimesh().unwrap())
            .collect();

        // Compute the size of the model, to scale it and have similar size for everything.
        let mut aabb = bounding_volume::point_cloud_aabb(&Isometry3::identity(), &meshes[0].coords[..]);

        for mesh in meshes[1..].iter() {
            aabb.merge(&bounding_volume::point_cloud_aabb(&Isometry3::identity(), &mesh.coords[..]));
        }

        let center = aabb.center().coords;
        let diag = na::norm(&(*aabb.maxs() - *aabb.mins()));

        for mut trimesh in meshes.iter_mut() {
            trimesh.translate_by(&Translation3::from_vector(-center));
            trimesh.scale_by_scalar(6.0 / diag);
            trimesh.transform_by(&Isometry3::new(na::zero(), Vector3::x() * f32::consts::FRAC_PI_2));
            trimesh.translate_by(&Translation3::new(0.0, 5.0, 0.0));
            trimesh.split_index_buffer(true);
        }

        let shape = meshes[0].clone().into();
//        let mut volume = MassSpringSurface::new(&shape, 1.0, 1000.0, 0.0);
//        volume.generate_neighbor_springs(1000.0, 0.0);
//        volume.generate_neighbor_springs(1000.0, 0.0);
//        volume.generate_neighbor_springs(1000.0, 0.0);
        let mut volume = MassConstraintSurface::new(&shape, 1.0, Some(100.0));
        volume.generate_neighbor_constraints(Some(100.0));
        volume.generate_neighbor_constraints(Some(100.0));
        let handle = world.add_body(Box::new(volume));
        world.add_deformable_collider(
            COLLIDER_MARGIN,
            shape,
            handle,
            None,
            None,
            Material::default(),
        );
        world.body_mut(handle).set_deactivation_threshold(None);


        meshes[0].translate_by(&Translation3::new(2.0, 4.5, 1.0));
        let shape = meshes[0].clone().into();
        let mut volume = MassConstraintSurface::new(&shape, 1.0, Some(0.1));
        volume.generate_neighbor_constraints(Some(0.1));
        volume.generate_neighbor_constraints(Some(0.1));
        let handle = world.add_body(Box::new(volume));
        world.add_deformable_collider(
            COLLIDER_MARGIN,
            shape,
            handle,
            None,
            None,
            Material::default(),
        );
        world.body_mut(handle).set_deactivation_threshold(None);
    }

//    // Add a cube to play around with.
//    let geom = ShapeHandle::new(Cuboid::new(Vector3::repeat(2.1 - COLLIDER_MARGIN)));
//    let inertia = geom.inertia(1.0);
//    let center_of_mass = geom.center_of_mass();
//    let pos = Isometry3::new(Vector3::y() * 5.0, na::zero());
//    let handle = world.add_rigid_body(pos, inertia, center_of_mass);
//
//    world.add_collider(
//        COLLIDER_MARGIN,
//        geom.clone(),
//        handle,
//        Isometry3::identity(),
//        Material::default(),
//    );


    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    // testbed.hide_performance_counters();
    testbed.look_at(Point3::new(0.0, 0.0, 2.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
