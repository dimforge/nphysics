extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use std::sync::Arc;
use std::f32;
use std::path::Path;
use na::{Isometry2, Point2, Point3, Vector2, Translation3};
use ncollide2d::shape::{Cuboid, ShapeHandle, Ball, Polyline};
use ncollide2d::procedural;
use ncollide2d::bounding_volume::{self, AABB, BoundingVolume};
use nphysics2d::object::{BodyPartHandle, Material, MassSpringSystem};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics2d::math::Inertia;
use nphysics_testbed2d::Testbed;


const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));
//    world.integration_parameters_mut().max_position_iterations = 0;
//    world.integration_parameters_mut().max_velocity_iterations = 1000;

    /*
     * Ground.
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector2::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry2::new(Vector2::y() * -ground_size, na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape.clone(),
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );


    let ground_size = 0.2;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector2::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry2::new(Vector2::new(0.0, 2.0), na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape.clone(),
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );

    let ground_pos = Isometry2::new(Vector2::new(7.0, 2.0), na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape.clone(),
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );




    let mut vertices = Vec::new();
    let n = 50;
    let width = 10.0;
    let y = 6.0;

    for i in 0..n {
        let step = width / (n as f32);
        vertices.push(Point2::new(step * (i as f32), 0.0 + y));
    }
    for i in (0..n).rev() {
        let step = width / (n as f32);
        vertices.push(Point2::new(step * (i as f32), 1.0 + y));
    }

    let mut indices: Vec<_> = (0..).map(|i| Point2::new(i, i + 1)).take(vertices.len() - 1).collect();
    indices.push(Point2::new(vertices.len() - 1, 0));
    let extra_springs1: Vec<_> = (0..).map(|i| Point2::new(i, vertices.len() - i - 2)).take(vertices.len() / 2).collect();
    let extra_springs2: Vec<_> = (1..).map(|i| Point2::new(i, vertices.len() - i)).take(vertices.len() / 2).collect();


    let polyline = Polyline::new(vertices, Some(indices));
    let mut system = MassSpringSystem::from_polyline(&polyline, 1.0, 100.0, 0.5);
    system.generate_neighbor_springs(100.0, 0.5);
    system.generate_neighbor_springs(100.0, 0.5);

    for spring in extra_springs1.iter().chain(extra_springs2.iter()) {
        system.add_spring(spring.x, spring.y, 100.0, 0.5);
    }

    let mass_spring_handle = world.add_body(Box::new(system.clone()));
    world.add_deformable_collider(
        COLLIDER_MARGIN,
        polyline,
        mass_spring_handle,
        None,
        None,
        Material::default(),
    );
    world.body_mut(mass_spring_handle).set_deactivation_threshold(None);

    // Add a cube to play around with.
    let geom = ShapeHandle::new(Cuboid::new(Vector2::repeat(0.5 - COLLIDER_MARGIN)));
    let inertia = geom.inertia(0.1);
    let center_of_mass = geom.center_of_mass();
    let pos = Isometry2::new(Vector2::y() * 15.0, na::zero());
    let handle = world.add_rigid_body(pos, inertia, center_of_mass);

    world.add_collider(
        COLLIDER_MARGIN,
        geom.clone(),
        handle,
        Isometry2::identity(),
        Material::default(),
    );


    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.set_body_color(mass_spring_handle, Point3::new(0.0, 0.0, 1.0));
    // testbed.hide_performance_counters();
//    testbed.look_at(Point3::new(0.0, 0.0, 2.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
