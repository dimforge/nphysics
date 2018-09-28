extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::sync::Arc;
use std::f32;
use na::{Isometry3, Point3, Vector3, Vector2};
use ncollide3d::shape::{Cuboid, ShapeHandle, Ball};
use nphysics3d::object::{BodyPartHandle, Material, MassSpringSurface, SpringType};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));
//    world.set_timestep(0.001);
//    world.integration_parameters_mut().max_position_iterations = 0;
//    world.integration_parameters_mut().max_velocity_iterations = 0;

    /*
     * Ground.
     */
//    let ground_size = 50.0;
    let ground_size = 0.2;
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
    let spring_type = SpringType::Elastic {
        stiffness: 2.0,
        damping: 0.1,
        min_length: None,
        max_length: Some(0.2),
    };

//    let spring_type = SpringType::Rigid;

    let volume = MassSpringSurface::quad(
        &Isometry3::new(Vector3::y() * 0.5, Vector3::x() * -f32::consts::FRAC_PI_2),
        &Vector2::new(1.0, 1.0),
        10, 10,
        1.0, spring_type);
    let mesh = volume.mesh();

    let handle = world.add_body(Box::new(volume));
    world.add_deformable_collider(
        COLLIDER_MARGIN,
        mesh,
        handle,
        None,
        None,
        Material::default(),
    );
    world.body_mut(handle).set_deactivation_threshold(None);


    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    // testbed.hide_performance_counters();
    testbed.look_at(Point3::new(0.0, 0.0, 2.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
