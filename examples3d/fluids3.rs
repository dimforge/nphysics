extern crate nalgebra as na;

use std::f32;
use na::{Point3, Vector3, Isometry3};
use ncollide3d::shape::{Cuboid, ShapeHandle, Multiball};
use nphysics3d::object::{ColliderDesc, RigidBodyDesc, DefaultBodySet, DefaultColliderSet, Ground, BodyPartHandle, PBFFluid};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::world::{DefaultMechanicalWorld, DefaultGeometricalWorld};
use nphysics_testbed3d::Testbed;


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector3::new(0.0, -9.81, 0.0));
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Ground
     */
    let ground_thickness = 0.2;
    let ground_half_width = 3.0;
    let ground_shape =
        ShapeHandle::new_shared(Cuboid::new(Vector3::new(ground_half_width, ground_thickness, ground_half_width)));

    let ground_handle = bodies.insert(Ground::new());

    let wall_poses = [
        Isometry3::translation(0.0, -ground_half_width, 0.0),
        Isometry3::new(Vector3::x() * ground_half_width, Vector3::z() * (f32::consts::PI / 2.0)),
        Isometry3::new(Vector3::x() * -ground_half_width, Vector3::z() * (f32::consts::PI / 2.0)),
        Isometry3::new(Vector3::z() * ground_half_width, Vector3::x() * (f32::consts::PI / 2.0)),
        Isometry3::new(Vector3::z() * -ground_half_width, Vector3::x() * (f32::consts::PI / 2.0)),
    ];

    for pose in wall_poses.into_iter() {
        let co = ColliderDesc::new(ground_shape.clone())
            .position(*pose)
            .build(BodyPartHandle(ground_handle, 0));
        colliders.insert(co);
    }

    /*
     * Create a cube
     */
//    let rb = RigidBodyDesc::new()
//        .translation(Vector3::y() * 10.0)
//        .build();
//    let rb_handle = bodies.insert(rb);
//    let cube = Cuboid::new(Vector3::repeat(0.4));
//    let co = ColliderDesc::new(ShapeHandle::new_owned(cube))
//        .density(1.0)
//        .build(BodyPartHandle(rb_handle, 0));
//    colliders.insert(co);

    /*
     * Create the fluid
     */
    let num = 10;
    let particles_radius = 0.1;

    let shift = (particles_radius + ColliderDesc::<f32>::default_margin()) * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0 + 2.0;
    let centerz = shift * (num as f32) / 2.0;
    let mut particle_centers = Vec::new();

    for i in 0usize.. num {
        for j in 0..num * 2 {
            for k in 0..num {
                let x = i as f32 * shift - centerx + j as f32 * 0.001;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift + centerz;
                particle_centers.push(Point3::new(x, y, z));
            }
        }
    }

    // Build the fluid.
    let fluid = PBFFluid::new(1.0, particles_radius, particle_centers);
    let fluid_collider_desc = fluid.particles_collider_desc();
    let fluid_handle = bodies.insert(fluid);
    let fluid_collider = fluid_collider_desc.build(fluid_handle);
    colliders.insert(fluid_collider);

    /*
     * Set up the testbed.
     */
    testbed.set_body_wireframe(ground_handle, true);
    testbed.set_ground_handle(Some(ground_handle));
    testbed.allow_grabbing_behind_ground(true);
    testbed.set_world(mechanical_world, geometrical_world, bodies, colliders, joint_constraints, force_generators);
    testbed.look_at(Point3::new(-11.0, 0.2, -9.0), Point3::new(-1.0, -0.6, 1.0));
}


fn main() {
    let testbed = Testbed::from_builders(0, vec![
        ("Fluids", init_world),
    ]);
    testbed.run()
}