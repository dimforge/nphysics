extern crate nalgebra as na;

use na::{Point2, Vector2, Isometry2};
use ncollide2d::shape::{Compound, Cuboid, ShapeHandle};
use nphysics2d::object::{ColliderDesc, RigidBodyDesc, DefaultBodySet, DefaultColliderSet, Ground, BodyPartHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::world::{DefaultDynamicWorld, DefaultColliderWorld};
use nphysics_testbed2d::Testbed;



pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let dynamic_world = DefaultDynamicWorld::new(Vector2::new(0.0, -9.81));
    let collider_world = DefaultColliderWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Ground
     */
    let ground_size = 25.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 1.0)));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(-Vector2::y())
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Cross shaped geometry
     */
    let large_rad = 1.0f32;
    let small_rad = 0.05f32;

    let delta1 = Isometry2::new(Vector2::new(0.0, large_rad), na::zero());
    let delta2 = Isometry2::new(Vector2::new(-large_rad, 0.0), na::zero());
    let delta3 = Isometry2::new(Vector2::new(large_rad, 0.0), na::zero());

    let mut cross_geoms = Vec::new();
    let vertical = ShapeHandle::new(Cuboid::new(Vector2::new(
        small_rad,
        large_rad,
    )));
    let horizontal = ShapeHandle::new(Cuboid::new(Vector2::new(
        large_rad,
        small_rad,
    )));
    cross_geoms.push((delta1, horizontal));
    cross_geoms.push((delta2, vertical.clone()));
    cross_geoms.push((delta3, vertical));

    let compound = Compound::new(cross_geoms);
    let cross = ShapeHandle::new(compound);

    /*
     * Create the rigid bodies.
     */
    let num = 15;
    let shift = 2.5 * large_rad;
    let centerx = (shift + ColliderDesc::<f32>::default_margin()) * (num as f32) / 2.0;
    let centery = (shift + ColliderDesc::<f32>::default_margin()) * (num as f32) / 2.0;

    for i in 0usize..num {
        for j in 0usize..num {
            let x = i as f32 * 2.5 * large_rad - centerx;
            let y = j as f32 * 2.5 * -large_rad + centery * 2.0;

            // Build the rigid body.
            let rb = RigidBodyDesc::new()
                .translation(Vector2::new(x, y))
                .build();
            let rb_handle = bodies.insert(rb);

            // Build the collider.
            let co = ColliderDesc::new(cross.clone())
                .density(1.0)
                .build(BodyPartHandle(rb_handle, 0));
            colliders.insert(co);
        }
    }

    /*
     * Run the simulation.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(dynamic_world, collider_world, bodies, colliders, joint_constraints, force_generators);
}


fn main() {
    let testbed = Testbed::from_builders(0, vec![
        ("Compound", init_world),
    ]);
    testbed.run()
}