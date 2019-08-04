extern crate nalgebra as na;

use std::f32;

use na::{Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::object::{ColliderDesc, RigidBodyDesc, DefaultBodySet, DefaultColliderSet, Ground, BodyPartHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::world::{DefaultDynamicWorld, DefaultColliderWorld};
use nphysics3d::math::Velocity;
use nphysics_testbed3d::Testbed;



pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let dynamic_world = DefaultDynamicWorld::new(na::zero());
    let collider_world = DefaultColliderWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Create the balls
     */
    let num = 10;
    let rad = 0.2;

    let cube = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    let subdiv = 1.0 / (num as f32);

    for i in 0usize..num {
        let (x, y) = (i as f32 * subdiv * f32::consts::PI * 2.0).sin_cos();
        let dir = Vector3::new(x, y, 0.0);

        // Build the rigid body.
        let rb = RigidBodyDesc::new()
            .translation(dir)
            .velocity(Velocity::new(dir * 10.0, Vector3::z() * 100.0))
            .linear_damping((i + 1) as f32 * subdiv * 10.0)
            .angular_damping((num - i) as f32 * subdiv * 10.0)
            .build();
        let rb_handle = bodies.insert(rb);

        // Build the collider.
        let co = ColliderDesc::new(cube.clone())
            .density(1.0)
            .build(BodyPartHandle(rb_handle, 0));
        colliders.insert(co);
    }

    /*
     * Set up the testbed.
     */
    // We add a ground to make dragging object work (this is a requirement of the testbed, not nphysics itself).
    let ground_handle = bodies.insert(Ground::new());
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(dynamic_world, collider_world, bodies, colliders, joint_constraints, force_generators);
    testbed.look_at(Point3::new(2.0, 2.5, 20.0), Point3::new(2.0, 2.5, 0.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![
        ("Damping", init_world),
    ]);
    testbed.run()
}