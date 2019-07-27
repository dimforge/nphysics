extern crate nalgebra as na;

use std::f32;

use na::{Point2, Vector2};
use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics2d::object::{ColliderDesc, RigidBodyDesc, DefaultBodySet, DefaultColliderSet, Ground, BodyPartHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::world::{DefaultDynamicWorld, DefaultColliderWorld};
use nphysics2d::math::Velocity;
use nphysics_testbed2d::Testbed;



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

    let cube = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let shift = (rad + ColliderDesc::<f32>::default_margin()) * 2.0 + 0.002;
    let subdiv = 1.0 / (num as f32);

    for i in 0usize..num {
        let (x, y) = (i as f32 * subdiv * f32::consts::PI * 2.0).sin_cos();
        let dir = Vector2::new(x, y);

        // Build the rigid body.
        let rb = RigidBodyDesc::new()
            .translation(dir)
            .velocity(Velocity::new(dir * 10.0, 100.0))
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
    testbed.look_at(Point2::new(-3.0, -2.0), 100.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![
        ("Damping", init_world),
    ]);
    testbed.run()
}