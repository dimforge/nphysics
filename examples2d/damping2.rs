extern crate nalgebra as na;

use std::f32;

use na::{Point2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::math::Velocity;
use nphysics2d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed2d::{r, Real, Testbed};

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(na::zero());
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Create the balls
     */
    let num = 10;
    let rad = r!(0.2);

    let cube = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let _shift = (rad + ColliderDesc::<Real>::default_margin()) * r!(2.0) + 0.002;
    let subdiv = r!(1.0) / r!(num);

    for i in 0usize..num {
        let (x, y) = (r!(i as f32) * subdiv * r!(f32::consts::PI) * r!(2.0)).sin_cos();
        let dir = Vector2::new(x, y);

        // Build the rigid body.
        let rb = RigidBodyDesc::new()
            .translation(dir)
            .velocity(Velocity::new(dir * r!(10.0), r!(100.0)))
            .linear_damping(r!((i + 1) as f32) * subdiv * r!(10.0))
            .angular_damping(r!((num - i) as f32) * subdiv * r!(10.0))
            .build();
        let rb_handle = bodies.insert(rb);

        // Build the collider.
        let co = ColliderDesc::new(cube.clone())
            .density(r!(1.0))
            .build(BodyPartHandle(rb_handle, 0));
        colliders.insert(co);
    }

    /*
     * Set up the testbed.
     */
    // We add a ground to make dragging object work (this is a requirement of the testbed, not nphysics itself).
    let ground_handle = bodies.insert(Ground::new());
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.look_at(Point2::new(3.0, 2.0), 75.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Damping", init_world)]);
    testbed.run()
}
