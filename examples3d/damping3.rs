extern crate nalgebra as na;

use std::f32;

use na::{Point3, RealField, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::math::Velocity;
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed3d::Testbed;

/*
 * NOTE: The `r` macro is only here to convert from f64 to the `N` scalar type.
 * This simplifies experimentation with various scalar types (f32, fixed-point numbers, etc.)
 */
pub fn init_world<N: RealField>(testbed: &mut Testbed<N>) {
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

    let cube = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    let subdiv = r!(1.0 / (num as f64));

    for i in 0usize..num {
        let (x, y) = (r!(i as f64) * subdiv * r!(std::f64::consts::PI) * r!(2.0)).sin_cos();
        let dir = Vector3::new(x, y, r!(0.0));

        // Build the rigid body.
        let rb = RigidBodyDesc::new()
            .translation(dir)
            .velocity(Velocity::new(dir * r!(10.0), Vector3::z() * r!(100.0)))
            .linear_damping(r!((i + 1) as f64) * subdiv * r!(10.0))
            .angular_damping(r!((num - i) as f64) * subdiv * r!(10.0))
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
    testbed.look_at(Point3::new(2.0, 2.5, 20.0), Point3::new(2.0, 2.5, 0.0));
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("Damping", init_world)]);
    testbed.run()
}
