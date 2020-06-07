extern crate nalgebra as na;

use na::{Isometry2, Point2, RealField, Vector2};
use ncollide2d::shape::{Compound, Cuboid, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed2d::Testbed;

/*
 * NOTE: The `r` macro is only here to convert from f64 to the `N` scalar type.
 * This simplifies experimentation with various scalar types (f32, fixed-point numbers, etc.)
 */
pub fn init_world<N: RealField>(testbed: &mut Testbed<N>) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector2::new(r!(0.0), r!(-9.81)));
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Ground
     */
    let ground_size = r!(25.0);
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, r!(1.0))));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(-Vector2::y())
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Cross shaped geometry
     */
    let large_rad = r!(1.0);
    let small_rad = r!(0.05);

    let delta1 = Isometry2::new(Vector2::new(r!(0.0), large_rad), na::zero());
    let delta2 = Isometry2::new(Vector2::new(-large_rad, r!(0.0)), na::zero());
    let delta3 = Isometry2::new(Vector2::new(large_rad, r!(0.0)), na::zero());

    let mut cross_geoms = Vec::new();
    let vertical = ShapeHandle::new(Cuboid::new(Vector2::new(small_rad, large_rad)));
    let horizontal = ShapeHandle::new(Cuboid::new(Vector2::new(large_rad, small_rad)));
    cross_geoms.push((delta1, horizontal));
    cross_geoms.push((delta2, vertical.clone()));
    cross_geoms.push((delta3, vertical));

    let compound = Compound::new(cross_geoms);
    let cross = ShapeHandle::new(compound);

    /*
     * Create the rigid bodies.
     */
    let num = 15;
    let shift = r!(2.5) * large_rad;
    let centerx = (shift + ColliderDesc::<N>::default_margin()) * r!(num as f64) / r!(2.0);
    let centery = (shift + ColliderDesc::<N>::default_margin()) * r!(num as f64) / r!(2.0);

    for i in 0usize..num {
        for j in 0usize..num {
            let x = r!(i as f64) * r!(2.5) * large_rad - centerx;
            let y = r!(j as f64) * r!(2.5) * -large_rad + centery * r!(2.0);

            // Build the rigid body.
            let rb = RigidBodyDesc::new().translation(Vector2::new(x, y)).build();
            let rb_handle = bodies.insert(rb);

            // Build the collider.
            let co = ColliderDesc::new(cross.clone())
                .density(r!(1.0))
                .build(BodyPartHandle(rb_handle, 0));
            colliders.insert(co);
        }
    }

    /*
     * Run the simulation.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.look_at(Point2::new(0.0, 3.0), 95.0);
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("Compound", init_world)]);
    testbed.run()
}
