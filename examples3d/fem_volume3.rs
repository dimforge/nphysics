extern crate nalgebra as na;

use na::{Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, FEMVolumeDesc, Ground,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed3d::{r, Testbed};

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector3::new(r!(0.0), r!(-9.81), r!(0.0)));
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Ground.
     */
    // Static body to which all the obstacles and the ground will be attached.
    let ground_handle = bodies.insert(Ground::new());

    let ground_thickness = r!(0.2);
    let ground = ShapeHandle::new(Cuboid::new(Vector3::new(
        r!(3.0),
        ground_thickness,
        r!(3.0),
    )));

    let co = ColliderDesc::new(ground)
        .translation(Vector3::y() * (-ground_thickness - r!(1.0)))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let ground_size = r!(3.0);
    let obstacle = ShapeHandle::new(Cuboid::new(Vector3::new(r!(0.02), r!(0.02), ground_size)));

    let mut obstacle_desc = ColliderDesc::new(obstacle);

    let co = obstacle_desc
        .set_translation(Vector3::new(r!(0.4), r!(-0.01), r!(0.0)))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let co = obstacle_desc
        .set_translation(Vector3::new(r!(-0.4), r!(-0.01), r!(0.0)))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Create the deformable body and a collider for its boundary.
     */
    let mut fem_body = FEMVolumeDesc::cube(20, 1, 1)
        .scale(Vector3::new(r!(1.0), r!(0.1), r!(0.1)))
        .translation(Vector3::y() * r!(0.1))
        .young_modulus(r!(1.0e3))
        .poisson_ratio(r!(0.2))
        .mass_damping(r!(0.2))
        .build();
    let boundary_desc = fem_body.boundary_collider_desc();
    let fem_body_handle = bodies.insert(fem_body);

    let co = boundary_desc.build(fem_body_handle);
    colliders.insert(co);

    /*
     * Set up the testbed.
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
    testbed.look_at(Point3::new(0.0, 0.0, 2.0), Point3::new(0.0, 0.0, 0.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("FEM volume", init_world)]);
    testbed.run()
}
