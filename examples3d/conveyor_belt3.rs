extern crate nalgebra as na;

use std::f32;

use na::{Point3, RealField, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::material::{BasicMaterial, MaterialHandle};
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed3d::Testbed;

pub fn init_world<N: RealField>(testbed: &mut Testbed<N>) {
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
     * Conveyor belts. We create 8 belts to form a "circle".
     */
    let conveyor_length = r!(5.0);
    let conveyor_half_width = r!(1.0);
    let conveyor_side_shape = ShapeHandle::new(Cuboid::new(Vector3::new(
        conveyor_length - conveyor_half_width,
        r!(0.2),
        conveyor_half_width,
    )));
    let conveyor_corner_shape = ShapeHandle::new(Cuboid::new(Vector3::new(
        conveyor_half_width,
        r!(0.2),
        conveyor_half_width,
    )));
    let conveyor_shift = conveyor_length;

    let conveyor_side_material = BasicMaterial {
        surface_velocity: Some(Vector3::x()),
        ..BasicMaterial::default()
    };
    let conveyor_corner_material = BasicMaterial {
        surface_velocity: Some(Vector3::new(r!(1.0), r!(0.0), r!(-2.0))),
        ..BasicMaterial::default()
    };

    // Build the sides of the conveyor belt "circle".
    //
    // Note that we can re-use the same material everywhere because the surface_velocity
    // is expressed in the local-space of the shape (and we are applying rotations to the
    // colliders so the surface_velocity points toward where we want it to).
    let ground_handle = bodies.insert(Ground::new());

    let mut side_desc = ColliderDesc::new(conveyor_side_shape.clone())
        .material(MaterialHandle::new(conveyor_side_material));

    let co = side_desc
        .set_translation(Vector3::new(r!(0.0), r!(-0.2), conveyor_shift))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let co = side_desc
        .set_translation(Vector3::new(conveyor_shift, r!(-0.2), r!(0.0)))
        .set_rotation(Vector3::y() * r!(std::f64::consts::FRAC_PI_2))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let co = side_desc
        .set_translation(Vector3::new(r!(0.0), r!(-0.2), -conveyor_shift))
        .set_rotation(Vector3::y() * r!(std::f64::consts::PI))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let co = side_desc
        .set_translation(Vector3::new(-conveyor_shift, r!(-0.2), r!(0.0)))
        .set_rotation(Vector3::y() * (r!(std::f64::consts::PI) + r!(std::f64::consts::FRAC_PI_2)))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    // Build the corners of the conveyor belt "circle".

    // Note that we can re-use the same material everywhere because the surface_velocity
    // is expressed in the local-space of the shape (and we are applying rotations to the
    // colliders so the surface_velocity points toward where we want it to).
    let mut corner_desc = ColliderDesc::new(conveyor_corner_shape)
        .material(MaterialHandle::new(conveyor_corner_material));

    let co = corner_desc
        .set_translation(Vector3::new(conveyor_shift, r!(-0.2), conveyor_shift))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let co = corner_desc
        .set_translation(Vector3::new(conveyor_shift, r!(-0.2), -conveyor_shift))
        .set_rotation(Vector3::y() * r!(std::f64::consts::FRAC_PI_2))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let co = corner_desc
        .set_translation(Vector3::new(-conveyor_shift, r!(-0.2), -conveyor_shift))
        .set_rotation(Vector3::y() * r!(std::f64::consts::PI))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let co = corner_desc
        .set_translation(Vector3::new(-conveyor_shift, r!(-0.2), conveyor_shift))
        .set_rotation(Vector3::y() * (r!(std::f64::consts::PI) + r!(std::f64::consts::FRAC_PI_2)))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Create some boxes
     */
    let num = 4;
    let rad = r!(0.1);

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));

    let shift = (rad + ColliderDesc::<N>::default_margin()) * r!(2.0);
    let centerx = shift * r!(num as f64) / r!(2.0) + conveyor_shift;
    let centery = shift / r!(2.0);
    let centerz = shift * r!(num as f64) / r!(2.0);
    let height = r!(0.0);

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = r!(i as f64) * shift - centerx;
                let y = r!(j as f64) * shift + centery + height;
                let z = r!(k as f64) * shift - centerz;

                // Build the rigid body.
                let rb = RigidBodyDesc::new()
                    .translation(Vector3::new(x, y, z))
                    .build();
                let rb_handle = bodies.insert(rb);

                // Build the collider.
                let co = ColliderDesc::new(cuboid.clone())
                    .density(r!(1.0))
                    .build(BodyPartHandle(rb_handle, 0));
                colliders.insert(co);
            }
        }
    }

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
    testbed.look_at(Point3::new(10.0, 4.0, -10.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("Conveyor belt", init_world)]);

    testbed.run()
}
