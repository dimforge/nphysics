extern crate nalgebra as na;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed3d::{r, Real, Testbed};
use std::f32;

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
    let ground_thickness = r!(0.2);
    let ground_half_width = 3.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(
        ground_half_width,
        ground_thickness,
        ground_half_width,
    )));

    let ground_handle = bodies.insert(Ground::new());

    let wall_poses = [
        Isometry3::translation(r!(0.0), -ground_half_width, r!(0.0)),
        Isometry3::translation(r!(0.0), ground_half_width, r!(0.0)),
        Isometry3::new(
            Vector3::x() * ground_half_width,
            Vector3::z() * r!(f32::consts::PI) / r!(2.0),
        ),
        Isometry3::new(
            Vector3::x() * -ground_half_width,
            Vector3::z() * r!(f32::consts::PI) / r!(2.0),
        ),
        Isometry3::new(
            Vector3::z() * ground_half_width,
            Vector3::x() * r!(f32::consts::PI) / r!(2.0),
        ),
        Isometry3::new(
            Vector3::z() * -ground_half_width,
            Vector3::x() * r!(f32::consts::PI) / r!(2.0),
        ),
    ];

    for pose in wall_poses.iter() {
        let co = ColliderDesc::new(ground_shape.clone())
            .position(*pose)
            .build(BodyPartHandle(ground_handle, 0));
        colliders.insert(co);
    }

    /*
     * Create the boxes.
     */
    let num = 4;
    let rad = r!(0.1);

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));

    let shift = (rad + ColliderDesc::<Real>::default_margin()) * r!(2.0);
    let centerx = shift * r!(num as f32) / r!(2.0);
    let centery = shift / r!(2.0);
    let centerz = shift * r!(num as f32) / r!(2.0);
    let height = 1.5;

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = r!(i as f32) * shift - centerx;
                let y = r!(j as f32) * shift + centery + height;
                let z = r!(k as f32) * shift - centerz;

                // Build the rigid body.
                let rb = RigidBodyDesc::new()
                    .translation(Vector3::new(x, y, z))
                    .velocity(nphysics3d::math::Velocity::linear(
                        r!(-100.0),
                        r!(-10.0),
                        r!(0.0),
                    ))
                    .build();
                let rb_handle = bodies.insert(rb);

                // Build the collider.
                let co = ColliderDesc::new(cuboid.clone())
                    .ccd_enabled(true)
                    .density(r!(1.0))
                    .build(BodyPartHandle(rb_handle, 0));
                colliders.insert(co);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_body_wireframe(ground_handle, true);
    testbed.set_ground_handle(Some(ground_handle));
    testbed.allow_grabbing_behind_ground(true);
    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.look_at(Point3::new(-11.0, 0.2, -9.0), Point3::new(-1.0, -0.6, 1.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("CCD", init_world)]);
    testbed.run()
}
