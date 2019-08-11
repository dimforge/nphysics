extern crate nalgebra as na;

use na::{Point3, Vector3, Isometry3};
use ncollide3d::shape::{Compound, Capsule, Cuboid, ShapeHandle};
use nphysics3d::object::{ColliderDesc, RigidBodyDesc, DefaultBodySet, DefaultColliderSet, Ground, BodyPartHandle};
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
     * Ground.
     */
    let ground_thickness = 0.2;
    let ground_shape =
        ShapeHandle::new_owned(Cuboid::new(Vector3::new(35.0, ground_thickness, 35.0)));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_thickness)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * U-shaped geometry.
     */
    let large_rad = 2.5f32;
    let small_rad = 0.1f32;

    let delta1 = Isometry3::new(Vector3::new(0.0, -large_rad, 0.0), na::zero());
    let delta2 = Isometry3::new(Vector3::new(-large_rad, 0.0, 0.0), na::zero());
    let delta3 = Isometry3::new(Vector3::new(large_rad, 0.0, 0.0), na::zero());

    let mut cross_geoms = Vec::new();
    let vertical = Capsule::new(large_rad, small_rad);
    let horizontal = Cuboid::new(Vector3::new(large_rad, small_rad, small_rad));
    cross_geoms.push((delta1, Box::new(horizontal) as Box<dyn Shape<_>>));
    cross_geoms.push((delta2, Box::new(vertical) as Box<dyn Shape<_>>));
    cross_geoms.push((delta3, Box::new(vertical) as Box<dyn Shape<_>>));

    let compound = Compound::new(cross_geoms);
    let cross = ShapeHandle::new_shared(compound);


    /*
     * Create the crosses
     */
    let num = 6;
    let rad = 5.0;
    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = 3.0 + shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                // Build the rigid body.
                let rb = RigidBodyDesc::new()
                    .translation(Vector3::new(x, y, z))
                    .build();
                let rb_handle = bodies.insert(rb);

                // Build the collider.
                let co = ColliderDesc::new(cross.clone())
                    .density(1.0)
                    .build(BodyPartHandle(rb_handle, 0));
                colliders.insert(co);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(mechanical_world, geometrical_world, bodies, colliders, joint_constraints, force_generators);
    testbed.look_at(Point3::new(-10.0, 10.0, -10.0), Point3::origin());
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![
        ("Compound", init_world),
    ]);

    testbed.run()
}
