extern crate nalgebra as na;

use na::{Point3, Vector3, Isometry3};
use ncollide3d::shape::{Compound, Cuboid, ShapeHandle};
use nphysics3d::object::{ColliderDesc, RigidBodyDesc, DefaultBodySet, DefaultColliderSet, Ground, BodyPartHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::world::{DefaultDynamicWorld, DefaultColliderWorld};
use nphysics_testbed3d::Testbed;


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let dynamic_world = DefaultDynamicWorld::new(Vector3::new(0.0, -9.81, 0.0));
    let collider_world = DefaultColliderWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Ground.
     */
    let ground_thickness = 0.2;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::new(3.0, ground_thickness, 3.0)));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_thickness)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Cross shaped geometry
     */
    let mut cross_geoms = Vec::new();

    let large_rad = 0.25f32;
    let small_rad = 0.01f32;

    let edge_x = Cuboid::new(Vector3::new(large_rad, small_rad, small_rad));
    let edge_y = Cuboid::new(Vector3::new(small_rad, large_rad, small_rad));
    let edge_z = Cuboid::new(Vector3::new(small_rad, small_rad, large_rad));

    cross_geoms.push((Isometry3::identity(), ShapeHandle::new(edge_x)));
    cross_geoms.push((Isometry3::identity(), ShapeHandle::new(edge_y)));
    cross_geoms.push((Isometry3::identity(), ShapeHandle::new(edge_z)));

    let compound = Compound::new(cross_geoms);
    let cross = ShapeHandle::new(compound);

    /*
     * Create the crosses
     */
    let num = 5;
    let shift = (large_rad + 0.08) * 2.0;
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
    testbed.set_world(dynamic_world, collider_world, bodies, colliders, joint_constraints, force_generators);
    testbed.look_at(Point3::new(-4.0, 1.0, -4.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![
        ("Crosses", init_world),
    ]);

    testbed.run()
}
