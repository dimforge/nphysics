extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Point3, Vector3};
use ncollide3d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics3d::object::{ColliderDesc, RigidBodyDesc, DefaultBodySet, DefaultColliderSet, Ground, BodyPartHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::world::{DefaultDynamicWorld, DefaultColliderWorld};
use nphysics_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut dynamic_world = DefaultDynamicWorld::new();
    let mut collider_world = DefaultColliderWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();

    dynamic_world.gravity = Vector3::new(0.0, -9.81, 0.0);

    /*
     * Ground.
     */
    let ground_handle = bodies.insert(Box::new(Ground::new()));

    let ground_thickness = 0.2;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::new(3.0, ground_thickness, 3.0)));

    let co = ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_thickness)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);


    /*
     * Create the boxes
     */
    let num = 8;
    let rad = 0.1;

    let ball = ShapeHandle::new(Ball::new(rad));
    let collider_desc = ColliderDesc::new(ball)
        .density(1.0);

    let shift = (rad + collider_desc.get_margin()) * 2.0 + 0.002;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;
    let height = 3.0;

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery + height;
                let z = k as f32 * shift - centerz;

                // Build the rigid body.
                let rb = RigidBodyDesc::new()
                    .translation(Vector3::new(x, y, z))
                    .build();
                let rb_handle = bodies.insert(Box::new(rb));

                // Build the collider.
                let co = collider_desc.build(BodyPartHandle(rb_handle, 0));
                colliders.insert(co);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(
        dynamic_world,
        collider_world,
        bodies,
        colliders,
        DefaultJointConstraintSet::new(),
        DefaultForceGeneratorSet::new()
    );
    testbed.look_at(Point3::new(-4.0, 1.0, -4.0), Point3::new(0.0, 1.0, 0.0));
}

fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}