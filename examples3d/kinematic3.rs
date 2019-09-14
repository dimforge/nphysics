extern crate nalgebra as na;

use na::{Point3, Vector3};
use ncollide3d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::{DefaultJointConstraintSet, RevoluteJoint};
use nphysics3d::object::{
    Body, BodyPartHandle, BodyStatus, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground,
    MultibodyDesc, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
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
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(10.0, ground_thickness, 10.0)));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_thickness)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Create the boxes
     */
    let num = 6;
    let rad = 0.2;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    let cuboid_collider_desc = ColliderDesc::new(cuboid).density(1.0);

    let shift = (rad + cuboid_collider_desc.get_margin()) * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0 + 3.04;
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
                let co = cuboid_collider_desc.build(BodyPartHandle(rb_handle, 0));
                colliders.insert(co);
            }
        }
    }

    /*
     * Setup a kinematic rigid body.
     */

    let platform_body = RigidBodyDesc::new()
        .translation(Vector3::new(0.0, 1.5 + 0.8, -10.0 * rad))
        .status(BodyStatus::Kinematic)
        .build();
    let platform_handle = bodies.insert(platform_body);

    let geom = ShapeHandle::new(Cuboid::new(Vector3::new(rad * 10.0, rad, rad * 10.0)));
    let platform_collider = ColliderDesc::new(geom)
        .density(1.0)
        .build(BodyPartHandle(platform_handle, 0));
    colliders.insert(platform_collider);

    /*
     * Setup a kinematic multibody.
     */
    let joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);

    let mut mb = MultibodyDesc::new(joint)
        .body_shift(Vector3::z() * 2.0)
        .parent_shift(Vector3::new(0.0, 2.0, 5.0))
        .build();

    mb.set_status(BodyStatus::Kinematic);
    mb.generalized_velocity_mut()[0] = 3.0;

    let mb_handle = bodies.insert(mb);
    let mb_collider = cuboid_collider_desc.build(BodyPartHandle(mb_handle, 0));
    colliders.insert(mb_collider);

    /*
     * Setup a motorized multibody.
     */
    let mut joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);
    joint.set_desired_angular_motor_velocity(-2.0);
    joint.set_max_angular_motor_torque(1.0);
    joint.enable_angular_motor();

    let mb = MultibodyDesc::new(joint)
        .body_shift(Vector3::z() * 2.0)
        .parent_shift(Vector3::new(0.0, 3.0, -4.0))
        .build();

    let mb_handle = bodies.insert(mb);
    let geom = ShapeHandle::new(Ball::new(2.0 * rad));
    let ball_collider_desc = ColliderDesc::new(geom).density(1.0);
    let mb_collider = ball_collider_desc.build(BodyPartHandle(mb_handle, 0));
    colliders.insert(mb_collider);

    /*
     * Setup a callback to control the platform.
     */
    testbed.add_callback(move |_, _, bodies, _, _, time| {
        let platform = bodies.rigid_body_mut(platform_handle).unwrap();
        let platform_z = platform.position().translation.z;

        let mut vel = *platform.velocity();
        vel.linear.y = (time * 5.0).sin() * 0.8;

        if platform_z >= rad * 10.0 {
            vel.linear.z = -1.0;
        }
        if platform_z <= -rad * 10.0 {
            vel.linear.z = 1.0;
        }

        platform.set_velocity(vel);
    });

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
    testbed.look_at(Point3::new(-10.0, 5.0, -10.0), Point3::new(0.0, 0.0, 0.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Kinematic body", init_world)]);
    testbed.run()
}
