extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::RevoluteJoint;
use nphysics3d::object::{BodyStatus, Body, BodyPart, ColliderDesc, RigidBodyDesc, MultibodyDesc};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Ground.
     */
    let ground_size = 10.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size)));

    ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_size)
        .build(&mut world);


    /*
     * Create the boxes
     */
    let num = 6;
    let rad = 0.2;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    let small_cuboid_collider_desc = ColliderDesc::new(cuboid)
        .density(1.0);

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&small_cuboid_collider_desc);

    let shift = (rad + small_cuboid_collider_desc.get_margin()) * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0 + 3.04;
    let centerz = shift * (num as f32) / 2.0;

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                // Build the rigid body and its collider.
                rb_desc
                    .set_translation(Vector3::new(x, y, z))
                    .build(&mut world);
            }
        }
    }

    /*
     * Setup a kinematic rigid body.
     */
    let geom = ShapeHandle::new(Cuboid::new(Vector3::new(rad * 10.0, rad, rad * 10.0)));

    let collider_desc = ColliderDesc::new(geom)
        .density(1.0);

    let platform_handle = RigidBodyDesc::new()
        .collider(&collider_desc)
        .translation(Vector3::new(0.0, 1.5 + 0.8, -10.0 * rad))
        .status(BodyStatus::Kinematic)
        .build(&mut world)
        .handle();

    /*
     * Setup a kinematic multibody.
     */
    let joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);

    let mb = MultibodyDesc::new(joint)
        .body_shift(Vector3::z() * 2.0)
        .parent_shift(Vector3::new(0.0, 2.0, 5.0))
        .collider(&small_cuboid_collider_desc)
        .build(&mut world);

    mb.set_status(BodyStatus::Kinematic);
    mb.generalized_velocity_mut()[0] = 3.0;

    /*
     * Setup a motorized multibody.
     */
    let mut joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);
    joint.set_desired_angular_motor_velocity(-2.0);
    joint.set_max_angular_motor_torque(1.0);
    joint.enable_angular_motor();

    MultibodyDesc::new(joint)
        .body_shift(Vector3::z() * 2.0)
        .parent_shift(Vector3::new(0.0, 3.0, -4.0))
        .collider(&small_cuboid_collider_desc)
        .build(&mut world);

    /*
     * Setup a callback to control the platform.
     */
    let mut testbed = Testbed::new(world);

    testbed.add_callback(move |world, _, time| {
        let platform = world.rigid_body_mut(platform_handle).unwrap();
        let platform_z = platform.position().translation.z;

        let mut vel = platform.velocity();
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
    testbed.look_at(Point3::new(-10.0, 5.0, -10.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
