extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Point2, Vector2};
use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics2d::joint::RevoluteJoint;
use nphysics2d::math::Velocity;
use nphysics2d::object::{BodyStatus, ColliderDesc, RigidBodyDesc, MultibodyDesc, Body};
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;


fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Plane
     */
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(20.0, 20.0)));

    ColliderDesc::new(ground_shape)
        .translation(Vector2::y() * -20.0)
        .build(&mut world);

    /*
     * Create the boxes
     */
    let num = 10;
    let rad = 0.2;

    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let collider_desc = ColliderDesc::new(cuboid)
        .density(1.0);

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    let shift = (rad + collider_desc.get_margin()) * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0 + 3.04;

    for i in 0usize..num {
        for j in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery;

            // Build the rigid body and its collider.
            rb_desc
                .set_translation(Vector2::new(x, y))
                .build(&mut world);
        }
    }

    /*
     * Setup a kinematic rigid body.
     */
    let geom = ShapeHandle::new(Cuboid::new(Vector2::new(rad * 10.0, rad)));
    let collider_desc = ColliderDesc::new(geom)
        .density(1.0);

    let platform_handle = RigidBodyDesc::new()
        .collider(&collider_desc)
        .translation(Vector2::new(0.0, 1.5))
        .velocity(Velocity::linear(1.0, 0.0))
        .status(BodyStatus::Kinematic)
        .build(&mut world);

    /*
     * Setup a kinematic multibody.
     */
    let joint = RevoluteJoint::new(0.0);

    let mb = MultibodyDesc::new(joint)
        .body_shift(Vector2::x() * 2.0)
        .parent_shift(Vector2::new(5.0, 2.0))
        .collider(&collider_desc)
        .build(&mut world);

    {
        let mut mb_write = world.multibody_mut(mb).unwrap();
        mb_write.set_status(BodyStatus::Kinematic);
        mb_write.generalized_velocity_mut()[0] = -3.0;
    }

    /*
     * Setup a motorized multibody.
     */
    let mut joint = RevoluteJoint::new(0.0);
    joint.set_desired_angular_motor_velocity(-2.0);
    joint.set_max_angular_motor_torque(2.0);
    joint.enable_angular_motor();

    let geom = ShapeHandle::new(Ball::new(2.0 * rad));
    let ball_collider_desc = ColliderDesc::new(geom).density(1.0);
    MultibodyDesc::new(joint)
        .body_shift(Vector2::x() * 2.0)
        .parent_shift(Vector2::new(-4.0, 3.0))
        .collider(&ball_collider_desc)
        .build(&mut world);

    /*
     * Setup a callback to control the platform.
     */
    let mut testbed = Testbed::new(world);
    testbed.add_callback(move |world, _, time| {
        let mut world = world.get_mut();
        if let Some(mut platform) = world.rigid_body_mut(platform_handle) {
            let platform_x = platform.position().translation.vector.x;

            let mut vel = *platform.velocity();
            vel.linear.y = (time * 5.0).sin() * 0.8;

            if platform_x >= rad * 10.0 {
                vel.linear.x = -1.0;
            }
            if platform_x <= -rad * 10.0 {
                vel.linear.x = 1.0;
            }

            platform.set_velocity(vel);
        };
    });

    /*
     * Run the simulation.
     */
    testbed.look_at(Point2::new(0.0, -5.0), 60.0);
    testbed.run();
}
