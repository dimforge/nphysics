extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::f32::consts::PI;
use na::{Isometry3, Point3, Translation3, Vector3};
use ncollide::shape::{Ball, Cuboid, Plane, ShapeHandle};
use nphysics3d::world::World;
use nphysics3d::object::{BodyHandle, BodyStatus, Material};
use nphysics3d::joint::RevoluteJoint;
use nphysics3d::math::{Inertia, Velocity};
use nphysics3d::volumetric::Volumetric;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    // Materials.
    let material = Material::default();

    /*
     * Plane
     */
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(10.0, 10.0, 10.0)));
    let ground_pos = Isometry3::new(Vector3::y() * -10.0, na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        material.clone(),
    );

    /*
     * Create the boxes
     */
    let num = 8;
    let rad = 0.2;
    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0 + 3.04;
    let centerz = shift * (num as f32) / 2.0;

    let geom = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad - COLLIDER_MARGIN)));
    let inertia = geom.inertia(1.0);

    for i in 0usize..num {
        for j in 0usize..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                /*
                 * Create the rigid body.
                 */
                let pos = Isometry3::new(Vector3::new(x, y, z), na::zero());
                let handle = world.add_rigid_body(pos, inertia);

                // if j == 5 {
                //     let mut rb = world.rigid_body_mut(handle).unwrap();
                //     rb.set_status(BodyStatus::Disabled);
                // }

                // if j == 7 {
                //     let mut rb = world.rigid_body_mut(handle).unwrap();
                //     rb.set_status(BodyStatus::Static);
                // }

                /*
                 * Create the collider.
                 */
                world.add_collider(
                    COLLIDER_MARGIN,
                    geom.clone(),
                    handle,
                    Isometry3::identity(),
                    material.clone(),
                );
            }
        }
    }

    /*
     * Setup a kinematic rigid body.
     */
    let geom = ShapeHandle::new(Cuboid::new(Vector3::new(rad * 10.0, rad, rad * 10.0)));
    let pos = Isometry3::new(Vector3::new(0.0, 1.5 + 0.8, -10.0 * rad), na::zero());
    let platform_handle = world.add_rigid_body(pos, Inertia::zero());
    {
        let rb = world.rigid_body_mut(platform_handle).unwrap();
        rb.set_status(BodyStatus::Kinematic);
        // rb.set_velocity(Velocity::linear(0.0, 0.0, 1.0));
    }
    world.add_collider(
        COLLIDER_MARGIN,
        geom.clone(),
        platform_handle,
        Isometry3::identity(),
        material.clone(),
    );

    /*
     * Setup a kinematic multibody.
     */
    let geom = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad - COLLIDER_MARGIN))); // Ball::new(rad - COLLIDER_MARGIN));
    let joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);
    let inertia = Inertia::zero();
    let handle = world.add_multibody_link(
        BodyHandle::ground(),
        joint,
        Vector3::new(0.0, 2.0, 5.0),
        Vector3::z() * 2.0,
        inertia,
    );

    {
        let mut mb = world.multibody_mut(handle).unwrap();
        mb.generalized_velocity_mut()[0] = 3.0;
        mb.set_status(BodyStatus::Kinematic);
    }

    world.add_collider(
        COLLIDER_MARGIN,
        geom.clone(),
        handle,
        Isometry3::identity(),
        material.clone(),
    );

    /*
     * Setup a motorized multibody.
     */
    let geom = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad - COLLIDER_MARGIN))); // Ball::new(rad - COLLIDER_MARGIN));
    let mut joint = RevoluteJoint::new(Vector3::x_axis(), 0.0);
    joint.set_desired_angular_motor_velocity(-2.0);
    joint.set_max_angular_motor_torque(1.0);
    joint.enable_angular_motor();
    let inertia = geom.inertia(1.0);
    let handle = world.add_multibody_link(
        BodyHandle::ground(),
        joint,
        Vector3::new(0.0, 3.0, -4.0),
        Vector3::z() * 2.0,
        inertia,
    );

    world.add_collider(
        COLLIDER_MARGIN,
        geom,
        handle,
        Isometry3::identity(),
        material.clone(),
    );

    /*
     * Setup a callback to control the platform.
     */
    let mut testbed = Testbed::new(world);
    testbed.add_callback(move |world, time| {
        let platform = world.rigid_body_mut(platform_handle).unwrap();
        let platform_z = platform.position().translation.vector.z;

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
    testbed.look_at(Point3::new(-10.0, 10.0, -10.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
