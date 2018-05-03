extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Point2, Translation2, Vector2};
use ncollide2d::shape::{Ball, Cuboid, Plane, ShapeHandle};
use nphysics2d::joint::RevoluteJoint;
use nphysics2d::math::{Inertia, Velocity};
use nphysics2d::object::{BodyHandle, BodyStatus, Material};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;
use std::f32::consts::PI;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, 9.81));

    /*
     * Plane
     */
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(20.0, 20.0)));
    let ground_pos = Isometry2::new(Vector2::y() * 20.0, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Create the boxes
     */
    let num = 8;
    let rad = 0.2;
    let mut parent = BodyHandle::ground();

    let revo = RevoluteJoint::new(0.0);
    let geom = ShapeHandle::new(Ball::new(rad));
    let inertia = geom.inertia(5.0);
    let center_of_mass = geom.center_of_mass();

    for j in 0usize..num {
        /*
         * Create the rigid body.
         */
        let mut body_shift = Vector2::zeros();
        if j == 0 {
            body_shift.y = -8.0;
        }

        parent = world.add_multibody_link(
            parent,
            revo,
            body_shift,
            Vector2::new(-rad * 3.0, 0.0),
            inertia,
            center_of_mass,
        );

        /*
         * Create the collider.
         */
        world.add_collider(
            COLLIDER_MARGIN,
            geom.clone(),
            parent,
            Isometry2::identity(),
            Material::default(),
        );
    }

    /*
     * Create the boxes
     */
    let num = 10;
    let rad = 0.2;
    let shift   = rad * 2.0 /*+ 1.0e-4*/;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0 + 3.04;
    let centerz = shift * (num as f32) / 2.0;

    let geom = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad - COLLIDER_MARGIN)));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for i in 0usize..num {
        for j in 0usize..num {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery;

            /*
             * Create the rigid body.
             */
            let pos = Isometry2::new(Vector2::new(x, -y), na::zero());
            let handle = world.add_rigid_body(pos, inertia, center_of_mass);

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
                Isometry2::identity(),
                Material::default(),
            );
        }
    }

    /*
     * Setup a kinematic rigid body.
     */
    let geom = ShapeHandle::new(Cuboid::new(Vector2::new(rad * 10.0, rad)));
    let pos = Isometry2::new(Vector2::new(0.0, -1.5), na::zero());
    let platform_handle = world.add_rigid_body(pos, Inertia::zero(), Point2::origin());
    {
        let rb = world.rigid_body_mut(platform_handle).unwrap();
        rb.set_status(BodyStatus::Kinematic);
        rb.set_velocity(Velocity::linear(1.0, 0.0));
    }
    world.add_collider(
        COLLIDER_MARGIN,
        geom.clone(),
        platform_handle,
        Isometry2::identity(),
        Material::default(),
    );

    // /*
    //  * Setup a kinematic multibody.
    //  */
    // let geom      = ShapeHandle::new(Ball::new(rad - COLLIDER_MARGIN));
    // let joint     = RevoluteJoint::new(0.0);
    // let inertia   = Inertia::zero();
    // let joint_pos = Isometry2::new(Vector2::new(5.0, -2.0), na::zero());
    // let handle    = world.add_multibody_link(BodyHandle::ground(), joint, Vector2::new(2.0, 0.0), joint_pos, inertia);

    // {
    //     let mut mb = world.multibody_mut(handle).unwrap();
    //     mb.generalized_velocity_mut()[0] = 3.0;
    //     mb.set_status(BodyStatus::Kinematic);
    // }

    // world.add_collider(COLLIDER_MARGIN, geom, handle, Isometry2::identity(), true);

    /*
     * Setup a motorized multibody.
     */
    let geom = ShapeHandle::new(Ball::new(2.0 * rad - COLLIDER_MARGIN));
    let mut joint = RevoluteJoint::new(0.0);
    joint.set_desired_angular_motor_velocity(-2.0);
    joint.set_max_angular_motor_torque(2.0);
    joint.enable_angular_motor();
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();
    let handle = world.add_multibody_link(
        BodyHandle::ground(),
        joint,
        Vector2::new(-4.0, -3.0),
        Vector2::x() * 2.0,
        inertia,
        center_of_mass,
    );

    world.add_collider(
        COLLIDER_MARGIN,
        geom,
        handle,
        Isometry2::identity(),
        Material::default(),
    );

    /*
     * Setup a callback to control the platform.
     */
    let mut testbed = Testbed::new(world);
    testbed.add_callback(move |world, time| {
        let platform = world.rigid_body_mut(platform_handle).unwrap();
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
    });

    /*
     * Run the simulation.
     */
    testbed.run();
}
