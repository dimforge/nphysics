extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::f32::consts::PI;
use na::{Isometry3, Point3, Unit, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::world::World;
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::joint::{BallConstraint, PinSlotConstraint, PrismaticConstraint, RevoluteConstraint};
use nphysics3d::volumetric::Volumetric;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    // Material.
    let material = Material::default();

    /*
     * Ground
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry3::new(Vector3::y() * (-ground_size - 5.0), na::zero());
    // let ground_shape = ShapeHandle::new(Plane::new(Vector3::y_axis()));
    // let ground_pos = Isometry3::identity();

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        material.clone(),
    );

    /*
     * Geometries that will be re-used for several multibody links..
     */
    let rad = 0.2;
    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    let cuboid_inertia = cuboid.inertia(1.0);
    let cuboid_center_of_mass = cuboid.center_of_mass();

    /*
     * Revolute joints.
     */
    let num = 6;
    let mut parent = BodyHandle::ground();
    let first_anchor = Point3::new(0.0, 5.0, 11.0);
    let mut pos = first_anchor.coords;

    for i in 0usize..num {
        let body_anchor = Point3::new(0.0, 0.0, 1.0) * (rad * 3.0 + 0.2);
        let parent_anchor = if i == 0 {
            first_anchor
        } else {
            Point3::origin()
        };

        pos -= body_anchor.coords;

        let rb = world.add_rigid_body(
            Isometry3::new(pos, na::zero()),
            cuboid_inertia,
            cuboid_center_of_mass,
        );

        world.add_collider(
            COLLIDER_MARGIN,
            cuboid.clone(),
            rb,
            Isometry3::identity(),
            material.clone(),
        );

        let constraint = RevoluteConstraint::new(
            parent,
            rb,
            parent_anchor,
            Vector3::x_axis(),
            body_anchor,
            Vector3::x_axis(),
        );

        world.add_constraint(constraint);

        parent = rb;
    }

    /*
     * Prismatic joint.
     */
    let first_anchor = Point3::new(0.0, 5.0, 5.0);
    let mut pos = first_anchor.coords;
    parent = BodyHandle::ground();

    for i in 0usize..2 {
        let mut body_anchor = Point3::origin();
        let mut parent_anchor = Point3::origin();
        if i == 0 {
            parent_anchor = first_anchor;
        } else {
            body_anchor = Point3::new(0.0, 0.0, 1.0) * (rad * 3.0);
        }

        pos -= body_anchor.coords;

        let rb = world.add_rigid_body(
            Isometry3::new(pos, na::zero()),
            cuboid_inertia,
            cuboid_center_of_mass,
        );

        world.add_collider(
            COLLIDER_MARGIN,
            cuboid.clone(),
            rb,
            Isometry3::identity(),
            material.clone(),
        );

        let constraint = PrismaticConstraint::new(
            parent,
            rb,
            parent_anchor,
            Vector3::y_axis(),
            body_anchor,
            Vector3::y_axis(),
        );

        world.add_constraint(constraint);

        parent = rb;
    }

    /*
     * Ball joint.
     */
    let first_anchor = Point3::new(0.0, 5.0, 0.0);
    let mut pos = first_anchor.coords;
    parent = BodyHandle::ground();

    for i in 0usize..num {
        let angle = i as f32 * 2.0 * PI / (num as f32);
        let mut body_anchor = Point3::origin();
        let mut parent_anchor = Point3::origin();
        if i == 0 {
            parent_anchor = first_anchor;
        } else {
            body_anchor = Point3::new(angle.cos(), 0.3, angle.sin()) * (rad * 5.0);
        }

        pos -= body_anchor.coords;

        let rb = world.add_rigid_body(
            Isometry3::new(pos, na::zero()),
            cuboid_inertia,
            cuboid_center_of_mass,
        );

        world.add_collider(
            COLLIDER_MARGIN,
            cuboid.clone(),
            rb,
            Isometry3::identity(),
            material.clone(),
        );

        let constraint = BallConstraint::new(parent, rb, parent_anchor, body_anchor);

        world.add_constraint(constraint);

        parent = rb;
    }

    /*
    /*
     * Ball joint.
     */
    parent = BodyHandle::ground();
    for i in 0usize..num {
        // The multibody links are initialized along a circle.
        let angle = i as f32 * 2.0 * PI / (num as f32);
        let shift = rad * 5.0;
        let mut parent_shift = Vector3::zeros();
        let mut body_shift = Vector3::new(angle.cos(), 0.3, angle.sin()) * shift;

        if i == 0 {
            parent_shift = Vector3::new(0.0, 5.0, 0.0);
        }

        let spherical = BallJoint::new(na::zero());
        parent = world.add_multibody_link(
            parent,
            spherical,
            parent_shift,
            body_shift,
            cuboid_inertia,
            cuboid_center_of_mass,
        );
        world.add_collider(
            COLLIDER_MARGIN,
            cuboid.clone(),
            parent,
            Isometry3::identity(),
            material.clone(),
        );
    }
    // Setup damping for the whole multibody.
    world.multibody_mut(parent).unwrap().damping_mut().fill(0.1);

    /*
     * Universal joint.
     */
    let axis1 = Vector3::x_axis();
    let axis2 = Vector3::z_axis();
    let fixed = FixedJoint::new(Isometry3::identity());
    let mut uni = UniversalJoint::new(axis1, axis2, 0.0, 0.0);
    uni.enable_angular_motor_2();
    uni.set_desired_angular_motor_velocity_2(5.0);

    let parent_shift = Vector3::new(0.0, 3.0, -5.0);
    let body_shift = -Vector3::z();

    parent = world.add_multibody_link(
        BodyHandle::ground(),
        fixed,
        parent_shift,
        na::zero(),
        cuboid_inertia,
        cuboid_center_of_mass,
    );
    world.add_collider(
        COLLIDER_MARGIN,
        cuboid.clone(),
        parent,
        Isometry3::identity(),
        material.clone(),
    );

    parent = world.add_multibody_link(
        parent,
        uni,
        na::zero(),
        body_shift,
        cuboid_inertia,
        cuboid_center_of_mass,
    );
    world.add_collider(
        COLLIDER_MARGIN,
        cuboid.clone(),
        parent,
        Isometry3::identity(),
        material.clone(),
    );

    /*
     * Helical joint.
     */
    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    let cuboid_inertia = cuboid.inertia(1.0);
    let axis = Vector3::y_axis();

    let mut hel = HelicalJoint::new(axis, 1.0, 0.0);
    hel.set_desired_angular_motor_velocity(4.0);

    let parent_shift = Vector3::new(0.0, -2.0, 10.0);

    let hel_handle = world.add_multibody_link(
        BodyHandle::ground(),
        hel,
        parent_shift,
        na::zero(),
        cuboid_inertia,
        cuboid_center_of_mass,
    );
    world.add_collider(
        COLLIDER_MARGIN,
        cuboid.clone(),
        hel_handle,
        Isometry3::identity(),
        material.clone(),
    );

    /*
     * Planar joint.
     */
    let axis1 = Vector3::z_axis();
    let axis2 = Vector3::y_axis();
    let shift = Vector3::new(0.0, -2.0, 5.0);
    let width = 5.0 * rad * 4.0;
    for i in 0..5 {
        for j in 0..5 {
            let mut x = i as f32 * rad * 4.0 - width / 2.0;
            let y = j as f32 * rad * 4.0 - width / 2.0;

            if j % 2 == 0 {
                x += rad * 2.0;
            }

            let mut planar = PlanarJoint::new(axis1, axis2, x, y, 0.0);
            planar.enable_min_offset_1(-width / 2.0);
            planar.enable_max_offset_1(width / 2.0);
            planar.enable_min_offset_2(-5.0);
            let handle = world.add_multibody_link(
                BodyHandle::ground(),
                planar,
                shift,
                na::zero(),
                cuboid_inertia,
                cuboid_center_of_mass,
            );
            world.add_collider(
                COLLIDER_MARGIN,
                cuboid.clone(),
                handle,
                Isometry3::identity(),
                material.clone(),
            );
        }
    }

    /*
     * Rectangular joint.
     */
    let axis1 = Vector3::z_axis();
    let axis2 = Vector3::y_axis();
    let shift = Vector3::new(0.0, -2.0, 0.0);
    let width = 5.0 * rad * 4.0;
    for i in 0..5 {
        for j in 0..5 {
            let mut x = i as f32 * rad * 4.0 - width / 2.0;
            let y = j as f32 * rad * 4.0 - width / 2.0;

            if j % 2 == 0 {
                x += rad * 2.0;
            }

            let mut rect = RectangularJoint::new(axis1, axis2, x, y);
            rect.enable_min_offset_1(-width / 2.0);
            rect.enable_max_offset_1(width / 2.0);
            rect.enable_min_offset_2(-5.0);
            let handle = world.add_multibody_link(
                BodyHandle::ground(),
                rect,
                shift,
                na::zero(),
                cuboid_inertia,
                cuboid_center_of_mass,
            );
            world.add_collider(
                COLLIDER_MARGIN,
                cuboid.clone(),
                handle,
                Isometry3::identity(),
                material.clone(),
            );
        }
    }*/

    /*
     * Pin-slot joint.
     */
    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::new(rad * 5.0, rad, rad * 5.0)));
    let cuboid_inertia = cuboid.inertia(1.0);
    let cuboid_center_of_mass = cuboid.center_of_mass();
    let rb = world.add_rigid_body(Isometry3::identity(), cuboid_inertia, cuboid_center_of_mass);

    world.add_collider(
        COLLIDER_MARGIN,
        cuboid.clone(),
        rb,
        Isometry3::identity(),
        material.clone(),
    );

    let constraint = PinSlotConstraint::new(
        BodyHandle::ground(),
        rb,
        Point3::origin(),
        Vector3::y_axis(),
        Vector3::x_axis(),
        Point3::origin(),
        Vector3::y_axis(),
        Vector3::x_axis(),
    );

    world.add_constraint(constraint);

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    /*
    testbed.add_callback(move |world, _| {
        /*
         * Activate the helical joint motor if it is to low.
         */
        // Might be None if the user interactively deleted the helical body.
        if let Some(mut helical) = world.multibody_link_mut(hel_handle) {
            let dof = helical
                .joint_mut()
                .downcast_mut::<HelicalJoint<f32>>()
                .unwrap();

            if dof.offset() < -5.0 {
                dof.enable_angular_motor();
            } else if dof.offset() > 0.0 {
                dof.disable_angular_motor();
            }
        }
    });

    testbed.add_callback(move |world, _| {
        /*
         * Activate the pin-slot joint linear motor if it is to low.
         */
        // Might be None if the user interactively deleted the pin-slot body.
        if let Some(mut pin_slot) = world.multibody_link_mut(pin_handle) {
            let dof = pin_slot
                .joint_mut()
                .downcast_mut::<PinSlotJoint<f32>>()
                .unwrap();

            if dof.offset() < -10.0 {
                dof.enable_linear_motor();
            } else if dof.offset() > -4.0 {
                dof.disable_linear_motor();
            }
        }
    });
    */

    testbed.look_at(Point3::new(30.0, -2.0, 0.0), Point3::new(0.0, -2.0, 0.0));
    testbed.run();
}
