extern crate env_logger;
extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::{BallConstraint, PinSlotConstraint, PlanarConstraint, PrismaticConstraint,
                        RectangularConstraint, RevoluteConstraint, UniversalConstraint};
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;
use std::f32::consts::{FRAC_PI_2, PI};

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    env_logger::init();
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Ground
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry3::new(Vector3::y() * (-ground_size - 5.0), na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
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
            Material::default(),
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
     * Prismatic constraint.
     */
    let first_anchor = Point3::new(0.0, 5.0, 4.0);
    let mut pos = first_anchor.coords;
    parent = BodyHandle::ground();

    for i in 0usize..3 {
        let mut body_anchor = Point3::origin();
        let mut parent_anchor = Point3::origin();
        if i == 0 {
            parent_anchor = first_anchor;
        } else {
            body_anchor = Point3::new(0.0, 0.0, -1.0) * (rad * 3.0);
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
            Material::default(),
        );

        let mut constraint =
            PrismaticConstraint::new(parent, rb, parent_anchor, Vector3::y_axis(), body_anchor);

        constraint.enable_min_offset(-rad * 2.0);

        world.add_constraint(constraint);

        parent = rb;
    }

    /*
     * Ball constraint.
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
            Material::default(),
        );

        let constraint = BallConstraint::new(parent, rb, parent_anchor, body_anchor);

        world.add_constraint(constraint);

        parent = rb;
    }

    /*
     * Universal constraint.
     */
    let parent_pos = Vector3::new(0.0, 5.0, -5.0);
    let child_pos = Vector3::new(0.0, 5.0, -6.0);

    world.add_collider(
        COLLIDER_MARGIN,
        cuboid.clone(),
        BodyHandle::ground(),
        Isometry3::new(parent_pos, na::zero()),
        Material::default(),
    );

    let rb = world.add_rigid_body(
        Isometry3::new(child_pos, na::zero()),
        cuboid_inertia,
        cuboid_center_of_mass,
    );

    world.add_collider(
        COLLIDER_MARGIN,
        cuboid.clone(),
        rb,
        Isometry3::identity(),
        Material::default(),
    );

    let constraint = UniversalConstraint::new(
        BodyHandle::ground(),
        rb,
        Point3::from_coordinates(parent_pos),
        Vector3::x_axis(),
        Point3::new(0.0, 0.0, 1.0),
        Vector3::z_axis(),
        FRAC_PI_2,
    );

    world.add_constraint(constraint);

    /*
     * Planar constraint.
     */
    let num = 5;
    let shift = Vector3::new(0.0, -2.0, 5.0);
    let width = 5.0 * rad * 4.0;

    for i in 0..num {
        for j in 0..num {
            let mut z = i as f32 * rad * 4.0 - width / 2.0;
            let y = j as f32 * rad * 4.0 - width / 2.0;

            if j % 2 == 0 {
                z += rad * 2.0;
            }

            let rb = world.add_rigid_body(
                Isometry3::new(shift + Vector3::new(na::zero(), y, z), na::zero()),
                cuboid_inertia,
                cuboid_center_of_mass,
            );

            world.add_collider(
                COLLIDER_MARGIN,
                cuboid.clone(),
                rb,
                Isometry3::identity(),
                Material::default(),
            );

            let constraint = PlanarConstraint::new(
                BodyHandle::ground(),
                rb,
                Point3::origin(),
                Vector3::x_axis(),
                Point3::origin(),
                Vector3::x_axis(),
            );

            world.add_constraint(constraint);
        }
    }

    /*
     * Rectangular constraint.
     */
    let shift = Vector3::new(0.0, -2.0, 0.0);
    let width = 5.0 * rad * 4.0;
    for i in 0..5 {
        for j in 0..5 {
            let mut z = i as f32 * rad * 4.0 - width / 2.0;
            let y = j as f32 * rad * 4.0 - width / 2.0;

            if j % 2 == 0 {
                z += rad * 2.0;
            }

            let rb = world.add_rigid_body(
                Isometry3::new(shift + Vector3::new(na::zero(), y, z), na::zero()),
                cuboid_inertia,
                cuboid_center_of_mass,
            );

            world.add_collider(
                COLLIDER_MARGIN,
                cuboid.clone(),
                rb,
                Isometry3::identity(),
                Material::default(),
            );

            let constraint = RectangularConstraint::new(
                BodyHandle::ground(),
                rb,
                Point3::origin(),
                Vector3::x_axis(),
                Point3::origin(),
            );

            world.add_constraint(constraint);
        }
    }

    /*
     * Pin-slot constraint.
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
        Material::default(),
    );

    let constraint = PinSlotConstraint::new(
        BodyHandle::ground(),
        rb,
        Point3::origin(),
        Vector3::y_axis(),
        Vector3::x_axis(),
        Point3::origin(),
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
         * Activate the helical constraint motor if it is to low.
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
         * Activate the pin-slot constraint linear motor if it is to low.
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
