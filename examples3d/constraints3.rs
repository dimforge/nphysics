extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::{BallConstraint, PinSlotConstraint, PlanarConstraint, PrismaticConstraint,
                        RectangularConstraint, RevoluteConstraint, UniversalConstraint};
use nphysics3d::object::{BodyPartHandle, ColliderDesc, RigidBodyDesc};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;
use std::f32::consts::{FRAC_PI_2, PI};


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Ground
     */
    let ground_thickness = 0.2;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::new(3.0, ground_thickness, 10.0)));

    ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * (-ground_thickness - 5.0))
        .build(&mut world);

    /*
     * Geometries that will be re-used for several multibody links..
     */
    let rad = 0.2;
    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));

    let collider_desc = ColliderDesc::new(cuboid.clone())
        .density(1.0);

    /*
     * Revolute joints.
     */
    let num = 6;
    let mut parent = BodyPartHandle::ground();
    let first_anchor = Point3::new(0.0, 5.0, 11.0);
    let mut pos = first_anchor.coords;

    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    for i in 0usize..num {
        let body_anchor = Point3::new(0.0, 0.0, 1.0) * (rad * 3.0 + 0.2);
        let parent_anchor = if i == 0 {
            first_anchor
        } else {
            Point3::origin()
        };

        pos -= body_anchor.coords;

        let rb_handle = rb_desc
            .set_translation(pos)
            .build(&mut world)
            .part_handle();

        let constraint = RevoluteConstraint::new(
            parent,
            rb_handle,
            parent_anchor,
            Vector3::x_axis(),
            body_anchor,
            Vector3::x_axis(),
        );

        world.add_constraint(constraint);

        parent = rb_handle;
    }

    /*
     * Prismatic constraint.
     */
    let first_anchor = Point3::new(0.0, 5.0, 4.0);
    let mut pos = first_anchor.coords;
    parent = BodyPartHandle::ground();

    for i in 0usize..3 {
        let mut body_anchor = Point3::origin();
        let mut parent_anchor = Point3::origin();
        if i == 0 {
            parent_anchor = first_anchor;
        } else {
            body_anchor = Point3::new(0.0, 0.0, -1.0) * (rad * 3.0);
        }

        pos -= body_anchor.coords;

        let rb_handle = rb_desc
            .set_translation(pos)
            .build(&mut world)
            .part_handle();

        let mut constraint =
            PrismaticConstraint::new(parent, rb_handle, parent_anchor, Vector3::y_axis(), body_anchor);

        constraint.enable_min_offset(-rad * 2.0);

        world.add_constraint(constraint);

        parent = rb_handle;
    }

    /*
     * Ball constraint.
     */
    let first_anchor = Point3::new(0.0, 5.0, 0.0);
    let mut pos = first_anchor.coords;
    parent = BodyPartHandle::ground();

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

        let rb_handle = rb_desc
            .set_translation(pos)
            .build(&mut world)
            .part_handle();

        let constraint = BallConstraint::new(parent, rb_handle, parent_anchor, body_anchor);
        world.add_constraint(constraint);
        parent = rb_handle;
    }

    /*
     * Universal constraint.
     */
    let parent_pos = Vector3::new(0.0, 5.0, -5.0);
    let child_pos = Vector3::new(0.0, 5.0, -6.0);

    ColliderDesc::new(cuboid)
        .set_translation(parent_pos)
        .build(&mut world);

    let rb_handle = rb_desc
        .set_translation(child_pos)
        .build(&mut world)
        .part_handle();

    let constraint = UniversalConstraint::new(
        BodyPartHandle::ground(),
        rb_handle,
        Point3::from(parent_pos),
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

            let rb_handle = rb_desc
                .set_translation(shift + Vector3::new(0.0, y, z))
                .build(&mut world)
                .part_handle();

            let constraint = PlanarConstraint::new(
                BodyPartHandle::ground(),
                rb_handle,
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

            let rb_handle = rb_desc
                .set_translation(shift + Vector3::new(0.0, y, z))
                .build(&mut world)
                .part_handle();

            let constraint = RectangularConstraint::new(
                BodyPartHandle::ground(),
                rb_handle,
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
    let collider_desc = ColliderDesc::new(cuboid)
        .density(1.0);

    let pin_handle = RigidBodyDesc::new()
        .collider(&collider_desc)
        .build(&mut world)
        .part_handle();

    let constraint = PinSlotConstraint::new(
        BodyPartHandle::ground(),
        pin_handle,
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
    testbed.set_world(world);
    testbed.look_at(Point3::new(30.0, -2.0, 0.0), Point3::new(0.0, -2.0, 0.0));
}

fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}
