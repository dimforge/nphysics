extern crate nalgebra as na;

use na::{Point3, RealField, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::joint::{
    BallConstraint, PinSlotConstraint, PlanarConstraint, PrismaticConstraint,
    RectangularConstraint, RevoluteConstraint, UniversalConstraint,
};
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};

use nphysics_testbed3d::Testbed;
use std::f64::consts::{FRAC_PI_2, PI};

/*
 * NOTE: The `r` macro is only here to convert from f64 to the `N` scalar type.
 * This simplifies experimentation with various scalar types (f32, fixed-point numbers, etc.)
 */
pub fn init_world<N: RealField>(testbed: &mut Testbed<N>) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector3::new(r!(0.0), r!(-9.81), r!(0.0)));
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let mut joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Ground
     */
    let ground_thickness = r!(0.2);
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(
        r!(3.0),
        ground_thickness,
        r!(10.0),
    )));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * (-ground_thickness - r!(5.0)))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Geometries that will be re-used for several multibody links..
     */
    let rad = r!(0.2);
    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));

    let collider_desc = ColliderDesc::new(cuboid.clone()).density(r!(1.0));

    /*
     * Revolute joints.
     */
    let num = 6;
    let mut parent = BodyPartHandle(ground_handle, 0);
    let first_anchor = Point3::new(r!(0.0), r!(5.0), r!(11.0));
    let mut pos = first_anchor.coords;

    for i in 0usize..num {
        let body_anchor = Point3::new(r!(0.0), r!(0.0), r!(1.0)) * (rad * r!(3.0) + r!(0.2));
        let parent_anchor = if i == 0 {
            first_anchor
        } else {
            Point3::origin()
        };

        pos -= body_anchor.coords;

        let rb = RigidBodyDesc::new().translation(pos).build();
        let rb_handle = bodies.insert(rb);

        let co = collider_desc.build(BodyPartHandle(rb_handle, 0));
        colliders.insert(co);

        let mut constraint = RevoluteConstraint::new(
            parent,
            BodyPartHandle(rb_handle, 0),
            parent_anchor,
            Vector3::x_axis(),
            body_anchor,
            Vector3::x_axis(),
        );

        constraint.set_break_force(r!(40.0));
        joint_constraints.insert(constraint);

        parent = BodyPartHandle(rb_handle, 0);
    }

    /*
     * Prismatic constraint.
     */
    let first_anchor = Point3::new(r!(0.0), r!(5.0), r!(4.0));
    let mut pos = first_anchor.coords;
    parent = BodyPartHandle(ground_handle, 0);

    for i in 0usize..3 {
        let mut body_anchor = Point3::origin();
        let mut parent_anchor = Point3::origin();
        if i == 0 {
            parent_anchor = first_anchor;
        } else {
            body_anchor = Point3::new(r!(0.0), r!(0.0), r!(-1.0)) * (rad * r!(3.0));
        }

        pos -= body_anchor.coords;

        let rb = RigidBodyDesc::new().translation(pos).build();
        let rb_handle = bodies.insert(rb);

        let co = collider_desc.build(BodyPartHandle(rb_handle, 0));
        colliders.insert(co);

        let mut constraint = PrismaticConstraint::new(
            parent,
            BodyPartHandle(rb_handle, 0),
            parent_anchor,
            Vector3::y_axis(),
            body_anchor,
        );
        constraint.set_break_force(r!(40.0));
        constraint.enable_min_offset(-rad * r!(2.0));
        joint_constraints.insert(constraint);

        parent = BodyPartHandle(rb_handle, 0);
    }

    /*
     * Ball constraint.
     */
    let first_anchor = Point3::new(r!(0.0), r!(5.0), r!(0.0));
    let mut pos = first_anchor.coords;
    parent = BodyPartHandle(ground_handle, 0);

    for i in 0usize..num {
        let angle = r!(i as f64) * r!(2.0) * r!(PI) / r!(num as f64);
        let mut body_anchor = Point3::origin();
        let mut parent_anchor = Point3::origin();
        if i == 0 {
            parent_anchor = first_anchor;
        } else {
            body_anchor = Point3::new(angle.cos(), r!(0.3), angle.sin()) * (rad * r!(5.0));
        }

        pos -= body_anchor.coords;

        let rb = RigidBodyDesc::new().translation(pos).build();
        let rb_handle = bodies.insert(rb);

        let co = collider_desc.build(BodyPartHandle(rb_handle, 0));
        colliders.insert(co);

        let mut constraint = BallConstraint::new(
            parent,
            BodyPartHandle(rb_handle, 0),
            parent_anchor,
            body_anchor,
        );
        constraint.set_break_force(r!(40.0));
        joint_constraints.insert(constraint);
        parent = BodyPartHandle(rb_handle, 0);
    }

    /*
     * Universal constraint.
     */
    let parent_pos = Vector3::new(r!(0.0), r!(5.0), r!(-5.0));
    let child_pos = Vector3::new(r!(0.0), r!(5.0), r!(-6.0));

    let co = ColliderDesc::new(cuboid)
        .translation(parent_pos)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    let rb = RigidBodyDesc::new().translation(child_pos).build();
    let rb_handle = bodies.insert(rb);
    let co = collider_desc.build(BodyPartHandle(rb_handle, 0));
    colliders.insert(co);

    let mut constraint = UniversalConstraint::new(
        BodyPartHandle(ground_handle, 0),
        BodyPartHandle(rb_handle, 0),
        Point3::from(parent_pos),
        Vector3::x_axis(),
        Point3::new(r!(0.0), r!(0.0), r!(1.0)),
        Vector3::z_axis(),
        r!(FRAC_PI_2),
    );

    constraint.set_break_force(r!(40.0));
    joint_constraints.insert(constraint);

    /*
     * Planar constraint.
     */
    let num = 5;
    let shift = Vector3::new(r!(0.0), r!(-2.0), r!(5.0));
    let width = r!(5.0) * rad * r!(4.0);

    for i in 0..num {
        for j in 0..num {
            let mut z = r!(i as f64) * rad * r!(4.0) - width / r!(2.0);
            let y = r!(j as f64) * rad * r!(4.0) - width / r!(2.0);

            if j % 2 == 0 {
                z += rad * r!(2.0);
            }

            let rb = RigidBodyDesc::new()
                .translation(shift + Vector3::new(r!(0.0), y, z))
                .build();
            let rb_handle = bodies.insert(rb);
            let co = collider_desc.build(BodyPartHandle(rb_handle, 0));
            colliders.insert(co);

            let mut constraint = PlanarConstraint::new(
                BodyPartHandle(ground_handle, 0),
                BodyPartHandle(rb_handle, 0),
                Point3::origin(),
                Vector3::x_axis(),
                Point3::origin(),
                Vector3::x_axis(),
            );

            constraint.set_break_force(r!(40.0));
            joint_constraints.insert(constraint);
        }
    }

    /*
     * Rectangular constraint.
     */
    let shift = Vector3::new(r!(0.0), r!(-2.0), r!(0.0));
    let width = r!(5.0) * rad * r!(4.0);
    for i in 0..5 {
        for j in 0..5 {
            let mut z = r!(i as f64) * rad * r!(4.0) - width / r!(2.0);
            let y = r!(j as f64) * rad * r!(4.0) - width / r!(2.0);

            if j % 2 == 0 {
                z += rad * r!(2.0);
            }

            let rb = RigidBodyDesc::new()
                .translation(shift + Vector3::new(r!(0.0), y, z))
                .build();
            let rb_handle = bodies.insert(rb);
            let co = collider_desc.build(BodyPartHandle(rb_handle, 0));
            colliders.insert(co);

            let mut constraint = RectangularConstraint::new(
                BodyPartHandle(ground_handle, 0),
                BodyPartHandle(rb_handle, 0),
                Point3::origin(),
                Vector3::x_axis(),
                Point3::origin(),
            );

            constraint.set_break_force(r!(40.0));
            joint_constraints.insert(constraint);
        }
    }

    /*
     * Pin-slot constraint.
     */
    let pin_rb = RigidBodyDesc::new().build();
    let pin_handle = bodies.insert(pin_rb);

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::new(rad * r!(5.0), rad, rad * r!(5.0))));
    let co = ColliderDesc::new(cuboid)
        .density(r!(1.0))
        .build(BodyPartHandle(pin_handle, 0));
    colliders.insert(co);

    let mut constraint = PinSlotConstraint::new(
        BodyPartHandle(ground_handle, 0),
        BodyPartHandle(pin_handle, 0),
        Point3::origin(),
        Vector3::y_axis(),
        Vector3::x_axis(),
        Point3::origin(),
        Vector3::x_axis(),
    );

    constraint.set_break_force(r!(40.0));
    joint_constraints.insert(constraint);

    /*
     * Set up the testbed.
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
    testbed.look_at(Point3::new(30.0, -2.0, 0.0), Point3::new(0.0, -2.0, 0.0));
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("Constraints", init_world)]);
    testbed.run()
}
