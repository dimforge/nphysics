extern crate nalgebra as na;

use na::{Point2, UnitComplex, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::{
    CartesianConstraint, DefaultJointConstraintSet, PrismaticConstraint, RevoluteConstraint,
};
use nphysics2d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector2::new(0.0, -9.81));
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let mut joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Ground.
     */
    let ground_size = 25.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 1.0)));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(-Vector2::y() * 10.0)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Revolute constraints.
     */
    let num = 10;
    let rad = 0.2;
    let mut parent = BodyPartHandle(ground_handle, 0);

    let geom = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let collider_desc = ColliderDesc::new(geom).density(1.0);

    for j in 0usize..num {
        /*
         * Create the rigid body.
         */
        let rb = RigidBodyDesc::new()
            .translation(Vector2::x() * (j + 1) as f32 * rad * 3.0)
            .build();
        let rb_handle = bodies.insert(rb);
        let co = collider_desc.build(BodyPartHandle(rb_handle, 0));
        colliders.insert(co);

        let revolute_constraint = RevoluteConstraint::new(
            parent,
            BodyPartHandle(rb_handle, 0),
            Point2::origin(),
            Point2::new(-rad * 3.0, 0.0),
        );

        joint_constraints.insert(revolute_constraint);

        /*
         * Parent for the next constraint.
         */
        parent = BodyPartHandle(rb_handle, 0);
    }

    /*
     * Prismatic constraints.
     */
    parent = BodyPartHandle(ground_handle, 0);
    let first_anchor = Point2::new(-1.0, 0.0);
    let other_anchor = Point2::new(-3.0 * rad, 0.0);
    let mut translation = first_anchor.coords;

    for j in 0usize..3 {
        /*
         * Create the rigid body.
         */
        let rb = RigidBodyDesc::new().translation(translation).build();
        let rb_handle = bodies.insert(rb);
        let co = collider_desc.build(BodyPartHandle(rb_handle, 0));
        colliders.insert(co);

        let mut constraint = PrismaticConstraint::new(
            parent,
            BodyPartHandle(rb_handle, 0),
            if j == 0 { first_anchor } else { other_anchor },
            Vector2::y_axis(),
            Point2::origin(),
        );

        constraint.enable_min_offset(-rad * 2.0);
        joint_constraints.insert(constraint);

        /*
         * Parent for the next constraint.
         */
        translation += other_anchor.coords;
        parent = BodyPartHandle(rb_handle, 0);
    }

    /*
     * Cartesian constraint.
     */
    let num = 10;
    let shift = Vector2::new(0.0, 2.0);
    let width = 20.0 * rad;

    for i in 0..num {
        for j in 0..num {
            let mut x = i as f32 * rad * 3.0 - width / 2.0 + 5.0;
            let y = j as f32 * rad * -3.0 + width / 2.0 + 4.0;

            if j % 2 == 0 {
                x += rad * 2.0;
            }

            let rb = RigidBodyDesc::new()
                .translation(shift + Vector2::new(x, y))
                .build();
            let rb_handle = bodies.insert(rb);
            let co = collider_desc.build(BodyPartHandle(rb_handle, 0));
            colliders.insert(co);

            let constraint = CartesianConstraint::new(
                BodyPartHandle(ground_handle, 0),
                BodyPartHandle(rb_handle, 0),
                Point2::origin(),
                UnitComplex::identity(),
                Point2::origin(),
                UnitComplex::identity(),
            );

            joint_constraints.insert(constraint);
        }
    }

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
    testbed.look_at(Point2::new(0.0, 4.0), 50.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Constraints", init_world)]);
    testbed.run()
}
