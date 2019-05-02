extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Point2, Vector2, UnitComplex};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::joint::{CartesianConstraint, PrismaticConstraint, RevoluteConstraint};
use nphysics2d::object::{BodyPartHandle, ColliderDesc, RigidBodyDesc};
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Ground.
     */
    let ground_size = 25.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 1.0)));

    ColliderDesc::new(ground_shape)
        .translation(-Vector2::y() * 10.0)
        .build(&mut world);

    /*
     * Revolute constraints.
     */
    let num = 10;
    let rad = 0.2;
    let mut parent = BodyPartHandle::ground();

    let geom = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let collider_desc = ColliderDesc::new(geom)
        .density(1.0);
    let mut rb_desc = RigidBodyDesc::new()
        .collider(&collider_desc);

    for j in 0usize..num {
        /*
         * Create the rigid body.
         */
        let rb_handle = rb_desc
            .set_translation(Vector2::x() * (j + 1) as f32 * rad * 3.0)
            .build(&mut world)
            .part_handle();

        let revolute_constraint =
            RevoluteConstraint::new(parent, rb_handle, Point2::origin(), Point2::new(-rad * 3.0, 0.0));

        world.add_constraint(revolute_constraint);

        /*
         * Parent for the next constraint.
         */
        parent = rb_handle;
    }

    /*
     * Prismatic constraints.
     */
    parent = BodyPartHandle::ground();
    let first_anchor = Point2::new(-1.0, 0.0);
    let other_anchor = Point2::new(-3.0 * rad, 0.0);
    let mut translation = first_anchor.coords;

    for j in 0usize..3 {
        /*
         * Create the rigid body.
         */
        let rb_handle = rb_desc
            .set_translation(translation)
            .build(&mut world)
            .part_handle();

        let mut constraint = PrismaticConstraint::new(
            parent,
            rb_handle,
            if j == 0 { first_anchor } else { other_anchor },
            Vector2::y_axis(),
            Point2::origin(),
        );

        constraint.enable_min_offset(-rad * 2.0);
        world.add_constraint(constraint);

        /*
         * Parent for the next constraint.
         */
        translation += other_anchor.coords;
        parent = rb_handle;
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

            let rb_handle = rb_desc
                .set_translation(shift + Vector2::new(x, y))
                .build(&mut world)
                .part_handle();

            let constraint = CartesianConstraint::new(
                BodyPartHandle::ground(),
                rb_handle,
                Point2::origin(),
                UnitComplex::identity(),
                Point2::origin(),
                UnitComplex::identity(),
            );

            world.add_constraint(constraint);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
    testbed.look_at(Point2::new(0.0, 4.0), 50.0);
}


fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}