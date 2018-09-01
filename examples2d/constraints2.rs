extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Point2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::joint::{CartesianConstraint, PrismaticConstraint, RevoluteConstraint};
use nphysics2d::object::{BodyPartHandle, Material};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Ground.
     */
    let ground_radx = 25.0;
    let ground_rady = 1.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        ground_radx - COLLIDER_MARGIN,
        ground_rady - COLLIDER_MARGIN,
    )));

    let ground_pos = Isometry2::new(Vector2::y() * -10.0, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Revolute constraints.
     */
    let num = 10;
    let rad = 0.2;
    let mut parent = BodyPartHandle::ground();

    let geom = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    for j in 0usize..num {
        /*
         * Create the rigid body.
         */
        let pos = Isometry2::new(Vector2::x() * (j + 1) as f32 * rad * 3.0, na::zero());
        let rb = world.add_rigid_body(pos, inertia, center_of_mass);

        let revolute_constraint =
            RevoluteConstraint::new(parent, rb, na::origin(), Point2::new(-rad * 3.0, 0.0));

        world.add_constraint(revolute_constraint);

        /*
         * Create the collider.
         */
        world.add_collider(
            COLLIDER_MARGIN,
            geom.clone(),
            rb,
            Isometry2::identity(),
            Material::default(),
        );

        /*
         * Parent for the next constraint.
         */
        parent = rb;
    }

    /*
     * Prismatic constraints.
     */
    parent = BodyPartHandle::ground();
    let first_anchor = Point2::new(-1.0, 0.0);
    let other_anchor = Point2::new(-3.0 * rad, 0.0);
    let mut pos = Isometry2::new(first_anchor.coords, na::zero());

    for j in 0usize..3 {
        /*
         * Create the rigid body.
         */
        let rb = world.add_rigid_body(pos, inertia, center_of_mass);

        let mut constraint = PrismaticConstraint::new(
            parent,
            rb,
            if j == 0 { first_anchor } else { other_anchor },
            Vector2::y_axis(),
            Point2::origin(),
        );

        constraint.enable_min_offset(-rad * 2.0);
        world.add_constraint(constraint);

        /*
         * Create the collider.
         */
        world.add_collider(
            COLLIDER_MARGIN,
            geom.clone(),
            rb,
            Isometry2::identity(),
            Material::default(),
        );

        /*
         * Parent for the next constraint.
         */
        pos.translation.vector += other_anchor.coords;
        parent = rb;
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

            let rb = world.add_rigid_body(
                Isometry2::new(shift + Vector2::new(x, y), na::zero()),
                inertia,
                center_of_mass,
            );

            world.add_collider(
                COLLIDER_MARGIN,
                geom.clone(),
                rb,
                Isometry2::identity(),
                Material::default(),
            );

            let constraint = CartesianConstraint::new(
                BodyPartHandle::ground(),
                rb,
                Isometry2::identity(),
                Isometry2::identity(),
            );

            world.add_constraint(constraint);
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.look_at(Point2::new(0.0, 4.0), 50.0);
    testbed.run();
}
