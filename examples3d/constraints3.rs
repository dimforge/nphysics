extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::sync::Arc;
use na::{Isometry3, Point3, Real, Translation3, Vector3};
use ncollide::shape::{Ball, Cuboid, Plane, ShapeHandle};
use nphysics3d::world::World;
use nphysics3d::object::BodyHandle;
use nphysics3d::joint::{BallConstraint, FixedConstraint, FixedJoint, FreeJoint, Joint,
                        RevoluteConstraint, RevoluteJoint};
use nphysics3d::volumetric::Volumetric;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Create the boxes
     */
    let num = 3;
    let rad = 0.2;
    let mut parent = BodyHandle::ground();

    let geom = ShapeHandle::new(Cuboid::new(Vector3::new(rad, rad, rad)));
    // let geom = ShapeHandle::new(Ball::new(rad));
    let inertia = geom.inertia(1.0);

    for j in 0usize..num {
        /*
         * Create the rigid body.
         */
        let pos = Isometry3::new(-Vector3::z() * (j + 1) as f32 * rad * 3.0, na::zero());
        let rb = world.add_rigid_body(pos, inertia);

        let constraint = RevoluteConstraint::new(
            parent,
            rb,
            // FIXME: unify the API regarding the choice of coordinate systems (what is relative to
            // what). For example, it seems weird to have a '-' here.
            na::origin(),
            Vector3::y_axis(),
            Point3::new(0.0, 0.0, rad * 3.0),
            Vector3::y_axis(),
        );

        // let constraint = BallConstraint::new(
        //     parent,
        //     rb,
        //     // FIXME: unify the API regarding the choice of coordinate systems (what is relative to
        //     // what). For example, it seems weird to have a '-' here.
        //     na::origin(),
        //     Point3::new(0.0, 0.0, rad * 3.0),
        // );

        // let constraint = FixedConstraint::new(
        //     parent,
        //     rb,
        //     // FIXME: unify the API regarding the choice of coordinate systems (what is relative to
        //     // what). For example, it seems weird to have a '-' here.
        //     na::one(),
        //     Isometry3::new(Vector3::z() * rad * 3.0, na::zero()),
        // );

        world.add_constraint(constraint);

        /*
         * Create the collider.
         */
        world.add_collider(COLLIDER_MARGIN, geom.clone(), rb, Isometry3::identity());

        /*
         * Parent for the next constraint.
         */
        parent = rb;
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(15.0, -2.0, 0.0), Point3::new(0.0, -2.0, 0.0));
    testbed.run();
}
