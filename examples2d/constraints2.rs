extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use std::sync::Arc;
use na::{Real, Point2, Vector2, Translation2, Isometry2};
use ncollide::shape::{ShapeHandle, Plane, Ball, Cuboid};
use nphysics2d::world::World;
use nphysics2d::object::BodyHandle;
use nphysics2d::joint::{FreeJoint, FixedJoint, Joint, RevoluteJoint, FixedConstraint, RevoluteConstraint};
use nphysics2d::volumetric::Volumetric;
use nphysics_testbed2d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, 9.81));

    /*
     * Create the boxes
     */
    let num        = 10;
    let rad        = 0.2;
    let mut parent = BodyHandle::ground();

    let geom    = ShapeHandle::new(Ball::new(rad));
    let inertia = geom.inertia(1.0);

    for j in 0usize .. num {
        /*
         * Create the rigid body.
         */
        let pos = Isometry2::new(Vector2::x() * (j + 1) as f32 * rad * 3.0, na::zero());
        let rb  = world.add_rigid_body(pos, inertia);

        let revolute_constraint = RevoluteConstraint::new(
            parent, rb,
            // FIXME: unify the API regarding the choice of coordinate systems (what is relative to
            // what). For example, it seems weird to have a '-' here.
            na::origin(), Point2::new(-rad * 3.0, 0.0));

        world.add_constraint(revolute_constraint);

        /*
         * Create the collider.
         */
        world.add_collider(COLLIDER_MARGIN, geom.clone(), rb, Isometry2::identity(), true);

        /*
         * Parent for the next constraint.
         */
        parent = rb;
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}
