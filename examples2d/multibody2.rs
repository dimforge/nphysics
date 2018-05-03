extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use std::sync::Arc;
use na::{Isometry2, Point2, Real, Translation2, Unit, Vector2};
use ncollide2d::shape::{Ball, Cuboid, Plane, ShapeHandle};
use nphysics2d::world::World;
use nphysics2d::object::BodyHandle;
use nphysics2d::joint::{FixedConstraint, FixedJoint, FreeJoint, Joint, PrismaticJoint,
                        RevoluteJoint};
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
    let num = 20;
    let rad = 0.2;
    let mut parent = BodyHandle::ground();

    let revo = RevoluteJoint::new(0.0);
    let geom = ShapeHandle::new(Ball::new(rad));
    let inertia = geom.inertia(1.0);

    for j in 0usize..num {
        /*
         * Create the rigid body.
         */
        parent = world.add_multibody_link(
            parent,
            revo,
            na::zero(),
            Vector2::new(-rad * 3.0, 0.0),
            inertia,
        );

        /*
         * Create the collider.
         */
        world.add_collider(COLLIDER_MARGIN, geom.clone(), parent, Isometry2::identity());
    }

    /*
     * PrismaticJoint joint.
     */
    let ramp = Vector2::new(0.0, 1.0);
    let mut parent = BodyHandle::ground();

    let mut prism = PrismaticJoint::new(Unit::new_normalize(ramp), 0.0);
    prism.enable_max_offset(rad * 3.0);
    prism.enable_min_offset(-rad * 3.0);

    for j in 0usize..num {
        parent = world.add_multibody_link(
            parent,
            prism,
            Vector2::new(-rad * 3.0, 0.0),
            na::zero(),
            inertia,
        );
        world.add_collider(COLLIDER_MARGIN, geom.clone(), parent, Isometry2::identity());
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}
