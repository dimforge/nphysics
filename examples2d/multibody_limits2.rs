extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use std::sync::Arc;
use na::{Real, Point2, Vector2, Translation2, Isometry2};
use ncollide2d::shape::{ShapeHandle, Plane, Ball, Cuboid};
use nphysics2d::world::World;
use nphysics2d::object::BodyHandle;
use nphysics2d::joint::{FreeJoint, FixedJoint, Joint, RevoluteJoint};
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
     * Setup the ground.
     */
    let plane     = ShapeHandle::new(Plane::new(Vector2::new(0.0, -1.0)));
    let plane_pos = Isometry2::new(Vector2::y() * 5.0, na::zero());
    world.add_collider(COLLIDER_MARGIN, plane, BodyHandle::ground(), plane_pos, false);

    /*
     * Setup the multibody.
     */
    let rad  = 0.2;
    let num  = 20;

    let geom = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let inertia = geom.inertia(1.0);

    // Setup the first link with a free joint.
    let free = FreeJoint::new(Isometry2::identity());
    let mut parent = BodyHandle::ground();
    parent = world.add_multibody_link(parent, free, Isometry2::identity(), inertia);

    // Setup the other links with revolute joints.
    let mut revo = RevoluteJoint::new(Point2::new(rad * 3.0, 0.0), -3.14 / 10.0);
    revo.enable_min_angle(-3.14 / 10.0);
    revo.enable_max_angle(-3.14 / 10.0);

    for j in 0usize .. num {
        parent = world.add_multibody_link(parent, revo, Isometry2::identity(), inertia);
        world.add_collider(COLLIDER_MARGIN, geom.clone(), parent, Isometry2::identity(), true);
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}
