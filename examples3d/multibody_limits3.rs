extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::sync::Arc;
use na::{Real, Point3, Vector3, Translation3, Isometry3};
use ncollide::shape::{ShapeHandle, Plane, Ball, Cuboid};
use nphysics3d::world::World;
use nphysics3d::object::BodyHandle;
use nphysics3d::joint::{FreeJoint, FixedJoint, Joint, RevoluteJoint};
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
     * Setup the ground.
     */
    let plane     = ShapeHandle::new(Plane::new(Vector3::new(0.0, 1.0, 0.0)));
    let plane_pos = Isometry3::new(Vector3::y() * -50.0, na::zero());
    world.add_collider(COLLIDER_MARGIN, plane, BodyHandle::ground(), plane_pos, false);

    /*
     * Setup the multibody.
     */
    let rad  = 1.0;
    let num  = 20;
    let axis = Vector3::x_axis();

    let geom = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    let inertia = geom.inertia(1.0);

    // Setup the first link with a free joint.
    let free = FreeJoint::new(Isometry3::identity());
    let mut parent = BodyHandle::ground();
    parent = world.add_multibody_link(parent, free, Isometry3::identity(), inertia);

    // Setup the other links with revolute joints.
    let mut revo = RevoluteJoint::new(Point3::new(0.0f32, 0.0, rad * 3.0), axis, -3.14 / 10.0);
    revo.enable_min_angle(-3.14 / 10.0);
    revo.enable_max_angle(-3.14 / 10.0);

    for j in 0usize .. num {
        parent = world.add_multibody_link(parent, revo, Isometry3::identity(), inertia);
        world.add_collider(COLLIDER_MARGIN, geom.clone(), parent, Isometry3::identity(), true);
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(40.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
