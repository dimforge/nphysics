extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Point3, Vector3, Translation3};
use ncollide::shape::Ball;
use nphysics3d::world::World;
use nphysics3d::object::{RigidBody, FreeJoint};
use nphysics3d::math::Twist;
use nphysics_testbed3d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Create the boxes
     */
    let num    = 2;
    let rad    = 1.0;
    let center = Translation3::new(0.0, 0.0, -rad * 3.0);
    // let axis   = Vector3::x_axis();
    let mut parent = None;
    let mut vel    = 0.0;

    for j in 0usize .. num {
        let geom   = Ball::new(rad);
        let mut rb = RigidBody::new_dynamic(geom, 1.0, 0.3, 0.5);
        let dof    = FreeJoint::new(na::convert(center), Twist::zero());
        rb.set_dof(Box::new(dof));
        rb.set_parent(parent, na::one());

        parent = Some(world.add_rigid_body(rb));
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(-30.0, 30.0, -30.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
