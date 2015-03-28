/*!
 * # Expected behaviour:
 * Run with `--pause` and execute two steps.
 * The small box will hit the second one at the second simulation step.
 * The hit should generate some contact points successfully.
 *
 * # Symptoms:
 * If we ignore the margins, the objects are 'just touching' because of the CCD *but* no contact
 * point is detected.
 *
 * # Cause:
 * The Minkowski sampling based algorithm to compute the penetration depth did not handle the case
 * of shapes that are _just touching_. Therefore, it failed (returning None).
 *
 * # Solution:
 * cf. e20eb1ee1d5ec0066aeefb782f1a6344c1fba87e
 * Fix the algorithm so that it handles positive gaps.
 *
 * # Limitations of the solution:
 */

extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics;
extern crate nphysics_testbed3d;

use na::{Pnt3, Vec3, Translation};
use ncollide::shape::{Plane, Cuboid};
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics_testbed3d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    /*
     * Planes
     */
    let rb = RigidBody::new_static(Plane::new(Vec3::new(0.0, 1.0, 0.0)), 0.3, 0.6);

    world.add_body(rb);

    /*
     * First cuboid.
     */
    let rad   = 5.0;
    let shift = (rad + 0.08) * 2.0;
    let x     = shift / 2.0;
    let y     = shift / 2.0;
    let z     = shift / 2.0;

    let mut rb = RigidBody::new_dynamic(Cuboid::new(Vec3::new(0.21, 4.96, 0.21)), 1.0, 0.3, 0.5);
    rb.append_translation(&Vec3::new(x, y, z));
    world.add_body(rb);

    /*
     * Create the ccd-enabled cube.
     */
    let mut rb = RigidBody::new_dynamic(Cuboid::new(Vec3::new(0.5, 0.5, 0.5)), 1.0, 0.3, 0.5);
    rb.append_translation(&Vec3::new(x - 1.0, y + 2.0, z - 1.0));
    rb.set_lin_vel(Vec3::new(10.0, 0.0, 10.0));
    let body_handle = world.add_body(rb);
    world.add_ccd_to(&body_handle, 0.0);

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Pnt3::new(-30.0, 30.0, -30.0), Pnt3::new(0.0, 0.0, 0.0));
    testbed.run();
}
