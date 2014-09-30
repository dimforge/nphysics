/*!
 * # Expected behaviour:
 * The box stands vertically until it falls asleep.
 * The box should not fall (horizontally) on the ground.
 * The box should not traverse the ground.
 *
 * # Symptoms:
 * The long, thin, box fails to collide with the plane: it just ignores it.
 *
 * # Cause:
 * The one shot contact manifold generator was incorrect in this case. This generator rotated the
 * object wrt its center to sample the contact manifold. If the object is long and the theoretical
 * contact surface is small, all contacts will be invalidated whenever the incremental contact
 * manifold will get a new point from the one-shot generator.
 *
 * # Solution:
 * Rotate the object wrt the contact center, not wrt the object center.
 *
 * # Limitations of the solution:
 * This will create only a three-points manifold for a small axis-alligned cube, instead of four.
 */

extern crate native;
extern crate "nalgebra" as na;
extern crate ncollide;
extern crate "nphysics3df32" as nphysics;
extern crate nphysics_testbed3d;

use na::{Vec3, Translation};
use ncollide::geom::{Plane, Cuboid};
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics_testbed3d::Testbed;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    /*
     * Plane
     */
    let geom = Plane::new(Vec3::new(0.0f32, 1.0, 0.0));

    world.add_body(RigidBody::new_static(geom, 0.3, 0.6));

    /*
     * Create the boxes
     */
    let rad = 1.0f32;
    let x   = rad;
    let y   = rad + 10.0;
    let z   = rad;

    let geom   = Cuboid::new(Vec3::new(rad, rad * 10.0, rad));
    let mut rb = RigidBody::new_dynamic(geom, 1.0f32, 0.3, 0.5);

    rb.append_translation(&Vec3::new(x, y, z));

    world.add_body(rb);

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Vec3::new(-30.0, 30.0, -30.0), Vec3::new(0.0, 0.0, 0.0));
    testbed.run();
}
