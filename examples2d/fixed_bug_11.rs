/*!
 * # Expected behaviour:
 * The top cube will fall asleep while the bottom cube goes right at constant speed.
 *
 * # Symptoms:
 * Bothe cube fall asleep at some point.
 *
 * # Cause:
 * There was no way of customizing the deactivation threshold.
 *
 * # Solution:
 * Rigid bodies now have a `set_deactivation_threshold` method to set the threshold to a
 * user-defined value, or to prevent completely any deactivation (by setting None as the
 * threshold).
 *
 * # Limitations of the solution:
 */


extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Vector2, Translation2};
use ncollide::shape::Cuboid;
use nphysics2d::world::World;
use nphysics2d::object::RigidBody;
use nphysics_testbed2d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, 0.0));

    /*
     * Create the box that will be deactivated.
     */
    let rad    = 1.0;
    let geom   = Cuboid::new(Vector2::new(rad, rad));
    let mut rb = RigidBody::new_dynamic(geom, 1.0, 0.3, 0.5);

    rb.set_lin_vel(Vector2::new(0.99, 0.0));

    world.add_rigid_body(rb.clone());

    /*
     * Create the box that will not be deactivated.
     */
    rb.set_deactivation_threshold(Some(0.5));
    rb.append_translation(&Translation2::new(0.0, 3.0));

    world.add_rigid_body(rb);

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.run();
}
