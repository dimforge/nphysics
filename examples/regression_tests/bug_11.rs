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


#![crate_type = "bin"]
#![warn(non_camel_case_types)]
extern crate native;
extern crate nphysics_testbed2d;
extern crate nalgebra;
extern crate ncollide = "ncollide2df32";
extern crate nphysics = "nphysics2df32";

use std::rc::Rc;
use std::cell::RefCell;
use nalgebra::na::{Vec2, Translation};
use ncollide::geom::Cuboid;
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics_testbed2d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(issue_11)
}

pub fn issue_11(graphics: &mut GraphicsManager) -> World {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec2::new(0.0f32, 0.0));

    /*
     * Create the box that will be deactivated.
     */
    let rad    = 1.0f32;
    let geom   = Cuboid::new(Vec2::new(rad, rad));
    let mut rb = RigidBody::new_dynamic(geom, 1.0f32, 0.3, 0.5);

    rb.set_lin_vel(Vec2::new(0.99, 0.0));

    let body = Rc::new(RefCell::new(rb.clone()));

    world.add_body(body.clone());
    graphics.add(body);

    /*
     * Create the box that will not be deactivated.
     */
    rb.set_deactivation_threshold(Some(0.5));
    rb.append_translation(&Vec2::new(0.0, 3.0));

    let body = Rc::new(RefCell::new(rb));

    world.add_body(body.clone());
    graphics.add(body);

    world
}
