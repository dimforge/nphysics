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


#[crate_type = "bin"];
#[warn(non_camel_case_types)];
#[feature(managed_boxes)];

extern mod std;
extern mod extra;
extern mod kiss3d;
extern mod graphics3d;
extern mod nphysics = "nphysics3df32";
extern mod nalgebra;
extern mod ncollide = "ncollide3df32";

use std::rc::Rc;
use std::cell::RefCell;
use nalgebra::na::{Vec3, Translation};
use kiss3d::window::Window;
use ncollide::geom::{Plane, Box};
use nphysics::world::World;
use nphysics::object::{RigidBody, Static, Dynamic};
use graphics3d::engine::GraphicsManager;

fn main() {
    GraphicsManager::simulate(boxes_vee_3d)
}

pub fn boxes_vee_3d(window: &mut Window, graphics: &mut GraphicsManager) -> World {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    /*
     * Plane
     */
    let geom = Plane::new(Vec3::new(0.0f32, 1.0, 0.0));
    let body = Rc::new(RefCell::new(RigidBody::new(geom, 0.0f32, Static, 0.3, 0.6)));

    world.add_body(body.clone());
    graphics.add(window, body);

    /*
     * Create the boxes
     */
    let rad = 1.0f32;
    let x   = rad;
    let y   = rad + 10.0;
    let z   = rad;

    let geom   = Box::new(Vec3::new(rad, rad * 10.0, rad));
    let mut rb = RigidBody::new(geom, 1.0f32, Dynamic, 0.3, 0.5);

    rb.append_translation(&Vec3::new(x, y, z));

    let body = Rc::new(RefCell::new(rb));

    world.add_body(body.clone());
    graphics.add(window, body);

    /*
     * Set up the camera and that is it!
     */
    graphics.look_at(Vec3::new(-30.0f32, 30.0, -30.0), Vec3::new(0.0, 0.0, 0.0));

    world
}
