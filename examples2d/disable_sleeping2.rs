extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::Vector2;
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::object::{ColliderDesc, RigidBodyDesc};
use nphysics2d::world::World;
use nphysics2d::math::Velocity;
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
    let rad = 0.1;
    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::new(rad, rad)));
    let collider_desc = ColliderDesc::new(cuboid).with_density(1.0);

    /*
     * Create the body that will be deactivated.
     */
    RigidBodyDesc::default()
        .with_collider(&collider_desc)
        .with_velocity(Velocity::linear(0.099, 0.0))
        .build(&mut world);

    /*
     * Create the body that cannot be deactivated by
     * setting its sleep threshold to None.
     */
    RigidBodyDesc::default()
        .with_collider(&collider_desc)
        .with_translation(Vector2::y() * 0.3)
        .with_velocity(Velocity::linear(0.099, 0.0))
        .with_sleep_threshold(None)
        .build(&mut world);

    /*
     * Set up the testbed.
     */
    let testbed = Testbed::new(world);
    testbed.run();
}
