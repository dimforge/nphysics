extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Compound, Cuboid, ShapeHandle};
use nphysics3d::object::{ColliderDesc, RigidBodyDesc};
use nphysics3d::world::World;
use nphysics3d::math::Velocity;
use nphysics_testbed3d::Testbed;


fn main() {
    /*
     * World
     */
    let mut world = World::new();

    /*
     * Create boxes to compute the inertia.
     */
    let mut shapes = Vec::new();
    shapes.push((
        Isometry3::identity(),
        ShapeHandle::new(Cuboid::new(Vector3::new(1.0, 0.1, 0.1))),
    ));
    shapes.push((
        Isometry3::translation(0.0, 0.4, 0.0),
        ShapeHandle::new(Cuboid::new(Vector3::new(0.1, 0.2, 0.1))),
    ));

    let geom = ShapeHandle::new(Compound::new(shapes));
    let collider_desc = ColliderDesc::new(geom)
        .with_density(1.0);

    /*
     * Create the rigid body.
     */
    RigidBodyDesc::new()
        .with_collider(&collider_desc)
        .with_velocity(Velocity::angular(0.0, 10.0, 0.1))
        .build(&mut world);

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(0.0, 0.0, 5.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
