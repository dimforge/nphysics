extern crate env_logger;
extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Compound, Cuboid, ShapeHandle};
use nphysics3d::object::Material;
use nphysics3d::world::World;
use nphysics3d::volumetric::Volumetric;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    env_logger::init();
    /*
     * World
     */
    let mut world = World::new();
    world.set_timestep(0.016);

    /*
     * Create boxes to compute the inertia.
     */
    let mut shapes = Vec::new();
    shapes.push((
        Isometry3::identity(),
        ShapeHandle::new(Cuboid::new(Vector3::new(1.0, 0.1, 0.1))),
    ));
    shapes.push((
        Isometry3::new(Vector3::y() * 0.4, na::zero()),
        ShapeHandle::new(Cuboid::new(Vector3::new(0.1, 0.2, 0.1))),
    ));

    let geom = ShapeHandle::new(Compound::new(shapes));
    let inertia = geom.inertia(1.0);
    let com = geom.center_of_mass();

    /*
     * Create the rigid body.
     */
    let handle = world.add_rigid_body(Isometry3::identity(), inertia, com);
    world
        .rigid_body_mut(handle)
        .unwrap()
        .set_angular_velocity(Vector3::new(0.0, 10.0, 0.1));

    /*
     * Create the collider.
     * XXX: This is not actually needed for the dynamics simulation, but we do it anyway so that the
     * testbed dislpays it.
     */
    world.add_collider(
        COLLIDER_MARGIN,
        geom.clone(),
        handle,
        Isometry3::identity(),
        Material::default(),
    );

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(0.0, 0.0, 5.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
