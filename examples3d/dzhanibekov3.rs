extern crate nalgebra as na;
extern crate ncollide;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Point3, Vector3, Isometry3, Matrix3};
use ncollide::shape::{ShapeHandle, Cuboid, Cylinder, Compound};
use nphysics3d::world::World;
use nphysics3d::volumetric::Volumetric;
use nphysics3d::math::Inertia;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_timestep(0.016);

    /*
     * Create boxes to compute the inertia.
     */
    let mut shapes = Vec::new();
    shapes.push((Isometry3::identity(), ShapeHandle::new(Cuboid::new(Vector3::new(1.0, 0.1, 0.1)))));
    shapes.push((Isometry3::new(Vector3::y() * 0.4, na::zero()), ShapeHandle::new(Cylinder::new(0.2, 0.1))));

    let geom = ShapeHandle::new(Compound::new(shapes));
    let inertia = geom.inertia(1.0);

    /*
     * Create the rigid body.
     */
    let handle = world.add_rigid_body(Isometry3::identity(), inertia);
    world.rigid_body_mut(handle).unwrap().set_angular_velocity(Vector3::new(0.0, 10.0, 0.1));

    /*
     * Create the collider.
     * XXX: This is not actually needed for the dynamics simulation, but we do it anyway so that the
     * testbed dislpays it.
     */
    world.add_collider(COLLIDER_MARGIN, geom.clone(), handle, Isometry3::identity(), true);

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(0.0, 0.0, 5.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
