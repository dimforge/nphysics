extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use std::sync::Arc;
use na::{Isometry3, Point3, Real, Translation3, Vector3};
use ncollide3d::shape::{Ball, Cuboid, Plane, ShapeHandle};
use nphysics3d::world::World;
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::joint::{FixedJoint, FreeJoint, Joint, RevoluteJoint};
use nphysics3d::volumetric::Volumetric;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Setup the ground.
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry3::new(Vector3::y() * -ground_size, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Setup the multibody.
     */
    let rad = 0.1;
    let num = 20;
    let axis = Vector3::x_axis();

    let cuboid = ShapeHandle::new(Cuboid::new(Vector3::repeat(rad)));
    let cuboid_inertia = cuboid.inertia(1.0);
    let cuboid_center_of_mass = cuboid.center_of_mass();

    // Setup the first link with a free joint.
    let free = FreeJoint::new(Isometry3::new(Vector3::y() * 3.0, na::zero()));
    let mut parent = BodyHandle::ground();
    parent = world.add_multibody_link(
        parent,
        free,
        na::zero(),
        na::zero(),
        cuboid_inertia,
        cuboid_center_of_mass,
    );

    // Setup the other links with revolute joints.
    let mut revo = RevoluteJoint::new(axis, -3.14 / 10.0);
    revo.enable_min_angle(-3.14 / 10.0);
    revo.enable_max_angle(-3.14 / 10.0);

    for j in 0usize..num {
        parent = world.add_multibody_link(
            parent,
            revo,
            na::zero(),
            Vector3::z() * (rad * 3.0),
            cuboid_inertia,
            cuboid_center_of_mass,
        );
        world.add_collider(
            COLLIDER_MARGIN,
            cuboid.clone(),
            parent,
            Isometry3::identity(),
            Material::default(),
        );
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(5.0, 1.0, 0.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
