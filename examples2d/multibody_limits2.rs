extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::joint::{FreeJoint, RevoluteJoint};
use nphysics2d::object::{BodyHandle, Material};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;
use std::f32::consts::PI;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Setup the ground.
     */
    let ground_radx = 25.0;
    let ground_rady = 1.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        ground_radx - COLLIDER_MARGIN,
        ground_rady - COLLIDER_MARGIN,
    )));

    let ground_pos = Isometry2::new(Vector2::y() * -ground_rady, na::zero());
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
    let rad = 0.2;
    let num = 20;

    let geom = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let inertia = geom.inertia(1.0);
    let center_of_mass = geom.center_of_mass();

    // Setup the first link with a free joint.
    let free = FreeJoint::new(Isometry2::new(Vector2::y(), na::zero()));
    let mut parent = BodyHandle::ground();
    parent = world.add_multibody_link(
        parent,
        free,
        na::zero(),
        na::zero(),
        inertia,
        center_of_mass,
    );

    // Setup the other links with revolute joints.
    let mut revo = RevoluteJoint::new(PI / 10.0);
    revo.enable_min_angle(PI / 10.0);
    revo.enable_max_angle(PI / 10.0);

    for _ in 0usize..num {
        parent = world.add_multibody_link(
            parent,
            revo,
            Vector2::x() * rad * 3.0,
            na::zero(),
            inertia,
            center_of_mass,
        );
        world.add_collider(
            COLLIDER_MARGIN,
            geom.clone(),
            parent,
            Isometry2::identity(),
            Material::default(),
        );
    }

    /*
     * Set up the testbed.
     */
    let testbed = Testbed::new(world);
    testbed.run();
}
