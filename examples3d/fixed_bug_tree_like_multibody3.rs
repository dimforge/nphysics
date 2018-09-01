// Issue #110
extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::{FixedJoint, FreeJoint, Joint};
use nphysics3d::object::{BodyPartHandle, Material};
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn new_shape<J: Joint<f32>>(world: &mut World<f32>, joint: J, parent: BodyPartHandle) -> BodyPartHandle {
    let shape = ShapeHandle::new(Cuboid::new(Vector3::new(0.2, 0.2, 0.2)));
    let handle = world.add_multibody_link(
        parent,
        joint,
        na::zero(),
        na::zero(),
        shape.inertia(1.0),
        shape.center_of_mass(),
    );

    world.add_collider(
        COLLIDER_MARGIN,
        shape,
        handle,
        Isometry3::identity(),
        Material::default(),
    );

    handle
}

fn main() {
    let mut world: World<f32> = World::new();
    world.set_gravity(Vector3::y() * -9.81);

    /*
     * Ground.
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry3::new(Vector3::y() * -ground_size, na::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyPartHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Tree-like structure.
     */
    let root = new_shape(
        &mut world,
        FreeJoint::new(Isometry3::identity()),
        BodyPartHandle::ground(),
    );

    let left_joint = FixedJoint::new(Isometry3::new(Vector3::new(-1.0, 1.5, 0.0), na::zero()));
    let right_joint = FixedJoint::new(Isometry3::new(Vector3::new(1.0, 1.5, 0.0), na::zero()));

    // two children under root
    let a = new_shape(&mut world, left_joint, root);
    let b = new_shape(&mut world, right_joint, root);

    // two children each to a and b
    new_shape(&mut world, left_joint, a);
    new_shape(&mut world, right_joint, a);

    new_shape(&mut world, left_joint, b);
    new_shape(&mut world, right_joint, b);

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.look_at(Point3::new(-1.0, 5.0, -1.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
