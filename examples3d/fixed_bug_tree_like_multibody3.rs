// Issue #110
extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::joint::{FixedJoint, FreeJoint, Joint};
use nphysics3d::object::{BodyPartHandle, ColliderDesc, MultibodyDesc};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;


fn new_link<J: Joint<f32>>(world: &mut World<f32>, joint: J, parent: BodyPartHandle) -> BodyPartHandle {
    let shape = ShapeHandle::new(Cuboid::new(Vector3::new(0.2, 0.2, 0.2)));
    let collider_desc = ColliderDesc::new(shape)
        .density(1.0);

    MultibodyDesc::new(joint)
        .collider(&collider_desc)
        .build_parent(parent, world)
        .unwrap()
        .part_handle()
}

fn main() {
    let mut world: World<f32> = World::new();
    world.set_gravity(Vector3::y() * -9.81);

    /*
     * Ground.
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size)));

    ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * (-ground_size - 5.0))
        .build(&mut world);

    /*
     * Tree-like structure.
     */
    let root = new_link(
        &mut world,
        FreeJoint::new(Isometry3::identity()),
        BodyPartHandle::ground(),
    );

    let left_joint = FixedJoint::new(Isometry3::translation(-1.0, 1.5, 0.0));
    let right_joint = FixedJoint::new(Isometry3::translation(1.0, 1.5, 0.0));

    // two children under root
    let a = new_link(&mut world, left_joint, root);
    let b = new_link(&mut world, right_joint, root);

    new_link(&mut world, left_joint, a);
    new_link(&mut world, right_joint, a);
    new_link(&mut world, right_joint, b);

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.look_at(Point3::new(-1.0, 5.0, -1.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}
