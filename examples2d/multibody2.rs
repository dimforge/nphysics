extern crate nalgebra as na;

use na::{Point2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::object::{ColliderDesc, MultibodyDesc, DefaultBodySet, DefaultColliderSet,
                         Ground, BodyPartHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::world::{DefaultDynamicWorld, DefaultColliderWorld};
use nphysics2d::joint::{CartesianJoint, PrismaticJoint, RevoluteJoint};
use nphysics_testbed2d::Testbed;


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let dynamic_world = DefaultDynamicWorld::new(Vector2::new(0.0, -9.81));
    let collider_world = DefaultColliderWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Ground
     */
    let ground_size = 25.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 1.0)));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(-Vector2::y() * 5.0)
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Shape that will be re-used for several multibody links.
     */
    let rad = 0.2;
    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let collider_desc = ColliderDesc::new(cuboid.clone()).density(1.0);

    /*
     * Revolute joint.
     */
    let num = 10;
    let revo = RevoluteJoint::new(-0.1);
    let body_shift = Vector2::x() * (rad * 3.0 + 0.2);

    let mut multibody_desc = MultibodyDesc::new(revo)
        .body_shift(body_shift)
        .parent_shift(Vector2::new(-4.0, 5.0));

    let mut curr = &mut multibody_desc;

    for _ in 0usize..num {
        curr = curr
            .add_child(revo)
            .set_body_shift(body_shift);
    }

    let multibody = multibody_desc.build();
    let multibody_handle = bodies.insert(multibody);

    // Create one collider for each link.
    for i in 0..num + 1 {
        let co = collider_desc.build(BodyPartHandle(multibody_handle, i));
        colliders.insert(co);
    }

    /*
     * Prismatic joint.
     */
    let mut prism = PrismaticJoint::new(Vector2::y_axis(), 0.0);
    // Joint limit so that it does not fall indefinitely.
    prism.enable_min_offset(-rad * 2.0);
    let mut multibody_desc = MultibodyDesc::new(prism)
        .parent_shift(Vector2::new(5.0, 5.0));

    let mut curr = &mut multibody_desc;

    for _ in 0usize..num {
        curr = curr
            .add_child(prism)
            .set_parent_shift(Vector2::x() * rad * 3.0);
    }

    let multibody = multibody_desc.build();
    let multibody_handle = bodies.insert(multibody);

    // Create one collider for each link.
    for i in 0..num + 1 {
        let co = collider_desc.build(BodyPartHandle(multibody_handle, i));
        colliders.insert(co);
    }

    /*
     * Cartesian joint.
     */
    let shift = Vector2::new(0.0, -2.0);
    let width = 5.0 * rad * 4.0;

    for i in 0..7 {
        for j in 0..7 {
            let mut x = i as f32 * rad * 4.0 - width / 2.0;
            let y = j as f32 * rad * 4.0 - width / 2.0 + 4.0;

            if j % 2 == 0 {
                x += rad * 2.0;
            }

            let cart = CartesianJoint::new(Vector2::new(x, y));

            let multibody = MultibodyDesc::new(cart)
                .parent_shift(shift)
                .build();
            let multibody_handle = bodies.insert(multibody);
            colliders.insert(collider_desc.build(BodyPartHandle(multibody_handle, 0)));
        }
    }


    /*
     * Set up the testbed.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(dynamic_world, collider_world, bodies, colliders, joint_constraints, force_generators);
    testbed.look_at(Point2::new(0.0, 4.0), 50.0);
}


fn main() {
    let testbed = Testbed::from_builders(0, vec![
        ("Multibody", init_world),
    ]);
    testbed.run()
}