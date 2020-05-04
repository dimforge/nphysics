extern crate nalgebra as na;

use na::{Point2, RealField, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::joint::{CartesianJoint, PrismaticJoint, RevoluteJoint};
use nphysics2d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, MultibodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed2d::Testbed;

pub fn init_world<N: RealField>(testbed: &mut Testbed<N>) {
    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector2::new(r!(0.0), r!(-9.81)));
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    /*
     * Ground
     */
    let ground_size = r!(25.0);
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, r!(1.0))));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .translation(-Vector2::y() * r!(5.0))
        .build(BodyPartHandle(ground_handle, 0));
    colliders.insert(co);

    /*
     * Shape that will be re-used for several multibody links.
     */
    let rad = r!(0.2);
    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));
    let collider_desc = ColliderDesc::new(cuboid.clone()).density(r!(1.0));

    /*
     * Revolute joint.
     */
    let num = 10;
    let revo = RevoluteJoint::new(r!(-0.1));
    let body_shift = Vector2::x() * (rad * r!(3.0) + r!(0.2));

    let mut multibody_desc = MultibodyDesc::new(revo)
        .body_shift(body_shift)
        .parent_shift(Vector2::new(r!(-4.0), r!(5.0)));

    let mut curr = &mut multibody_desc;

    for _ in 0usize..num {
        curr = curr.add_child(revo).set_body_shift(body_shift);
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
    let mut prism = PrismaticJoint::new(Vector2::y_axis(), r!(0.0));
    // Joint limit so that it does not fall indefinitely.
    prism.enable_min_offset(-rad * r!(2.0));
    let mut multibody_desc = MultibodyDesc::new(prism).parent_shift(Vector2::new(r!(5.0), r!(5.0)));

    let mut curr = &mut multibody_desc;

    for _ in 0usize..num {
        curr = curr
            .add_child(prism)
            .set_parent_shift(Vector2::x() * rad * r!(3.0));
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
    let shift = Vector2::new(r!(0.0), r!(-2.0));
    let width = r!(5.0) * rad * r!(4.0);

    for i in 0..7 {
        for j in 0..7 {
            let mut x = r!(i as f64) * rad * r!(4.0) - width / r!(2.0);
            let y = r!(j as f64) * rad * r!(4.0) - width / r!(2.0) + r!(4.0);

            if j % 2 == 0 {
                x += rad * r!(2.0);
            }

            let cart = CartesianJoint::new(Vector2::new(x, y));

            let multibody = MultibodyDesc::new(cart).parent_shift(shift).build();
            let multibody_handle = bodies.insert(multibody);
            colliders.insert(collider_desc.build(BodyPartHandle(multibody_handle, 0)));
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.look_at(Point2::new(0.0, -4.0), 50.0);
}

fn main() {
    let testbed = Testbed::<f32>::from_builders(0, vec![("Multibody", init_world)]);
    testbed.run()
}
