extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::Vector2;
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::joint::{CartesianJoint, PrismaticJoint, RevoluteJoint};
use nphysics2d::object::{ColliderDesc, MultibodyDesc};
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;


pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Ground
     */
    let ground_size = 25.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 1.0)));

    ColliderDesc::new(ground_shape)
        .translation(-Vector2::y() * 5.0)
        .build(&mut world);

    /*
     * Shape that will be re-used for several multibody links.
     */
    let rad = 0.2;
    let cuboid = ShapeHandle::new(Cuboid::new(Vector2::repeat(rad)));

    /*
     * Revolute joint.
     */
    let num = 10;
    let revo = RevoluteJoint::new(-0.1);
    let body_shift = Vector2::x() * (rad * 3.0 + 0.2);

    let collider = ColliderDesc::new(cuboid.clone()).density(1.0);
    let mut multibody = MultibodyDesc::new(revo)
        .body_shift(body_shift)
        .parent_shift(Vector2::new(-4.0, 5.0))
        .collider(&collider);

    let mut curr = &mut multibody;

    for _ in 0usize..num {
        curr = curr
            .add_child(revo)
            .set_body_shift(body_shift)
            .add_collider(&collider);
    }

    multibody.build(&mut world);

    /*
     * Prismatic joint.
     */
    let mut prism = PrismaticJoint::new(Vector2::y_axis(), 0.0);
    // Joint limit so that it does not fall indefinitely.
    prism.enable_min_offset(-rad * 2.0);
    let mut multibody = MultibodyDesc::new(prism)
        .parent_shift(Vector2::new(5.0, 5.0))
        .collider(&collider);

    let mut curr = &mut multibody;

    for _ in 0usize..num {
        curr = curr
            .add_child(prism)
            .set_parent_shift(Vector2::x() * rad * 3.0)
            .add_collider(&collider);
    }

    multibody.build(&mut world);

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

            let rect = CartesianJoint::new(Vector2::new(x, y));

            MultibodyDesc::new(rect)
                .parent_shift(shift)
                .collider(&collider)
                .build(&mut world);
        }
    }


    /*
     * Set up the testbed.
     */
    testbed.set_world(world);
}


fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}