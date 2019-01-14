extern crate nalgebra as na;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics3d::joint::{BallJoint, FreeJoint};
use nphysics3d::object::{ColliderDesc, MultibodyDesc};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;


fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Ground.
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size)));

    ColliderDesc::new(ground_shape)
        .with_translation(Vector3::y() * -ground_size)
        .build(&mut world);

    /*
     * Create the ragdolls
     */
    build_ragdolls(&mut world);

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(-20.0, 10.0, -20.0), Point3::new(0.0, 1.0, 0.0));
    testbed.run();
}

fn build_ragdolls(world: &mut World<f32>) {
    let body_rady = 1.2;
    let body_radz = 0.4;
    let body_radx = 0.2;
    let head_rad = 0.4;
    let member_rad = 0.1;
    let arm_length = 0.9;
    let leg_length = 1.4;
    let space = 0.1;

    let body_geom = ShapeHandle::new(Cuboid::new(Vector3::new(body_radx, body_rady, body_radz)));
    let head_geom = ShapeHandle::new(Ball::new(head_rad));
    let arm_geom = ShapeHandle::new(Cuboid::new(Vector3::new(
        member_rad, arm_length, member_rad,
    )));
    let leg_geom = ShapeHandle::new(Cuboid::new(Vector3::new(
        member_rad, leg_length, member_rad,
    )));

    // The position of the free joint will be modified in the
    // final loop of this function (before we actually build the
    // ragdoll body into the World.
    let free = FreeJoint::new(Isometry3::identity());
    let spherical = BallJoint::new(na::zero());

    /*
     * Body.
     */
    let body_collider = ColliderDesc::new(body_geom).with_density(0.3);
    let mut body = MultibodyDesc::new(free)
        .with_collider(&body_collider);

    /*
     * Head.
     */
    let head_collider = ColliderDesc::new(head_geom).with_density(0.3);
    body.add_child(spherical)
        .add_collider(&head_collider)
        .set_parent_shift(Vector3::new(0.0, body_rady + head_rad + space * 2.0, 0.0));


    /*
     * Arms.
     */
    let arm_collider = ColliderDesc::new(arm_geom).with_density(0.3);
    body.add_child(spherical)
        .add_collider(&arm_collider)
        .set_parent_shift(Vector3::new(0.0, body_rady, body_radx + 2.0 * space))
        .set_body_shift(Vector3::new(0.0, arm_length + space, 0.0));

    body.add_child(spherical)
        .add_collider(&arm_collider)
        .set_parent_shift(Vector3::new(0.0, body_rady, -body_radx - 2.0 * space))
        .set_body_shift(Vector3::new(0.0, arm_length + space, 0.0));

    /*
     * Legs.
     */
    let leg_collider = ColliderDesc::new(leg_geom).with_density(0.3);
    body.add_child(spherical)
        .add_collider(&leg_collider)
        .set_parent_shift(Vector3::new(0.0, -body_rady, body_radx))
        .set_body_shift(Vector3::new(0.0, leg_length + space, 0.0));


    body.add_child(spherical)
        .add_collider(&leg_collider)
        .set_parent_shift(Vector3::new(0.0, -body_rady, -body_radx))
        .set_body_shift(Vector3::new(0.0, leg_length + space, 0.0));

    let n = 3;
    let shift = 5.0;
    let shifty = 6.0;

    for i in 0usize..n {
        for j in 0usize..n {
            for k in 0usize..n {
                let x = i as f32 * shift - n as f32 * shift / 2.0;
                let y = j as f32 * shifty + 10.0;
                let z = k as f32 * shift - n as f32 * shift / 2.0;

                let free = FreeJoint::new(Isometry3::translation(x, y, z));
                body.set_joint(free)
                    .build(world);
            }
        }
    }
}
