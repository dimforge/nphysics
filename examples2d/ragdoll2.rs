extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Point2, Vector2};
use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics2d::joint::{FreeJoint, RevoluteJoint};
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
     * A plane for the ground
     */
    let ground_size = 25.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 1.0)));

    ColliderDesc::new(ground_shape)
        .translation(-Vector2::y())
        .build(&mut world);

    /*
     * Create the ragdolls
     */
    add_ragdolls(&mut world);

    /*
     * Run the simulation.
     */
    testbed.set_world(world);
    testbed.look_at(Point2::new(0.0, -5.0), 25.0);
}

fn add_ragdolls(world: &mut World<f32>) {
    let body_rady = 1.2;
    let body_radx = 0.2;
    let head_rad = 0.4;
    let member_rad = 0.1;
    let arm_length = 0.9;
    let leg_length = 1.4;
    let space = 0.1;

    let body_geom = ShapeHandle::new(Cuboid::new(Vector2::new(body_radx, body_rady)));
    let head_geom = ShapeHandle::new(Ball::new(head_rad));
    let arm_geom = ShapeHandle::new(Cuboid::new(Vector2::new(member_rad, arm_length)));
    let leg_geom = ShapeHandle::new(Cuboid::new(Vector2::new(member_rad, leg_length)));

    // The position of the free joint will be modified in the
    // final loop of this function (before we actually build the
    // ragdoll body into the World.
    let free = FreeJoint::new(Isometry2::new(Vector2::zeros(), na::zero()));
    let spherical = RevoluteJoint::new(na::zero());

    /*
     * Body.
     */
    let body_collider = ColliderDesc::new(body_geom).density(0.3);
    let mut body = MultibodyDesc::new(free)
        .collider(&body_collider);

    /*
     * Head.
     */
    let head_collider = ColliderDesc::new(head_geom).density(0.3);
    body.add_child(spherical)
        .add_collider(&head_collider)
        .set_parent_shift(Vector2::new(0.0, body_rady + head_rad + space * 2.0));

    /*
     * Arms.
     */
    let arm_collider = ColliderDesc::new(arm_geom).density(0.3);
    body.add_child(spherical)
        .add_collider(&arm_collider)
        .set_parent_shift(Vector2::new(body_radx + 2.0 * space, body_rady))
        .set_body_shift(Vector2::new(0.0, arm_length + space));

    body.add_child(spherical)
        .add_collider(&arm_collider)
        .set_parent_shift(Vector2::new(-body_radx - 2.0 * space, body_rady))
        .set_body_shift(Vector2::new(0.0, arm_length + space));

    /*
     * Legs.
     */
    let leg_collider = ColliderDesc::new(leg_geom).density(0.3);
    body.add_child(spherical)
        .add_collider(&leg_collider)
        .set_parent_shift(Vector2::new(body_radx, -body_rady))
        .set_body_shift(Vector2::new(0.0, leg_length + space));


    body.add_child(spherical)
        .add_collider(&leg_collider)
        .set_parent_shift(Vector2::new(-body_radx, -body_rady))
        .set_body_shift(Vector2::new(0.0, leg_length + space));


    let n = 5;
    let shiftx = 2.0;
    let shifty = 6.5;

    for i in 0usize..n {
        for j in 0usize..n {
            let x = i as f32 * shiftx - n as f32 * shiftx / 2.0;
            let y = j as f32 * shifty + 6.0;

            let free = FreeJoint::new(Isometry2::translation(x, y));
            body.set_joint(free)
                .build(world);
        }
    }
}


fn main() {
    let mut testbed = Testbed::new_empty();
    init_world(&mut testbed);
    testbed.run();
}