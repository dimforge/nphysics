extern crate native;
extern crate "nalgebra" as na;
extern crate ncollide;
extern crate nphysics;
extern crate nphysics_testbed2d;

use std::num::Float;
use na::{Pnt2, Vec1, Vec2, Translation, Rotation};
use ncollide::shape::{Plane, Cuboid, Ball};
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics::detection::joint::{Anchor, BallInSocket};
use nphysics_testbed2d::Testbed;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec2::new(0.0, 9.81));

    /*
     * A plane for the ground
     */
    let ground_geom = Plane::new(Vec2::new(0.0, -1.0));

    world.add_body(RigidBody::new_static(ground_geom, 0.3, 0.6));

    /*
     * Create the ragdolls
     */
    let n     = 5;
    let shift = 10.0;

    for i in range(0u, n) {
        for j in range(0u, n) {
            let x = i as f32 * shift - n as f32 * shift / 2.0;
            let y = j as f32 * (-shift) - 10.0;

            add_ragdoll(Vec2::new(x, y), &mut world);
        }
    }

    /*
     * Run the simulation.
     */
    let mut testbed = Testbed::new(world);

    testbed.run();
}

fn add_ragdoll(pos: Vec2<f32>, world: &mut World) {
    // head
    let     head_geom = Ball::new(0.8);
    let mut head      = RigidBody::new_dynamic(head_geom, 1.0, 0.3, 0.5);
    head.append_translation(&(pos + Vec2::new(0.0, -2.4)));

    // body
    let     body_geom = Cuboid::new(Vec2::new(1.2, 0.5));
    let mut body      = RigidBody::new_dynamic(body_geom, 1.0, 0.3, 0.5);
    body.append_rotation(&-Vec1::new(Float::frac_pi_2()));
    body.append_translation(&pos);

    // right arm
    let     rarm_geom = Cuboid::new(Vec2::new(1.6, 0.2));
    let mut rarm      = RigidBody::new_dynamic(rarm_geom, 1.0, 0.3, 0.5);
    rarm.append_translation(&(pos + Vec2::new(2.4, -1.0)));

    // left arm
    let mut larm      = rarm.clone();
    larm.append_translation(&Vec2::new(-4.8, 0.0));

    // right foot
    let     rfoot_geom = Cuboid::new(Vec2::new(1.6, 0.2));
    let mut rfoot      = RigidBody::new_dynamic(rfoot_geom, 1.0, 0.3, 0.5);
    rfoot.append_rotation(&-Vec1::new(Float::frac_pi_2()));
    rfoot.append_translation(&(pos + Vec2::new(0.4, 3.0)));

    // left foot
    let mut lfoot      = rfoot.clone();
    lfoot.append_translation(&Vec2::new(-0.8, 0.0));

    let head  = world.add_body(head);
    let body  = world.add_body(body);
    let rarm  = world.add_body(rarm);
    let larm  = world.add_body(larm);
    let rfoot = world.add_body(rfoot);
    let lfoot = world.add_body(lfoot);

    /*
     * Create joints.
     */
    let body_anchor_head  = Anchor::new(Some(body.clone()), Pnt2::new(1.4, 0.0));
    let body_anchor_rarm  = Anchor::new(Some(body.clone()), Pnt2::new(1.0, 0.76));
    let body_anchor_larm  = Anchor::new(Some(body.clone()), Pnt2::new(1.0, -0.76));
    let body_anchor_rfoot = Anchor::new(Some(body.clone()), Pnt2::new(-1.5, 0.3));
    let body_anchor_lfoot = Anchor::new(Some(body.clone()), Pnt2::new(-1.5, -0.3));

    let head_anchor       = Anchor::new(Some(head), Pnt2::new(0.0, 0.9));
    let rarm_anchor       = Anchor::new(Some(rarm), Pnt2::new(-1.5, 0.0));
    let larm_anchor       = Anchor::new(Some(larm), Pnt2::new(1.5, 0.0));
    let rfoot_anchor      = Anchor::new(Some(rfoot), Pnt2::new(1.5, 0.0));
    let lfoot_anchor      = Anchor::new(Some(lfoot), Pnt2::new(1.5, 0.0));

    let head_joint  = BallInSocket::new(body_anchor_head,   head_anchor);
    let rarm_joint  = BallInSocket::new(body_anchor_rarm,   rarm_anchor);
    let larm_joint  = BallInSocket::new(body_anchor_larm,   larm_anchor);
    let rfoot_joint = BallInSocket::new(body_anchor_rfoot, rfoot_anchor);
    let lfoot_joint = BallInSocket::new(body_anchor_lfoot, lfoot_anchor);

    world.add_ball_in_socket(head_joint);
    world.add_ball_in_socket(rarm_joint);
    world.add_ball_in_socket(larm_joint);
    world.add_ball_in_socket(rfoot_joint);
    world.add_ball_in_socket(lfoot_joint);
}
