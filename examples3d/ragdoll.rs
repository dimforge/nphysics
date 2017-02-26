extern crate nphysics_testbed3d;
extern crate ncollide;
extern crate nphysics3d;
extern crate nalgebra as na;

use std::f32;
use na::{Point3, Vector3, Translation3, UnitQuaternion};
use ncollide::shape::{Plane, Ball, Cylinder};
use nphysics3d::world::World;
use nphysics3d::object::RigidBody;
use nphysics3d::detection::joint::{Anchor, BallInSocket};
use nphysics_testbed3d::Testbed;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * A plane for the ground
     */
    let ground_geom = Plane::new(Vector3::new(0.0, 1.0, 0.0));

    world.add_rigid_body(RigidBody::new_static(ground_geom, 0.3, 0.6));

    /*
     * Create the ragdolls
     */
    let n     = 5;
    let shift = 10.0;

    for i in 0usize .. n {
        for j in 0usize .. n {
            for k in 0usize .. n {
                let x = i as f32 * shift - n as f32 * shift / 2.0;
                let y = j as f32 * shift + 10.0;
                let z = k as f32 * shift - n as f32 * shift / 2.0;

                add_ragdoll(Vector3::new(x, y, z), &mut world);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);

    testbed.look_at(Point3::new(-30.0, 30.0, -30.0), Point3::new(0.0, 0.0, 0.0));
    testbed.run();
}

fn add_ragdoll(pos: Vector3<f32>, world: &mut World<f32>) {
    // head
    let     head_geom = Ball::new(0.8);
    let mut head      = RigidBody::new_dynamic(head_geom, 1.0, 0.3, 0.5);
    head.append_translation(&Translation3::from_vector(pos + Vector3::new(0.0, 2.4, 0.0)));

    // body
    let     body_geom = Cylinder::new(1.2, 0.5);
    let mut body      = RigidBody::new_dynamic(body_geom, 1.0, 0.3, 0.5);
    body.append_translation(&Translation3::from_vector(pos));

    // right arm
    let     rarm_geom = Cylinder::new(1.6, 0.2);
    let mut rarm      = RigidBody::new_dynamic(rarm_geom, 1.0, 0.3, 0.5);
    rarm.append_rotation(&UnitQuaternion::from_scaled_axis(Vector3::x() * f32::consts::FRAC_PI_2));
    rarm.append_translation(&Translation3::from_vector(pos + Vector3::new(0.0, 1.0, 2.4)));

    // left arm
    let mut larm      = rarm.clone();
    larm.append_translation(&Translation3::new(0.0, 0.0, -4.8));

    // right foot
    let     rfoot_geom = Cylinder::new(1.6, 0.2);
    let mut rfoot      = RigidBody::new_dynamic(rfoot_geom, 1.0, 0.3, 0.5);
    rfoot.append_translation(&Translation3::from_vector(pos + Vector3::new(0.0, -3.0, 0.4)));

    // left foot
    let mut lfoot      = rfoot.clone();
    lfoot.append_translation(&Translation3::new(0.0, 0.0, -0.8));

    let head  = world.add_rigid_body(head);
    let body  = world.add_rigid_body(body);
    let rarm  = world.add_rigid_body(rarm);
    let larm  = world.add_rigid_body(larm);
    let rfoot = world.add_rigid_body(rfoot);
    let lfoot = world.add_rigid_body(lfoot);

    /*
     * Create joints.
     */
    let body_anchor_head  = Anchor::new(Some(body.clone()), Point3::new(0.0, 1.5, 0.0));
    let body_anchor_rarm  = Anchor::new(Some(body.clone()), Point3::new(0.0, 1.0, 0.75));
    let body_anchor_larm  = Anchor::new(Some(body.clone()), Point3::new(0.0, 1.0, -0.75));
    let body_anchor_rfoot = Anchor::new(Some(body.clone()), Point3::new(0.0, -1.5, 0.2));
    let body_anchor_lfoot = Anchor::new(Some(body.clone()), Point3::new(0.0, -1.5, -0.2));

    let head_anchor       = Anchor::new(Some(head), Point3::new(0.0, -0.9, 0.0));
    let rarm_anchor       = Anchor::new(Some(rarm), Point3::new(0.0, -1.7, 0.0));
    let larm_anchor       = Anchor::new(Some(larm), Point3::new(0.0, 1.7, 0.0));
    let rfoot_anchor      = Anchor::new(Some(rfoot), Point3::new(0.0, 1.7, 0.0));
    let lfoot_anchor      = Anchor::new(Some(lfoot), Point3::new(0.0, 1.7, 0.0));

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
