#![crate_type = "bin"]
#![warn(non_camel_case_types)]

extern crate std;
extern crate native;
extern crate rsfml;
extern crate nalgebra;
extern crate ncollide = "ncollide2df32";
extern crate nphysics = "nphysics2df32";
extern crate graphics2d;

use std::rc::Rc;
use std::cell::RefCell;
use nalgebra::na::{Vec1, Vec2, Translation, Rotation};
use ncollide::geom::{Plane, Cuboid, Ball};
use nphysics::world::World;
use nphysics::object::RigidBody;
use nphysics::detection::joint::{Anchor, BallInSocket};
use graphics2d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(boxes_vee_3d)
}

pub fn boxes_vee_3d(graphics: &mut GraphicsManager) -> World {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec2::new(0.0f32, 9.81));

    /*
     * A plane for the ground
     */
    let ground_geom = Plane::new(Vec2::new(0.0f32, -1.0));
    let ground      = Rc::new(RefCell::new(RigidBody::new_static(ground_geom, 0.3, 0.6)));

    world.add_body(ground.clone());
    graphics.add(ground);

    /*
     * Create the ragdolls
     */
    let n     = 5;
    let shift = 10.0;

    for i in range(0u, n) {
        for j in range(0u, n) {
            let x = i as f32 * shift - n as f32 * shift / 2.0;
            let y = j as f32 * (-shift) - 10.0;

            add_ragdoll(Vec2::new(x, y), &mut world, graphics);
        }
    }

    /*
     * Set up the camera and that is it!
     */

    world
}

fn add_ragdoll(pos: Vec2<f32>, world: &mut World, graphics: &mut GraphicsManager) {
    // head
    let     head_geom = Ball::new(0.8);
    let mut head      = RigidBody::new_dynamic(head_geom, 1.0f32, 0.3, 0.5);
    head.append_translation(&(pos + Vec2::new(0.0f32, -2.4)));

    // body
    let     body_geom = Cuboid::new(Vec2::new(1.2, 0.5));
    let mut body      = RigidBody::new_dynamic(body_geom, 1.0f32, 0.3, 0.5);
    body.append_rotation(&-Vec1::new(Float::frac_pi_2()));
    body.append_translation(&pos);

    // right arm
    let     rarm_geom = Cuboid::new(Vec2::new(1.6, 0.2));
    let mut rarm      = RigidBody::new_dynamic(rarm_geom, 1.0f32, 0.3, 0.5);
    rarm.append_translation(&(pos + Vec2::new(2.4f32, -1.0)));

    // left arm
    let mut larm      = rarm.clone();
    larm.append_translation(&Vec2::new(-4.8f32, 0.0));

    // right foot
    let     rfoot_geom = Cuboid::new(Vec2::new(1.6, 0.2));
    let mut rfoot      = RigidBody::new_dynamic(rfoot_geom, 1.0f32, 0.3, 0.5);
    rfoot.append_rotation(&-Vec1::new(Float::frac_pi_2()));
    rfoot.append_translation(&(pos + Vec2::new(0.4f32, 3.0)));

    // left foot
    let mut lfoot      = rfoot.clone();
    lfoot.append_translation(&Vec2::new(-0.8f32, 0.0));


    let head  = Rc::new(RefCell::new(head));
    let body  = Rc::new(RefCell::new(body));
    let rarm  = Rc::new(RefCell::new(rarm));
    let larm  = Rc::new(RefCell::new(larm));
    let rfoot = Rc::new(RefCell::new(rfoot));
    let lfoot = Rc::new(RefCell::new(lfoot));

    world.add_body(head.clone());
    world.add_body(body.clone());
    world.add_body(rarm.clone());
    world.add_body(larm.clone());
    world.add_body(rfoot.clone());
    world.add_body(lfoot.clone());

    // let color = graphics.gen_color();
    graphics.add(head.clone());
    graphics.add(body.clone());
    graphics.add(rarm.clone());
    graphics.add(larm.clone());
    graphics.add(rfoot.clone());
    graphics.add(lfoot.clone());

    /*
     * Create joints.
     */
    let body_anchor_head  = Anchor::new(Some(body.clone()), Vec2::new(1.3, 0.0));
    let body_anchor_rarm  = Anchor::new(Some(body.clone()), Vec2::new(1.0, 0.75));
    let body_anchor_larm  = Anchor::new(Some(body.clone()), Vec2::new(1.0, -0.75));
    let body_anchor_rfoot = Anchor::new(Some(body.clone()), Vec2::new(-1.3, 0.2));
    let body_anchor_lfoot = Anchor::new(Some(body.clone()), Vec2::new(-1.3, -0.2));

    let head_anchor       = Anchor::new(Some(head), Vec2::new(0.0, 0.9));
    let rarm_anchor       = Anchor::new(Some(rarm), Vec2::new(-1.5, 0.0));
    let larm_anchor       = Anchor::new(Some(larm), Vec2::new(1.5, 0.0));
    let rfoot_anchor      = Anchor::new(Some(rfoot), Vec2::new(1.5, 0.0));
    let lfoot_anchor      = Anchor::new(Some(lfoot), Vec2::new(1.5, 0.0));

    let head_joint  = BallInSocket::new(body_anchor_head,   head_anchor);
    let rarm_joint  = BallInSocket::new(body_anchor_rarm,   rarm_anchor);
    let larm_joint  = BallInSocket::new(body_anchor_larm,   larm_anchor);
    let rfoot_joint = BallInSocket::new(body_anchor_rfoot, rfoot_anchor);
    let lfoot_joint = BallInSocket::new(body_anchor_lfoot, lfoot_anchor);

    world.add_ball_in_socket(Rc::new(RefCell::new(head_joint)));
    world.add_ball_in_socket(Rc::new(RefCell::new(rarm_joint)));
    world.add_ball_in_socket(Rc::new(RefCell::new(larm_joint)));
    world.add_ball_in_socket(Rc::new(RefCell::new(rfoot_joint)));
    world.add_ball_in_socket(Rc::new(RefCell::new(lfoot_joint)));
}
