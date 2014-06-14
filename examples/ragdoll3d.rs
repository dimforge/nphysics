#![crate_type = "bin"]
#![warn(non_camel_case_types)]

extern crate std;
extern crate native;
extern crate kiss3d;
extern crate graphics3d;
extern crate nphysics = "nphysics3df32";
extern crate ncollide = "ncollide3df32";
extern crate nalgebra;

use std::rc::Rc;
use std::cell::RefCell;
use nalgebra::na::{Vec3, Translation, Rotation};
use kiss3d::window::Window;
use ncollide::geom::{Plane, Ball, Cylinder};
use nphysics::world::World;
use nphysics::object::{RigidBody, Static, Dynamic};
use nphysics::detection::joint::{Anchor, BallInSocket};
use graphics3d::engine::GraphicsManager;

#[start]
fn start(argc: int, argv: **u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    GraphicsManager::simulate(boxes_vee_3d)
}

pub fn boxes_vee_3d(window: &mut Window, graphics: &mut GraphicsManager) -> World {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vec3::new(0.0f32, -9.81, 0.0));

    /*
     * A plane for the ground
     */
    let ground_geom = Plane::new(Vec3::new(0.0f32, 1.0, 0.0));
    let ground      = Rc::new(RefCell::new(RigidBody::new(ground_geom, 0.0f32, Static, 0.3, 0.6)));

    world.add_body(ground.clone());
    graphics.add(window, ground);

    /*
     * Create the ragdolls
     */
    let n     = 5;
    let shift = 10.0;

    for i in range(0, n) {
        for j in range(0, n) {
            for k in range(0, n) {
                let x = i as f32 * shift - n as f32 * shift / 2.0;
                let y = j as f32 * shift + 10.0;
                let z = k as f32 * shift - n as f32 * shift / 2.0;

                add_ragdoll(Vec3::new(x, y, z), &mut world, window, graphics);
            }
        }
    }

    /*
     * Set up the camera and that is it!
     */
    graphics.look_at(Vec3::new(-30.0, 30.0, -30.0), Vec3::new(0.0, 0.0, 0.0));

    world
}

fn add_ragdoll(pos:      Vec3<f32>,
               world:    &mut World,
               window:   &mut Window,
               graphics: &mut GraphicsManager) {
    // head
    let     head_geom = Ball::new(0.8);
    let mut head      = RigidBody::new(head_geom, 1.0f32, Dynamic, 0.3, 0.5);
    head.append_translation(&(pos + Vec3::new(0.0f32, 2.4, 0.0)));

    // body
    let     body_geom = Cylinder::new(1.2, 0.5);
    let mut body      = RigidBody::new(body_geom, 1.0f32, Dynamic, 0.3, 0.5);
    body.append_translation(&pos);

    // right arm
    let     rarm_geom = Cylinder::new(1.6, 0.2);
    let mut rarm      = RigidBody::new(rarm_geom, 1.0f32, Dynamic, 0.3, 0.5);
    rarm.append_rotation(&Vec3::new(Float::frac_pi_2(), 0.0, 0.0));
    rarm.append_translation(&(pos + Vec3::new(0.0f32, 1.0, 2.4)));

    // left arm
    let mut larm      = rarm.clone();
    larm.append_translation(&Vec3::new(0.0f32, 0.0, -4.8));

    // right foot
    let     rfoot_geom = Cylinder::new(1.6, 0.2);
    let mut rfoot      = RigidBody::new(rfoot_geom, 1.0f32, Dynamic, 0.3, 0.5);
    rfoot.append_translation(&(pos + Vec3::new(0.0f32, -3.0, 0.4)));

    // left foot
    let mut lfoot      = rfoot.clone();
    lfoot.append_translation(&Vec3::new(0.0f32, 0.0, -0.8));


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

    let color = graphics.gen_color();
    graphics.add_with_color(window, head.clone(), color);
    graphics.add_with_color(window, body.clone(), color);
    graphics.add_with_color(window, rarm.clone(), color);
    graphics.add_with_color(window, larm.clone(), color);
    graphics.add_with_color(window, rfoot.clone(), color);
    graphics.add_with_color(window, lfoot.clone(), color);

    /*
     * Create joints.
     */
    let body_anchor_head  = Anchor::new(Some(body.clone()), Vec3::new(0.0, 1.5, 0.0));
    let body_anchor_rarm  = Anchor::new(Some(body.clone()), Vec3::new(0.0, 1.0, 0.75));
    let body_anchor_larm  = Anchor::new(Some(body.clone()), Vec3::new(0.0, 1.0, -0.75));
    let body_anchor_rfoot = Anchor::new(Some(body.clone()), Vec3::new(0.0, -1.5, 0.2));
    let body_anchor_lfoot = Anchor::new(Some(body.clone()), Vec3::new(0.0, -1.5, -0.2));

    let head_anchor       = Anchor::new(Some(head), Vec3::new(0.0, -0.9, 0.0));
    let rarm_anchor       = Anchor::new(Some(rarm), Vec3::new(0.0, -1.7, 0.0));
    let larm_anchor       = Anchor::new(Some(larm), Vec3::new(0.0, 1.7, 0.0));
    let rfoot_anchor      = Anchor::new(Some(rfoot), Vec3::new(0.0, 1.7, 0.0));
    let lfoot_anchor      = Anchor::new(Some(lfoot), Vec3::new(0.0, 1.7, 0.0));

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
