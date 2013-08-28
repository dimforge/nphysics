use std::ptr;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use detection::detector::Detector;
use detection::joint::ball_in_socket::BallInSocket;
use detection::constraint::{Constraint, BallInSocket};
use object::body::Body;

pub struct JointManager<N, LV, AV, M, II> {
    joints: HashMap<uint, Constraint<N, LV, AV, M, II>, UintTWHash>
}

impl<N, LV, AV, M, II> JointManager<N, LV, AV, M, II> {
    pub fn add_ball_in_socket(&mut self, joint: @mut BallInSocket<N, LV, AV, M, II>) {
        self.joints.insert(ptr::to_mut_unsafe_ptr(joint) as uint, BallInSocket(joint));
    }

    pub fn remove_ball_in_socket(&mut self, joint: @mut BallInSocket<N, LV, AV, M, II>) {
        self.joints.remove(&(ptr::to_mut_unsafe_ptr(joint) as uint));
    }
}

impl<N:  Clone,
     LV: Clone,
     AV,
     M,
     II>
Detector<N, Body<N, LV, AV, M, II>, Constraint<N, LV, AV, M, II>> for JointManager<N, LV, AV, M, II> {
    fn add(&mut self, _: @mut Body<N, LV, AV, M, II>) {
    }

    fn remove(&mut self, _: @mut Body<N, LV, AV, M, II>) {
        // XXX Remove every joint involving this body
        fail!("Not yet implemented.")
    }

    fn update(&mut self) {
    }

    fn interferences(&mut self, constraint: &mut ~[Constraint<N, LV, AV, M, II>]) {
        for joint in self.joints.elements().iter() {
            constraint.push(joint.value.clone())
        }
    }

    fn priority(&self) -> f64 { 50.0 }
}
