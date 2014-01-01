use std::ptr;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use detection::detector::Detector;
use detection::joint::ball_in_socket::BallInSocket;
use detection::joint::fixed::Fixed;
use detection::constraint::{Constraint, BallInSocket, Fixed, RBRB};
use object::Body;
use signal::signal::SignalEmiter;

pub struct JointManager {
    events: @mut SignalEmiter<Body, Constraint>,
    joints: HashMap<uint, Constraint, UintTWHash>
}

impl JointManager {
    pub fn new(events: @mut SignalEmiter<Body, Constraint>) -> @mut JointManager {
        /*
         * NOTE: we dont listen to `body_activated` and `body_deactivated` signals because this is
         * not worth it:
         *  1) it is not easy to implement since we have to keep track of a list of joint per rigid
         *     body.
         *  2) most of the time we dont have a number of joints significant enough to justify the
         *     overhead.
         */
        @mut JointManager {
            events: events,
            joints: HashMap::new(UintTWHash::new())
        }
    }

    pub fn add_ball_in_socket(&mut self, joint: @mut BallInSocket) {
        self.joints.insert(ptr::to_mut_unsafe_ptr(joint) as uint, BallInSocket(joint));
    }

    pub fn remove_ball_in_socket(&mut self, joint: @mut BallInSocket) {
        self.joints.remove(&(ptr::to_mut_unsafe_ptr(joint) as uint));
    }

    pub fn add_fixed(&mut self, joint: @mut Fixed) {
        self.joints.insert(ptr::to_mut_unsafe_ptr(joint) as uint, Fixed(joint));
    }

    pub fn remove_fixed(&mut self, joint: @mut Fixed) {
        self.joints.remove(&(ptr::to_mut_unsafe_ptr(joint) as uint));
    }
}

impl Detector<Body, Constraint> for JointManager {
    fn add(&mut self, _: @mut Body) {
    }

    fn remove(&mut self, _: @mut Body) {
        let mut keys_to_remove = ~[];

        // Remove any joint attached to this body
        // NOTE: this could be improved keeping track of the list of bodies having a joint. This
        // would avoid traversing the joint list to find that a body does not have any joint.
        for elts in self.joints.elements().iter() {
            match elts.value {
                BallInSocket(bis) => {
                    bis.anchor2().body.map(|b| keys_to_remove.push(ptr::to_mut_unsafe_ptr(b) as uint));
                    bis.anchor1().body.map(|b| keys_to_remove.push(ptr::to_mut_unsafe_ptr(b) as uint));
                },
                Fixed(f) => {
                    f.anchor2().body.map(|b| keys_to_remove.push(ptr::to_mut_unsafe_ptr(b) as uint));
                    f.anchor1().body.map(|b| keys_to_remove.push(ptr::to_mut_unsafe_ptr(b) as uint));
                }
                RBRB(_, _, _) => fail!("Internal error: a contact RBRB should not be here.")
            }
        }

        for k in keys_to_remove.iter() {
            self.joints.remove(k);
        }
    }

    fn update(&mut self) {
    }

    fn interferences(&mut self, constraint: &mut ~[Constraint]) {
        for joint in self.joints.elements().iter() {
            match joint.value {
                BallInSocket(bis) =>
                    if !bis.up_to_date() {
                        // the joint has been invalidated by the user: wake up the attached bodies
                        bis.update();
                        match bis.anchor1().body {
                            Some(b) => self.events.request_body_activation(b),
                            None    => { }
                        }
                        match bis.anchor2().body {
                            Some(b) => self.events.request_body_activation(b),
                            None    => { }
                        }
                },
                Fixed(bis) => // FIXME: code duplication from BallInSocket
                    if !bis.up_to_date() {
                        // the joint has been invalidated by the user: wake up the attached bodies
                        bis.update();
                        match bis.anchor1().body {
                            Some(b) => self.events.request_body_activation(b),
                            None    => { }
                        }
                        match bis.anchor2().body {
                            Some(b) => self.events.request_body_activation(b),
                            None    => { }
                        }
                },
                RBRB(_, _ , _) => { }
            }

            constraint.push(joint.value.clone())
        }
    }

    fn priority(&self) -> f64 { 49.0 }
}
