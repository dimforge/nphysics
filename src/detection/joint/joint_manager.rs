use std::ptr;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use detection::detector::Detector;
use detection::joint::ball_in_socket::BallInSocket;
use detection::constraint::{Constraint, BallInSocket, RBRB};
use object::Body;
use signal::signal::SignalEmiter;

pub struct JointManager<N, LV, AV, M, II> {
    events: @mut SignalEmiter<N, Body<N, LV, AV, M, II>, Constraint<N, LV, AV, M, II>>,
    joints: HashMap<uint, Constraint<N, LV, AV, M, II>, UintTWHash>
}

impl<N: 'static, LV: 'static, AV: 'static, M: 'static, II: 'static> JointManager<N, LV, AV, M, II> {
    pub fn new(events: @mut SignalEmiter<N, Body<N, LV, AV, M, II>, Constraint<N, LV, AV, M, II>>)
               -> @mut JointManager<N, LV, AV, M, II> {
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
        let mut keys_to_remove = ~[];

        // Remove any joint attached to this body
        // NOTE: this could be improved keeping track of the list of bodies having a joint. This
        // would avoid traversing the joint list to find that a body does not have any joint.
        for elts in self.joints.elements().iter() {
            match elts.value {
                BallInSocket(bis) => {
                    bis.anchor2().body.map(|b| keys_to_remove.push(ptr::to_mut_unsafe_ptr(*b) as uint));
                    bis.anchor1().body.map(|b| keys_to_remove.push(ptr::to_mut_unsafe_ptr(*b) as uint));
                },
                RBRB(_, _, _)     => fail!("Internal error: a contact RBRB should not be here.")
            }
        }

        for k in keys_to_remove.iter() {
            self.joints.remove(k);
        }
    }

    fn update(&mut self) {
    }

    fn interferences(&mut self, constraint: &mut ~[Constraint<N, LV, AV, M, II>]) {
        for joint in self.joints.elements().iter() {
            match joint.value {
                BallInSocket(bis) =>
                    if !bis.up_to_date() {
                        // the joint has been invalidated by the user: wake up the attached bodies
                        bis.update();
                        match bis.anchor1().body {
                            Some(b) => self.events.emit_body_activated(b, constraint),
                            None    => { }
                        }
                        match bis.anchor2().body {
                            Some(b) => self.events.emit_body_activated(b, constraint),
                            None    => { }
                        }
                },
                RBRB(_, _ , _) => { }
            }

            constraint.push(joint.value.clone())
        }
    }

    fn priority(&self) -> f64 { 50.0 }
}
