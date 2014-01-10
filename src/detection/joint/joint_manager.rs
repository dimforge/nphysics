use std::borrow;
use std::cell::RefCell;
use std::rc::Rc;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use detection::detector::Detector;
use detection::joint::ball_in_socket::BallInSocket;
use detection::joint::fixed::Fixed;
use detection::constraint::{Constraint, BallInSocket, Fixed, RBRB};
use object::RigidBody;

pub struct JointManager {
    joints: HashMap<uint, Constraint, UintTWHash>
}

impl JointManager {
    pub fn new() -> JointManager {
        JointManager {
            joints: HashMap::new(UintTWHash::new())
        }
    }

    pub fn add_ball_in_socket(&mut self, joint: Rc<RefCell<BallInSocket>>) {
        self.joints.insert(borrow::to_uint(joint.borrow()), BallInSocket(joint));
    }

    pub fn remove_ball_in_socket(&mut self, joint: &Rc<RefCell<BallInSocket>>) {
        self.joints.remove(&borrow::to_uint(joint.borrow()));
    }

    pub fn add_fixed(&mut self, joint: Rc<RefCell<Fixed>>) {
        self.joints.insert(borrow::to_uint(joint.borrow()), Fixed(joint));
    }

    pub fn remove_fixed(&mut self, joint: &Rc<RefCell<Fixed>>) {
        self.joints.remove(&borrow::to_uint(joint.borrow()));
    }
}

impl<BF> Detector<RigidBody, Constraint, BF> for JointManager {

    /* XXX: object removal related code
    fn remove(&mut self, _: @mut RigidBody) {
        let mut keys_to_remove = ~[];

        // Remove any joint attached to this body
        // NOTE: this could be improved keeping track of the list of bodies having a joint. This
        // would avoid traversing the joint list to find that a body does not have any joint.
        for elts in self.joints.elements().iter() {
            match elts.value {
                BallInSocket(bis) => {
                    bis.anchor2().body.map(|b| keys_to_remove.push(borrow::to_uint(b.borrow())));
                    bis.anchor1().body.map(|b| keys_to_remove.push(borrow::to_uint(b.borrow())));
                },
                Fixed(f) => {
                    f.anchor2().body.map(|b| keys_to_remove.push(borrow::to_uint(b.borrow())));
                    f.anchor1().body.map(|b| keys_to_remove.push(borrow::to_uint(b.borrow())));
                }
                RBRB(_, _, _) => fail!("Internal error: a contact RBRB should not be here.")
            }
        }

        for k in keys_to_remove.iter() {
            self.joints.remove(k);
        }
    }
    */

    fn update(&mut self, _: &mut BF) {
    }

    fn interferences(&mut self, constraint: &mut ~[Constraint], _: &mut BF) {
        for joint in self.joints.elements().iter() {
            match joint.value {
                BallInSocket(ref bis) => {
                    let mut bbis = bis.borrow().borrow_mut();
                    if !bbis.get().up_to_date() {
                        // the joint has been invalidated by the user: wake up the attached bodies
                        bbis.get().update();
                        match bbis.get().anchor1().body {
                            Some(ref b) => { }, // XXX: self.events.request_body_activation(b),
                            None        => { }
                        }
                        match bbis.get().anchor2().body {
                            Some(ref b) => { }, // XXX: self.events.request_body_activation(b),
                            None        => { }
                        }
                    }
                },
                Fixed(ref f) => { // FIXME: code duplication from BallInSocket
                    let mut bf = f.borrow().borrow_mut();
                    if !bf.get().up_to_date() {
                        // the joint has been invalidated by the user: wake up the attached bodies
                        bf.get().update();
                        match bf.get().anchor1().body {
                            Some(ref b) => { }, // XXX: self.events.request_body_activation(b),
                            None        => { }
                        }
                        match bf.get().anchor2().body {
                            Some(ref b) => { }, // XXX: self.events.request_body_activation(b),
                            None        => { }
                        }
                    }
                },
                RBRB(_, _ , _) => { }
            }

            constraint.push(joint.value.clone())
        }
    }
}
