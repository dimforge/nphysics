use std::cell::RefCell;
use std::rc::Rc;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use detection::activation_manager::ActivationManager;
use detection::detector::Detector;
use detection::joint::ball_in_socket::BallInSocket;
use detection::joint::fixed::Fixed;
use detection::constraint::{Constraint, BallInSocket, Fixed, RBRB};
use object::RigidBody;

/// Structure that handles creation and removal of joints.
pub struct JointManager {
    priv joints: HashMap<uint, Constraint, UintTWHash>
}

impl JointManager {
    /// Creates a new `JointManager`.
    pub fn new() -> JointManager {
        JointManager {
            joints: HashMap::new(UintTWHash::new())
        }
    }

    /// Add a `BallInSocket` joint to this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn add_ball_in_socket(&mut self,
                              joint:      Rc<RefCell<BallInSocket>>,
                              activation: &mut ActivationManager) {
        if self.joints.insert(joint.deref() as *RefCell<BallInSocket> as uint, BallInSocket(joint.clone())) {
            let _  = joint.borrow().anchor1().body.as_ref().map(|b| activation.will_activate(b));
            let _  = joint.borrow().anchor2().body.as_ref().map(|b| activation.will_activate(b));
        }
    }

    /// Removes a `BallInSocket` joint from this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn remove_ball_in_socket(&mut self, joint: &Rc<RefCell<BallInSocket>>, activation: &mut ActivationManager) {
        if self.joints.remove(&(joint.deref() as *RefCell<BallInSocket> as uint)) {
            let _  = joint.borrow().anchor1().body.as_ref().map(|b| activation.will_activate(b));
            let _  = joint.borrow().anchor2().body.as_ref().map(|b| activation.will_activate(b));
        }
    }

    /// Add a `Fixed` joint to this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn add_fixed(&mut self, joint: Rc<RefCell<Fixed>>, activation: &mut ActivationManager) {
        if self.joints.insert(joint.deref() as *RefCell<Fixed> as uint, Fixed(joint.clone())) {
            let _  = joint.borrow().anchor1().body.as_ref().map(|b| activation.will_activate(b));
            let _  = joint.borrow().anchor2().body.as_ref().map(|b| activation.will_activate(b));
        }
    }

    /// Removes a `Fixed` joint from this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn remove_fixed(&mut self, joint: &Rc<RefCell<Fixed>>, activation: &mut ActivationManager) {
        if self.joints.remove(&(joint.deref() as *RefCell<Fixed> as uint)) {
            let _  = joint.borrow().anchor1().body.as_ref().map(|b| activation.will_activate(b));
            let _  = joint.borrow().anchor2().body.as_ref().map(|b| activation.will_activate(b));
        }
    }

    /// Removes every joint attached to a given rigid body.
    ///
    /// This will force the activation of every object attached to the deleted joints.
    pub fn remove(&mut self, b: &Rc<RefCell<RigidBody>>, activation: &mut ActivationManager) {
        let mut keys_to_remove = Vec::new();

        // Remove any joint attached to this body
        // NOTE: this could be improved keeping track of the list of bodies having a joint. This
        // would avoid traversing the joint list to find that a body does not have any joint.
        for elts in self.joints.elements().iter() {
            fn try_remove(b:              &Rc<RefCell<RigidBody>>,
                          ref_b:          &Rc<RefCell<RigidBody>>,
                          keys_to_remove: &mut Vec<uint>,
                          activation:     &mut ActivationManager) {
                if b.deref() as *RefCell<RigidBody> == ref_b.deref() as *RefCell<RigidBody> {
                    keys_to_remove.push(b.deref() as *RefCell<RigidBody> as uint);
                }
                else {
                    activation.will_activate(b);
                }
            }

            match elts.value {
                BallInSocket(ref bis) => {
                    let _ = bis.borrow().anchor2().body.as_ref().map(|r| try_remove(r, b, &mut keys_to_remove, activation));
                    let _ = bis.borrow().anchor1().body.as_ref().map(|r| try_remove(r, b, &mut keys_to_remove, activation));
                },
                Fixed(ref f) => {
                    let _ = f.borrow().anchor2().body.as_ref().map(|r| try_remove(r, b, &mut keys_to_remove, activation));
                    let _ = f.borrow().anchor1().body.as_ref().map(|r| try_remove(r, b, &mut keys_to_remove, activation));
                }
                RBRB(_, _, _) => fail!("Internal error: a contact RBRB should not be here.")
            }
        }

        for k in keys_to_remove.iter() {
            self.joints.remove(k);
        }
    }
}

impl<BF> Detector<RigidBody, Constraint, BF> for JointManager {
    fn update(&mut self, _: &mut BF, activation: &mut ActivationManager) {
        for joint in self.joints.elements().iter() {
            match joint.value {
                BallInSocket(ref bis) => {
                    let mut bbis = bis.borrow_mut();
                    if !bbis.up_to_date() {
                        // the joint has been invalidated by the user: wake up the attached bodies
                        bbis.update();
                        match bbis.anchor1().body {
                            Some(ref b) => activation.will_activate(b),
                            None        => { }
                        }
                        match bbis.anchor2().body {
                            Some(ref b) => activation.will_activate(b),
                            None        => { }
                        }
                    }
                },
                Fixed(ref f) => { // FIXME: code duplication from BallInSocket
                    let mut bf = f.borrow_mut();
                    if !bf.up_to_date() {
                        // the joint has been invalidated by the user: wake up the attached bodies
                        bf.update();
                        match bf.anchor1().body {
                            Some(ref b) => activation.will_activate(b),
                            None        => { }
                        }
                        match bf.anchor2().body {
                            Some(ref b) => activation.will_activate(b),
                            None        => { }
                        }
                    }
                },
                RBRB(_, _, _) => fail!("Internal error: a contact RBRB should not be here.")
 
            }
        }
    }

    fn interferences(&mut self, constraint: &mut Vec<Constraint>, _: &mut BF) {
        for joint in self.joints.elements().iter() {
            constraint.push(joint.value.clone())
        }
    }
}
