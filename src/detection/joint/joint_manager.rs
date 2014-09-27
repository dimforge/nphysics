use std::cell::RefCell;
use std::rc::Rc;
use std::ptr;
use ncollide::utils::data::hash_map::HashMap;
use ncollide::utils::data::hash::UintTWHash;
use detection::activation_manager::ActivationManager;
use detection::detector::Detector;
use detection::joint::ball_in_socket::BallInSocket;
use detection::joint::fixed::Fixed;
use detection::joint::joint::Joint;
use detection::constraint::{Constraint, BallInSocketConstraint, FixedConstraint, RBRB};
use object::RigidBody;

/// Structure that handles creation and removal of joints.
pub struct JointManager {
    joints:      HashMap<uint, Constraint, UintTWHash>,
    body2joints: HashMap<uint, Vec<Constraint>, UintTWHash>
}

impl JointManager {
    /// Creates a new `JointManager`.
    pub fn new() -> JointManager {
        JointManager {
            joints:      HashMap::new(UintTWHash::new()),
            body2joints: HashMap::new(UintTWHash::new())
        }
    }

    /// Joints handled by this manager.
    #[inline]
    pub fn joints<'a>(&'a self) -> &'a HashMap<uint, Constraint, UintTWHash> {
        &self.joints
    }

    /// List of joints attached to a specific body.
    #[inline]
    pub fn joints_with_body<'a>(&'a self, body: &Rc<RefCell<RigidBody>>) -> Option<&'a [Constraint]> {
        self.body2joints.find(&(body.deref() as *const RefCell<RigidBody> as uint)).map(|v| v.as_slice())
    }

    /// Add a `BallInSocket` joint to this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn add_ball_in_socket(&mut self,
                              joint:      Rc<RefCell<BallInSocket>>,
                              activation: &mut ActivationManager) {
        if self.joints.insert(joint.deref() as *const RefCell<BallInSocket> as uint, BallInSocketConstraint(joint.clone())) {
            match joint.borrow().anchor1().body.as_ref() {
                Some(b) => {
                    activation.will_activate(b);
                    let js = self.body2joints.find_or_insert_lazy(b.deref() as *const RefCell<RigidBody> as uint,
                                                                  || Some(Vec::new()));
                    js.unwrap().push(BallInSocketConstraint(joint.clone()));
                },
                _ => { }
            }

            match joint.borrow().anchor2().body.as_ref() {
                Some(b) => {
                    activation.will_activate(b);
                    let js = self.body2joints.find_or_insert_lazy(b.deref() as *const RefCell<RigidBody> as uint,
                                                                  || Some(Vec::new()));
                    js.unwrap().push(BallInSocketConstraint(joint.clone()));
                },
                _ => { }
            }
        }
    }

    /// Removes a `BallInSocket` joint from this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn remove_ball_in_socket(&mut self, joint: &Rc<RefCell<BallInSocket>>, activation: &mut ActivationManager) {
        if self.joints.remove(&(joint.deref() as *const RefCell<BallInSocket> as uint)) {
            let _  = joint.borrow().anchor1().body.as_ref().map(|b| activation.will_activate(b));
            let _  = joint.borrow().anchor2().body.as_ref().map(|b| activation.will_activate(b));
        }
    }

    /// Add a `Fixed` joint to this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn add_fixed(&mut self, joint: Rc<RefCell<Fixed>>, activation: &mut ActivationManager) {
        if self.joints.insert(joint.deref() as *const RefCell<Fixed> as uint, FixedConstraint(joint.clone())) {
            match joint.borrow().anchor1().body.as_ref() {
                Some(b) => {
                    activation.will_activate(b);
                    let js = self.body2joints.find_or_insert_lazy(b.deref() as *const RefCell<RigidBody> as uint,
                                                                  || Some(Vec::new()));
                    js.unwrap().push(FixedConstraint(joint.clone()));
                },
                _ => { }
            }

            match joint.borrow().anchor2().body.as_ref() {
                Some(b) => {
                    activation.will_activate(b);
                    let js = self.body2joints.find_or_insert_lazy(b.deref() as *const RefCell<RigidBody> as uint,
                                                                  || Some(Vec::new()));
                    js.unwrap().push(FixedConstraint(joint.clone()));
                },
                _ => { }
            }
        }
    }

    /// Removes a joint from this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn remove_joint<T: Joint<M>, M>(&mut self,
                                        joint:      &Rc<RefCell<T>>,
                                        activation: &mut ActivationManager) {
        if self.joints.remove(&(joint.deref() as *const RefCell<T> as uint)) {
            self.remove_joint_for_body(joint, joint.borrow().anchor1().body.as_ref(), activation);
            self.remove_joint_for_body(joint, joint.borrow().anchor2().body.as_ref(), activation);
        }
    }

    fn remove_joint_for_body<T: Joint<M>, M>(&mut self,
                                             joint:      &Rc<RefCell<T>>,
                                             body:       Option<&Rc<RefCell<RigidBody>>>,
                                             activation: &mut ActivationManager) {
        match body {
            Some(b) => {
                activation.will_activate(b);
                let key = b.deref() as *const RefCell<RigidBody> as uint;
                match self.body2joints.find_mut(&key) {
                    Some(ref mut js) => {
                        let jkey = joint.deref() as *const RefCell<T>;
                        js.retain(|j| {
                            // we do not know the type of the joint, so cast it to uint for
                            // comparison.
                            let id = match *j {
                                RBRB(_, _, _)                   => ptr::null::<uint>() as uint,
                                BallInSocketConstraint(ref bis) => bis.deref() as *const RefCell<BallInSocket> as uint,
                                FixedConstraint(ref f)          => f.deref() as *const RefCell<Fixed> as uint
                            };

                            id != jkey as uint
                        });
                    }
                    None => { }
                }
            }
            None => { }
        }
    }

    /// Removes every joint attached to a given rigid body.
    ///
    /// This will force the activation of every object attached to the deleted joints.
    pub fn remove(&mut self, b: &Rc<RefCell<RigidBody>>, activation: &mut ActivationManager) {
        for joints in self.body2joints.get_and_remove(&(b.deref() as *const RefCell<RigidBody> as uint)).iter() {
            for joint in joints.value.iter() {
                fn do_remove<T: Joint<M>, M>(_self:      &mut JointManager,
                                             joint:      &Rc<RefCell<T>>,
                                             b:          &Rc<RefCell<RigidBody>>,
                                             activation: &mut ActivationManager) {
                    let bj    = joint.borrow();
                    let body1 = bj.anchor1().body.as_ref();
                    let body2 = bj.anchor2().body.as_ref();

                    for body in bj.anchor1().body.as_ref().iter() {
                        if body.deref() as *const RefCell<RigidBody> == b.deref() as *const RefCell<RigidBody> {
                            _self.remove_joint_for_body(joint, body2, activation);
                        }
                        else {
                            _self.remove_joint_for_body(joint, body1, activation);
                        }
                    }
                }

                match *joint {
                    BallInSocketConstraint(ref bis) => do_remove(self, bis, b, activation),
                    FixedConstraint(ref f)          => do_remove(self, f, b, activation),
                    RBRB(_, _, _) => fail!("Internal error: a contact RBRB should not be here.")
                }
            }
        }
    }
}

impl<BF> Detector<RigidBody, Constraint, BF> for JointManager {
    // FIXME: do we really want to handle this here instead of in the activation manager directly?
    fn update(&mut self, _: &mut BF, activation: &mut ActivationManager) {
        for joint in self.joints.elements().iter() {
            match joint.value {
                BallInSocketConstraint(ref bis) => {
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
                FixedConstraint(ref f) => { // FIXME: code duplication from BallInSocket
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
