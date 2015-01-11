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
use detection::constraint::Constraint;
use object::RigidBody;

/// Structure that handles creation and removal of joints.
pub struct JointManager {
    joints:      HashMap<usize, Constraint, UintTWHash>,
    body2joints: HashMap<usize, Vec<Constraint>, UintTWHash>
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
    pub fn joints(&self) -> &HashMap<usize, Constraint, UintTWHash> {
        &self.joints
    }

    /// List of joints attached to a specific body.
    #[inline]
    pub fn joints_with_body(&self, body: &Rc<RefCell<RigidBody>>) -> Option<&[Constraint]> {
        self.body2joints.find(&(&**body as *const RefCell<RigidBody> as usize)).map(|v| v.as_slice())
    }

    /// Add a `BallInSocket` joint to this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn add_ball_in_socket(&mut self,
                              joint:      Rc<RefCell<BallInSocket>>,
                              activation: &mut ActivationManager) {
        if self.joints.insert(&*joint as *const RefCell<BallInSocket> as usize,
                              Constraint::BallInSocket(joint.clone())) {
            match joint.borrow().anchor1().body.as_ref() {
                Some(b) => {
                    activation.will_activate(b);
                    let js = self.body2joints.find_or_insert_lazy(&**b as *const RefCell<RigidBody> as usize,
                                                                  || Some(Vec::new()));
                    js.unwrap().push(Constraint::BallInSocket(joint.clone()));
                },
                _ => { }
            }

            match joint.borrow().anchor2().body.as_ref() {
                Some(b) => {
                    activation.will_activate(b);
                    let js = self.body2joints.find_or_insert_lazy(&**b as *const RefCell<RigidBody> as usize,
                                                                  || Some(Vec::new()));
                    js.unwrap().push(Constraint::BallInSocket(joint.clone()));
                },
                _ => { }
            }
        }
    }

    /// Removes a `BallInSocket` joint from this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn remove_ball_in_socket(&mut self, joint: &Rc<RefCell<BallInSocket>>, activation: &mut ActivationManager) {
        if self.joints.remove(&(&**joint as *const RefCell<BallInSocket> as usize)) {
            let _  = joint.borrow().anchor1().body.as_ref().map(|b| activation.will_activate(b));
            let _  = joint.borrow().anchor2().body.as_ref().map(|b| activation.will_activate(b));
        }
    }

    /// Add a `Fixed` joint to this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn add_fixed(&mut self, joint: Rc<RefCell<Fixed>>, activation: &mut ActivationManager) {
        if self.joints.insert(&*joint as *const RefCell<Fixed> as usize, Constraint::Fixed(joint.clone())) {
            match joint.borrow().anchor1().body.as_ref() {
                Some(b) => {
                    activation.will_activate(b);
                    let js = self.body2joints.find_or_insert_lazy(&**b as *const RefCell<RigidBody> as usize,
                                                                  || Some(Vec::new()));
                    js.unwrap().push(Constraint::Fixed(joint.clone()));
                },
                _ => { }
            }

            match joint.borrow().anchor2().body.as_ref() {
                Some(b) => {
                    activation.will_activate(b);
                    let js = self.body2joints.find_or_insert_lazy(&**b as *const RefCell<RigidBody> as usize,
                                                                  || Some(Vec::new()));
                    js.unwrap().push(Constraint::Fixed(joint.clone()));
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
        if self.joints.remove(&(&**joint as *const RefCell<T> as usize)) {
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
                let key = &**b as *const RefCell<RigidBody> as usize;
                match self.body2joints.find_mut(&key) {
                    Some(ref mut js) => {
                        let jkey = &**joint as *const RefCell<T>;
                        js.retain(|j| {
                            // we do not know the type of the joint, so cast it to usize for
                            // comparison.
                            let id = match *j {
                                Constraint::RBRB(_, _, _) => ptr::null::<usize>() as usize,
                                Constraint::BallInSocket(ref b) => &**b as *const RefCell<BallInSocket> as usize,
                                Constraint::Fixed(ref f) => &**f as *const RefCell<Fixed> as usize
                            };

                            id != jkey as usize
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
        for joints in self.body2joints.get_and_remove(&(&**b as *const RefCell<RigidBody> as usize)).iter() {
            for joint in joints.value.iter() {
                fn do_remove<T: Joint<M>, M>(_self:      &mut JointManager,
                                             joint:      &Rc<RefCell<T>>,
                                             b:          &Rc<RefCell<RigidBody>>,
                                             activation: &mut ActivationManager) {
                    let bj    = joint.borrow();
                    let body1 = bj.anchor1().body.as_ref();
                    let body2 = bj.anchor2().body.as_ref();

                    for body in bj.anchor1().body.as_ref().iter() {
                        if &**(*body) as *const RefCell<RigidBody> == &**b as *const RefCell<RigidBody> {
                            _self.remove_joint_for_body(joint, body2, activation);
                        }
                        else {
                            _self.remove_joint_for_body(joint, body1, activation);
                        }
                    }
                }

                match *joint {
                    Constraint::BallInSocket(ref bis) => do_remove(self, bis, b, activation),
                    Constraint::Fixed(ref f)          => do_remove(self, f, b, activation),
                    Constraint::RBRB(_, _, _) => panic!("Internal error: a contact RBRB should not be here.")
                }
            }
        }
    }

    // FIXME: do we really want to handle this here instead of in the activation manager directly?
    /// Activates the objects that interact with an activated object through a joint.
    pub fn update(&mut self, activation: &mut ActivationManager) {
        for joint in self.joints.elements().iter() {
            match joint.value {
                Constraint::BallInSocket(ref bis) => {
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
                Constraint::Fixed(ref f) => { // FIXME: code duplication from BallInSocket
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
                Constraint::RBRB(_, _, _) => panic!("Internal error: a contact RBRB should not be here.")
 
            }
        }
    }

    /// Collects all the constraints caused by joints.
    pub fn interferences(&mut self, constraint: &mut Vec<Constraint>) {
        for joint in self.joints.elements().iter() {
            constraint.push(joint.value.clone())
        }
    }
}
