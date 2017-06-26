use std::ptr;

use alga::general::Real;
use ncollide::utils::data::hash_map::HashMap;
use ncollide::utils::data::hash::UintTWHash;
use detection::activation_manager::ActivationManager;
use detection::joint::ball_in_socket::BallInSocket;
use detection::joint::fixed::Fixed;
use detection::joint::joint::Joint;
use detection::constraint::Constraint;
use object::RigidBody;

/// Structure that handles creation and removal of joints.
pub struct JointManager<N: Real> {
    joints:      HashMap<usize, Constraint<N>, UintTWHash>,
    body2joints: HashMap<usize, Vec<Constraint<N>>, UintTWHash>
}

impl<N: Real> JointManager<N> {
    /// Creates a new `JointManager`.
    pub fn new() -> JointManager<N> {
        JointManager {
            joints:      HashMap::new(UintTWHash::new()),
            body2joints: HashMap::new(UintTWHash::new())
        }
    }

    /// Joints handled by this manager.
    #[inline]
    pub fn joints(&self) -> &HashMap<usize, Constraint<N>, UintTWHash> {
        &self.joints
    }

    /// List of joints attached to a specific body.
    #[inline]
    pub fn joints_with_body(&self, body: &::Rc<RigidBody<N>>) -> Option<&[Constraint<N>]> {
        self.body2joints.find(&(body.ptr() as usize)).map(|v| &v[..])
    }

    /// Add a `BallInSocket` joint to this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn add_ball_in_socket(&mut self,
                              joint:      ::Rc<BallInSocket<N>>,
                              activation: &mut ActivationManager<N>) {
        if self.joints.insert(joint.ptr() as usize,
                              Constraint::BallInSocket(joint.clone())) {
            match joint.borrow().anchor1().body.as_ref() {
                Some(b) => {
                    activation.deferred_activate(b);
                    let js = self.body2joints.find_or_insert_lazy(b.ptr() as usize,
                                                                  || Some(Vec::new()));
                    js.unwrap().push(Constraint::BallInSocket(joint.clone()));
                },
                _ => { }
            }

            match joint.borrow().anchor2().body.as_ref() {
                Some(b) => {
                    activation.deferred_activate(b);
                    let js = self.body2joints.find_or_insert_lazy(b.ptr() as usize,
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
    pub fn remove_ball_in_socket(&mut self, joint: &::Rc<BallInSocket<N>>, activation: &mut ActivationManager<N>) {
        if self.joints.remove(&(joint.ptr() as usize)) {
            let _  = joint.borrow().anchor1().body.as_ref().map(|b| activation.deferred_activate(b));
            let _  = joint.borrow().anchor2().body.as_ref().map(|b| activation.deferred_activate(b));
        }
    }

    /// Add a `Fixed` joint to this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn add_fixed(&mut self, joint: ::Rc<Fixed<N>>, activation: &mut ActivationManager<N>) {
        if self.joints.insert(joint.ptr() as usize, Constraint::Fixed(joint.clone())) {
            match joint.borrow().anchor1().body.as_ref() {
                Some(b) => {
                    activation.deferred_activate(b);
                    let js = self.body2joints.find_or_insert_lazy(b.ptr() as usize,
                                                                  || Some(Vec::new()));
                    js.unwrap().push(Constraint::Fixed(joint.clone()));
                },
                _ => { }
            }

            match joint.borrow().anchor2().body.as_ref() {
                Some(b) => {
                    activation.deferred_activate(b);
                    let js = self.body2joints.find_or_insert_lazy(b.ptr() as usize,
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
    pub fn remove_joint<T: Joint<N, M>, M>(&mut self,
                                           joint:      &::Rc<T>,
                                           activation: &mut ActivationManager<N>) {
        if self.joints.remove(&(joint.ptr() as usize)) {
            self.remove_joint_for_body(joint, joint.borrow().anchor1().body.as_ref(), activation);
            self.remove_joint_for_body(joint, joint.borrow().anchor2().body.as_ref(), activation);
        }
    }

    fn remove_joint_for_body<T: Joint<N, M>, M>(&mut self,
                                                joint:      &::Rc<T>,
                                                body:       Option<&::Rc<RigidBody<N>>>,
                                                activation: &mut ActivationManager<N>) {
        match body {
            Some(b) => {
                activation.deferred_activate(b);
                let key = b.ptr() as usize;
                match self.body2joints.find_mut(&key) {
                    Some(ref mut js) => {
                        let jkey = joint.ptr();
                        js.retain(|j| {
                            // we do not know the type of the joint, so cast it to usize for
                            // comparison.
                            let id = match *j {
                                Constraint::RBRB(_, _, _) => ptr::null::<usize>() as usize,
                                Constraint::BallInSocket(ref b) => b.ptr() as usize,
                                Constraint::Fixed(ref f) => f.ptr() as usize
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
    pub fn remove(&mut self, b: &::Rc<RigidBody<N>>, activation: &mut ActivationManager<N>) {
        for joints in self.body2joints.get_and_remove(&(b.ptr() as usize)).iter() {
            for joint in joints.value.iter() {
                fn do_remove<N: Real, T: Joint<N, M>, M>(_self:      &mut JointManager<N>,
                                                         joint:      &::Rc<T>,
                                                         b:          &::Rc<RigidBody<N>>,
                                                         activation: &mut ActivationManager<N>) {
                    let bj    = joint.borrow();
                    let body1 = bj.anchor1().body.as_ref();
                    let body2 = bj.anchor2().body.as_ref();

                    for body in bj.anchor1().body.as_ref().iter() {
                        if body.ptr() == b.ptr() {
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
    pub fn update(&mut self, activation: &mut ActivationManager<N>) {
        for joint in self.joints.elements().iter() {
            match joint.value {
                Constraint::BallInSocket(ref bis) => {
                    let mut bbis = bis.borrow_mut();
                    if !bbis.up_to_date() {
                        // the joint has been invalidated by the user: wake up the attached bodies
                        bbis.update();
                        match bbis.anchor1().body {
                            Some(ref b) => activation.deferred_activate(b),
                            None        => { }
                        }
                        match bbis.anchor2().body {
                            Some(ref b) => activation.deferred_activate(b),
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
                            Some(ref b) => activation.deferred_activate(b),
                            None        => { }
                        }
                        match bf.anchor2().body {
                            Some(ref b) => activation.deferred_activate(b),
                            None        => { }
                        }
                    }
                },
                Constraint::RBRB(_, _, _) => panic!("Internal error: a contact RBRB should not be here.")
 
            }
        }
    }

    /// Collects all the constraints caused by joints.
    pub fn constraints(&mut self, constraint: &mut Vec<Constraint<N>>) {
        for joint in self.joints.elements().iter() {
            constraint.push(joint.value.clone())
        }
    }
}
