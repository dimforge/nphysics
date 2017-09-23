use alga::general::Real;
use ncollide::utils::data::hash_map::HashMap;
use ncollide::utils::data::hash::UintTWHash;
use detection::activation_manager::ActivationManager;
use detection::joint::ball_in_socket::BallInSocket;
use detection::joint::fixed::Fixed;
use detection::joint::joint::Joint;
use detection::constraint::Constraint;
use object::RigidBody;
use world::RigidBodyStorage;
use utils::IndexVec;

/// Structure that handles creation and removal of joints.
pub struct JointManager<N: Real> {
    joints:      IndexVec<Constraint<N>>,
    body2joints: HashMap<usize, Vec<usize>, UintTWHash>
}

impl<N: Real> JointManager<N> {
    /// Creates a new `JointManager`.
    pub fn new() -> JointManager<N> {
        JointManager {
            joints:      IndexVec::new(0),
            body2joints: HashMap::new(UintTWHash::new())
        }
    }

    /// Joints handled by this manager.
    #[inline]
    pub fn joints(&self) -> &IndexVec<Constraint<N>> {
        &self.joints
    }

    // TODO
    // /// List of joints attached to a specific body.
    // #[inline]
    // pub fn joints_with_body(&self, body: &RigidBody<N>) -> Option<&[Constraint<N>]> {
    //     self.body2joints.find(&body.uid()).map(|v| v.iter().map(|&index| self.joints[index]))
    // }

    // TODO: return index
    /// Add a `BallInSocket` joint to this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn add_ball_in_socket(&mut self,
                              joint: BallInSocket<N>,
                              activation: &mut ActivationManager<N>,
                              bodies: &RigidBodyStorage<N>) -> usize {
        let anchor1 = joint.anchor1().body;
        let anchor2 = joint.anchor2().body;

        let index = self.joints.insert(Constraint::BallInSocket(joint));
        if let Some(b) = anchor1 {
            activation.deferred_activate(&bodies[b]);
            let js = self.body2joints.find_or_insert_lazy(b, || Some(Vec::new()));
            js.unwrap().push(index);
        }
        if let Some(b) = anchor2 {
            activation.deferred_activate(&bodies[b]);
            let js = self.body2joints.find_or_insert_lazy(b, || Some(Vec::new()));
            js.unwrap().push(index);
        }
        index
    }

    /// Removes a `BallInSocket` joint from this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn remove_ball_in_socket(&mut self, joint: usize, activation: &mut ActivationManager<N>, bodies: &RigidBodyStorage<N>) {
        let constraint = self.joints.remove(joint);
        match constraint {
            Constraint::BallInSocket(bis) => {
                if let Some(b) = bis.anchor1().body {
                    self.remove_joint_for_body(joint, &bodies[b], activation);
                }
                if let Some(b) = bis.anchor2().body {
                    self.remove_joint_for_body(joint, &bodies[b], activation);
                }
            },
            _ => panic!("remove expect ball in socket"),
        }
    }

    /// Add a `Fixed` joint to this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn add_fixed(&mut self, joint: Fixed<N>, activation: &mut ActivationManager<N>, bodies: &RigidBodyStorage<N>) -> usize {
        let index = self.joints.insert(Constraint::Fixed(joint.clone()));
        if let Some(b) = joint.anchor1().body {
            activation.deferred_activate(&bodies[b]);
            let js = self.body2joints.find_or_insert_lazy(b, || Some(Vec::new()));
            js.unwrap().push(index);
        }

        if let Some(b) = joint.anchor2().body {
            activation.deferred_activate(&bodies[b]);
            let js = self.body2joints.find_or_insert_lazy(b, || Some(Vec::new()));
            js.unwrap().push(index);
        }
        index
    }

    /// Removes a joint from this manager.
    ///
    /// This will force the activation of the two objects attached to the joint.
    pub fn remove_joint<T: Joint<M>, M>(&mut self,
                                        joint: usize,
                                        activation: &mut ActivationManager<N>,
                                        bodies: &RigidBodyStorage<N>) {
        let constraint = self.joints.remove(joint);
        let mut if_body_then_remove = |joint: usize, body: Option<usize>| {
            if let Some(body) = body {
                self.remove_joint_for_body(joint, &bodies[body], activation)
            }
        };

        match constraint {
            Constraint::BallInSocket(bis) => {
                if_body_then_remove(joint, bis.anchor1().body);
                if_body_then_remove(joint, bis.anchor1().body);
            },
            Constraint::Fixed(f) => {
                if_body_then_remove(joint, f.anchor1().body);
                if_body_then_remove(joint, f.anchor1().body);
            },
            _ => panic!("remove joint uid is not a joint"),
        }
    }

    fn remove_joint_for_body(&mut self,
                             joint:      usize,
                             body:       &RigidBody<N>,
                             activation: &mut ActivationManager<N>) {
        activation.deferred_activate(body);
        // TODO: expect Internal error: instead of unwrap
        self.body2joints.find_mut(&body.uid()).unwrap().retain(|&j| j != joint);
    }

    /// Removes every joint attached to a given rigid body.
    ///
    /// This will force the activation of every object attached to the deleted joints.
    pub fn remove(&mut self, body: usize, activation: &mut ActivationManager<N>, bodies: &RigidBodyStorage<N>) {
        // TODO: expect it ???
        if let Some(joints) = self.body2joints.get_and_remove(&body) {
            for &joint_index in joints.value.iter() {
                let joint = self.joints.remove(joint_index);
                let (anchor1_body, anchor2_body) = match joint {
                    Constraint::BallInSocket(ref bis) => (bis.anchor1().body, bis.anchor2().body),
                    Constraint::Fixed(ref f) => (f.anchor1().body, f.anchor2().body),
                    Constraint::RBRB(_, _, _) => panic!("Internal error: a contact RBRB should not be here.")
                };

                for anchor_body in [anchor1_body, anchor2_body].iter().filter_map(|e| *e) {
                    if anchor_body != body {
                        self.remove_joint_for_body(joint_index, &bodies[anchor_body], activation);
                    }
                }
            }
        }
    }

    // FIXME: do we really want to handle this here instead of in the activation manager directly?
    /// Activates the objects that interact with an activated object through a joint.
    pub fn update(&mut self, activation: &mut ActivationManager<N>, bodies: &RigidBodyStorage<N>) {
        for joint in self.joints.iter_mut() {
            match *joint {
                Constraint::BallInSocket(ref mut bis) => {
                    if !bis.up_to_date() {
                        // the joint has been invalidated by the user: wake up the attached bodies
                        bis.update();
                        if let Some(b) = bis.anchor1().body {
                            activation.deferred_activate(&bodies[b]);
                        }
                        if let Some(b) = bis.anchor2().body {
                            activation.deferred_activate(&bodies[b]);
                        }
                    }
                },
                Constraint::Fixed(ref mut f) => { // FIXME: code duplication from BallInSocket
                    if !f.up_to_date() {
                        // the joint has been invalidated by the user: wake up the attached bodies
                        f.update();
                        if let Some(b) = f.anchor1().body {
                            activation.deferred_activate(&bodies[b]);
                        }
                        if let Some(b) = f.anchor2().body {
                            activation.deferred_activate(&bodies[b]);
                        }
                    }
                },
                Constraint::RBRB(_, _, _) => panic!("Internal error: a contact RBRB should not be here.")
            }
        }
    }

    /// Collects all the constraints caused by joints.
    pub fn constraints(&mut self, constraint: &mut Vec<Constraint<N>>) {
        for joint in self.joints.iter() {
            constraint.push(joint.clone())
        }
    }
}
