use std::rc::Rc;
use std::cell::RefCell;
use nalgebra::na;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use ncollide::math::Scalar;
use ncollide::broad::{InterferencesBroadPhase};
use ncollide::narrow::{CollisionDetector, GeomGeomCollisionDetector};
use detection::constraint::{Constraint, RBRB, BallInSocket, Fixed};
use detection::joint::{JointManager, Joint};
use object::{RigidBody, Deleted};
use utils::union_find::UnionFindSet;
use utils::union_find;

/// Structure that monitors island-based activation/deactivation of objects.
///
/// It is responsible for making objects sleep or wake up.
pub struct ActivationManager {
    threshold:      Scalar,
    mix_factor:     Scalar,
    ufind:          Vec<UnionFindSet>,
    can_deactivate: Vec<bool>,
    collector:      Vec<Constraint>,
    to_activate:    Vec<Rc<RefCell<RigidBody>>>,
    to_deactivate:  Vec<uint>
}

impl ActivationManager {
    /// Creates a new `ActivationManager`.
    ///
    /// # Arguments:
    /// * `thresold`   - the minimum energy required to keep an object awake.
    /// * `mix_factor` - the ratio of energy to keep between two frames.
    pub fn new(threshold:  Scalar, mix_factor: Scalar) -> ActivationManager {
        assert!(mix_factor >= na::zero() && threshold <= na::one(),
                "The energy mixing factor must be between 0.0 and 1.0.");

        ActivationManager {
            threshold:      threshold,
            mix_factor:     mix_factor,
            ufind:          Vec::new(),
            can_deactivate: Vec::new(),
            collector:      Vec::new(),
            to_activate:    Vec::new(),
            to_deactivate:  Vec::new()
        }
    }

    /// Notify the `ActivationManager` that is has to activate an object at the next update.
    // FIXME: this is not a very good name
    pub fn will_activate(&mut self, b: &Rc<RefCell<RigidBody>>) {
        if b.borrow().can_move() && !b.borrow().is_active() {
            self.to_activate.push(b.clone());
        }
    }

    fn update_energy(&self, b: &mut RigidBody) {
        // FIXME: take the time in account (to make a true RWA)
        let _1         = na::one::<Scalar>();
        let new_energy = (_1 - self.mix_factor) * b.activation_state().energy() +
            self.mix_factor * (na::sqnorm(&b.lin_vel()) + na::sqnorm(&b.ang_vel()));

        b.activate(new_energy.min(self.threshold * na::cast(4.0)));
    }

    /// Update the activation manager, activating and deactivating objects when needed.
    pub fn update<BF: InterferencesBroadPhase<Rc<RefCell<RigidBody>>, ~GeomGeomCollisionDetector>>(
                  &mut self,
                  broad_phase: &mut BF,
                  joints:      &JointManager,
                  bodies:      &mut HashMap<uint, Rc<RefCell<RigidBody>>, UintTWHash>) {
        /*
         *
         * Update bodies energy
         *
         */
        for (i, b) in bodies.elements().iter().enumerate() {
            let mut b = b.value.borrow_mut();

            self.update_energy(&mut *b);
            b.set_index(i as int);
        }


        /*
         *
         * Build islands to deactivate those with low-energy objects only.
         *
         */
        // resize buffers
        if bodies.len() > self.ufind.len() {
            let to_add = bodies.len() - self.ufind.len();
            self.ufind.grow(to_add, &UnionFindSet::new(0));
            self.can_deactivate.grow(to_add, &false);
        }
        else {
            self.ufind.truncate(bodies.len());
            self.can_deactivate.truncate(bodies.len());
        }

        // init the union find
        for (i, u) in self.ufind.mut_iter().enumerate() {
            u.reinit(i)
        }

        for d in self.can_deactivate.mut_iter() {
            *d = true
        }

        // run the union-find
        fn make_union(b1: &Rc<RefCell<RigidBody>>, b2: &Rc<RefCell<RigidBody>>, ufs: &mut [UnionFindSet]) {
            let rb1 = b1.borrow();
            let rb2 = b2.borrow();

            if rb1.is_active() && rb2.is_active() {
                union_find::union(rb1.index() as uint, rb2.index() as uint, ufs)
            }
        }

        broad_phase.for_each_pair(|b1, b2, cd| {
            if cd.num_colls() != 0 {
                make_union(b1, b2, self.ufind.as_mut_slice())
            }
        });

        for e in joints.joints().elements().iter() {
            match e.value {
                RBRB(ref b1, ref b2, _) => make_union(b1, b2, self.ufind.as_mut_slice()),
                BallInSocket(ref bis)   => {
                    match (bis.borrow().anchor1().body.as_ref(), bis.borrow().anchor2().body.as_ref()) {
                        (Some(b1), Some(b2)) => make_union(b1, b2, self.ufind.as_mut_slice()),
                        _ => { }
                    }
                },
                Fixed(ref f)   => {
                    match (f.borrow().anchor1().body.as_ref(), f.borrow().anchor2().body.as_ref()) {
                        (Some(b1), Some(b2)) => make_union(b1, b2, self.ufind.as_mut_slice()),
                        _ => { }
                    }
                }
            }
        }

        /*
         *
         * Find deactivable islands and deactivate them
         *
         */
        // find out whether islands can be deactivated
        for i in range(0u, self.ufind.len()) {
            let root = union_find::find(i, self.ufind.as_mut_slice());
            *self.can_deactivate.get_mut(root) =
                *self.can_deactivate.get(root) &&
                bodies.elements()[i].value.borrow().activation_state().energy() < self.threshold
        }

        // deactivate islands having only deactivable objects
        for i in range(0u, self.ufind.len()) {
            let root = union_find::find(i, self.ufind.as_mut_slice());

            if *self.can_deactivate.get(root) { // everybody in this set can be deactivacted
                let b = &bodies.elements()[i].value;
                b.borrow_mut().deactivate();
                broad_phase.deactivate(b);
                self.to_deactivate.push(b.deref() as *RefCell<RigidBody> as uint);
            }
        }

        for b in self.to_deactivate.iter() {
            bodies.remove(b);
        }

        self.to_deactivate.clear();

        /*
         *
         * Activate any deactivated island interfering with an active object.
         *
         */
        while !self.to_activate.is_empty() { // the len will change
            let to_activate = self.to_activate.pop().unwrap();

            {
                let mut b = to_activate.borrow_mut();

                if *b.activation_state() == Deleted {
                    continue
                }
                else {
                    if !b.is_active() {
                        bodies.insert(to_activate.deref() as *RefCell<RigidBody> as uint, to_activate.clone());
                    }

                    b.activate(self.threshold * na::cast(2.0))
                }
            }

            fn add_to_activation_list(body: &Rc<RefCell<RigidBody>>, list: &mut Vec<Rc<RefCell<RigidBody>>>) {
                let mut rb = body.borrow_mut();

                if !rb.is_active() && rb.can_move() {
                    list.push(body.clone());
                }
            }

            broad_phase.activate(&to_activate, |b1, b2, cd| {
                if cd.num_colls() > 0 {
                    add_to_activation_list(b1, &mut self.to_activate);
                    add_to_activation_list(b2, &mut self.to_activate);
                }
            });

            // propagate through joints too
            for joints in joints.joints_with_body(&to_activate).iter() {
                for joint in joints.iter() {
                    match *joint {
                        RBRB(ref b1, ref b2, _) => {
                            add_to_activation_list(b1, &mut self.to_activate);
                            add_to_activation_list(b2, &mut self.to_activate);
                        },
                        BallInSocket(ref bis) => {
                            let _ = bis.borrow().anchor1().body.as_ref().map(|b| add_to_activation_list(b, &mut self.to_activate));
                            let _ = bis.borrow().anchor2().body.as_ref().map(|b| add_to_activation_list(b, &mut self.to_activate));
                        }
                        Fixed(ref f) => {
                            let _ = f.borrow().anchor1().body.as_ref().map(|b| add_to_activation_list(b, &mut self.to_activate));
                            let _ = f.borrow().anchor2().body.as_ref().map(|b| add_to_activation_list(b, &mut self.to_activate));
                        }
                    }
                }
            }
        }
    }
}
