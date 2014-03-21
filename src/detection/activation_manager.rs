use std::rc::Rc;
use std::cell::RefCell;
use std::vec::Vec;
use nalgebra::na;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use ncollide::math::Scalar;
use ncollide::broad::{InterferencesBroadPhase};
use ncollide::narrow::{CollisionDetector, GeomGeomCollisionDetector};
use detection::constraint::Constraint;
use object::{RigidBody, Deleted};
use utils::union_find::UnionFindSet;
use utils::union_find;

/// Structure that monitors island-based activation/deactivation of objects.
///
/// It is responsible for making objects sleep or wake up.
pub struct ActivationManager {
    priv threshold:      Scalar,
    priv mix_factor:     Scalar,
    priv ufind:          Vec<UnionFindSet>,
    priv can_deactivate: Vec<bool>,
    priv collector:      Vec<Constraint>,
    priv to_activate:    Vec<Rc<RefCell<RigidBody>>>,
    priv to_deactivate:  Vec<uint>
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
        broad_phase.for_each_pair(|b1, b2, cd| {
            let rb1 = b1.borrow();
            let rb2 = b2.borrow();

            if rb1.is_active() && rb2.is_active() && cd.num_colls() != 0 {
                union_find::union(rb1.index() as uint, rb2.index() as uint, self.ufind.as_mut_slice())
            }
        });

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

            broad_phase.activate(&to_activate, |b1, b2, cd| {
                if cd.num_colls() > 0 {
                    let mut bb1 = b1.borrow_mut();
                    let mut bb2 = b2.borrow_mut();

                    let rb1 = bb1.get();
                    let rb2 = bb2.get();

                    if !rb1.is_active() && rb1.can_move() {
                        self.to_activate.push(b1.clone());
                    }

                    if !rb2.is_active() && rb2.can_move() {
                        self.to_activate.push(b2.clone());
                    }
                }
            });
        }
    }
}
