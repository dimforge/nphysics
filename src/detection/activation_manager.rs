use std::rc::Rc;
use std::cell::RefCell;
use std::borrow;
use nalgebra::na;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use ncollide::math::N;
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
    priv threshold:      N,
    priv mix_factor:     N,
    priv ufind:          ~[UnionFindSet],
    priv can_deactivate: ~[bool],
    priv collector:      ~[Constraint],
    priv to_activate:    ~[Rc<RefCell<RigidBody>>],
    priv to_deactivate:  ~[uint]
}

impl ActivationManager {
    /// Creates a new `ActivationManager`.
    ///
    /// # Arguments:
    /// * `thresold`   - the minimum energy required to keep an object awake.
    /// * `mix_factor` - the ratio of energy to keep between two frames.
    pub fn new(threshold:  N, mix_factor: N) -> ActivationManager {
        assert!(mix_factor >= na::zero() && threshold <= na::one(),
                "The energy mixing factor must be between 0.0 and 1.0.");

        ActivationManager {
            threshold:      threshold,
            mix_factor:     mix_factor,
            ufind:          ~[],
            can_deactivate: ~[],
            collector:      ~[],
            to_activate:    ~[],
            to_deactivate:  ~[]
        }
    }

    /// Notify the `ActivationManager` that is has to activate an object at the next update.
    // FIXME: this is not a very good name
    pub fn will_activate(&mut self, b: &Rc<RefCell<RigidBody>>) {
        if b.borrow().with(|b| b.can_move() && !b.is_active()) {
            self.to_activate.push(b.clone());
        }
    }

    fn update_energy(&self, b: &mut RigidBody) {
        // FIXME: take the time in account (to make a true RWA)
        let _1         = na::one::<N>();
        let new_energy = (_1 - self.mix_factor) * b.activation_state().energy() +
            self.mix_factor * (na::sqnorm(&b.lin_vel()) + na::sqnorm(&b.ang_vel()));

        b.activate(new_energy.min(&(self.threshold * na::cast(4.0))));
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
            b.value.borrow().with_mut(|b| {
                self.update_energy(b);
                b.set_index(i as int);
            });
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
            let bb1 = b1.borrow().borrow();
            let bb2 = b2.borrow().borrow();

            let rb1 = bb1.get();
            let rb2 = bb2.get();

            if rb1.is_active() && rb2.is_active() && cd.num_colls() != 0 {
                union_find::union(rb1.index() as uint, rb2.index() as uint, self.ufind)
            }
        });

        /*
         *
         * Find deactivable islands and deactivate them
         *
         */
        // find out whether islands can be deactivated
        for i in range(0u, self.ufind.len()) {
            let root = union_find::find(i, self.ufind);
            self.can_deactivate[root] = self.can_deactivate[root] &&
                                        bodies.elements()[i].value.borrow().with(|b| b.activation_state().energy() < self.threshold)
        }

        // deactivate islands having only deactivable objects
        for i in range(0u, self.ufind.len()) {
            let root = union_find::find(i, self.ufind);

            if self.can_deactivate[root] { // everybody in this set can be deactivacted
                let b = &bodies.elements()[i].value;
                b.borrow().with_mut(|b| b.deactivate());
                broad_phase.deactivate(b);
                self.to_deactivate.push(borrow::to_uint(b.borrow()));
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
            let to_activate = self.to_activate.pop();

            {
                let mut bto_activate = to_activate.borrow().borrow_mut();
                let     b            = bto_activate.get();

                if *b.activation_state() == Deleted {
                    continue
                }
                else {
                    if !b.is_active() {
                        bodies.insert(borrow::to_uint(to_activate.borrow()), to_activate.clone());
                    }

                    b.activate(self.threshold * na::cast(2.0))
                }
            }

            broad_phase.activate(&to_activate, |b1, b2, cd| {
                if cd.num_colls() > 0 {
                    let mut bb1 = b1.borrow().borrow_mut();
                    let mut bb2 = b2.borrow().borrow_mut();

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
