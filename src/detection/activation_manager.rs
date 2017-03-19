use std::iter;
use na;
use alga::general::Real;
use ncollide::utils::data::hash_map::HashMap;
use ncollide::utils::data::hash::UintTWHash;
use world::RigidBodyCollisionWorld;
use detection::constraint::Constraint;
use detection::joint::{JointManager, Joint};
use object::{WorldObject, RigidBody, RigidBodyHandle, ActivationState};
use utils::union_find::UnionFindSet;
use utils::union_find;

/// Structure that monitors island-based activation/deactivation of objects.
///
/// It is responsible for making objects sleep or wake up.
pub struct ActivationManager<N: Real> {
    mix_factor:     N,
    ufind:          Vec<UnionFindSet>,
    can_deactivate: Vec<bool>,
    to_activate:    Vec<RigidBodyHandle<N>>,
}

impl<N: Real> ActivationManager<N> {
    /// Creates a new `ActivationManager`.
    ///
    /// # Arguments:
    /// * `thresold`   - the minimum energy required to keep an object awake.
    /// * `mix_factor` - the ratio of energy to keep between two frames.
    pub fn new(mix_factor: N) -> ActivationManager<N> {
        assert!(mix_factor >= na::zero(), "The energy mixing factor must be between 0.0 and 1.0.");

        ActivationManager {
            mix_factor:     mix_factor,
            ufind:          Vec::new(),
            can_deactivate: Vec::new(),
            to_activate:    Vec::new(),
        }
    }

    /// Notify the `ActivationManager` that is has to activate an object at the next update.
    // FIXME: this is not a very good name
    pub fn deferred_activate(&mut self, b: &RigidBodyHandle<N>) {
        if b.borrow().can_move() && !b.borrow().is_active() {
            self.to_activate.push(b.clone());
        }
    }

    fn update_energy(&self, b: &mut RigidBody<N>) {
        match b.deactivation_threshold() {
            Some(threshold) => {
                // FIXME: take the time in account (to make a true RWA)
                let _1         = na::one::<N>();
                let new_energy = (_1 - self.mix_factor) * b.activation_state().energy() +
                    self.mix_factor * (na::norm_squared(&b.lin_vel()) + na::norm_squared(&b.ang_vel()));

                b.activate(new_energy.min(threshold * na::convert(4.0f64)));
            },
            None => { }
        }
    }

    /// Update the activation manager, activating and deactivating objects when needed.
    pub fn update(&mut self,
                  world:  &mut RigidBodyCollisionWorld<N>,
                  joints: &JointManager<N>,
                  bodies: &HashMap<usize, RigidBodyHandle<N>, UintTWHash>) {
        /*
         *
         * Update bodies energy
         *
         */
        for (i, b) in bodies.elements().iter().enumerate() {
            let mut b = b.value.borrow_mut();

            assert!(*b.activation_state() != ActivationState::Deleted);
            if b.is_active() {
                self.update_energy(&mut *b);
            }

            b.set_index(i as isize);
        }

        /*
         *
         * Activate bodies that need it.
         *
         */
        for b in self.to_activate.iter() {
            let mut rb = b.borrow_mut();

            match rb.deactivation_threshold() {
                Some(threshold) => rb.activate(threshold * na::convert(2.0f64)),
                None => { }
            }
        }

        self.to_activate.clear();

        /*
         *
         * Build islands to deactivate those with low-energy objects only.
         *
         */
        // Resize buffers.
        if bodies.len() > self.ufind.len() {
            let to_add = bodies.len() - self.ufind.len();
            self.ufind.extend(iter::repeat(UnionFindSet::new(0)).take(to_add));
            self.can_deactivate.extend(iter::repeat(false).take(to_add));
        }
        else {
            self.ufind.truncate(bodies.len());
            self.can_deactivate.truncate(bodies.len());
        }

        // Init the union find.
        for (i, u) in self.ufind.iter_mut().enumerate() {
            u.reinit(i)
        }

        for d in self.can_deactivate.iter_mut() {
            *d = true
        }

        // Run the union-find.
        fn make_union<N: Real>(b1: &RigidBodyHandle<N>, b2: &RigidBodyHandle<N>, ufs: &mut [UnionFindSet]) {
            let rb1 = b1.borrow();
            let rb2 = b2.borrow();

            if rb1.can_move() && rb2.can_move() {
                union_find::union(rb1.index() as usize, rb2.index() as usize, ufs)
            }
        }

        for (b1, b2, cd) in world.contact_pairs() {
            if let (&WorldObject::RigidBody(ref rb1), &WorldObject::RigidBody(ref rb2)) = (&b1.data, &b2.data) {
                if cd.num_contacts() != 0 {
                    make_union(&rb1, &rb2, &mut self.ufind[..])
                }
            }
        }

        for e in joints.joints().elements().iter() {
            match e.value {
                Constraint::RBRB(ref b1, ref b2, _) => make_union(b1, b2, &mut self.ufind[..]),
                Constraint::BallInSocket(ref b)   => {
                    match (b.borrow().anchor1().body.as_ref(), b.borrow().anchor2().body.as_ref()) {
                        (Some(b1), Some(b2)) => make_union(b1, b2, &mut self.ufind[..]),
                        _ => { }
                    }
                },
                Constraint::Fixed(ref f)   => {
                    match (f.borrow().anchor1().body.as_ref(), f.borrow().anchor2().body.as_ref()) {
                        (Some(b1), Some(b2)) => make_union(b1, b2, &mut self.ufind[..]),
                        _ => { }
                    }
                }
            }
        }

        /*
         * Body activation/deactivation.
         */
        // Find deactivable islands.
        for i in 0usize .. self.ufind.len() {
            let root = union_find::find(i, &mut self.ufind[..]);
            let b    = bodies.elements()[i].value.borrow();

            self.can_deactivate[root] =
                match b.deactivation_threshold() {
                    Some(threshold) => {
                        self.can_deactivate[root] && b.activation_state().energy() < threshold
                    },
                    None => false
                };
        }

        // Activate/deactivate islands.
        for i in 0usize .. self.ufind.len() {
            let root = union_find::find(i, &mut self.ufind[..]);
            let mut b = bodies.elements()[i].value.borrow_mut();

            if self.can_deactivate[root] { // Everybody in this set can be deactivacted.
                b.deactivate();
            }
            else { // Everybody in this set must be reactivated.
                if !b.is_active() && b.can_move() {
                    match b.deactivation_threshold() {
                        Some(threshold) => b.activate(threshold * na::convert::<f64, N>(2.0f64)),
                        None => { }
                    }
                }
            }
        }
    }
}
