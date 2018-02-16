use std::iter;
use na;
use alga::general::Real;
use world::{RigidBodyCollisionWorld, RigidBodyStorage};
use detection::constraint::Constraint;
use detection::joint::{JointManager, Joint};
use object::{WorldObject, RigidBody, ActivationState};
use utils::union_find::UnionFindSet;
use utils::union_find;

/// Structure that monitors island-based activation/deactivation of objects.
///
/// It is responsible for making objects sleep or wake up.
pub struct ActivationManager<N: Real> {
    mix_factor:     N,
    ufind:          Vec<UnionFindSet>,
    can_deactivate: Vec<bool>,
    to_activate:    Vec<usize>,
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
    pub fn deferred_activate(&mut self, b: &RigidBody<N>) {
        if b.can_move() && !b.is_active() {
            self.to_activate.push(b.uid());
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
                  bodies: &mut RigidBodyStorage<N>) {
        /*
         *
         * Update bodies energy
         *
         */
        for (i, b) in bodies.iter_mut().enumerate() {
            assert!(*b.activation_state() != ActivationState::Deleted);
            if b.is_active() {
                self.update_energy(b);
            }

            b.set_index(i as isize);
        }

        /*
         *
         * Activate bodies that need it.
         *
         */
        for &b in &self.to_activate {
            let rb = &mut bodies[b];

            if let Some(threshold) = rb.deactivation_threshold() {
                rb.activate(threshold * na::convert(2.0f64));
            }
        }

        self.to_activate.clear();

        /*
         *
         * Build islands to deactivate those with low-energy objects only.
         *
         */
        // Resize buffers.
        self.ufind.resize(bodies.len(), UnionFindSet::new(0));
        self.can_deactivate.resize(bodies.len(), false);

        // Init the union find.
        for (i, u) in self.ufind.iter_mut().enumerate() {
            u.reinit(i)
        }

        for d in self.can_deactivate.iter_mut() {
            *d = true
        }

        // Run the union-find.
        fn make_union<N: Real>(rb1: &RigidBody<N>, rb2: &RigidBody<N>, ufs: &mut [UnionFindSet]) {
            if rb1.can_move() && rb2.can_move() {
                union_find::union(rb1.index() as usize, rb2.index() as usize, ufs)
            }
        }

        for (b1, b2, cd) in world.contact_pairs() {
            if let (&WorldObject::RigidBody(rb1), &WorldObject::RigidBody(rb2)) = (&b1.data, &b2.data) {
                if cd.num_contacts() != 0 {
                    make_union(&bodies[rb1], &bodies[rb2], &mut self.ufind[..])
                }
            }
        }

        for e in joints.joints().iter() {
            match *e {
                Constraint::RBRB(b1, b2, _) => make_union(&bodies[b1], &bodies[b2], &mut self.ufind[..]),
                Constraint::BallInSocket(ref b)   => {
                    if let (Some(b1), Some(b2)) = (b.anchor1().body, b.anchor2().body) {
                        make_union(&bodies[b1], &bodies[b2], &mut self.ufind[..]);
                    }
                },
                Constraint::Fixed(ref f)   => {
                    if let (Some(b1), Some(b2)) = (f.anchor1().body, f.anchor2().body) {
                        make_union(&bodies[b1], &bodies[b2], &mut self.ufind[..]);
                    }
                },
            }
        }

        /*
         * Body activation/deactivation.
         */
        // Find deactivable islands.
        for (i, b) in bodies.iter().enumerate() {
            let root = union_find::find(i, &mut self.ufind[..]);

            self.can_deactivate[root] =
                match b.deactivation_threshold() {
                    Some(threshold) => {
                        self.can_deactivate[root] && b.activation_state().energy() < threshold
                    },
                    None => false
                };
        }

        // Activate/deactivate islands.
        for (i, b) in bodies.iter_mut().enumerate() {
            let root = union_find::find(i, &mut self.ufind[..]);

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
