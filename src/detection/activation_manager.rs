use slab::Slab;

use na::{self, RealField};
use crate::world::ColliderWorld;
use crate::object::{BodyHandle, Body, BodySet};
use crate::joint::JointConstraint;
use crate::utils::union_find::UnionFindSet;
use crate::utils::union_find;

/// Structure that monitors island-based activation/deactivation of bodies.
///
/// It is responsible for making objects sleep or wake up.
pub struct ActivationManager<N: RealField> {
    mix_factor: N,
    ufind: Vec<UnionFindSet>,
    can_deactivate: Vec<bool>,
    to_activate: Vec<BodyHandle>,
    id_to_body: Vec<BodyHandle>,
}

impl<N: RealField> ActivationManager<N> {
    /// Creates a new `ActivationManager2`.
    ///
    /// # Arguments:
    /// * `thresold`   - the minimum energy required to keep an object awake.
    /// * `mix_factor` - the ratio of energy to keep between two frames.
    pub fn new(mix_factor: N) -> ActivationManager<N> {
        assert!(
            mix_factor >= na::zero(),
            "The energy mixing factor must be between 0.0 and 1.0."
        );

        ActivationManager {
            mix_factor: mix_factor,
            ufind: Vec::new(),
            can_deactivate: Vec::new(),
            to_activate: Vec::new(),
            id_to_body: Vec::new(),
        }
    }

    /// Notify the `ActivationManager2` that is has to activate an object at the next update.
    // FIXME: this is not a very good name
    pub fn deferred_activate(&mut self, handle: BodyHandle) {
        self.to_activate.push(handle);
    }

    fn update_energy(&self, body: &mut Body<N>) {
        // FIXME: avoid the Copy when NLL lands ?
        let status = *body.activation_status();

        if let Some(threshold) = status.deactivation_threshold() {
            // FIXME: take the time in account (to make a true RWA)
            let new_energy = (N::one() - self.mix_factor) * status.energy()
                + self.mix_factor * (body.generalized_velocity().norm_squared());

            body.activate_with_energy(new_energy.min(threshold * na::convert(4.0f64)));
        }
    }

    /// Update the activation manager, activating and deactivating objects when needed.
    pub fn update(
        &mut self,
        bodies: &mut BodySet<N>,
        cworld: &ColliderWorld<N>,
        constraints: &Slab<Box<JointConstraint<N>>>,
        active_bodies: &mut Vec<BodyHandle>,
    ) {
        /*
         *
         * Update bodies energy
         *
         */
        self.id_to_body.clear();

        for body in bodies.bodies_mut() {
            if body.status_dependent_ndofs() != 0 {
                if body.is_active() {
                    self.update_energy(body);
                }

                body.set_companion_id(self.id_to_body.len());
                self.id_to_body.push(body.handle());
            }

            if body.is_kinematic() {
                body.set_companion_id(self.id_to_body.len());
                self.id_to_body.push(body.handle());
            }
        }

        /*
         *
         * Activate bodies that need it.
         *
         */
        for handle in self.to_activate.iter() {
            let body = try_continue!(bodies.body_mut(*handle));

            if body.activation_status().deactivation_threshold().is_some() {
                body.activate()
            }
        }

        self.to_activate.clear();

        /*
         *
         * Build islands.
         *
         */
        // Resize buffers.
        self.ufind
            .resize(self.id_to_body.len(), UnionFindSet::new(0));
        self.can_deactivate.resize(self.id_to_body.len(), true);

        // Init the union find.
        // FIXME: are there more efficient ways of doing those?
        for (i, u) in self.ufind.iter_mut().enumerate() {
            u.reinit(i)
        }

        for d in self.can_deactivate.iter_mut() {
            *d = true
        }

        // Run the union-find.
        // FIXME: use the union-find from petgraph?
        #[inline(always)]
        fn make_union<N: RealField>(
            bodies: &BodySet<N>,
            b1: BodyHandle,
            b2: BodyHandle,
            ufs: &mut [UnionFindSet],
        ) {
            let b1 = try_ret!(bodies.body(b1));
            let b2 = try_ret!(bodies.body(b2));
            if (b1.status_dependent_ndofs() != 0 || b1.is_kinematic())
                && (b2.status_dependent_ndofs() != 0 || b2.is_kinematic())
                {
                    union_find::union(b1.companion_id(), b2.companion_id(), ufs)
                }
        }

        for (c1, c2, _, manifold) in cworld.contact_pairs(false) {
            if manifold.len() > 0 {
                make_union(bodies, c1.body(), c2.body(), &mut self.ufind)
            }
        }

        for (_, c) in constraints.iter() {
            let (b1, b2) = c.anchors();
            make_union(bodies, b1.0, b2.0, &mut self.ufind);
        }

        /*
         * Body activation/deactivation.
         */
        // Find deactivable islands.
        for i in 0usize..self.ufind.len() {
            let root = union_find::find(i, &mut self.ufind[..]);
            let handle = self.id_to_body[i];
            let body = try_continue!(bodies.body(handle));
            // FIXME: avoid the Copy when NLL lands ?
            let status = *body.activation_status();

            self.can_deactivate[root] = match status.deactivation_threshold() {
                Some(threshold) => self.can_deactivate[root] && status.energy() < threshold,
                None => false,
            };
        }

        // Activate/deactivate islands.
        for i in 0usize..self.ufind.len() {
            let root = union_find::find(i, &mut self.ufind[..]);
            let handle = self.id_to_body[i];
            let body = try_continue!(bodies.body_mut(handle));

            if self.can_deactivate[root] {
                // Everybody in this set can be deactivacted.
                if body.is_active() {
                    body.deactivate();
                }
            } else if !body.is_kinematic() {
                // Everybody in this set must be reactivated.
                active_bodies.push(handle);

                // FIXME: avoid the Copy when NLL lands ?
                let status = *body.activation_status();

                if !status.is_active() && status.deactivation_threshold().is_some() {
                    body.activate()
                }
            }
        }
    }
}
