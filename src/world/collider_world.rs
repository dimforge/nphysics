use std::collections::{hash_map, HashMap};


use na::RealField;

use ncollide::pipeline::{
    self,
    DefaultProximityDispatcher, DefaultContactDispatcher,
    InteractionGraph, Interaction, ContactAlgorithm, ProximityDetector, NarrowPhase, ContactEvents, ProximityEvents,
    DBVTBroadPhase, BroadPhase, BroadPhasePairFilter,
    CollisionGroups
};
use ncollide::query::{Ray, ContactManifold, Proximity};
use ncollide::bounding_volume::AABB;

use crate::volumetric::Volumetric;
use crate::object::{Collider, ColliderHandle, DefaultColliderHandle, ColliderAnchor,
                    ColliderSet, BodySet, DefaultBodyHandle, Body, BodyHandle};

use crate::math::Point;


/// The default collider world, that can be used with a `DefaultBodyHandle` and `DefaultColliderHandle`.
pub type DefaultColliderWorld<N> = ColliderWorld<N, DefaultBodyHandle, DefaultColliderHandle>;

/// The world managing all geometric queries.
///
/// This is a wrapper over the `CollisionWorld` structure from `ncollide` to simplify
/// its use with the [object::Collider] structure.
pub struct ColliderWorld<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle> {
    /// The broad phase used by this collision world.
    pub(crate) broad_phase: Box<BroadPhase<N, AABB<N>, CollHandle>>,
    /// The narrow-phase used by this collision world.
    pub(crate) narrow_phase: NarrowPhase<N, CollHandle>,
    /// The graph of interactions detected so far.
    pub(crate) interactions: InteractionGraph<N, CollHandle>,
    /// A user-defined broad-phase pair filter.
    // FIXME: we don't actually use this currently.
    pub(crate) pair_filters: Option<Box<BroadPhasePairFilter<N, Collider<N, Handle>, CollHandle>>>,
    body_colliders: HashMap<Handle, Vec<CollHandle>>,
}

impl<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle> ColliderWorld<N, Handle, CollHandle> {
    /// Creates a collider world from the provided broad-phase and narrow-phase structures.
    pub fn from_parts<BF>(broad_phase: BF, narrow_phase: NarrowPhase<N, CollHandle>) -> Self
        where BF: BroadPhase<N, AABB<N>, CollHandle> {

        ColliderWorld {
            broad_phase: Box::new(broad_phase),
            narrow_phase,
            interactions: InteractionGraph::new(),
            pair_filters: None,
            body_colliders: HashMap::new(),
        }
    }

    /// Creates a new collision world.
    // FIXME: use default values for `margin` and allow its modification by the user ?
    pub fn new() -> Self {
        let coll_dispatcher = Box::new(DefaultContactDispatcher::new());
        let prox_dispatcher = Box::new(DefaultProximityDispatcher::new());
        let broad_phase = DBVTBroadPhase::new(na::convert(0.01));
        let narrow_phase = NarrowPhase::new(coll_dispatcher, prox_dispatcher);
        Self::from_parts(broad_phase, narrow_phase)

    }

    fn register_collider(&mut self, handle: CollHandle, collider: &mut Collider<N, Handle>) {
        assert!(collider.proxy_handle().is_none(), "Cannot register a collider that is already registered.");
        assert!(collider.graph_index().is_none(), "Cannot register a collider that is already registered.");

        let proxies = pipeline::create_proxies(handle, &mut *self.broad_phase, &mut self.interactions, collider.position(), collider.shape(), collider.query_type());

        self.body_colliders.entry(collider.body()).or_insert(Vec::new()).push(handle);
        collider.set_proxy_handle(Some(proxies.0));
        collider.set_graph_index(Some(proxies.1));
    }

    /// Maintain the internal structures of the collider world by handling body removals and colliders insersion and removals.
    pub fn maintain<Bodies, Colliders>(&mut self, bodies: &mut Bodies, colliders: &mut Colliders)
        where Bodies: BodySet<N, Handle = Handle>,
              Colliders: ColliderSet<N, Handle, Handle = CollHandle> {
        self.handle_removals(bodies, colliders);
        self.handle_insertions(bodies, colliders);
    }

    fn handle_insertions<Bodies, Colliders>(&mut self, bodies: &mut Bodies, colliders: &mut Colliders)
        where Bodies: BodySet<N, Handle = Handle>,
              Colliders: ColliderSet<N, Handle, Handle = CollHandle> {
        while let Some(handle) = colliders.pop_insertion_event() {
            if let Some(collider) = colliders.get_mut(handle) {
                self.register_collider(handle, collider);

                match collider.anchor() {
                    ColliderAnchor::OnBodyPart { body_part, position_wrt_body_part } => {
                        let body = bodies.get_mut(body_part.0).expect("Invalid parent body part handle.");

                        // Update the parent body's inertia.
                        if !collider.density().is_zero() {
                            let (com, inertia) = collider.shape().transformed_mass_properties(collider.density(), position_wrt_body_part);
                            body.add_local_inertia_and_com(body_part.1, com, inertia);
                        }

                        // Set the position and ndofs (note this will be done in the `sync_collider` too.
                        let ndofs = body.status_dependent_ndofs();
                        let part = body.part(body_part.1).expect("Invalid parent body part handle.");
                        let pos = part.position() * position_wrt_body_part;

                        collider.set_body_status_dependent_ndofs(ndofs);
                        collider.set_position(pos);
                    }
                    _ => {}
                }
            }
        }
    }

    fn handle_removals<Bodies, Colliders>(&mut self, bodies: &mut Bodies, colliders: &mut Colliders)
        where Bodies: BodySet<N, Handle = Handle>,
              Colliders: ColliderSet<N, Handle, Handle = CollHandle> {
        let mut graph_id_remapping = HashMap::new();

        while let Some((removed_handle, mut removed)) = colliders.pop_removal_event() {
            // Adjust the graph index of the removed collider if it was affected by other removals.
            if let Some(new_id) = graph_id_remapping.get(&removed_handle) {
                removed.graph_index = *new_id
            }

            // Activate the bodies in contact with the deleted collider.
            for (coll1, coll2, _, _) in self.interactions.contacts_with(removed.graph_index, false) {
                if coll1 == removed_handle {
                    if let Some(coll) = colliders.get(coll2) {
                        if let Some(body) = bodies.get_mut(coll.body()) {
                            body.activate()
                        }
                    }
                }

                if coll2 == removed_handle {
                    if let Some(coll) = colliders.get(coll1) {
                        if let Some(body) = bodies.get_mut(coll.body()) {
                            body.activate()
                        }
                    }
                }
            }

            // Activate the body the deleted collider was attached to.
            if let Some(body) = bodies.get_mut(removed.anchor.body()) {
                // Update the parent body's inertia.
                if !removed.density.is_zero() {
                    if let ColliderAnchor::OnBodyPart { body_part, position_wrt_body_part } = &removed.anchor {
                        let (com, inertia) = removed.shape.transformed_mass_properties(removed.density, position_wrt_body_part);
                        body.add_local_inertia_and_com(body_part.1, -com, -inertia)
                    }
                }

                body.activate()
            }

            // Remove the collider from the list of colliders for this body.
            match self.body_colliders.entry(removed.anchor.body()) {
                hash_map::Entry::Occupied(mut e) => {
                    if let Some(i) = e.get().iter().position(|h| *h == removed_handle) {
                        let _ = e.get_mut().swap_remove(i);
                    }

                    if e.get().is_empty() {
                        let _ = e.remove_entry();
                    }
                },
                hash_map::Entry::Vacant(_) => {}
            }


            // Remove proxies and handle the graph index remapping.
            if let Some(to_change) = pipeline::remove_proxies(&mut *self.broad_phase, &mut self.interactions, removed.proxy_handle, removed.graph_index) {
                if let Some(collider) = colliders.get_mut(to_change.0) {
                    // Apply the graph index remapping.
                    collider.set_graph_index(Some(to_change.1))
                } else {
                    // Register the graph index remapping for other removed colliders.
                    let _ = graph_id_remapping.insert(to_change.0, to_change.1);
                }
            }
        }
    }

    /// Synchronize all colliders with their body parent and the underlying collision world.
    pub fn sync_colliders<Bodies, Colliders>(&mut self, bodies: &Bodies, colliders: &mut Colliders)
        where Bodies: BodySet<N, Handle = Handle>,
              Colliders: ColliderSet<N, Handle, Handle = CollHandle> {
        colliders.foreach_mut(|_collider_id, collider| {
            let body = try_ret!(bodies.get(collider.body()));

            collider.set_body_status_dependent_ndofs(body.status_dependent_ndofs());

            if !body.update_status().colliders_need_update() {
                return;
            }

            let new_pos = match collider.anchor() {
                ColliderAnchor::OnBodyPart { body_part, position_wrt_body_part } => {
                    let part = try_ret!(body.part(body_part.1));
                    let part_pos1 = part.safe_position();
                    let part_pos2 = part.position();
                    Some((part_pos1 * position_wrt_body_part, part_pos2 * position_wrt_body_part))
                }
                ColliderAnchor::OnDeformableBody { .. } => {
                    None
                }
            };

            match new_pos {
                Some(pos) => collider.set_position_with_prediction(pos.0, pos.1),
                None => collider.set_deformations(body.deformed_positions().unwrap().1)
            }
        });
    }

    /// Returns the set of colliders attached to the specified body.
    ///
    /// Returns `None` if the body has no collider attached to it, of if the body does not exist.
    pub fn body_colliders(&self, body: Handle) -> Option<&[CollHandle]> {
        self.body_colliders.get(&body).map(|c| &c[..])
    }

    /*
    /// Customize the selection of narrow-phase collision detection algorithms
    pub fn set_narrow_phase(&mut self, narrow_phase: NarrowPhase<N, CollHandle>) {
        self.cworld.set_narrow_phase(narrow_phase);
    }
*/
    /// Empty the contact and proximity event pools.
    pub fn clear_events(&mut self) {
        self.narrow_phase.clear_events()
    }
/*
    /// Adds a filter that tells if a potential collision pair should be ignored or not.
    ///
    /// The proximity filter returns `false` for a given pair of colliders if they should
    /// be ignored by the narrow phase. Keep in mind that modifying the proximity filter will have
    /// a non-trivial overhead during the next update as it will force re-detection of all
    /// collision pairs.
    pub fn register_broad_phase_pair_filter<F>(&mut self, name: &str, filter: F)
        where F: BroadPhasePairFilter<N, ColliderData<N>> {
        self.cworld.register_broad_phase_pair_filter(name, filter)
    }

    /// Removes the pair filter named `name`.
    pub fn unregister_broad_phase_pair_filter(&mut self, name: &str) {
        self.cworld.unregister_broad_phase_pair_filter(name)
    }
    */

    /// Executes the broad phase of the collision detection pipeline.
    pub fn perform_broad_phase<Colliders>(&mut self, colliders: &Colliders)
        where Colliders: ColliderSet<N, Handle, Handle = CollHandle> {
        pipeline::perform_broad_phase(
            colliders,
            &mut *self.broad_phase,
            &mut self.narrow_phase,
            &mut self.interactions,
            Some(&BodyStatusCollisionFilter),
//            self.pair_filters.as_ref().map(|f| &**f)
        )
    }

    /// Executes the narrow phase of the collision detection pipeline.
    pub fn perform_narrow_phase<Colliders>(&mut self, colliders: &Colliders)
        where Colliders: ColliderSet<N, Handle, Handle = CollHandle> {
        pipeline::perform_narrow_phase(colliders, &mut self.narrow_phase, &mut self.interactions)
    }

    /// The broad-phase used by this collider world.
    pub fn broad_phase(&self) -> &BroadPhase<N, AABB<N>, CollHandle> {
        &*self.broad_phase
    }

    /// Computes the interferences between every rigid bodies on this world and a ray.
    #[inline]
    pub fn interferences_with_ray<'a, 'b, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(
        &'a self,
        colliders: &'a Colliders,
        ray: &'b Ray<N>,
        groups: &'b CollisionGroups,
    ) -> pipeline::InterferencesWithRay<'a, 'b, N, Colliders>
    {
        pipeline::interferences_with_ray(&colliders, &*self.broad_phase, ray, groups)
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a point.
    #[inline]
    pub fn interferences_with_point<'a, 'b, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(
        &'a self,
        colliders: &'a Colliders,
        point: &'b Point<N>,
        groups: &'b CollisionGroups,
    ) -> pipeline::InterferencesWithPoint<'a, 'b, N, Colliders>
    {
        pipeline::interferences_with_point(&colliders, &*self.broad_phase, point, groups)
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a aabb.
    #[inline]
    pub fn interferences_with_aabb<'a, 'b, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(
        &'a self,
        colliders: &'a Colliders,
        aabb: &'b AABB<N>,
        groups: &'b CollisionGroups,
    ) -> pipeline::InterferencesWithAABB<'a, 'b, N, Colliders>
    {
        pipeline::interferences_with_aabb(&colliders, &*self.broad_phase, aabb, groups)
    }

    /// The contact events pool.
    pub fn contact_events(&self) -> &ContactEvents<CollHandle> {
        self.narrow_phase.contact_events()
    }

    /// The proximity events pool.
    pub fn proximity_events(&self) -> &ProximityEvents<CollHandle> {
        self.narrow_phase.proximity_events()
    }

    /*
     *
     * Iterators on contacts/proximity pairs.
     *
     */
    fn is_interaction_effective(c1: &Collider<N, Handle>, c2: &Collider<N, Handle>, interaction: &Interaction<N>) -> bool {
        match interaction {
            Interaction::Contact(_, manifold) => Self::is_contact_effective(c1, c2, manifold),
            Interaction::Proximity(_, prox) => *prox == Proximity::Intersecting
        }
    }

    fn is_contact_effective(c1: &Collider<N, Handle>, c2: &Collider<N, Handle>, manifold: &ContactManifold<N>) -> bool {
        if let Some(c) = manifold.deepest_contact() {
            c.contact.depth >= -(c1.margin() + c2.margin())
        } else {
            false
        }
    }

    #[inline(always)]
    fn filter_interactions<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(colliders: &'a Colliders, iter: impl Iterator<Item = (CollHandle, CollHandle, &'a Interaction<N>)>, effective_only: bool)
                               -> impl Iterator<Item = (CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a Interaction<N>)> {
        iter.filter_map(move |inter| {
            let c1 = colliders.get(inter.0)?;
            let c2 = colliders.get(inter.1)?;
            if !effective_only || Self::is_interaction_effective(c1, c2, inter.2) {
                Some((inter.0, c1, inter.1, c2, inter.2))
            } else {
                None
            }
        })
    }

    #[inline(always)]
    fn filter_contacts<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(colliders: &'a Colliders, iter: impl Iterator<Item = (CollHandle, CollHandle, &'a ContactAlgorithm<N>, &'a ContactManifold<N>)>, effective_only: bool)
                           -> impl Iterator<Item = (CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a ContactAlgorithm<N>, &'a ContactManifold<N>)> {
        iter.filter_map(move |inter| {
            let c1 = colliders.get(inter.0)?;
            let c2 = colliders.get(inter.1)?;
            if !effective_only || Self::is_contact_effective(c1, c2, inter.3) {
                Some((inter.0, c1, inter.1, c2, inter.2, inter.3))
            } else {
                None
            }
        })
    }

    #[inline(always)]
    fn filter_proximities<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(colliders: &'a Colliders, iter: impl Iterator<Item = (CollHandle, CollHandle, &'a ProximityDetector<N>, Proximity)>)
                           -> impl Iterator<Item = (CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a ProximityDetector<N>, Proximity)> {
        iter.filter_map(move |prox| {
            Some((prox.0, colliders.get(prox.0)?, prox.1, colliders.get(prox.1)?, prox.2, prox.3))
        })
    }

    // FIXME: we need to be careful with the notion of "effective_only" when dealing with
    // contacts. Perhaps we should filter out contacts with depths that are not considered
    // as actual contacts by the solver?

    /// All the potential interactions pairs.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interaction_pairs<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, effective_only: bool) -> impl Iterator<Item = (
        CollHandle,
        &'a Collider<N, Handle>,
        CollHandle,
        &'a Collider<N, Handle>,
        &'a Interaction<N>
    )> {
        Self::filter_interactions(colliders, self.interactions.interaction_pairs(false), effective_only)
    }


    /// All the potential contact pairs.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn contact_pairs<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, effective_only: bool) -> impl Iterator<Item = (
        CollHandle,
        &'a Collider<N, Handle>,
        CollHandle,
        &'a Collider<N, Handle>,
        &'a ContactAlgorithm<N>,
        &'a ContactManifold<N>,
    )> {
        Self::filter_contacts(colliders, self.interactions.contact_pairs(false), effective_only)
    }


    /// All the potential proximity pairs.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn proximity_pairs<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, effective_only: bool) -> impl Iterator<Item = (
        CollHandle,
        &'a Collider<N, Handle>,
        CollHandle,
        &'a Collider<N, Handle>,
        &'a ProximityDetector<N>,
        Proximity,
    )> {
        Self::filter_proximities(colliders, self.interactions.proximity_pairs(effective_only))
    }


    /// The potential interaction pair between the two specified colliders.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interaction_pair<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, handle1: CollHandle, handle2: CollHandle, effective_only: bool)
                            -> Option<(CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a Interaction<N>)> {
        let id1 = colliders.get(handle1)?.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        let id2 = colliders.get(handle2)?.graph_index().expect(crate::NOT_REGISTERED_ERROR);

        self.interactions.interaction_pair(id1, id2, false).and_then(move |inter| {
            let c1 = colliders.get(inter.0)?;
            let c2 = colliders.get(inter.1)?;

            if !effective_only || Self::is_interaction_effective(c1, c2, inter.2) {
                Some((inter.0, c1, inter.1, c2, inter.2))
            } else {
                None
            }
        })
    }

    /// The potential contact pair between the two specified colliders.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn contact_pair<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, handle1: CollHandle, handle2: CollHandle, effective_only: bool)
                        -> Option<(CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a ContactAlgorithm<N>, &'a ContactManifold<N>)> {
        let id1 = colliders.get(handle1)?.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        let id2 = colliders.get(handle2)?.graph_index().expect(crate::NOT_REGISTERED_ERROR);

        self.interactions.contact_pair(id1, id2, false).and_then(move |inter| {
            let c1 = colliders.get(inter.0)?;
            let c2 = colliders.get(inter.1)?;
            if !effective_only || Self::is_contact_effective(c1, c2, inter.3) {
                Some((inter.0, c1, inter.1, c2, inter.2, inter.3))
            } else {
                None
            }
        })
    }

    /// The potential proximity pair between the two specified colliders.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn proximity_pair<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, handle1: CollHandle, handle2: CollHandle, effective_only: bool)
                          -> Option<(CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a ProximityDetector<N>, Proximity)> {
        let id1 = colliders.get(handle1)?.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        let id2 = colliders.get(handle2)?.graph_index().expect(crate::NOT_REGISTERED_ERROR);

        self.interactions.proximity_pair(id1, id2, effective_only).and_then(move |prox| {
            Some((prox.0, colliders.get(prox.0)?, prox.1, colliders.get(prox.1)?, prox.2, prox.3))
        })
    }

    /// All the interaction pairs involving the specified collider.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interactions_with<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, handle: CollHandle, effective_only: bool)
                             -> Option<impl Iterator<Item = (CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a Interaction<N>)>> {
        let idx = colliders.get(handle)?.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        Some(Self::filter_interactions(colliders, self.interactions.interactions_with(idx, false), effective_only))
    }

    /// All the contact pairs involving the specified collider.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn contacts_with<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, handle: CollHandle, effective_only: bool)
                         -> Option<impl Iterator<Item = (CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a ContactAlgorithm<N>, &'a ContactManifold<N>)>> {
        let idx = colliders.get(handle)?.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        Some(Self::filter_contacts(colliders, self.interactions.contacts_with(idx, false), effective_only))
    }

    /// All the proximity pairs involving the specified collider.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn proximities_with<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, handle: CollHandle, effective_only: bool)
                            -> Option<impl Iterator<Item = (CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a ProximityDetector<N>, Proximity)>> {
        let idx = colliders.get(handle)?.graph_index().expect(crate::NOT_REGISTERED_ERROR);
        Some(Self::filter_proximities(colliders, self.interactions.proximities_with(idx, effective_only)))
    }

    /// All the collider handles of colliders interacting with the specified collider.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn colliders_interacting_with<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, handle: CollHandle) -> Option<impl Iterator<Item = (CollHandle, &'a Collider<N, Handle>)>> {
        Some(self.interactions_with(colliders, handle, true)?
            .map(move |(h1, c1, h2, c2, _)| {
                if h1 == handle { (h2, c2) } else { (h1, c1) }
            }))
    }

    /// All the collider handles of colliders in potential contact with the specified collision
    /// object.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn colliders_in_contact_with<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, handle: CollHandle) -> Option<impl Iterator<Item = (CollHandle, &'a Collider<N, Handle>)>> {
        Some(self.contacts_with(colliders, handle, true)?
            .map(move |(h1, c1, h2, c2, _, _)| {
                if h1 == handle { (h2, c2) } else { (h1, c1) }
            }))
    }

    /// All the collider handles of colliders in potential proximity of with the specified
    /// collider.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn colliders_in_proximity_of<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, handle: CollHandle) -> Option<impl Iterator<Item = (CollHandle, &'a Collider<N, Handle>)>> {
        Some(self.proximities_with(colliders, handle, true)?
            .map(move |(h1, c1, h2, c2, _, _)| {
                if h1 == handle { (h2, c2) } else { (h1, c1) }
            }))
    }
}

struct BodyStatusCollisionFilter;

impl<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle> BroadPhasePairFilter<N, Collider<N, Handle>, CollHandle> for BodyStatusCollisionFilter {
    /// Activate an action for when two objects start or stop to be close to each other.
    fn is_pair_valid(&self, c1: &Collider<N, Handle>, c2: &Collider<N, Handle>, _: CollHandle, _: CollHandle) -> bool {
        c1.body_status_dependent_ndofs() != 0 || c2.body_status_dependent_ndofs() != 0
    }
}