use std::collections::{hash_map, HashMap};

use na::RealField;
use ncollide::pipeline::world::CollisionWorld;
use ncollide::pipeline::object::{GeometricQueryType, CollisionGroups};
use ncollide::broad_phase::{BroadPhase, BroadPhasePairFilter};
use ncollide::narrow_phase::{InteractionGraph, Interaction, ContactAlgorithm, ProximityAlgorithm, NarrowPhase, ContactEvents, ProximityEvents};
use ncollide::query::{Ray, RayIntersection, ContactManifold, Proximity};
use ncollide::shape::ShapeHandle;
use ncollide::bounding_volume::AABB;
use ncollide::pipeline::glue;

use crate::object::{Collider, ColliderData, ColliderHandle, ColliderAnchor, ColliderSet,
                    BodySet, BodySlabHandle, BodyPartHandle, Body, BodyHandle};
use crate::material::{BasicMaterial, MaterialHandle};
use crate::math::{Isometry, Point};


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
//    /// A user-defined broad-phase pair filter.
//    pub pair_filters: Option<Box<for <'a> BroadPhasePairFilter<N, &'a Collider<N, T>>>>,
    collider_lists: HashMap<Handle, (CollHandle, CollHandle)>, // (head, tail)
    colliders_w_parent: Vec<CollHandle>,
    default_material: MaterialHandle<N, Handle>
}

impl<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle> ColliderWorld<N, Handle, CollHandle> {
    /// Creates a new collision world.
    // FIXME: use default values for `margin` and allow its modification by the user ?
    pub fn new(margin: N) -> Self {
        unimplemented!()
//        let mut cworld = CollisionWorld::new(margin);
//        cworld.set_broad_phase_pair_filter(Some(BodyStatusCollisionFilter));
//
//        ColliderWorld {
//            collider_lists: HashMap::new(),
//            colliders_w_parent: Vec::new(),
//            default_material: MaterialHandle::new(BasicMaterial::default())
//        }
    }

    /// Synchronize all colliders with their body parent and the underlying collision world.
    pub fn sync_colliders<Bodies, Colliders>(&mut self, bodies: &Bodies, colliders: &mut Colliders)
        where Bodies: BodySet<N, Handle = Handle>,
              Colliders: ColliderSet<N, Handle, Handle = CollHandle> {
        unimplemented!()
        /*
        colliders.foreach_mut(|collider_id, collider| {
            // FIXME: update only if the position changed (especially for static bodies).
            let body = try_ret!(bodies.get(collider.data().body()));

            collider
                .data_mut()
                .set_body_status_dependent_ndofs(body.status_dependent_ndofs());

            if !body.is_active() || !body.update_status().colliders_need_update() {
                return true;
            }

            let new_pos = match collider.data().anchor() {
                ColliderAnchor::OnBodyPart { body_part, position_wrt_body_part } => {
                    let part = try_ret!(body.part(body_part.1), false);
                    let part_pos1 = part.safe_position();
                    let part_pos2 = part.position();
                    Some((part_pos1 * position_wrt_body_part, part_pos2 * position_wrt_body_part))
                }
                ColliderAnchor::OnDeformableBody { .. } => {
                    None
                }
            };

            match new_pos {
                Some(pos) => cworld.set_position(*collider_id, pos.1), // XXX: cworld.set_position_with_prediction(*collider_id, pos.0, &pos.1),
                None => cworld.set_deformations(*collider_id, body.deformed_positions().unwrap().1)
            }

            true
        });
        */
    }

    /// The material given to colliders without user-defined materials.
    pub fn default_material(&self) -> MaterialHandle<N, Handle> {
        self.default_material.clone()
    }

    /*
    /// Customize the selection of narrow-phase collision detection algorithms
    pub fn set_narrow_phase(&mut self, narrow_phase: NarrowPhase<N, CollHandle>) {
        self.cworld.set_narrow_phase(narrow_phase);
    }

    /// Adds a collider to the world.
    pub(crate) fn add(
        &mut self,
        position: Isometry<N>,
        shape: ShapeHandle<N>,
        collision_groups: CollisionGroups,
        query_type: GeometricQueryType<N>,
        data: ColliderData<N, Handle>,
    ) -> &mut Collider<N, Handle>
    {
        let co = Collider::from_mut(self.cworld.add(position, shape, collision_groups, query_type, data));
        let parent = co.body();
        let result = co.handle();

        if !parent.is_ground() {
            self.colliders_w_parent.push(result);
        }

        // Update the colliders list.
        match self.collider_lists.entry(parent) {
            hash_map::Entry::Vacant(e) => {
                let _ = e.insert((result, result));
            }
            hash_map::Entry::Occupied(mut e) => {
                let (head, tail) = *e.get();
                let _ = e.insert((head, result));
                co.set_prev(Some(tail));

                let tail = Collider::from_mut(self.cworld.collision_object_mut(tail).unwrap());
                assert!(tail.next().is_none());
                tail.set_next(Some(result));

            }
        }

        // Return the result.
        self.collider_mut(result).unwrap()
    }

    /// Updates the collision world.
    ///
    /// This executes the whole collision detection pipeline:
    /// 1. Clears the event pools.
    /// 2. Executes the broad phase first.
    /// 3. Executes the narrow phase.
    pub fn update(&mut self) {
        self.cworld.update()
    }
*/
    /// Empty the contact and proximity event pools.
    pub fn clear_events(&mut self) {
        self.narrow_phase.clear_events()
    }
/*
    /// Removed the specified set of colliders from the world.
    ///
    /// Panics of any handle is invalid, or if the list contains duplicates.
    pub(crate) fn remove(&mut self, handles: &[CollHandle]) {
        // Update the collider lists.
        for handle in handles {
            if let Some(co) = colliders.get(*handle) {
                let (prev, next, body) = (co.prev(), co.next(), co.body());

                match (prev, next) {
                    (Some(prev), Some(next)) => {
                        self.collider_mut(next).unwrap().set_prev(Some(prev));
                        self.collider_mut(prev).unwrap().set_next(Some(next));
                    }
                    (Some(prev), None) => {
                        self.collider_mut(prev).unwrap().set_next(None);
                        self.collider_lists.get_mut(&body).unwrap().1 = prev;
                    }
                    (None, Some(next)) => {
                        self.collider_mut(next).unwrap().set_prev(None);
                        self.collider_lists.get_mut(&body).unwrap().0 = next;
                    }
                    (None, None) => {
                        let _ = self.collider_lists.remove(&body);
                    }
                }
            }
        }

        // Remove the colliders.
        self.cworld.remove(handles)
    }

    /// Remove all the colliders attached to `body`.
    pub(crate) fn remove_body_colliders(&mut self, body: Handle) {
        let mut curr = try_ret!(self.collider_lists.get(&body)).0;

        loop {
            let co = try_ret!(colliders.get(curr));
            let next = co.next();
            self.cworld.remove(&[curr]);
            curr = try_ret!(next);
        }
    }

    pub(crate) fn remove_body(&mut self, handle: Handle) {
        self.remove_body_colliders(handle);
        let _ = self.collider_lists.remove(&handle);
    }

    /// Iterator through all the colliders attached to the body with the given `handle`.
    ///
    /// Returns an empty iterator if the body does not exists.
    pub fn body_colliders(&self, handle: Handle) -> ColliderChain<N, Handle, CollHandle> {
        let first = self.collider_lists.get(&handle).map(|l| l.0);

        ColliderChain {
            cworld: self,
            curr: first
        }
    }

    /// Iterator through all the colliders attached to the body part with the given `handle`.
    ///
    /// Returns an empty iterator if the body part does not exists.
    /// Does not return deformable colliders.
    pub fn body_part_colliders(&self, handle: BodyPartHandle<Handle>) -> impl Iterator<Item = &Collider<N, Handle>> {
        self.body_colliders(handle.0).filter(move |co| {
            match co.anchor() {
                ColliderAnchor::OnBodyPart { body_part, .. } => *body_part == handle,
                _ => false
            }
        })
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
    }*/
*/
    /// Executes the broad phase of the collision detection pipeline.
    pub fn perform_broad_phase<Colliders>(&mut self, colliders: &Colliders)
        where Colliders: ColliderSet<N, Handle, Handle = CollHandle> {
        glue::perform_broad_phase(
            colliders,
            &mut *self.broad_phase,
            &mut self.narrow_phase,
            &mut self.interactions,
            None::<&()>,
//            self.pair_filters.as_ref().map(|f| &**f)
        )
    }

    /// Executes the narrow phase of the collision detection pipeline.
    pub fn perform_narrow_phase<Colliders>(&mut self, colliders: &Colliders)
        where Colliders: ColliderSet<N, Handle, Handle = CollHandle> {
        glue::perform_narrow_phase(colliders, &mut self.narrow_phase, &mut self.interactions)
    }
/*
    /// The broad-phase used by this collider world.
    pub fn broad_phase(&self) -> &BroadPhase<N, AABB<N>, CollHandle> {
        &*self.cworld.broad_phase
    }

    /// The broad-phase AABB used for this collider.
    pub fn broad_phase_aabb(&self, handle: CollHandle) -> Option<&AABB<N>> {
        self.cworld.broad_phase_aabb(handle)
    }

    /*
    /// Computes the interferences between every rigid bodies on this world and a ray.
    #[inline]
    pub fn interferences_with_ray<'a>(
        &'a self,
        ray: &'a Ray<N>,
        groups: &'a CollisionGroups,
    ) -> impl Iterator<Item = (&'a Collider<N, Handle>, RayIntersection<N>)>
    {
        self.cworld.interferences_with_ray(ray, groups).map(|res| (Collider::from_ref(res.0), res.1))
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a point.
    #[inline]
    pub fn interferences_with_point<'a>(
        &'a self,
        point: &'a Point<N>,
        groups: &'a CollisionGroups,
    ) -> impl Iterator<Item = &'a Collider<N, Handle>>
    {
        self.cworld.interferences_with_point(point, groups).map(|co| Collider::from_ref(co))
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a aabb.
    #[inline]
    pub fn interferences_with_aabb<'a>(
        &'a self,
        aabb: &'a AABB<N>,
        groups: &'a CollisionGroups,
    ) -> impl Iterator<Item = &'a Collider<N, Handle>>
    {
        self.cworld.interferences_with_aabb(aabb, groups).map(|co| Collider::from_ref(co))
    }
    */

    /// The contact events pool.
    pub fn contact_events(&self) -> &ContactEvents<CollHandle> {
        self.cworld.contact_events()
    }

    /// The proximity events pool.
    pub fn proximity_events(&self) -> &ProximityEvents<CollHandle> {
        self.cworld.proximity_events()
    }*/

    /*
     *
     * Iterators on contacts/proximity pairs.
     *
     */
    fn is_interaction_effective(c1: &Collider<N, Handle>, c2: &Collider<N, Handle>, interaction: &Interaction<N>) -> bool {
        match interaction {
            Interaction::Contact(_, manifold) => Self::is_contact_effective(c1, c2, manifold),
            Interaction::Proximity(prox) => prox.proximity() == Proximity::Intersecting
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
    fn filter_interactions<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, iter: impl Iterator<Item = (CollHandle, CollHandle, &'a Interaction<N>)>, effective_only: bool)
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
    fn filter_contacts<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, iter: impl Iterator<Item = (CollHandle, CollHandle, &'a ContactAlgorithm<N>, &'a ContactManifold<N>)>, effective_only: bool)
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
    fn filter_proximities<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, iter: impl Iterator<Item = (CollHandle, CollHandle, &'a ProximityAlgorithm<N>)>)
                           -> impl Iterator<Item = (CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a ProximityAlgorithm<N>)> {
        iter.filter_map(move |prox| {
            Some((prox.0, colliders.get(prox.0)?, prox.1, colliders.get(prox.1)?, prox.2))
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
        self.filter_interactions(colliders, self.interactions.interaction_pairs(false), effective_only)
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
        self.filter_contacts(colliders, self.interactions.contact_pairs(false), effective_only)
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
        &'a ProximityAlgorithm<N>,
    )> {
        self.filter_proximities(colliders, self.interactions.proximity_pairs(effective_only))
    }


    /// The potential interaction pair between the two specified colliders.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interaction_pair<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, handle1: CollHandle, handle2: CollHandle, effective_only: bool)
                            -> Option<(CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a Interaction<N>)> {
        let id1 = colliders.get(handle1)?.graph_index();
        let id2 = colliders.get(handle2)?.graph_index();

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
        let id1 = colliders.get(handle1)?.graph_index();
        let id2 = colliders.get(handle2)?.graph_index();

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
                          -> Option<(CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a ProximityAlgorithm<N>)> {
        let id1 = colliders.get(handle1)?.graph_index();
        let id2 = colliders.get(handle2)?.graph_index();

        self.interactions.proximity_pair(id1, id2, effective_only).and_then(move |prox| {
            Some((prox.0, colliders.get(prox.0)?, prox.1, colliders.get(prox.1)?, prox.2))
        })
    }

    /// All the interaction pairs involving the specified collider.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interactions_with<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, handle: CollHandle, effective_only: bool)
                             -> Option<impl Iterator<Item = (CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a Interaction<N>)>> {
        let idx = colliders.get(handle)?.graph_index();
        Some(self.filter_interactions(colliders, self.interactions.interactions_with(idx, false), effective_only))
    }

    /// All the contact pairs involving the specified collider.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn contacts_with<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, handle: CollHandle, effective_only: bool)
                         -> Option<impl Iterator<Item = (CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a ContactAlgorithm<N>, &'a ContactManifold<N>)>> {
        let idx = colliders.get(handle)?.graph_index();
        Some(self.filter_contacts(colliders, self.interactions.contacts_with(idx, false), effective_only))
    }

    /// All the proximity pairs involving the specified collider.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn proximities_with<'a, Colliders: ColliderSet<N, Handle, Handle = CollHandle>>(&'a self, colliders: &'a Colliders, handle: CollHandle, effective_only: bool)
                            -> Option<impl Iterator<Item = (CollHandle, &'a Collider<N, Handle>, CollHandle, &'a Collider<N, Handle>, &'a ProximityAlgorithm<N>)>> {
        let idx = colliders.get(handle)?.graph_index();
        Some(self.filter_proximities(colliders, self.interactions.proximities_with(idx, effective_only)))
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
            .map(move |(h1, c1, h2, c2, _)| {
                if h1 == handle { (h2, c2) } else { (h1, c1) }
            }))
    }
}

struct BodyStatusCollisionFilter;

impl<'a, N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle> BroadPhasePairFilter<N, &'a Collider<N, Handle>, CollHandle> for BodyStatusCollisionFilter {
    /// Activate an action for when two objects start or stop to be close to each other.
    fn is_pair_valid(&self, c1: &'a Collider<N, Handle>, c2: &'a Collider<N, Handle>, _: CollHandle, _: CollHandle) -> bool {
        c1.0.data().body_status_dependent_ndofs() != 0 || c2.0.data().body_status_dependent_ndofs() != 0
    }
}