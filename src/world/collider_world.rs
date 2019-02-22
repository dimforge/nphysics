use std::collections::{hash_map, HashMap};

use na::Real;
use ncollide::world::{CollisionWorld, GeometricQueryType, CollisionGroups, CollisionObject};
use ncollide::broad_phase::BroadPhasePairFilter;
use ncollide::narrow_phase::{Interaction, ContactAlgorithm, ProximityAlgorithm};
use ncollide::query::{Ray, RayIntersection, ContactManifold, Proximity};
use ncollide::shape::ShapeHandle;
use ncollide::bounding_volume::AABB;
use ncollide::events::{ContactEvents, ProximityEvents};

use crate::object::{Collider, ColliderData, ColliderHandle, ColliderAnchor, BodySet, BodyHandle, BodyPartHandle};
use crate::material::{BasicMaterial, MaterialHandle};
use crate::math::{Isometry, Point};

/// The world managing all geometric queries.
///
/// This is a wrapper over the `CollisionWorld` structure from `ncollide` to simplify
/// its use with the [object::Collider] structure.
pub struct ColliderWorld<N: Real> {
    cworld: CollisionWorld<N, ColliderData<N>>,
    collider_lists: HashMap<BodyHandle, (ColliderHandle, ColliderHandle)>, // (head, tail)
    colliders_w_parent: Vec<ColliderHandle>,
    default_material: MaterialHandle<N>
}

impl<N: Real> ColliderWorld<N> {
    /// Creates a new collision world.
    // FIXME: use default values for `margin` and allow its modification by the user ?
    pub fn new(margin: N) -> Self {
        let mut cworld = CollisionWorld::new(margin);
        cworld.register_broad_phase_pair_filter(
            "__nphysics_internal_body_status_collision_filter",
            BodyStatusCollisionFilter,
        );

        ColliderWorld {
            cworld,
            collider_lists: HashMap::new(),
            colliders_w_parent: Vec::new(),
            default_material: MaterialHandle::new(BasicMaterial::default())
        }
    }

    /// Synchronize all colliders with their body parent and the underlying collision world.
    pub fn sync_colliders(&mut self, bodies: &BodySet<N>) {
        let cworld = &mut self.cworld;
        self.colliders_w_parent.retain(|collider_id| {
            // FIXME: update only if the position changed (especially for static bodies).
            let collider = try_ret!(cworld.collision_object_mut(*collider_id), false);
            let body = try_ret!(bodies.body(collider.data().body()), false);

            collider
                .data_mut()
                .set_body_status_dependent_ndofs(body.status_dependent_ndofs());

            if !body.is_active() || !body.update_status().colliders_need_update() {
                return true;
            }

            let new_pos = match collider.data().anchor() {
                ColliderAnchor::OnBodyPart { body_part, position_wrt_body_part } => {
                    let part = try_ret!(body.part(body_part.1), false);
                    let part_pos = part.position();
                    Some(part_pos * position_wrt_body_part)
                }
                ColliderAnchor::OnDeformableBody { .. } => {
                    None
                }
            };

            match new_pos {
                Some(pos) => cworld.set_position(*collider_id, pos),
                None => cworld.set_deformations(*collider_id, body.deformed_positions().unwrap().1)
            }

            true
        });
    }

    /// The material given to colliders without user-defined materials.
    pub fn default_material(&self) -> MaterialHandle<N> {
        self.default_material.clone()
    }

    /// The underlying collision world from the ncollide crate.
    pub fn as_collider_world(&self) -> &CollisionWorld<N, ColliderData<N>> {
        &self.cworld
    }
    
    /// The underlying collision world from the ncollide crate.
    pub fn as_collider_world_mut(&mut self) -> &mut CollisionWorld<N, ColliderData<N>> {
        &mut self.cworld
    }

    /// Unwraps the underlying collision world from the ncollide crate.
    pub fn into_inner(self) -> CollisionWorld<N, ColliderData<N>> {
        self.cworld
    }

    /// Adds a collider to the world.
    pub(crate) fn add(
        &mut self,
        position: Isometry<N>,
        shape: ShapeHandle<N>,
        collision_groups: CollisionGroups,
        query_type: GeometricQueryType<N>,
        data: ColliderData<N>,
    ) -> &mut Collider<N>
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

    /// Empty the contact and proximity event pools.
    pub fn clear_events(&mut self) {
        self.cworld.clear_events()
    }

    /// Removed the specified set of colliders from the world.
    ///
    /// Panics of any handle is invalid, or if the list contains duplicates.
    pub(crate) fn remove(&mut self, handles: &[ColliderHandle]) {
        // Update the collider lists.
        for handle in handles {
            if let Some(co) = self.collider(*handle) {
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
    pub(crate) fn remove_body_colliders(&mut self, body: BodyHandle) {
        let mut curr = try_ret!(self.collider_lists.get(&body)).0;

        loop {
            let co = try_ret!(self.collider(curr));
            let next = co.next();
            self.cworld.remove(&[curr]);
            curr = try_ret!(next);
        }
    }

    pub(crate) fn remove_body(&mut self, handle: BodyHandle) {
        self.remove_body_colliders(handle);
        let _ = self.collider_lists.remove(&handle);
    }

    /// Iterator through all the colliders with the given name.
    pub fn colliders_with_name<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Collider<N>> {
        self.colliders().filter(move |co| co.name() == name)
    }

    /// Iterator through all the colliders attached to the body with the given `handle`.
    ///
    /// Returns an empty iterator if the body does not exists.
    pub fn body_colliders(&self, handle: BodyHandle) -> ColliderChain<N> {
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
    pub fn body_part_colliders(&self, handle: BodyPartHandle) -> impl Iterator<Item = &Collider<N>> {
        self.body_colliders(handle.0).filter(move |co| {
            match co.anchor() {
                ColliderAnchor::OnBodyPart { body_part, .. } => *body_part == handle,
                _ => false
            }
        })
    }

    /// Sets the position the collider attached to the specified object.
    pub fn set_position(&mut self, handle: ColliderHandle, pos: Isometry<N>) {
        self.cworld.set_position(handle, pos)
    }

//    /// Apply the given deformations to the specified object.
//    pub(crate) fn set_deformations(
//        &mut self,
//        handle: ColliderHandle,
//        coords: &[N],
//    )
//    {
//        self.cworld.set_deformations(handle, coords)
//    }

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

    /// Executes the broad phase of the collision detection pipeline.
    pub fn perform_broad_phase(&mut self) {
        self.cworld.perform_broad_phase()
    }

    /// Executes the narrow phase of the collision detection pipeline.
    pub fn perform_narrow_phase(&mut self) {
        self.cworld.perform_narrow_phase()
    }

    /// Iterates through all colliders.
    #[inline]
    pub fn colliders(&self) -> impl Iterator<Item = &Collider<N>> {
        self.cworld.collision_objects().map(|co| Collider::from_ref(co))
    }

    /// Returns a reference to the collider identified by its handle.
    #[inline]
    pub fn collider(&self, handle: ColliderHandle) -> Option<&Collider<N>> {
        self.cworld.collision_object(handle).map(|co| Collider::from_ref(co))
    }

    /// Returns a mutable reference to the collider identified by its handle.
    #[inline]
    pub fn collider_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider<N>> {
        self.cworld.collision_object_mut(handle).map(|co| Collider::from_mut(co))
    }

    /// Sets the collision groups of the given collider.
    #[inline]
    pub fn set_collision_groups(&mut self, handle: ColliderHandle, groups: CollisionGroups) {
        self.cworld.set_collision_groups(handle, groups)
    }

    /// Computes the interferences between every rigid bodies on this world and a ray.
    #[inline]
    pub fn interferences_with_ray<'a>(
        &'a self,
        ray: &'a Ray<N>,
        groups: &'a CollisionGroups,
    ) -> impl Iterator<Item = (&'a Collider<N>, RayIntersection<N>)>
    {
        self.cworld.interferences_with_ray(ray, groups).map(|res| (Collider::from_ref(res.0), res.1))
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a point.
    #[inline]
    pub fn interferences_with_point<'a>(
        &'a self,
        point: &'a Point<N>,
        groups: &'a CollisionGroups,
    ) -> impl Iterator<Item = &'a Collider<N>>
    {
        self.cworld.interferences_with_point(point, groups).map(|co| Collider::from_ref(co))
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a aabb.
    #[inline]
    pub fn interferences_with_aabb<'a>(
        &'a self,
        aabb: &'a AABB<N>,
        groups: &'a CollisionGroups,
    ) -> impl Iterator<Item = &'a Collider<N>>
    {
        self.cworld.interferences_with_aabb(aabb, groups).map(|co| Collider::from_ref(co))
    }

    /// The contact events pool.
    pub fn contact_events(&self) -> &ContactEvents {
        self.cworld.contact_events()
    }

    /// The proximity events pool.
    pub fn proximity_events(&self) -> &ProximityEvents {
        self.cworld.proximity_events()
    }

    /*
     *
     * Iterators on contacts/proximity pairs.
     *
     */
    fn is_interaction_effective(c1: &Collider<N>, c2: &Collider<N>, interaction: &Interaction<N>) -> bool {
        match interaction {
            Interaction::Contact(_, manifold) => Self::is_contact_effective(c1, c2, manifold),
            Interaction::Proximity(prox) => prox.proximity() == Proximity::Intersecting
        }
    }

    fn is_contact_effective(c1: &Collider<N>, c2: &Collider<N>, manifold: &ContactManifold<N>) -> bool {
        if let Some(c) = manifold.deepest_contact() {
            c.contact.depth >= -(c1.margin() + c2.margin())
        } else {
            false
        }
    }

    #[inline(always)]
    fn filter_interactions<'a>(&'a self, iter: impl Iterator<Item = (ColliderHandle, ColliderHandle, &'a Interaction<N>)>, effective_only: bool)
                               -> impl Iterator<Item = (&'a Collider<N>, &'a Collider<N>, &'a Interaction<N>)> {
        iter.filter_map(move |inter| {
            let c1 = self.collider(inter.0)?;
            let c2 = self.collider(inter.1)?;
            if !effective_only || Self::is_interaction_effective(c1, c2, inter.2) {
                Some((c1, c2, inter.2))
            } else {
                None
            }
        })
    }

    #[inline(always)]
    fn filter_contacts<'a>(&'a self, iter: impl Iterator<Item = (ColliderHandle, ColliderHandle, &'a ContactAlgorithm<N>, &'a ContactManifold<N>)>, effective_only: bool)
                           -> impl Iterator<Item = (&'a Collider<N>, &'a Collider<N>, &'a ContactAlgorithm<N>, &'a ContactManifold<N>)> {
        iter.filter_map(move |inter| {
            let c1 = self.collider(inter.0)?;
            let c2 = self.collider(inter.1)?;
            if !effective_only || Self::is_contact_effective(c1, c2, inter.3) {
                Some((c1, c2, inter.2, inter.3))
            } else {
                None
            }
        })
    }

    #[inline(always)]
    fn filter_proximities<'a>(&'a self, iter: impl Iterator<Item = (ColliderHandle, ColliderHandle, &'a ProximityAlgorithm<N>)>)
                           -> impl Iterator<Item = (&'a Collider<N>, &'a Collider<N>, &'a ProximityAlgorithm<N>)> {
        iter.filter_map(move |prox| {
            Some((self.collider(prox.0)?, self.collider(prox.1)?, prox.2))
        })
    }

    // FIXME: we need to be careful with the notion of "effective_only" when dealing with
    // contacts. Perhaps we should filter out contacts with depths that are not considered
    // as actual contacts by the solver?

    /// All the potential interactions pairs.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interaction_pairs(&self, effective_only: bool) -> impl Iterator<Item = (
        &Collider<N>,
        &Collider<N>,
        &Interaction<N>
    )> {
        self.filter_interactions(self.cworld.interaction_pairs(false), effective_only)
    }


    /// All the potential contact pairs.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn contact_pairs(&self, effective_only: bool) -> impl Iterator<Item = (
        &Collider<N>,
        &Collider<N>,
        &ContactAlgorithm<N>,
        &ContactManifold<N>,
    )> {
        self.filter_contacts(self.cworld.contact_pairs(false), effective_only)
    }


    /// All the potential proximity pairs.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn proximity_pairs(&self, effective_only: bool) -> impl Iterator<Item = (
        &Collider<N>,
        &Collider<N>,
        &ProximityAlgorithm<N>,
    )> {
        self.filter_proximities(self.cworld.proximity_pairs(effective_only))
    }


    /// The potential interaction pair between the two specified colliders.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interaction_pair(&self, handle1: ColliderHandle, handle2: ColliderHandle, effective_only: bool)
                            -> Option<(&Collider<N>, &Collider<N>, &Interaction<N>)> {
        self.cworld.interaction_pair(handle1, handle2, false).and_then(move |inter| {
            let c1 = self.collider(inter.0)?;
            let c2 = self.collider(inter.1)?;
            if !effective_only || Self::is_interaction_effective(c1, c2, inter.2) {
                Some((c1, c2, inter.2))
            } else {
                None
            }
        })
    }

    /// The potential contact pair between the two specified colliders.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn contact_pair(&self, handle1: ColliderHandle, handle2: ColliderHandle, effective_only: bool)
                        -> Option<(&Collider<N>, &Collider<N>, &ContactAlgorithm<N>, &ContactManifold<N>)> {
        self.cworld.contact_pair(handle1, handle2, false).and_then(move |inter| {
            let c1 = self.collider(inter.0)?;
            let c2 = self.collider(inter.1)?;
            if !effective_only || Self::is_contact_effective(c1, c2, inter.3) {
                Some((c1, c2, inter.2, inter.3))
            } else {
                None
            }
        })
    }

    /// The potential proximity pair between the two specified colliders.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn proximity_pair(&self, handle1: ColliderHandle, handle2: ColliderHandle, effective_only: bool)
                          -> Option<(&Collider<N>, &Collider<N>, &ProximityAlgorithm<N>)> {
        self.cworld.proximity_pair(handle1, handle2, effective_only).and_then(move |prox| {
            Some((self.collider(prox.0)?, self.collider(prox.1)?, prox.2))
        })
    }


    /// All the interaction pairs involving the specified collider.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn interactions_with(&self, handle: ColliderHandle, effective_only: bool)
                             -> Option<impl Iterator<Item = (&Collider<N>, &Collider<N>, &Interaction<N>)>> {
        Some(self.filter_interactions(self.cworld.interactions_with(handle, false)?, effective_only))
    }

    /// All the contact pairs involving the specified collider.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn contacts_with(&self, handle: ColliderHandle, effective_only: bool)
                         -> Option<impl Iterator<Item = (&Collider<N>, &Collider<N>, &ContactAlgorithm<N>, &ContactManifold<N>)>> {
        Some(self.filter_contacts(self.cworld.contacts_with(handle, false)?, effective_only))
    }

    /// All the proximity pairs involving the specified collider.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn proximities_with(&self, handle: ColliderHandle, effective_only: bool)
                            -> Option<impl Iterator<Item = (&Collider<N>, &Collider<N>, &ProximityAlgorithm<N>)>> {
        Some(self.filter_proximities(self.cworld.proximities_with(handle, effective_only)?))
    }

    /// All the collider handles of colliders interacting with the specified collider.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn colliders_interacting_with<'a>(&'a self, handle: ColliderHandle) -> Option<impl Iterator<Item = &Collider<N>> + 'a> {
        Some(self.interactions_with(handle, true)?
            .map(move |(c1, c2, _)| {
                if c1.handle() == handle { c2 } else { c1 }
            }))
    }

    /// All the collider handles of colliders in potential contact with the specified collision
    /// object.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn colliders_in_contact_with<'a>(&'a self, handle: ColliderHandle) -> Option<impl Iterator<Item = &Collider<N>> + 'a> {
        Some(self.contacts_with(handle, true)?
            .map(move |(c1, c2, _, _)| {
                if c1.handle() == handle { c2 } else { c1 }
            }))
    }

    /// All the collider handles of colliders in potential proximity of with the specified
    /// collider.
    ///
    /// Refer to the official [user guide](https://nphysics.org/interaction_handling_and_sensors/#interaction-iterators)
    /// for details.
    pub fn colliders_in_proximity_of<'a>(&'a self, handle: ColliderHandle) -> Option<impl Iterator<Item = &Collider<N>> + 'a> {
        Some(self.proximities_with(handle, true)?
            .map(move |(c1, c2, _)| {
                if c1.handle() == handle { c2 } else { c1 }
            }))
    }
}

struct BodyStatusCollisionFilter;

impl<N: Real> BroadPhasePairFilter<N, ColliderData<N>> for BodyStatusCollisionFilter {
    /// Activate an action for when two objects start or stop to be close to each other.
    fn is_pair_valid(&self, b1: &CollisionObject<N, ColliderData<N>>, b2: &CollisionObject<N, ColliderData<N>>) -> bool {
        b1.data().body_status_dependent_ndofs() != 0 || b2.data().body_status_dependent_ndofs() != 0
    }
}

pub struct ColliderChain<'a, N: Real> {
    cworld: &'a ColliderWorld<N>,
    curr: Option<ColliderHandle>,
}

impl<'a, N: Real> Iterator for ColliderChain<'a, N> {
    type Item = &'a Collider<N>;

    fn next(&mut self) -> Option<&'a Collider<N>> {
        let coll = self.cworld.collider(self.curr?)?;
        self.curr = coll.next();
        Some(coll)
    }
}
