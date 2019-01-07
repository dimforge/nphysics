use either::Either;
use na::Real;
use ncollide::world::{CollisionWorld, GeometricQueryType, CollisionGroups,
                      CollisionObject, CollisionObjects, InterferencesWithRay,
                      InterferencesWithPoint, InterferencesWithAABB};
use ncollide::broad_phase::BroadPhasePairFilter;
use ncollide::narrow_phase::{NarrowPhase, ContactAlgorithm, ContactPairs, ProximityPairs, ProximityAlgorithm};
use ncollide::query::{ContactManifold, Ray, RayIntersection};
use ncollide::shape::ShapeHandle;
use ncollide::bounding_volume::AABB;
use ncollide::events::{ContactEvents, ProximityEvents};

use crate::object::{Collider, ColliderData, ColliderHandle, ColliderAnchor, BodySet};
use crate::math::{Isometry, Point};

pub struct ColliderWorld<N: Real> {
    cworld: CollisionWorld<N, ColliderData<N>>,
    colliders_w_parent: Vec<ColliderHandle>,
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
            colliders_w_parent: Vec::new()
        }
    }

    pub fn sync_colliders(&mut self, bodies: &BodySet<N>) {
        let cworld = &mut self.cworld;
        self.colliders_w_parent.retain(|collider_id| {
            // FIXME: update only if the position changed (especially for static bodies).
            let collider = try_ret!(cworld.collision_object_mut(*collider_id), false);
            let body = try_ret!(bodies.body(collider.data().body()), false);

            collider
                .data_mut()
                .set_body_status_dependent_ndofs(body.status_dependent_ndofs());

            if !body.is_active() {
                return true;
            }

            let new_pos = match collider.data().anchor() {
                ColliderAnchor::OnBodyPart { body_part, position_wrt_body_part } => {
                    let part = try_ret!(body.part(body_part.1), false);
                    let part_pos = part.position();
                    Either::Left(part_pos * position_wrt_body_part)
                }
                ColliderAnchor::OnDeformableBody { indices, .. } => {
                    // FIXME: too bad we have to clone the indices here
                    // (that's why this is an arc) to avoid borrowing issue.
                    Either::Right(indices.clone())
                }
            };

            match new_pos {
                Either::Left(pos) => cworld.set_position(*collider_id, pos),
                Either::Right(indices) => cworld.set_deformations(*collider_id, body.deformed_positions().unwrap().1, indices.as_ref().map(|idx| &idx[..]))
            }

            true
        });
    }

    fn as_collision_world(&self) -> &CollisionWorld<N, ColliderData<N>> {
        &self.cworld
    }

    fn into_inner(self) -> CollisionWorld<N, ColliderData<N>> {
        self.cworld
    }

    /// Adds a collider to the world.
    pub fn add(
        &mut self,
        position: Isometry<N>,
        shape: ShapeHandle<N>,
        collision_groups: CollisionGroups,
        query_type: GeometricQueryType<N>,
        data: ColliderData<N>,
    ) -> &mut Collider<N>
    {
        let co = self.cworld.add(position, shape, collision_groups, query_type, data);

        if !co.data().body().is_ground() {
            self.colliders_w_parent.push(co.handle());
        }

        Collider::from_mut(co)
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
    pub fn remove(&mut self, handles: &[ColliderHandle]) {
        self.cworld.remove(handles)
    }

    /// Sets the position the collider attached to the specified object.
    pub fn set_position(&mut self, handle: ColliderHandle, pos: Isometry<N>) {
        self.cworld.set_position(handle, pos)
    }

    /// Apply the given deformations to the specified object.
    pub fn set_deformations(
        &mut self,
        handle: ColliderHandle,
        coords: &[N],
        indices: Option<&[usize]>,
    )
    {
        self.cworld.set_deformations(handle, coords, indices)
    }

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

    /// Sets a new narrow phase and returns the previous one.
    ///
    /// Keep in mind that modifying the narrow-pase will have a non-trivial overhead during the
    /// next update as it will force re-detection of all collision pairs and their associated
    /// contacts.
    pub fn set_narrow_phase(
        &mut self,
        narrow_phase: Box<NarrowPhase<N, ColliderData<N>>>,
    ) -> Box<NarrowPhase<N, ColliderData<N>>>
    {
        self.cworld.set_narrow_phase(narrow_phase)
    }

    /// The contact pair, if any, between the given colliders.
    #[inline]
    pub fn contact_pair(
        &self,
        handle1: ColliderHandle,
        handle2: ColliderHandle,
    ) -> Option<(&ContactAlgorithm<N>, &ContactManifold<N>)>
    {
        self.cworld.contact_pair(handle1, handle2)
    }

    /// Iterates through all the contact pairs detected since the last update.
    #[inline]
    pub fn contact_pairs(&self)
        -> impl Iterator<Item = (
            &Collider<N>,
            &Collider<N>,
            &ContactAlgorithm<N>,
            &ContactManifold<N>,
        )> {
        self.cworld.contact_pairs().map(|p| (Collider::from_ref(p.0), Collider::from_ref(p.1), p.2, p.3))
    }

    /// Iterates through all the proximity pairs detected since the last update.
    #[inline]
    pub fn proximity_pairs(&self)
        -> impl Iterator<Item = (
            &Collider<N>,
            &Collider<N>,
            &ProximityAlgorithm<N>,
        )> {
        self.cworld.proximity_pairs().map(|p| (Collider::from_ref(p.0), Collider::from_ref(p.1), p.2))
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
    pub fn collider_mut(
        &mut self,
        handle: ColliderHandle,
    ) -> Option<&mut Collider<N>>
    {
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
}

struct BodyStatusCollisionFilter;

impl<N: Real> BroadPhasePairFilter<N, ColliderData<N>> for BodyStatusCollisionFilter {
    /// Activate an action for when two objects start or stop to be close to each other.
    fn is_pair_valid(&self, b1: &CollisionObject<N, ColliderData<N>>, b2: &CollisionObject<N, ColliderData<N>>) -> bool {
        b1.data().body_status_dependent_ndofs() != 0 || b2.data().body_status_dependent_ndofs() != 0
    }
}