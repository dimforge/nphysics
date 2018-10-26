use slab::{Iter, IterMut, Slab};
use std::iter::Map;

use joint::Joint;
use math::{Inertia, Isometry, Point, Vector};
use na::Real;
use object::{
    Body, BodyMut, BodyPart, BodyPartMut, Ground, Multibody, MultibodyLinkId, MultibodyLinkMut,
    MultibodyLinkRef, MultibodyWorkspace, RigidBody,
};
use solver::IntegrationParameters;

// FIXME: remove the pub(crate) after this is replaced by -> impl Iterator
pub(crate) type RigidBodies<'a, N> =
    Map<Iter<'a, RigidBody<N>>, fn((usize, &RigidBody<N>)) -> &RigidBody<N>>;
pub(crate) type RigidBodiesMut<'a, N> =
    Map<IterMut<'a, RigidBody<N>>, fn((usize, &mut RigidBody<N>)) -> &mut RigidBody<N>>;
pub(crate) type Multibodies<'a, N> =
    Map<Iter<'a, Multibody<N>>, fn((usize, &Multibody<N>)) -> &Multibody<N>>;
pub(crate) type MultibodiesMut<'a, N> =
    Map<IterMut<'a, Multibody<N>>, fn((usize, &mut Multibody<N>)) -> &mut Multibody<N>>;

/// A unique identifier of a body added to the world.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct BodyHandle {
    handle: usize,
    reserved: usize, // NOTE: reserved for future use (e.g. for soft bodies).
}

impl BodyHandle {
    pub(crate) fn new(handle: usize) -> Self {
        BodyHandle {
            handle: handle,
            reserved: 0,
        }
    }

    /// The unique identifier of the ground.
    pub fn ground() -> Self {
        Self::new(usize::max_value())
    }

    /// Tests if this handle corresponds to the ground.
    pub fn is_ground(&self) -> bool {
        self.handle == usize::max_value()
    }
}

// FIXME: remove the pub(crate) after this is replaced by -> impl Iterator
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum BodyId {
    MultibodyLinkId(usize, MultibodyLinkId),
    RigidBodyId(usize),
}

impl BodyId {
    #[inline]
    pub fn is_same_body(&self, other: BodyId) -> bool {
        match (*self, other) {
            (BodyId::MultibodyLinkId(id1, _), BodyId::MultibodyLinkId(id2, _)) => id1 == id2,
            (BodyId::RigidBodyId(id1), BodyId::RigidBodyId(id2)) => id1 == id2,
            _ => false,
        }
    }
}

/// A set containing all the bodies added to the world.
pub struct BodySet<N: Real> {
    ground: Ground<N>,
    ids: Slab<BodyId>,
    mbs: Slab<Multibody<N>>,
    rbs: Slab<RigidBody<N>>,
}

impl<N: Real> BodySet<N> {
    /// Create a new empty set of bodies.
    pub fn new() -> Self {
        BodySet {
            ground: Ground::new(),
            ids: Slab::new(),
            mbs: Slab::new(),
            rbs: Slab::new(),
        }
    }

    /// The number of bodies in this set.
    pub fn len(&self) -> usize {
        self.mbs.len() + self.rbs.len()
    }

    /// Check if the two given handles identify the same body.
    ///
    /// In particular, returns `true` if both handles indentify multibody links belonging
    /// to the same multibody.
    pub fn are_same_body(&self, body1: BodyHandle, body2: BodyHandle) -> bool {
        if body1.handle == body2.handle {
            return true;
        }

        let ids = (self.ids[body1.handle], self.ids[body2.handle]);
        if let (BodyId::MultibodyLinkId(mb1, _), BodyId::MultibodyLinkId(mb2, _)) = ids {
            return mb1 == mb2;
        }

        false
    }

    /// Update the kinematics of all the bodies.
    pub fn update_kinematics(&mut self) {
        for (_, mb) in &mut self.mbs {
            mb.update_kinematics();
        }
    }

    /// Clear the dynamics of all the bodies.
    pub fn clear_dynamics(&mut self) {
        for (_, mb) in &mut self.mbs {
            mb.clear_dynamics();
        }

        for (_, rb) in &mut self.rbs {
            rb.clear_dynamics()
        }
    }

    /// Update the dynamics of all the bodies.
    pub fn update_dynamics(
        &mut self,
        gravity: &Vector<N>,
        params: &IntegrationParameters<N>,
        workspace: &mut MultibodyWorkspace<N>,
    ) {
        for (_, mb) in &mut self.mbs {
            mb.update_dynamics(gravity, params, workspace);
        }

        for (_, rb) in &mut self.rbs {
            rb.update_dynamics(gravity, params)
        }
    }

    /// Add a rigid body to the set and return its handle.
    pub fn add_rigid_body(
        &mut self,
        position: Isometry<N>,
        local_inertia: Inertia<N>,
        local_com: Point<N>,
    ) -> BodyHandle {
        let rb_entry = self.rbs.vacant_entry();
        let rb_id = rb_entry.key();
        let rb_handle = BodyHandle::new(self.ids.insert(BodyId::RigidBodyId(rb_id)));

        let _ = rb_entry.insert(RigidBody::new(
            rb_handle,
            position,
            local_inertia,
            local_com,
        ));
        rb_handle
    }

    /// Add a multibody link to the set and return its handle.
    pub fn add_multibody_link<J: Joint<N>>(
        &mut self,
        parent: BodyHandle,
        joint: J,
        parent_shift: Vector<N>,
        body_shift: Vector<N>,
        local_inertia: Inertia<N>,
        local_com: Point<N>,
    ) -> BodyHandle {
        if parent.is_ground() {
            let mb_entry = self.mbs.vacant_entry();
            let mb_id = mb_entry.key();
            let mb_handle = BodyHandle::new(
                self.ids
                    .insert(BodyId::MultibodyLinkId(mb_id, MultibodyLinkId::ground())),
            );
            let mb = mb_entry.insert(Multibody::new());
            let id = mb
                .add_link(
                    mb_handle,
                    MultibodyLinkId::ground(),
                    joint,
                    parent_shift,
                    body_shift,
                    local_inertia,
                    local_com,
                )
                .id();
            self.ids[mb_handle.handle] = BodyId::MultibodyLinkId(mb_id, id);
            mb_handle
        } else {
            if let BodyId::MultibodyLinkId(mb_id, parent_id) = self.ids[parent.handle] {
                let mb_handle = BodyHandle::new(
                    self.ids
                        .insert(BodyId::MultibodyLinkId(mb_id, MultibodyLinkId::ground())),
                );
                let id = self.mbs[mb_id]
                    .add_link(
                        mb_handle,
                        parent_id,
                        joint,
                        parent_shift,
                        body_shift,
                        local_inertia,
                        local_com,
                    )
                    .id();
                self.ids[mb_handle.handle] = BodyId::MultibodyLinkId(mb_id, id);
                mb_handle
            } else {
                panic!("Attempting to attach a multibody link to a body that is neither a link nor the ground.")
            }
        }
    }

    /// Remove a body from this set.
    ///
    /// If `body` identify a mutibody link, the whole multibody is removed.
    pub fn remove_body(&mut self, body: BodyHandle) {
        if !body.is_ground() {
            let body_id = self.ids[body.handle];

            match body_id {
                BodyId::MultibodyLinkId(id, _) => {
                    let _ = self.mbs.remove(id);
                }
                BodyId::RigidBodyId(id) => {
                    let _ = self.rbs.remove(id);
                }
            }

            self.ids.retain(|_, id| !id.is_same_body(body_id));
        }
    }

    /// Remove some multibody links.
    ///
    /// If a multibody link which has descendent is removed, its immediat descendent in
    /// the kinematic tree becomes the root of a new multibody and its joint is replaced by
    /// a free joint attached to the ground.
    pub fn remove_multibody_links(&mut self, body_parts: &[BodyHandle]) {
        if body_parts.len() != 0 {
            if let BodyId::MultibodyLinkId(parent_id, _) = self.ids[body_parts[0].handle] {
                let mb = self.mbs.remove(parent_id);
                let mut links = Vec::with_capacity(body_parts.len());

                for part in body_parts {
                    if let BodyId::MultibodyLinkId(mb_id, id) = self.ids.remove(part.handle) {
                        if mb_id != parent_id {
                            panic!("Multibody link removal: all multibody link must belong to the same multibody.")
                        }

                        links.push(id);
                    } else {
                        panic!("Multibody link removal: all body must indentify a multibody link.")
                    }
                }

                let mbs = mb.remove_links(&links);

                // Update the links identifiers.
                for mb in mbs {
                    let mb_key = self.mbs.insert(mb);

                    for link in self.mbs[mb_key].links() {
                        self.ids[link.handle().handle] = BodyId::MultibodyLinkId(mb_key, link.id());
                    }
                }
            } else {
                panic!("Multibody link removal: all body must indentify a multibody link.")
            }
        }
    }

    /// Checks that the given handle identifies a valid body part.
    ///
    /// Returns `true` if `body.is_ground()` too.
    pub fn contains(&self, body: BodyHandle) -> bool {
        // FIXME: do we have to take body.reserved into account?
        body.is_ground() || self.ids.contains(body.handle)
    }

    /// Reference to the body identified by `body`.
    ///
    /// Panics if the body part is not found.
    #[inline]
    pub fn body(&self, body: BodyHandle) -> Body<N> {
        if body.is_ground() {
            Body::Ground(&self.ground)
        } else {
            match self.ids[body.handle] {
                BodyId::MultibodyLinkId(mb_id, _) => Body::Multibody(&self.mbs[mb_id]),
                BodyId::RigidBodyId(id) => Body::RigidBody(&self.rbs[id]),
            }
        }
    }

    /// Mutable reference to the body identified by `body`.
    ///
    /// Panics if the body part is not found.
    #[inline]
    pub fn body_mut(&mut self, body: BodyHandle) -> BodyMut<N> {
        if body.is_ground() {
            BodyMut::Ground(&mut self.ground)
        } else {
            match self.ids[body.handle] {
                BodyId::MultibodyLinkId(mb_id, _) => BodyMut::Multibody(&mut self.mbs[mb_id]),
                BodyId::RigidBodyId(id) => BodyMut::RigidBody(&mut self.rbs[id]),
            }
        }
    }

    /// Reference to the body part identified by `body`.
    ///
    /// Panics if the body part is not found.
    #[inline]
    pub fn body_part(&self, body: BodyHandle) -> BodyPart<N> {
        if body.is_ground() {
            BodyPart::Ground(&self.ground)
        } else {
            match self.ids[body.handle] {
                BodyId::MultibodyLinkId(mb_id, link_id) => {
                    BodyPart::MultibodyLink(self.mbs[mb_id].link(link_id))
                }
                BodyId::RigidBodyId(id) => BodyPart::RigidBody(&self.rbs[id]),
            }
        }
    }

    /// Mutable reference to the body part identified by `body`.
    ///
    /// Panics if the body part is not found.
    #[inline]
    pub fn body_part_mut(&mut self, body: BodyHandle) -> BodyPartMut<N> {
        if body.is_ground() {
            BodyPartMut::Ground(&mut self.ground)
        } else {
            match self.ids[body.handle] {
                BodyId::MultibodyLinkId(mb_id, link_id) => {
                    BodyPartMut::MultibodyLink(self.mbs[mb_id].link_mut(link_id))
                }
                BodyId::RigidBodyId(id) => BodyPartMut::RigidBody(&mut self.rbs[id]),
            }
        }
    }

    /// Reference to the multibody containing the multibody link identified by `body`.
    ///
    /// Returns `None` if it is not found or does not identify a multibody link.
    #[inline]
    pub fn multibody(&self, body: BodyHandle) -> Option<&Multibody<N>> {
        if let Some(&BodyId::MultibodyLinkId(mb_id, _)) = self.ids.get(body.handle) {
            Some(&self.mbs[mb_id])
        } else {
            None
        }
    }

    /// Mutable reference to the multibody containing the multibody link identified by `body`.
    ///
    /// Returns `None` if it is not found or does not identify a multibody link.
    #[inline]
    pub fn multibody_mut(&mut self, body: BodyHandle) -> Option<&mut Multibody<N>> {
        if let Some(&BodyId::MultibodyLinkId(mb_id, _)) = self.ids.get(body.handle) {
            Some(&mut self.mbs[mb_id])
        } else {
            None
        }
    }

    /// Reference to the multibody link identified by `body`.
    ///
    /// Returns `None` if it is not found or does not identify a multibody link.
    #[inline]
    pub fn multibody_link(&self, body: BodyHandle) -> Option<MultibodyLinkRef<N>> {
        if let Some(&BodyId::MultibodyLinkId(mb_id, link_id)) = self.ids.get(body.handle) {
            Some(self.mbs[mb_id].link(link_id))
        } else {
            None
        }
    }

    /// Mutable reference to the multibody link identified by `body`.
    ///
    /// Returns `None` if it is not found or does not identify a multibody link.
    #[inline]
    pub fn multibody_link_mut(&mut self, body: BodyHandle) -> Option<MultibodyLinkMut<N>> {
        if let Some(&BodyId::MultibodyLinkId(mb_id, link_id)) = self.ids.get(body.handle) {
            Some(self.mbs[mb_id].link_mut(link_id))
        } else {
            None
        }
    }

    /// Reference to the rigid body identified by `body`.
    ///
    /// Returns `None` if it is not found or does not identify a rigid body.
    #[inline]
    pub fn rigid_body(&self, body: BodyHandle) -> Option<&RigidBody<N>> {
        if let Some(&BodyId::RigidBodyId(id)) = self.ids.get(body.handle) {
            Some(&self.rbs[id])
        } else {
            None
        }
    }

    /// Mutable reference to the rigid body identified by `body`.
    ///
    /// Returns `None` if it is not found or does not identify a rigid body.
    #[inline]
    pub fn rigid_body_mut(&mut self, body: BodyHandle) -> Option<&mut RigidBody<N>> {
        if let Some(&BodyId::RigidBodyId(id)) = self.ids.get(body.handle) {
            Some(&mut self.rbs[id])
        } else {
            None
        }
    }

    /// Iterator yielding all the rigid bodies on this set.
    #[inline]
    pub fn rigid_bodies(&self) -> RigidBodies<N> {
        self.rbs.iter().map(|e| e.1)
    }

    /// Mutable iterator yielding all the rigid bodies on this set.
    #[inline]
    pub fn rigid_bodies_mut(&mut self) -> RigidBodiesMut<N> {
        self.rbs.iter_mut().map(|e| e.1)
    }

    /// Iterator yielding all the multibodies on this set.
    #[inline]
    pub fn multibodies(&self) -> Multibodies<N> {
        self.mbs.iter().map(|e| e.1)
    }

    /// Mutable iterator yielding all the multibodies on this set.
    #[inline]
    pub fn multibodies_mut(&mut self) -> MultibodiesMut<N> {
        self.mbs.iter_mut().map(|e| e.1)
    }

    /*
    #[inline]
    pub fn bodies(&self) -> Bodies<N> {
        Bodies {
            rbs_iter: self.rbs.iter(),
            mbs_iter: self.mbs.iter()
        }
    }
    */

    /// Mutable iterator yielding all the bodies on this set.
    #[inline]
    pub fn bodies_mut(&mut self) -> BodiesMut<N> {
        BodiesMut {
            rbs_iter: self.rbs.iter_mut(),
            mbs_iter: self.mbs.iter_mut(),
        }
    }
}

/// Iterator yielding all the bodies on a body set.
pub struct Bodies<'a, N: Real> {
    rbs_iter: Iter<'a, RigidBody<N>>,
    mbs_iter: Iter<'a, Multibody<N>>,
}

impl<'a, N: Real> Iterator for Bodies<'a, N> {
    type Item = Body<'a, N>;

    #[inline]
    fn next(&mut self) -> Option<Body<'a, N>> {
        if let Some(res) = self.rbs_iter.next() {
            return Some(Body::RigidBody(res.1));
        }

        if let Some(res) = self.mbs_iter.next() {
            return Some(Body::Multibody(res.1));
        }

        return None;
    }
}

/// Mutable iterator yielding all the bodies on a body set.
pub struct BodiesMut<'a, N: Real> {
    rbs_iter: IterMut<'a, RigidBody<N>>,
    mbs_iter: IterMut<'a, Multibody<N>>,
}

impl<'a, N: Real> Iterator for BodiesMut<'a, N> {
    type Item = BodyMut<'a, N>;

    #[inline]
    fn next(&mut self) -> Option<BodyMut<'a, N>> {
        if let Some(res) = self.rbs_iter.next() {
            Some(BodyMut::RigidBody(res.1))
        } else if let Some(res) = self.mbs_iter.next() {
            Some(BodyMut::Multibody(res.1))
        } else {
            None
        }
    }
}
