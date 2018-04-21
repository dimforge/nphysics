use std::iter::Map;
use slab::{Iter, IterMut, Slab};

use na::Real;
use object::{Body, BodyMut, BodyPart, BodyPartMut, Ground, Multibody, MultibodyLinkId,
             MultibodyLinkMut, MultibodyLinkRef, MultibodyWorkspace, RigidBody};
use joint::Joint;
use solver::IntegrationParameters;
use math::{Inertia, Isometry, Point, Vector};

// FIXME: remove the pub(crate) after this is replaced by -> impl Iterator
pub(crate) type RigidBodies<'a, N> =
    Map<Iter<'a, RigidBody<N>>, fn((usize, &RigidBody<N>)) -> &RigidBody<N>>;
pub(crate) type RigidBodiesMut<'a, N> =
    Map<IterMut<'a, RigidBody<N>>, fn((usize, &mut RigidBody<N>)) -> &mut RigidBody<N>>;
pub(crate) type Multibodies<'a, N> =
    Map<Iter<'a, Multibody<N>>, fn((usize, &Multibody<N>)) -> &Multibody<N>>;
pub(crate) type MultibodiesMut<'a, N> =
    Map<IterMut<'a, Multibody<N>>, fn((usize, &mut Multibody<N>)) -> &mut Multibody<N>>;

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

    pub fn ground() -> Self {
        Self::new(usize::max_value())
    }

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

pub struct BodySet<N: Real> {
    ground: Ground<N>,
    ids: Slab<BodyId>,
    mbs: Slab<Multibody<N>>,
    rbs: Slab<RigidBody<N>>,
}

impl<N: Real> BodySet<N> {
    pub fn new() -> Self {
        BodySet {
            ground: Ground::new(),
            ids: Slab::new(),
            mbs: Slab::new(),
            rbs: Slab::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.mbs.len() + self.rbs.len()
    }

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

    pub fn update_kinematics(&mut self) {
        for (_, mb) in &mut self.mbs {
            mb.update_kinematics();
        }
    }

    pub fn clear_dynamics(&mut self) {
        for (_, mb) in &mut self.mbs {
            mb.clear_dynamics();
        }

        for (_, rb) in &mut self.rbs {
            rb.clear_dynamics()
        }
    }

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
            let id = mb.add_link(
                mb_handle,
                MultibodyLinkId::ground(),
                joint,
                parent_shift,
                body_shift,
                local_inertia,
                local_com,
            ).id();
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

    pub fn contains(&self, body: BodyHandle) -> bool {
        // FIXME: do we have to take body.reserved into account?
        body.is_ground() || self.ids.contains(body.handle)
    }

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

    #[inline]
    pub fn multibody(&self, body: BodyHandle) -> Option<&Multibody<N>> {
        if let Some(&BodyId::MultibodyLinkId(mb_id, _)) = self.ids.get(body.handle) {
            Some(&self.mbs[mb_id])
        } else {
            None
        }
    }

    #[inline]
    pub fn multibody_mut(&mut self, body: BodyHandle) -> Option<&mut Multibody<N>> {
        if let Some(&BodyId::MultibodyLinkId(mb_id, _)) = self.ids.get(body.handle) {
            Some(&mut self.mbs[mb_id])
        } else {
            None
        }
    }

    #[inline]
    pub fn multibody_link(&self, body: BodyHandle) -> Option<MultibodyLinkRef<N>> {
        if let Some(&BodyId::MultibodyLinkId(mb_id, link_id)) = self.ids.get(body.handle) {
            Some(self.mbs[mb_id].link(link_id))
        } else {
            None
        }
    }

    #[inline]
    pub fn multibody_link_mut(&mut self, body: BodyHandle) -> Option<MultibodyLinkMut<N>> {
        if let Some(&BodyId::MultibodyLinkId(mb_id, link_id)) = self.ids.get(body.handle) {
            Some(self.mbs[mb_id].link_mut(link_id))
        } else {
            None
        }
    }

    #[inline]
    pub fn rigid_body(&self, body: BodyHandle) -> Option<&RigidBody<N>> {
        if let Some(&BodyId::RigidBodyId(id)) = self.ids.get(body.handle) {
            Some(&self.rbs[id])
        } else {
            None
        }
    }

    #[inline]
    pub fn rigid_body_mut(&mut self, body: BodyHandle) -> Option<&mut RigidBody<N>> {
        if let Some(&BodyId::RigidBodyId(id)) = self.ids.get(body.handle) {
            Some(&mut self.rbs[id])
        } else {
            None
        }
    }

    #[inline]
    pub fn rigid_bodies(&self) -> RigidBodies<N> {
        self.rbs.iter().map(|e| e.1)
    }

    #[inline]
    pub fn rigid_bodies_mut(&mut self) -> RigidBodiesMut<N> {
        self.rbs.iter_mut().map(|e| e.1)
    }

    #[inline]
    pub fn multibodies(&self) -> Multibodies<N> {
        self.mbs.iter().map(|e| e.1)
    }

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

    #[inline]
    pub fn bodies_mut(&mut self) -> BodiesMut<N> {
        BodiesMut {
            rbs_iter: self.rbs.iter_mut(),
            mbs_iter: self.mbs.iter_mut(),
        }
    }
}

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

pub struct BodiesMut<'a, N: Real> {
    rbs_iter: IterMut<'a, RigidBody<N>>,
    mbs_iter: IterMut<'a, Multibody<N>>,
}

impl<'a, N: Real> Iterator for BodiesMut<'a, N> {
    type Item = BodyMut<'a, N>;

    #[inline]
    fn next(&mut self) -> Option<BodyMut<'a, N>> {
        if let Some(res) = self.rbs_iter.next() {
            return Some(BodyMut::RigidBody(res.1));
        }

        if let Some(res) = self.mbs_iter.next() {
            return Some(BodyMut::Multibody(res.1));
        }

        return None;
    }
}
