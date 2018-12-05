use slab::{Iter, IterMut, Slab};
use std::iter::Map;

use joint::Joint;
use math::{Inertia, Isometry, Point, Vector};
use na::Real;
use object::{Body, BodyPart, Ground, Multibody,
             MultibodyLink, MultibodyWorkspace, RigidBody};
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

/// A world-specific body handle.
///
/// This structure is automatically allocated by the physics world.
/// It cannot be constructed by the end-user.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct BodyHandle(BodyVariant);

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
enum BodyVariant {
    Ground,
    RigidBody(usize),
    Multibody(usize),
    AbstractBody(usize),
}

/// A unique identifier of a body part added to the world.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct BodyPartHandle {
    /// The handle of the body this body part belongs to.
    pub body_handle: BodyHandle,
    /// The body-dependent identifier of this body part.
    pub part_id: usize,
}

impl BodyHandle {
    /// The body handle of the ground.
    #[inline]
    pub fn ground() -> Self {
        BodyHandle(BodyVariant::Ground)
    }

    /// Tests if this handle corresponds to the ground.
    #[inline]
    pub fn is_ground(&self) -> bool {
        self.0 == BodyVariant::Ground
    }
}

impl BodyPartHandle {
    /// Initializes a body handle.
    pub fn new(body_handle: BodyHandle, part_id: usize) -> Self {
        BodyPartHandle {
            body_handle,
            part_id,
        }
    }

    /// The body part handle of the ground.
    pub fn ground() -> Self {
        BodyPartHandle::new(BodyHandle(BodyVariant::Ground), 0)
    }

    /// Tests if this handle corresponds to the ground.
    pub fn is_ground(&self) -> bool {
        self.body_handle.0 == BodyVariant::Ground
    }
}

/// A set containing all the bodies added to the world.
pub struct BodySet<N: Real> {
    ground: Ground<N>,
    rbs: Slab<RigidBody<N>>,
    mbs: Slab<Multibody<N>>,
    bodies: Slab<Box<Body<N>>>,
}

impl<N: Real> BodySet<N> {
    /// Create a new empty set of bodies.
    pub fn new() -> Self {
        BodySet {
            ground: Ground::new(),
            rbs: Slab::new(),
            mbs: Slab::new(),
            bodies: Slab::new(),
        }
    }

    /// The number of bodies in this set.
    pub fn len(&self) -> usize {
        self.rbs.len() + self.mbs.len() + self.bodies.len()
    }

    /// Update the kinematics of all the bodies.
    pub fn update_kinematics(&mut self) {
        for (_, mb) in &mut self.mbs {
            mb.update_kinematics();
        }
        for (_, b) in &mut self.bodies {
            b.update_kinematics();
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

        for (_, b) in &mut self.bodies {
            b.clear_dynamics()
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
            mb.update_dynamics2(gravity, params, workspace); // XXX
        }

        for (_, rb) in &mut self.rbs {
            rb.update_dynamics(gravity, params)
        }

        for (_, b) in &mut self.bodies {
            b.update_dynamics(gravity, params)
        }
    }

    /// Adds a body to the world.
    pub fn add_body(&mut self, mut body: Box<Body<N>>) -> BodyHandle {
        let b_entry = self.bodies.vacant_entry();
        let b_id = b_entry.key();
        let handle = BodyHandle(BodyVariant::AbstractBody(b_id));
        body.set_handle(Some(handle));
        let _ = b_entry.insert(body);

        handle
    }

    /// Add a rigid body to the set and return its handle.
    pub fn add_rigid_body(
        &mut self,
        position: Isometry<N>,
        local_inertia: Inertia<N>,
        local_com: Point<N>,
    ) -> BodyPartHandle {
        let rb_entry = self.rbs.vacant_entry();
        let rb_id = rb_entry.key();
        let handle = BodyHandle(BodyVariant::RigidBody(rb_id));
        let mut rb = RigidBody::new(
            position,
            local_inertia,
            local_com,
        );
        rb.set_handle(Some(handle));
        let _ = rb_entry.insert(rb);

        BodyPartHandle::new(handle, 0)
    }

    /// Add a multibody link to the set and return its handle.
    pub fn add_multibody_link<J: Joint<N>>(
        &mut self,
        parent: BodyPartHandle,
        joint: J,
        parent_shift: Vector<N>,
        body_shift: Vector<N>,
        local_inertia: Inertia<N>,
        local_com: Point<N>,
    ) -> BodyPartHandle {
        if parent.is_ground() {
            let mb_entry = self.mbs.vacant_entry();
            let mb_id = mb_entry.key();
            let mb = mb_entry.insert(Multibody::new());
            mb.set_handle(Some(BodyHandle(BodyVariant::Multibody(mb_id))));

            mb.add_link(
                BodyPartHandle::ground(),
                joint,
                parent_shift,
                body_shift,
                local_inertia,
                local_com,
            ).handle().unwrap()
        } else {
            if let BodyVariant::Multibody(parent_id) = parent.body_handle.0 {
                self.mbs[parent_id]
                    .add_link(
                        parent,
                        joint,
                        parent_shift,
                        body_shift,
                        local_inertia,
                        local_com,
                    ).handle().unwrap()
            } else {
                panic!("Attempting to attach a multibody link to a body that is neither a link nor the ground.")
            }
        }
    }

    /// Remove a body from this set.
    ///
    /// If `body` identify a mutibody link, the whole multibody is removed.
    pub fn remove_body(&mut self, body: BodyHandle) {
        match body.0 {
            BodyVariant::RigidBody(id) => {
                let _ = self.rbs.remove(id);
            }
            BodyVariant::Multibody(id) => {
                let _ = self.mbs.remove(id);
            }
            BodyVariant::AbstractBody(id) => {
                let _ = self.bodies.remove(id);
            }
            BodyVariant::Ground => {}
        }
    }

    /// Remove some multibody links.
    ///
    /// If a multibody link which has descendant is removed, its immediate descendent in
    /// the kinematic tree becomes the root of a new multibody and its joint is replaced by
    /// a free joint attached to the ground.
    pub fn remove_multibody_links(&mut self, link_handles: &[BodyPartHandle]) {
        if link_handles.len() != 0 {
            if let BodyVariant::Multibody(parent_id) = link_handles[0].body_handle.0 {
                let mut mb = self.mbs.remove(parent_id);
                mb.set_handle(None);
                let mbs = mb.remove_links(&link_handles);

                // Update the links identifiers.
                for mb in mbs {
                    let id = self.mbs.insert(mb);
                    self.mbs[id].set_handle(Some(BodyHandle(BodyVariant::Multibody(id))));
                }
            } else {
                panic!("Multibody link removal: all body must indentify a multibody link.")
            }
        }
    }

    /// Checks that the given handle identifies a valid body.
    ///
    /// Returns `true` if `handle.is_ground()` too.
    pub fn contains_body(&self, handle: BodyHandle) -> bool {
        match handle.0 {
            BodyVariant::Ground => true,
            BodyVariant::RigidBody(id) => self.rbs.contains(id),
            BodyVariant::Multibody(id) => self.mbs.contains(id),
            BodyVariant::AbstractBody(id) => self.bodies.contains(id),
        }
    }

    /// Checks that the given handle identifies a valid body part.
    ///
    /// Returns `true` if `handle.is_ground()` too.
    pub fn contains_body_part(&self, handle: BodyPartHandle) -> bool {
        match handle.body_handle.0 {
            BodyVariant::Ground => true,
            BodyVariant::RigidBody(id) => self.rbs.contains(id),
            BodyVariant::Multibody(id) => self.mbs.contains(id) && self.mbs[id].contains_part(handle),
            BodyVariant::AbstractBody(id) => self.bodies.contains(id) && self.bodies[id].contains_part(handle)
        }
    }

    /// Reference to the body identified by `body`.
    ///
    /// Panics if the body is not found.
    #[inline]
    pub fn body(&self, handle: BodyHandle) -> &Body<N> {
        match handle.0 {
            BodyVariant::Ground => &self.ground,
            BodyVariant::RigidBody(id) => &self.rbs[id],
            BodyVariant::Multibody(id) => &self.mbs[id],
            BodyVariant::AbstractBody(id) => &*self.bodies[id],
        }
    }

    /// Mutable reference to the body identified by `body`.
    ///
    /// Panics if the body is not found.
    #[inline]
    pub fn body_mut(&mut self, handle: BodyHandle) -> &mut Body<N> {
        match handle.0 {
            BodyVariant::Ground => &mut self.ground,
            BodyVariant::RigidBody(id) => &mut self.rbs[id],
            BodyVariant::Multibody(id) => &mut self.mbs[id],
            BodyVariant::AbstractBody(id) => &mut *self.bodies[id],
        }
    }

    /// Reference to the body part identified by `handle`.
    ///
    /// Panics if the body part is not found.
    #[inline]
    pub fn body_part(&self, handle: BodyPartHandle) -> &BodyPart<N> {
        match handle.body_handle.0 {
            BodyVariant::Ground => &self.ground,
            BodyVariant::RigidBody(id) => &self.rbs[id],
            BodyVariant::Multibody(id) => self.mbs[id].part(handle),
            BodyVariant::AbstractBody(id) => self.bodies[id].part(handle),
        }
    }

    /// Mutable reference to the body part identified by `handle`.
    ///
    /// Panics if the body part is not found.
    #[inline]
    pub fn body_part_mut(&mut self, handle: BodyPartHandle) -> &mut BodyPart<N> {
        match handle.body_handle.0 {
            BodyVariant::Ground => &mut self.ground,
            BodyVariant::RigidBody(id) => &mut self.rbs[id],
            BodyVariant::Multibody(id) => self.mbs[id].part_mut(handle),
            BodyVariant::AbstractBody(id) => self.bodies[id].part_mut(handle),
        }
    }


    /// Reference to the body and body_part identified by `handle`.
    ///
    /// Panics if the body or body part is not found.
    #[inline]
    pub fn body_and_part(&self, handle: BodyPartHandle) -> (&Body<N>, &BodyPart<N>) {
        match handle.body_handle.0 {
            BodyVariant::Ground => (&self.ground, &self.ground),
            BodyVariant::RigidBody(id) => (&self.rbs[id], &self.rbs[id]),
            BodyVariant::Multibody(id) => (&self.mbs[id], self.mbs[id].part(handle)),
            BodyVariant::AbstractBody(id) => (&*self.bodies[id], self.bodies[id].part(handle)),
        }
    }

    /// Reference to the multibody containing the multibody link identified by `body`.
    ///
    /// Returns `None` if it is not found or does not identify a multibody link.
    #[inline]
    pub fn multibody(&self, handle: BodyHandle) -> Option<&Multibody<N>> {
        if let BodyVariant::Multibody(id) = handle.0 {
            Some(&self.mbs[id])
        } else {
            None
        }
    }

    /// Mutable reference to the multibody containing the multibody link identified by `body`.
    ///
    /// Returns `None` if it is not found or does not identify a multibody link.
    #[inline]
    pub fn multibody_mut(&mut self, handle: BodyHandle) -> Option<&mut Multibody<N>> {
        if let BodyVariant::Multibody(id) = handle.0 {
            Some(&mut self.mbs[id])
        } else {
            None
        }
    }

    /// Reference to the multibody link identified by `body`.
    ///
    /// Returns `None` if it is not found or does not identify a multibody link.
    #[inline]
    pub fn multibody_link(&self, handle: BodyPartHandle) -> Option<&MultibodyLink<N>> {
        if let BodyVariant::Multibody(id) = handle.body_handle.0 {
            self.mbs[id].link(handle)
        } else {
            None
        }
    }

    /// Mutable reference to the multibody link identified by `body`.
    ///
    /// Returns `None` if it is not found or does not identify a multibody link.
    #[inline]
    pub fn multibody_link_mut(&mut self, handle: BodyPartHandle) -> Option<&mut MultibodyLink<N>> {
        if let BodyVariant::Multibody(id) = handle.body_handle.0 {
            self.mbs[id].link_mut(handle)
        } else {
            None
        }
    }

    /// Reference to the rigid body identified by `body`.
    ///
    /// Returns `None` if it is not found or does not identify a rigid body.
    #[inline]
    pub fn rigid_body(&self, handle: BodyHandle) -> Option<&RigidBody<N>> {
        if let BodyVariant::RigidBody(id) = handle.0 {
            Some(&self.rbs[id])
        } else {
            None
        }
    }

    /// Mutable reference to the rigid body identified by `body`.
    ///
    /// Returns `None` if it is not found or does not identify a rigid body.
    #[inline]
    pub fn rigid_body_mut(&mut self, handle: BodyHandle) -> Option<&mut RigidBody<N>> {
        if let BodyVariant::RigidBody(id) = handle.0 {
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

    /// Iterator yielding all the bodies on this set.
    #[inline]
    pub fn bodies(&self) -> Bodies<N> {
        Bodies {
            rbs_iter: self.rbs.iter(),
            mbs_iter: self.mbs.iter(),
            bs_iter: self.bodies.iter(),
        }
    }

    /// Mutable iterator yielding all the bodies on this set.
    #[inline]
    pub fn bodies_mut(&mut self) -> BodiesMut<N> {
        BodiesMut {
            rbs_iter: self.rbs.iter_mut(),
            mbs_iter: self.mbs.iter_mut(),
            bs_iter: self.bodies.iter_mut(),
        }
    }
}

/// Iterator yielding all the bodies on a body set.
pub struct Bodies<'a, N: Real> {
    rbs_iter: Iter<'a, RigidBody<N>>,
    mbs_iter: Iter<'a, Multibody<N>>,
    bs_iter: Iter<'a, Box<Body<N>>>,
}

impl<'a, N: Real> Iterator for Bodies<'a, N> {
    type Item = &'a Body<N>;

    #[inline]
    fn next(&mut self) -> Option<&'a Body<N>> {
        if let Some(res) = self.rbs_iter.next() {
            return Some(res.1);
        }

        if let Some(res) = self.mbs_iter.next() {
            return Some(res.1);
        }

        if let Some(res) = self.bs_iter.next() {
            return Some(&**res.1);
        }

        return None;
    }
}

/// Mutable iterator yielding all the bodies on a body set.
pub struct BodiesMut<'a, N: Real> {
    rbs_iter: IterMut<'a, RigidBody<N>>,
    mbs_iter: IterMut<'a, Multibody<N>>,
    bs_iter: IterMut<'a, Box<Body<N>>>,
}

impl<'a, N: Real> Iterator for BodiesMut<'a, N> {
    type Item = &'a mut Body<N>;

    #[inline]
    fn next(&mut self) -> Option<&'a mut Body<N>> {
        if let Some(res) = self.rbs_iter.next() {
            return Some(res.1);
        }

        if let Some(res) = self.mbs_iter.next() {
            return Some(res.1);
        }

        if let Some(res) = self.bs_iter.next() {
            return Some(&mut **res.1);
        }

        return None;
    }
}
