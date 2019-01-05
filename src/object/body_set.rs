use slab::{Iter, IterMut, Slab};
use std::iter::Map;

use crate::joint::Joint;
use crate::math::{Inertia, Isometry, Point, Vector};
use na::Real;
use crate::world::CollisionWorld;
use crate::object::{Body, BodyPart, Ground, Multibody,
             MultibodyLink, MultibodyWorkspace, RigidBody};
use crate::solver::IntegrationParameters;

/// A world-specific body handle.
///
/// This structure is automatically allocated by the physics world.
/// It cannot be constructed by the end-user.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct BodyHandle(usize);

/// A unique identifier of a body part added to the world.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct BodyPartHandle(pub BodyHandle, pub usize);

impl BodyHandle {
    /// The body handle of the ground.
    #[inline]
    pub fn ground() -> Self {
        BodyHandle(usize::max_value())
    }

    /// Tests if this handle corresponds to the ground.
    #[inline]
    pub fn is_ground(&self) -> bool {
        self.0 == usize::max_value()
    }
}

impl BodyPartHandle {
    /// The body part handle of the ground.
    pub fn ground() -> Self {
        BodyPartHandle(BodyHandle::ground(), 0)
    }

    /// Tests if this handle corresponds to the ground.
    pub fn is_ground(&self) -> bool {
        self.0.is_ground()
    }
}

/*
pub trait AbstractBodySet<'a, N: Real> {
    type BodyHandle;
    type Body: ?Sized + Body<N>;
    type Bodies: Iterator<Item = &'a Self::Body>;
    type BodiesMut: Iterator<Item = &'a mut Self::Body>;

    fn add_body(&mut self, body: impl Body<N>) -> &mut Self::Body;
    fn remove_body(&mut self, key: Self::BodyHandle);
    fn body(&self, handle: Self::BodyHandle) -> &Self::Body;
    fn body_mut(&mut self, handle: Self::BodyHandle) -> &mut Self::Body;
    fn bodies(&self) -> Self::Bodies;
    fn bodies_mut(&mut self) -> Self::BodiesMut;
}

impl<'a, N: Real> AbstractBodySet<'a, N> for BodySet<N> {
    type BodyHandle = BodyHandle;
    type Body = Body<N>;
    type Bodies = Bodies<'a, N>;
    type BodiesMut = BodiesMut<'a, N>;

    fn add_body(&mut self, mut body: impl Body<N>) -> &mut Self::Body {
        let b_entry = self.bodies.vacant_entry();
        let b_id = b_entry.handle();
        let handle = BodyHandle(BodyVariant::AbstractBody(b_id));
        body.set_handle(Some(handle));
        &mut **b_entry.insert(body)
    }

    fn remove_body(&mut self, handle: Self::BodyHandle) {
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

    fn body(&self, handle: Self::BodyHandle) -> &Self::Body {
        unimplemented!()
    }

    fn body_mut(&mut self, handle: Self::BodyHandle) -> &mut Self::Body {
        unimplemented!()
    }

    fn bodies(&self) -> Self::Bodies {
        unimplemented!()
    }

    fn bodies_mut(&mut self) -> Self::BodiesMut {
        unimplemented!()
    }
}
*/

/// A set containing all the bodies added to the world.
pub struct BodySet<N: Real> {
    ground: Ground<N>,
    bodies: Slab<Box<Body<N>>>,
}

impl<N: Real> BodySet<N> {
    /// Create a new empty set of bodies.
    pub fn new() -> Self {
        BodySet {
            ground: Ground::new(),
            bodies: Slab::new(),
        }
    }

    /// The number of bodies in this set.
    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    /// Adds a body to the world.
    pub fn add_body(&mut self, mut body: Box<Body<N>>) -> &mut Body<N> {
        let b_entry = self.bodies.vacant_entry();
        let b_id = b_entry.key();
        let handle = BodyHandle(b_id);
        body.set_handle(Some(handle));
        &mut **b_entry.insert(body)
    }

    /// Remove a body from this set.
    ///
    /// If `body` identify a mutibody link, the whole multibody is removed.
    pub fn remove_body(&mut self, body: BodyHandle) {
        if !body.is_ground() {
            let _ = self.bodies.remove(body.0);
        }
    }

    /// Checks that the given handle identifies a valid body.
    ///
    /// Returns `true` if `handle.is_ground()` too.
    pub fn contains_body(&self, handle: BodyHandle) -> bool {
        handle.is_ground() || self.bodies.contains(handle.0)
    }

    /// Reference to the body identified by `body`.
    ///
    /// Panics if the body is not found.
    #[inline]
    pub fn body(&self, handle: BodyHandle) -> Option<&Body<N>> {
        if handle.is_ground() {
            Some(&self.ground)
        } else {
            self.bodies.get(handle.0).map(|b| &**b)
        }
    }

    /// Mutable reference to the body identified by `body`.
    ///
    /// Panics if the body is not found.
    #[inline]
    pub fn body_mut(&mut self, handle: BodyHandle) -> Option<&mut Body<N>> {
        if handle.is_ground() {
            Some(&mut self.ground)
        } else {
            self.bodies.get_mut(handle.0).map(|b| &mut **b)
        }
    }

    /// Iterator yielding all the bodies on this set.
    #[inline]
    pub fn bodies(&self) -> impl Iterator<Item = &Body<N>> {
        self.bodies.iter().map(|e| &**e.1)
    }

    /// Mutable iterator yielding all the bodies on this set.
    #[inline]
    pub fn bodies_mut(&mut self) -> impl Iterator<Item = &mut Body<N>> {
        self.bodies.iter_mut().map(|e| &mut **e.1)
    }
}

/// Iterator yielding all the bodies on a body set.
pub type Bodies<'a, N: Real> = Iter<'a, Box<Body<N>>>;
/// Mutable iterator yielding all the bodies on a body set.
pub type BodiesMut<'a, N: Real> = IterMut<'a, Box<Body<N>>>;
