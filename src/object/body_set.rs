use slab::{Iter, IterMut, Slab};
use std::hash::Hash;

use na::RealField;
use crate::world::ColliderWorld;
use crate::object::{Body, Ground, DefaultColliderHandle};

pub trait BodyHandle: Copy + Hash + PartialEq + Eq + 'static + Send + Sync {
    fn is_ground(&self) -> bool;
}

impl BodyHandle for () {
    fn is_ground(&self) -> bool {
        false
    }
}

pub trait BodySet<N: RealField> {
    type Body: ?Sized + Body<N>;
    type Handle: BodyHandle;

    fn get(&self, handle: Self::Handle) -> Option<&Self::Body>;
    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::Body>;
    fn get_pair_mut(&mut self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&mut Self::Body>, Option<&mut Self::Body>);

    fn contains(&self, handle: Self::Handle) -> bool;

    fn foreach(&self, f: impl FnMut(Self::Handle, &Self::Body));
    fn foreach_mut(&mut self, f: impl FnMut(Self::Handle, &mut Self::Body));
}

impl<N: RealField> BodySet<N> for DefaultBodySet<N> {
    type Body = Body<N>;
    type Handle = DefaultBodyHandle;

    fn get(&self, handle: Self::Handle) -> Option<&Self::Body> {
        self.get(handle)
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::Body> {
        self.get_mut(handle)
    }

    fn get_pair_mut(&mut self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&mut Self::Body>, Option<&mut Self::Body>) {
        self.get_pair_mut(handle1, handle2)
    }


    fn contains(&self, handle: Self::Handle) -> bool {
        self.contains(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &Self::Body)) {
        for (h, b) in self.bodies.iter() {
            f(DefaultBodyHandle(h), &**b)
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Self::Handle, &mut Self::Body)) {
        for (h, b) in self.bodies.iter_mut() {
            f(DefaultBodyHandle(h), &mut **b)
        }
    }
}


/// A world-specific body handle.
///
/// This structure is automatically allocated by the physics world.
/// It cannot be constructed by the end-user.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct DefaultBodyHandle(usize);
pub type DefaultBodyPartHandle = BodyPartHandle<DefaultBodyHandle>;

impl BodyHandle for DefaultBodyHandle {
    #[inline]
    fn is_ground(&self) -> bool {
        self.0 == usize::max_value()
    }
}

/// A unique identifier of a body part added to the world.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct BodyPartHandle<Handle: BodyHandle>(pub Handle, pub usize);

impl DefaultBodyHandle {
    #[inline]
    pub fn ground() -> Self {
        DefaultBodyHandle(usize::max_value())
    }

    /// Tests if this handle corresponds to the ground.
    #[inline]
    pub fn is_ground(&self) -> bool {
        self.0 == usize::max_value()
    }
}

impl<Handle: BodyHandle> BodyPartHandle<Handle> {
    /// Tests if this handle corresponds to the ground.
    pub fn is_ground(&self) -> bool {
        self.0.is_ground()
    }

    #[inline]
    pub fn ground() -> Self {
        panic!("This should be removed. Though I don't know yet what to do with this.\
                It is currently used by multibodies. We probably want to replace this with an Option.")
    }
}

/// A abstract body descriptor to be passed to the physics `World` to create a body.
pub trait BodyDesc<N: RealField> {
    /// The type of body being generated.
    type Body: Body<N>;

    /// Called by the `World` to create a body with the given allocated handle.
    fn build_with_handle(&self, cworld: &mut ColliderWorld<N, DefaultBodyHandle, DefaultColliderHandle>, handle: DefaultBodyHandle) -> Self::Body;
}

/// A set containing all the bodies added to the world.
pub struct DefaultBodySet<N: RealField> {
    ground: Ground<N>,
    bodies: Slab<Box<Body<N>>>,
}

impl<N: RealField> DefaultBodySet<N> {
    /// Create a new empty set of bodies.
    pub fn new() -> Self {
        DefaultBodySet {
            ground: Ground::new(),
            bodies: Slab::new(),
        }
    }

    /// The number of bodies in this set.
    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    /// Adds a body to the world.
    pub fn add<B: BodyDesc<N>>(&mut self, desc: &B, cworld: &mut ColliderWorld<N, DefaultBodyHandle, DefaultColliderHandle>) -> &mut B::Body {
        let b_entry = self.bodies.vacant_entry();
        let b_id = b_entry.key();
        let handle = DefaultBodyHandle(b_id);
        let body = desc.build_with_handle(cworld, handle);
        b_entry.insert(Box::new(body)).downcast_mut::<B::Body>().expect("Body construction failed with type mismatch.")
    }

    /// Remove a body from this set.
    ///
    /// If `body` identify a mutibody link, the whole multibody is removed.
    pub fn remove(&mut self, body: DefaultBodyHandle) {
        if !body.is_ground() {
            let _ = self.bodies.remove(body.0);
        }
    }

    /// Returns `true` if the given body exists.
    #[inline]
    pub fn contains(&self, handle: DefaultBodyHandle) -> bool {
        handle.is_ground() || self.bodies.contains(handle.0)
    }

    /// Reference to the body identified by `body`.
    ///
    /// Returns `None` if the body is not found.
    #[inline]
    pub fn get(&self, handle: DefaultBodyHandle) -> Option<&Body<N>> {
        if handle.is_ground() {
            Some(&self.ground)
        } else {
            self.bodies.get(handle.0).map(|b| &**b)
        }
    }

    /// Mutable reference to the specified body.
    ///
    /// Returns `None` if the body is not found.
    #[inline]
    pub fn get_mut(&mut self, handle: DefaultBodyHandle) -> Option<&mut Body<N>> {
        if handle.is_ground() {
            Some(&mut self.ground)
        } else {
            self.bodies.get_mut(handle.0).map(|b| &mut **b)
        }
    }

    /// Mutable reference to the two specified bodies.
    ///
    /// Returns `None` if the body is not found.
    /// Panics if both handles are equal.
    #[inline]
    pub fn get_pair_mut(&mut self, handle1: DefaultBodyHandle, handle2: DefaultBodyHandle) -> (Option<&mut Body<N>>, Option<&mut Body<N>>) {
        assert_ne!(handle1, handle2, "Both body handles must not be equal.");
        let b1 = self.get_mut(handle1).map(|b| b as *mut Body<N>);
        let b2 = self.get_mut(handle2).map(|b| b as *mut Body<N>);
        unsafe {
            use std::mem;
            (b1.map(|b| mem::transmute(b)), b2.map(|b| mem::transmute(b)))
        }
    }

    /// Iterator yielding all the bodies on this set.
    #[inline]
    pub fn iter(&self) -> impl Iterator<Item = (DefaultBodyHandle, &Body<N>)> {
        self.bodies.iter().map(|e| (DefaultBodyHandle(e.0), &**e.1))
    }

    /// Mutable iterator yielding all the bodies on this set.
    #[inline]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (DefaultBodyHandle, &mut Body<N>)> {
        self.bodies.iter_mut().map(|e| (DefaultBodyHandle(e.0), &mut **e.1))
    }
}

/// Iterator yielding all the bodies on a body set.
pub type Bodies<'a, N> = Iter<'a, Box<Body<N>>>;
/// Mutable iterator yielding all the bodies on a body set.
pub type BodiesMut<'a, N> = IterMut<'a, Box<Body<N>>>;
