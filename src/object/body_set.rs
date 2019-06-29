use slab::{Iter, IterMut, Slab};
use std::hash::Hash;

use na::RealField;
use crate::world::ColliderWorld;
use crate::object::{Body, Ground};

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

impl<N: RealField> BodySet<N> for BodySlab<N> {
    type Body = Body<N>;
    type Handle = BodySlabHandle;

    fn get(&self, handle: Self::Handle) -> Option<&Self::Body> {
        self.body(handle)
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::Body> {
        self.body_mut(handle)
    }

    fn get_pair_mut(&mut self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&mut Self::Body>, Option<&mut Self::Body>) {
        self.body_pair_mut(handle1, handle2)
    }


    fn contains(&self, handle: Self::Handle) -> bool {
        self.contains(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &Self::Body)) {
        for (h, b) in self.bodies.iter() {
            f(BodySlabHandle(h), &**b)
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Self::Handle, &mut Self::Body)) {
        for (h, b) in self.bodies.iter_mut() {
            f(BodySlabHandle(h), &mut **b)
        }
    }
}


/// A world-specific body handle.
///
/// This structure is automatically allocated by the physics world.
/// It cannot be constructed by the end-user.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct BodySlabHandle(usize);

impl BodyHandle for BodySlabHandle {
    #[inline]
    fn is_ground(&self) -> bool {
        self.0 == usize::max_value()
    }
}

/// A unique identifier of a body part added to the world.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct BodyPartHandle<Handle: BodyHandle>(pub Handle, pub usize);

impl BodySlabHandle {
    #[inline]
    pub fn ground() -> Self {
        BodySlabHandle(usize::max_value())
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

/*
pub trait AbstractBodySlab<'a, N: RealField> {
    type BodySlabHandle;
    type Body: ?Sized + Body<N>;
    type Bodies: Iterator<Item = &'a Self::Body>;
    type BodiesMut: Iterator<Item = &'a mut Self::Body>;

    fn add_body(&mut self, body: impl Body<N>) -> &mut Self::Body;
    fn remove_body(&mut self, key: Self::BodySlabHandle);
    fn body(&self, handle: Self::BodySlabHandle) -> &Self::Body;
    fn body_mut(&mut self, handle: Self::BodySlabHandle) -> &mut Self::Body;
    fn bodies(&self) -> Self::Bodies;
    fn bodies_mut(&mut self) -> Self::BodiesMut;
}

impl<'a, N: RealField> AbstractBodySlab<'a, N> for BodySlab<N> {
    type BodySlabHandle = BodySlabHandle;
    type Body = Body<N>;
    type Bodies = Bodies<'a, N>;
    type BodiesMut = BodiesMut<'a, N>;

    fn add_body(&mut self, mut body: impl Body<N>) -> &mut Self::Body {
        let b_entry = self.bodies.vacant_entry();
        let b_id = b_entry.handle();
        let handle = BodySlabHandle(BodyVariant::AbstractBody(b_id));
        body.set_handle(Some(handle));
        &mut **b_entry.insert(body)
    }

    fn remove_body(&mut self, handle: Self::BodySlabHandle) {
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

    fn body(&self, handle: Self::BodySlabHandle) -> &Self::Body {
        unimplemented!()
    }

    fn body_mut(&mut self, handle: Self::BodySlabHandle) -> &mut Self::Body {
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

/// A abstract body descriptor to be passed to the physics `World` to create a body.
pub trait BodyDesc<N: RealField> {
    /// The type of body being generated.
    type Body: Body<N>;

    /// Called by the `World` to create a body with the given allocated handle.
    fn build_with_handle(&self, cworld: &mut ColliderWorld<N, BodySlabHandle>, handle: BodySlabHandle) -> Self::Body;
}

/// A set containing all the bodies added to the world.
pub struct BodySlab<N: RealField> {
    ground: Ground<N>,
    bodies: Slab<Box<Body<N>>>,
}

impl<N: RealField> BodySlab<N> {
    /// Create a new empty set of bodies.
    pub fn new() -> Self {
        BodySlab {
            ground: Ground::new(),
            bodies: Slab::new(),
        }
    }

    /// The number of bodies in this set.
    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    /// Adds a body to the world.
    pub fn add_body<B: BodyDesc<N>>(&mut self, desc: &B, cworld: &mut ColliderWorld<N, BodySlabHandle>) -> &mut B::Body {
        let b_entry = self.bodies.vacant_entry();
        let b_id = b_entry.key();
        let handle = BodySlabHandle(b_id);
        let body = desc.build_with_handle(cworld, handle);
        b_entry.insert(Box::new(body)).downcast_mut::<B::Body>().expect("Body construction failed with type mismatch.")
    }

    /// Remove a body from this set.
    ///
    /// If `body` identify a mutibody link, the whole multibody is removed.
    pub fn remove_body(&mut self, body: BodySlabHandle) {
        if !body.is_ground() {
            let _ = self.bodies.remove(body.0);
        }
    }

    /// Returns `true` if the given body exists.
    #[inline]
    pub fn contains(&self, handle: BodySlabHandle) -> bool {
        handle.is_ground() || self.bodies.contains(handle.0)
    }

    /// Reference to the body identified by `body`.
    ///
    /// Returns `None` if the body is not found.
    #[inline]
    pub fn body(&self, handle: BodySlabHandle) -> Option<&Body<N>> {
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
    pub fn body_mut(&mut self, handle: BodySlabHandle) -> Option<&mut Body<N>> {
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
    pub fn body_pair_mut(&mut self, handle1: BodySlabHandle, handle2: BodySlabHandle) -> (Option<&mut Body<N>>, Option<&mut Body<N>>) {
        assert_ne!(handle1, handle2, "Both body handles must not be equal.");
        let b1 = self.body_mut(handle1).map(|b| b as *mut Body<N>);
        let b2 = self.body_mut(handle2).map(|b| b as *mut Body<N>);
        unsafe {
            use std::mem;
            (b1.map(|b| mem::transmute(b)), b2.map(|b| mem::transmute(b)))
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
pub type Bodies<'a, N> = Iter<'a, Box<Body<N>>>;
/// Mutable iterator yielding all the bodies on a body set.
pub type BodiesMut<'a, N> = IterMut<'a, Box<Body<N>>>;
