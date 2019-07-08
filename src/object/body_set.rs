use slab::{Iter, IterMut, Slab};
use std::hash::Hash;

use na::RealField;
use crate::world::ColliderWorld;
use crate::object::{Body, Ground, DefaultColliderHandle};

pub trait BodyHandle: Copy + Hash + PartialEq + Eq + 'static + Send + Sync {
}

impl<T: Copy + Hash + PartialEq + Eq + 'static + Send + Sync> BodyHandle for T {
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
        self.get(handle).map(|e| &**e)
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::Body> {
        self.get_mut(handle).map(|e| &mut **e)
    }

    fn get_pair_mut(&mut self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&mut Self::Body>, Option<&mut Self::Body>) {
        assert_ne!(handle1, handle2, "Both body handles must not be equal.");
        let b1 = self.get_mut(handle1).map(|b| &mut **b as *mut Body<N>);
        let b2 = self.get_mut(handle2).map(|b| &mut **b as *mut Body<N>);
        unsafe {
            use std::mem;
            (b1.map(|b| mem::transmute(b)), b2.map(|b| mem::transmute(b)))
        }
    }


    fn contains(&self, handle: Self::Handle) -> bool {
        self.contains(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &Self::Body)) {
        for (h, b) in self.iter() {
            f(h, &**b)
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Self::Handle, &mut Self::Body)) {
        for (h, b) in self.iter_mut() {
            f(h, &mut **b)
        }
    }
}


pub type DefaultBodyHandle = usize;
pub type DefaultBodyPartHandle = BodyPartHandle<DefaultBodyHandle>;

/// A unique identifier of a body part added to the world.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct BodyPartHandle<Handle: BodyHandle>(pub Handle, pub usize);

/// A abstract body descriptor to be passed to the physics `World` to create a body.
pub trait BodyDesc<N: RealField> {
    /// The type of body being generated.
    type Body: Body<N>;

    /// Called by the `World` to create a body with the given allocated handle.
    fn build_with_handle(&self, cworld: &mut ColliderWorld<N, DefaultBodyHandle, DefaultColliderHandle>, handle: DefaultBodyHandle) -> Self::Body;
}

/// A set containing all the bodies added to the world.
pub type DefaultBodySet<N: RealField> = Slab<Box<Body<N>>>;

/// Iterator yielding all the bodies on a body set.
pub type Bodies<'a, N> = Iter<'a, Box<Body<N>>>;
/// Mutable iterator yielding all the bodies on a body set.
pub type BodiesMut<'a, N> = IterMut<'a, Box<Body<N>>>;
