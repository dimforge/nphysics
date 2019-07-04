use std::hash::Hash;
use slab::Slab;

use na::RealField;
use ncollide::pipeline::object::{CollisionObjectHandle, CollisionObjectSet};
use crate::object::{Collider, BodyHandle};

pub trait ColliderHandle: CollisionObjectHandle {
}

impl<T: CollisionObjectHandle> ColliderHandle for T { }

pub trait ColliderSet<N: RealField, Handle: BodyHandle>: for <'a> CollisionObjectSet<'a, N, CollisionObjectHandle = <Self as ColliderSet<N, Handle>>::Handle> {
    type Handle: ColliderHandle;

    fn get(&self, handle: Self::Handle) -> Option<&Collider<N, Handle>>;
    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Collider<N, Handle>>;
    fn get_pair_mut(&mut self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&mut Collider<N, Handle>>, Option<&mut Collider<N, Handle>>);

    fn contains(&self, handle: Self::Handle) -> bool;

    fn foreach(&self, f: impl FnMut(Self::Handle, &Collider<N, Handle>));
    fn foreach_mut(&mut self, f: impl FnMut(Self::Handle, &mut Collider<N, Handle>));
}

pub type ColliderSlabHandle = usize;
pub struct ColliderSlab<N: RealField, Handle: BodyHandle>(pub Slab<Collider<N, Handle>>);

impl<N: RealField, Handle: BodyHandle> ColliderSlab<N, Handle> {
    pub fn new() -> Self {
        ColliderSlab(Slab::new())
    }
}

impl<'a, N: RealField, Handle: BodyHandle> CollisionObjectSet<'a, N> for ColliderSlab<N, Handle> {
    type CollisionObject = &'a Collider<N, Handle>;
    type Iter = slab::Iter<'a, Collider<N, Handle>>;
    type CollisionObjectHandle = ColliderSlabHandle;

    fn collision_object(&'a self, handle: Self::CollisionObjectHandle) -> Option<Self::CollisionObject> {
        self.0.get(handle)
    }

    fn iter(&'a self) -> Self::Iter {
        self.0.iter()
    }
}

impl<N: RealField, Handle: BodyHandle> ColliderSet<N, Handle> for ColliderSlab<N, Handle> {
    type Handle = ColliderSlabHandle;

    fn get(&self, handle: Self::Handle) -> Option<&Collider<N, Handle>> {
        self.0.get(handle)
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Collider<N, Handle>> {
        self.0.get_mut(handle)
    }

    fn get_pair_mut(&mut self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&mut Collider<N, Handle>>, Option<&mut Collider<N, Handle>>) {
        unimplemented!()
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.0.contains(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &Collider<N, Handle>)) {
        for (h, b) in self.0.iter() {
            f(h, b)
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Self::Handle, &mut Collider<N, Handle>)) {
        for (h, b) in self.0.iter_mut() {
            f(h, b)
        }
    }
}