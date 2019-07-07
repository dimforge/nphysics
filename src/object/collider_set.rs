use std::hash::Hash;
use std::ops::{Deref, DerefMut};
use slab::Slab;

use na::RealField;
use ncollide::pipeline::object::{CollisionObjectHandle, CollisionObjectSet};
use crate::object::{Collider, BodyHandle, DefaultBodyHandle};

pub trait ColliderHandle: CollisionObjectHandle {
}

impl<T: CollisionObjectHandle> ColliderHandle for T { }

pub trait ColliderSet<N: RealField, Handle: BodyHandle>: CollisionObjectSet<N, CollisionObject = Collider<N, Handle>, CollisionObjectHandle = <Self as ColliderSet<N, Handle>>::Handle> {
    type Handle: ColliderHandle;

    fn get(&self, handle: Self::Handle) -> Option<&Collider<N, Handle>>;
    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Collider<N, Handle>>;
    fn get_pair_mut(&mut self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&mut Collider<N, Handle>>, Option<&mut Collider<N, Handle>>);

    fn contains(&self, handle: Self::Handle) -> bool;

    fn foreach(&self, f: impl FnMut(Self::Handle, &Collider<N, Handle>));
    fn foreach_mut(&mut self, f: impl FnMut(Self::Handle, &mut Collider<N, Handle>));
}

pub type DefaultColliderHandle = usize;
pub struct DefaultColliderSet<N: RealField>(Slab<Collider<N, DefaultBodyHandle>>);

impl<N: RealField> Deref for DefaultColliderSet<N> {
    type Target = Slab<Collider<N, DefaultBodyHandle>>;

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<N: RealField> DerefMut for DefaultColliderSet<N> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<N: RealField> DefaultColliderSet<N> {
    pub fn new() -> Self {
        DefaultColliderSet(Slab::new())
    }
}

impl<N: RealField> CollisionObjectSet<N> for DefaultColliderSet<N> {
    type CollisionObject = Collider<N, DefaultBodyHandle>;
    type CollisionObjectHandle = DefaultColliderHandle;

    fn collision_object(&self, handle: Self::CollisionObjectHandle) -> Option<&Self::CollisionObject> {
        self.0.get(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::CollisionObjectHandle, &Self::CollisionObject)) {
        for (handle, co) in self.0.iter() {
            f(handle, &co)
        }
    }
}

impl<N: RealField> ColliderSet<N, DefaultBodyHandle> for DefaultColliderSet<N> {
    type Handle = DefaultColliderHandle;

    fn get(&self, handle: Self::Handle) -> Option<&Collider<N, DefaultBodyHandle>> {
        self.0.get(handle)
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Collider<N, DefaultBodyHandle>> {
        self.0.get_mut(handle)
    }

    fn get_pair_mut(&mut self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&mut Collider<N, DefaultBodyHandle>>, Option<&mut Collider<N, DefaultBodyHandle>>) {
        unimplemented!()
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.0.contains(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &Collider<N, DefaultBodyHandle>)) {
        for (h, b) in self.0.iter() {
            f(h, b)
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Self::Handle, &mut Collider<N, DefaultBodyHandle>)) {
        for (h, b) in self.0.iter_mut() {
            f(h, b)
        }
    }
}