

use generational_arena::Arena;

use na::RealField;
use ncollide::pipeline::object::{CollisionObjectHandle, CollisionObjectSet};
use crate::object::{Collider, BodyHandle, DefaultBodyHandle, ColliderRemovalData};

pub trait ColliderHandle: CollisionObjectHandle + std::fmt::Debug {
}

impl<T: CollisionObjectHandle + std::fmt::Debug> ColliderHandle for T { }

pub trait ColliderSet<N: RealField, Handle: BodyHandle>: CollisionObjectSet<N, CollisionObject = Collider<N, Handle>, CollisionObjectHandle = <Self as ColliderSet<N, Handle>>::Handle> {
    type Handle: ColliderHandle;

    fn get(&self, handle: Self::Handle) -> Option<&Collider<N, Handle>>;
    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Collider<N, Handle>>;
    fn get_pair_mut(&mut self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&mut Collider<N, Handle>>, Option<&mut Collider<N, Handle>>);
    fn get_pair(&self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&Collider<N, Handle>>, Option<&Collider<N, Handle>>) {
        (self.get(handle1), self.get(handle2))
    }

    fn contains(&self, handle: Self::Handle) -> bool;

    fn foreach(&self, f: impl FnMut(Self::Handle, &Collider<N, Handle>));
    fn foreach_mut(&mut self, f: impl FnMut(Self::Handle, &mut Collider<N, Handle>));

    fn pop_insertion_event(&mut self) -> Option<Self::Handle>;
    fn pop_removal_event(&mut self) -> Option<(Self::Handle, ColliderRemovalData<N, Handle>)>;
    fn remove(&mut self, to_remove: Self::Handle) -> Option<&mut ColliderRemovalData<N, Handle>>;
}

pub type DefaultColliderHandle = generational_arena::Index;

pub struct DefaultColliderSet<N: RealField, Handle: BodyHandle = DefaultBodyHandle> {
    colliders: Arena<Collider<N, Handle>>,
    removed: Vec<(DefaultColliderHandle, ColliderRemovalData<N, Handle>)>,
    inserted: Vec<DefaultColliderHandle>,
}


impl<N: RealField, Handle: BodyHandle> DefaultColliderSet<N, Handle> {
    pub fn new() -> Self {
        DefaultColliderSet {
            colliders: Arena::new(),
            removed: Vec::new(),
            inserted: Vec::new(),
        }
    }

    pub fn insert(&mut self, collider: Collider<N, Handle>) -> DefaultColliderHandle {
        let res = self.colliders.insert(collider);
        self.inserted.push(res);
        res
    }

    pub fn remove(&mut self, to_remove: DefaultColliderHandle) -> Option<Collider<N, Handle>> {
        println!("Collider removed.");
        let res = self.colliders.remove(to_remove)?;

        if let Some(data) = res.removal_data() {
            self.removed.push((to_remove, data));
        }

        Some(res)
    }

    pub fn get(&self, handle: DefaultColliderHandle) -> Option<&Collider<N, Handle>> {
        self.colliders.get(handle)
    }

    pub fn get_mut(&mut self, handle: DefaultColliderHandle) -> Option<&mut Collider<N, Handle>> {
        self.colliders.get_mut(handle)
    }

    pub fn iter(&self) -> impl Iterator<Item = (DefaultColliderHandle, &Collider<N, Handle>)> {
        self.colliders.iter()
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (DefaultColliderHandle, &mut Collider<N, Handle>)> {
        self.colliders.iter_mut()
    }
}

impl<N: RealField, Handle: BodyHandle> CollisionObjectSet<N> for DefaultColliderSet<N, Handle> {
    type CollisionObject = Collider<N, Handle>;
    type CollisionObjectHandle = DefaultColliderHandle;

    fn collision_object(&self, handle: Self::CollisionObjectHandle) -> Option<&Self::CollisionObject> {
        self.get(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::CollisionObjectHandle, &Self::CollisionObject)) {
        for (handle, co) in self.iter() {
            f(handle, &co)
        }
    }
}

impl<N: RealField, Handle: BodyHandle> ColliderSet<N, Handle> for DefaultColliderSet<N, Handle> {
    type Handle = DefaultColliderHandle;

    fn get(&self, handle: Self::Handle) -> Option<&Collider<N, Handle>> {
        self.get(handle)
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Collider<N, Handle>> {
        self.get_mut(handle)
    }

    fn get_pair_mut(&mut self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&mut Collider<N, Handle>>, Option<&mut Collider<N, Handle>>) {
        assert_ne!(handle1, handle2, "Both body handles must not be equal.");
        let b1 = self.get_mut(handle1).map(|b| b as *mut Collider<N, Handle>);
        let b2 = self.get_mut(handle2).map(|b| b as *mut Collider<N, Handle>);
        unsafe {
            use std::mem;
            (b1.map(|b| mem::transmute(b)), b2.map(|b| mem::transmute(b)))
        }
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.colliders.contains(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &Collider<N, Handle>)) {
        for (h, b) in self.iter() {
            f(h, b)
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Self::Handle, &mut Collider<N, Handle>)) {
        for (h, b) in self.iter_mut() {
            f(h, b)
        }
    }

    fn pop_insertion_event(&mut self) -> Option<Self::Handle> {
        self.inserted.pop()
    }

    fn pop_removal_event(&mut self) -> Option<(Self::Handle, ColliderRemovalData<N, Handle>)> {
        self.removed.pop()
    }

    fn remove(&mut self, to_remove: Self::Handle) -> Option<&mut ColliderRemovalData<N, Handle>> {
        let res = self.colliders.remove(to_remove)?;
        if let Some(data) = res.removal_data() {
            self.removed.push((to_remove, data));
            self.removed.last_mut().map(|r| &mut r.1)
        } else {
            None
        }
    }
}