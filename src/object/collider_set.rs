

use generational_arena::Arena;

use na::RealField;
use ncollide::pipeline::{CollisionObjectHandle, CollisionObjectSet};
use crate::object::{Collider, BodyHandle, DefaultBodyHandle, ColliderRemovalData};

/// Trait auto-implemented for types that can be used as a Collider handle.
///
/// Collider handles must be unique, i.e., they should not suffer from the ABA problem.
pub trait ColliderHandle: CollisionObjectHandle + std::fmt::Debug {
}

impl<T: CollisionObjectHandle + std::fmt::Debug> ColliderHandle for T { }

/// Trait implemented by sets of colliders.
///
/// A set of colliders maps a collider handle to a collider instance. In addition, it must maintain a set of
/// data related to colliders that have been inserted or removed (see the `pop_insertion_event` and `pop_removal_event` methods for details).
pub trait ColliderSet<N: RealField, Handle: BodyHandle>: CollisionObjectSet<N, CollisionObject = Collider<N, Handle>, CollisionObjectHandle = <Self as ColliderSet<N, Handle>>::Handle> {
    /// Type of a collider handle identifying a collider in this set.
    type Handle: ColliderHandle;

    /// Gets a reference to the collider identified by `handle`.
    fn get(&self, handle: Self::Handle) -> Option<&Collider<N, Handle>>;
    /// Gets a mutable reference to the collider identified by `handle`.
    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Collider<N, Handle>>;
    /// Gets a mutable reference to the two colliders identified by `handle1` and `handle2`.
    ///
    /// Panics if both handles are equal.
    fn get_pair_mut(&mut self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&mut Collider<N, Handle>>, Option<&mut Collider<N, Handle>>);
    /// Gets a reference to the two colliders identified by `handle1` and `handle2`.
    ///
    /// Both handles are allowed to be equal.
    fn get_pair(&self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&Collider<N, Handle>>, Option<&Collider<N, Handle>>) {
        (self.get(handle1), self.get(handle2))
    }

    /// Check if this set contains a collider identified by `handle`.
    fn contains(&self, handle: Self::Handle) -> bool;

    /// Iterate through all the colliders on this set, applying the closure `f` on them.
    fn foreach(&self, f: impl FnMut(Self::Handle, &Collider<N, Handle>));
    /// Mutable iterates through all the colliders on this set, applying the closure `f` on them.
    fn foreach_mut(&mut self, f: impl FnMut(Self::Handle, &mut Collider<N, Handle>));


    /// Gets the handle of one collider that has been inserted.
    ///
    /// A collider set must keep track (using typically a stack or a queue) of every collider that has been
    /// inserted from it. This is used by  nphysics to perform some internal setup actions, or
    /// physical actions like waking colliders touching one that has been inserted.
    ///
    /// This method should return a removed collider handle only once.
    fn pop_insertion_event(&mut self) -> Option<Self::Handle>;
    /// Gets the handle and removal data of one collider that has been removed.
    ///
    /// A collider set must keep track (using typically a stack or a queue) of every collider that has been
    /// removed from it. This is used by nphysics to perform some internal cleanup actions, or
    /// physical actions like waking colliders touching one that has been removed.
    ///
    /// This method should return a removed collider handle and its associated `ColliderRemovalData` only once.
    fn pop_removal_event(&mut self) -> Option<(Self::Handle, ColliderRemovalData<N, Handle>)>;
    /// Removes a collider from this set.
    ///
    /// A collider can be removed automatically by nphysics whene the collider it was attached too has been removed.
    fn remove(&mut self, to_remove: Self::Handle) -> Option<&mut ColliderRemovalData<N, Handle>>;
}

/// The collider handle used by the `DefaultColliderSet`.
pub type DefaultColliderHandle = generational_arena::Index;

/// The default set containing all the colliders added to the world.
///
/// It is based on an arena using generational indices to avoid the ABA problem.
pub struct DefaultColliderSet<N: RealField, Handle: BodyHandle = DefaultBodyHandle> {
    colliders: Arena<Collider<N, Handle>>,
    removed: Vec<(DefaultColliderHandle, ColliderRemovalData<N, Handle>)>,
    inserted: Vec<DefaultColliderHandle>,
}


impl<N: RealField, Handle: BodyHandle> DefaultColliderSet<N, Handle> {
    /// Creates an empty set.
    pub fn new() -> Self {
        DefaultColliderSet {
            colliders: Arena::new(),
            removed: Vec::new(),
            inserted: Vec::new(),
        }
    }

    /// Adds a collider to this set.
    pub fn insert(&mut self, collider: Collider<N, Handle>) -> DefaultColliderHandle {
        let res = self.colliders.insert(collider);
        self.inserted.push(res);
        res
    }

    /// Removes a collider from this set.
    pub fn remove(&mut self, to_remove: DefaultColliderHandle) -> Option<Collider<N, Handle>> {
        let res = self.colliders.remove(to_remove)?;

        if let Some(data) = res.removal_data() {
            self.removed.push((to_remove, data));
        }

        Some(res)
    }

    /// Check if this set contains a collider identified by `handle`.
    fn contains(&self, handle: DefaultColliderHandle) -> bool {
        self.colliders.contains(handle)
    }


    /// Gets a reference to the collider identified by `handle`.
    pub fn get(&self, handle: DefaultColliderHandle) -> Option<&Collider<N, Handle>> {
        self.colliders.get(handle)
    }

    /// Gets a mutable reference to the collider identified by `handle`.
    pub fn get_mut(&mut self, handle: DefaultColliderHandle) -> Option<&mut Collider<N, Handle>> {
        self.colliders.get_mut(handle)
    }

    /// Iterates through all the colliders and their handles.
    pub fn iter(&self) -> impl Iterator<Item = (DefaultColliderHandle, &Collider<N, Handle>)> {
        self.colliders.iter()
    }

    /// Mutably iterate through all the colliders and their handles.
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
        self.colliders.get2_mut(handle1, handle2)
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.contains(handle)
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