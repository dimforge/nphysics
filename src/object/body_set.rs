use std::hash::Hash;
use generational_arena::Arena;

use na::RealField;
use crate::world::ColliderWorld;
use crate::object::{Body, DefaultColliderHandle, RigidBody, Multibody};

/// Trait auto-implemented for types that can be used as a Body handle.
///
/// Body handles must be unique, i.e., they should not suffer from the ABA problem.
pub trait BodyHandle: Copy + Hash + PartialEq + Eq + 'static + Send + Sync {
}

impl<T: Copy + Hash + PartialEq + Eq + 'static + Send + Sync> BodyHandle for T {
}

/// Trait implemented by sets of bodies.
///
/// A set of bodies maps a body handle to a body instance. In addition, it must maintain a set of
/// body handle of bodies that have been removed (see the `pop_removal_event` method for details).
pub trait BodySet<N: RealField> {
    /// Type of a body stored in this set.
    type Body: ?Sized + Body<N>;
    /// Type of a body handle identifying a body in this set.
    type Handle: BodyHandle;

    /// Gets a reference to the body identified by `handle`.
    fn get(&self, handle: Self::Handle) -> Option<&Self::Body>;
    /// Gets a mutable reference to the body identified by `handle`.
    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::Body>;
    /// Gets a mutable reference to the two bodies identified by `handle1` and `handle2`.
    ///
    /// Panics if both handles are equal.
    fn get_pair_mut(&mut self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&mut Self::Body>, Option<&mut Self::Body>);
    /// Gets a reference to the two bodies identified by `handle1` and `handle2`.
    ///
    /// Both handles are allowed to be equal.
    fn get_pair(&self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&Self::Body>, Option<&Self::Body>) {
        (self.get(handle1), self.get(handle2))
    }

    /// Check if this set contains a body identified by `handle`.
    fn contains(&self, handle: Self::Handle) -> bool;

    /// Iterate through all the bodies on this set, applying the closure `f` on them.
    fn foreach(&self, f: impl FnMut(Self::Handle, &Self::Body));
    /// Mutable iterate through all the bodies on this set, applying the closure `f` on them.
    fn foreach_mut(&mut self, f: impl FnMut(Self::Handle, &mut Self::Body));

    /// Gets the handle of one body that has been removed.
    ///
    /// A body set must keep track (using typically a stack or a queue) of every body that has been
    /// removed from it. This is used by  nphysics to perform some internal cleanup actions, or
    /// physical actions like waking bodies touching one that has been removed.
    ///
    /// This method should return a removed body handle only once.
    fn pop_removal_event(&mut self) -> Option<Self::Handle>;
}

/// The default set containing all the bodies added to the world.
///
/// It is based on an arena using generational indices to avoid the ABA problem.
pub struct DefaultBodySet<N: RealField> {
    bodies: Arena<Box<Body<N>>>,
    removed: Vec<DefaultBodyHandle>,
}

impl<N: RealField> DefaultBodySet<N> {
    /// Creates an empty set.
    pub fn new() -> Self {
        DefaultBodySet {
            bodies: Arena::new(),
            removed: Vec::new(),
        }
    }

    /// Adds a body to this set.
    pub fn insert(&mut self, body: impl Body<N>) -> DefaultBodyHandle {
        self.bodies.insert(Box::new(body))
    }

    /// Adds a body (represented as a boxed trait-object) to this set.
    pub fn insert_boxed(&mut self, body: Box<Body<N>>) -> DefaultBodyHandle {
        self.bodies.insert(body)
    }

    /// Removes a body from this set.
    pub fn remove(&mut self, to_remove: DefaultBodyHandle) -> Option<Box<Body<N>>> {
        let res = self.bodies.remove(to_remove)?;
        self.removed.push(to_remove);
        Some(res)
    }

    /// Check if this set contains a body identified by `handle`.
    fn contains(&self, handle: DefaultBodyHandle) -> bool {
        self.bodies.contains(handle)
    }

    /// Gets a reference to the body identified by `handle`.
    pub fn get(&self, handle: DefaultBodyHandle) -> Option<&Body<N>> {
        self.bodies.get(handle).map(|b| &**b)
    }

    /// Gets a mutable reference to the body identified by `handle`.
    pub fn get_mut(&mut self, handle: DefaultBodyHandle) -> Option<&mut Body<N>> {
        self.bodies.get_mut(handle).map(|b| &mut **b)
    }

    /// Iterate through all the bodies and their handles.
    pub fn iter(&self) -> impl Iterator<Item = (DefaultBodyHandle, &Body<N>)> {
        self.bodies.iter().map(|b| (b.0, &**b.1))
    }

    /// Mutably iterate through all the bodies and their handles.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (DefaultBodyHandle, &mut Body<N>)> {
        self.bodies.iter_mut().map(|b| (b.0, &mut **b.1))
    }

    /// Gets the rigid body identified by `handle`.
    ///
    /// Returns `None` if the body does not exists, of if it exists but is not a rigid body.
    pub fn rigid_body(&self, handle: DefaultBodyHandle) -> Option<&RigidBody<N>> {
        self.get(handle).and_then(|b| b.downcast_ref())
    }

    /// Gets a mutable reference to the rigid body identified by `handle`.
    ///
    /// Returns `None` if the body does not exists, of if it exists but is not a rigid body.
    pub fn rigid_body_mut(&mut self, handle: DefaultBodyHandle) -> Option<&mut RigidBody<N>> {
        self.get_mut(handle).and_then(|b| b.downcast_mut())
    }

    /// Gets the multibody identified by `handle`.
    ///
    /// Returns `None` if the body does not exists, of if it exists but is not a multibody.
    pub fn multibody(&self, handle: DefaultBodyHandle) -> Option<&Multibody<N>> {
        self.get(handle).and_then(|b| b.downcast_ref())
    }

    /// Gets a mutable reference to the multibody identified by `handle`.
    ///
    /// Returns `None` if the body does not exists, of if it exists but is not a multibody.
    pub fn multibody_mut(&mut self, handle: DefaultBodyHandle) -> Option<&mut Multibody<N>> {
        self.get_mut(handle).and_then(|b| b.downcast_mut())
    }
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
        assert_ne!(handle1, handle2, "Both body handles must not be equal.");
        let b1 = self.get_mut(handle1).map(|b| b as *mut Body<N>);
        let b2 = self.get_mut(handle2).map(|b| b as *mut Body<N>);
        unsafe {
            use std::mem;
            (
                b1.map(|b| mem::transmute(b)),
                b2.map(|b| mem::transmute(b))
            )
        }
    }


    fn contains(&self, handle: Self::Handle) -> bool {
        self.contains(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &Self::Body)) {
        for (h, b) in self.iter() {
            f(h, b)
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Self::Handle, &mut Self::Body)) {
        for (h, b) in self.iter_mut() {
            f(h, b)
        }
    }

    fn pop_removal_event(&mut self) -> Option<Self::Handle> {
        self.removed.pop()
    }
}


/// The body handle used by the `DefaultBodySet`.
pub type DefaultBodyHandle = generational_arena::Index;
/// The body part handle used by the `DefaultBodySet`.
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