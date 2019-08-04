use std::hash::Hash;
use generational_arena::Arena;

use na::RealField;
use crate::world::ColliderWorld;
use crate::object::{Body, Ground, DefaultColliderHandle, RigidBody, Multibody};

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
    fn get_pair(&self, handle1: Self::Handle, handle2: Self::Handle) -> (Option<&Self::Body>, Option<&Self::Body>) {
        (self.get(handle1), self.get(handle2))
    }

    fn contains(&self, handle: Self::Handle) -> bool;

    fn foreach(&self, f: impl FnMut(Self::Handle, &Self::Body));
    fn foreach_mut(&mut self, f: impl FnMut(Self::Handle, &mut Self::Body));

    fn pop_removal_event(&mut self) -> Option<Self::Handle>;
}

/// A set containing all the bodies added to the world.
pub struct DefaultBodySet<N: RealField> {
    bodies: Arena<Box<Body<N>>>,
    removed: Vec<DefaultBodyHandle>,
}

impl<N: RealField> DefaultBodySet<N> {
    pub fn new() -> Self {
        DefaultBodySet {
            bodies: Arena::new(),
            removed: Vec::new(),
        }
    }

    pub fn insert(&mut self, body: impl Body<N>) -> DefaultBodyHandle {
        self.bodies.insert(Box::new(body))
    }

    pub fn insert_boxed(&mut self, body: Box<Body<N>>) -> DefaultBodyHandle {
        self.bodies.insert(body)
    }

    pub fn remove(&mut self, to_remove: DefaultBodyHandle) -> Option<Box<Body<N>>> {
        let res = self.bodies.remove(to_remove)?;
        self.removed.push(to_remove);
        Some(res)
    }

    pub fn get(&self, handle: DefaultBodyHandle) -> Option<&Body<N>> {
        self.bodies.get(handle).map(|b| &**b)
    }

    pub fn get_mut(&mut self, handle: DefaultBodyHandle) -> Option<&mut Body<N>> {
        self.bodies.get_mut(handle).map(|b| &mut **b)
    }

    pub fn iter(&self) -> impl Iterator<Item = (DefaultBodyHandle, &Body<N>)> {
        self.bodies.iter().map(|b| (b.0, &**b.1))
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (DefaultBodyHandle, &mut Body<N>)> {
        self.bodies.iter_mut().map(|b| (b.0, &mut **b.1))
    }

    pub fn rigid_body(&self, handle: DefaultBodyHandle) -> Option<&RigidBody<N>> {
        self.get(handle).and_then(|b| b.downcast_ref())
    }

    pub fn rigid_body_mut(&mut self, handle: DefaultBodyHandle) -> Option<&mut RigidBody<N>> {
        self.get_mut(handle).and_then(|b| b.downcast_mut())
    }

    pub fn multibody(&self, handle: DefaultBodyHandle) -> Option<&Multibody<N>> {
        self.get(handle).and_then(|b| b.downcast_ref())
    }

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


pub type DefaultBodyHandle = generational_arena::Index;
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