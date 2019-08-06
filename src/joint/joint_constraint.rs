#![allow(missing_docs)] // For downcast.

use downcast_rs::Downcast;
use generational_arena::Arena;
use na::{DVector, RealField};

use crate::object::{BodyPartHandle, BodySet, Body, DefaultBodySet};
use crate::solver::{LinearConstraints, IntegrationParameters, NonlinearConstraintGenerator};

/// Trait implemented by sets of constraint-based joints.
///
/// A set of constraint-based joints maps a joint handle to a joint instance. In addition, it must maintain a set of
/// joint handle of joints that have been inserted removed (see the `pop_insertion_event` and `pop_removal_event` methods for details).
pub trait JointConstraintSet<N: RealField, Bodies: BodySet<N>> {
    /// Type of a constraint-based joints stored in this set.
    type JointConstraint: ?Sized + JointConstraint<N, Bodies>;
    /// Type of a joint handle identifying a joint in this set.
    type Handle: Copy;

    /// Gets a reference to the joint identified by `handle`.
    fn get(&self, handle: Self::Handle) -> Option<&Self::JointConstraint>;
    /// Gets a mutable reference to the joint identified by `handle`.
    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::JointConstraint>;


    /// Check if this set contains a joint identified by `handle`.
    fn contains(&self, handle: Self::Handle) -> bool;

    /// Iterate through all the bodies on this set, applying the closure `f` on them.
    fn foreach(&self, f: impl FnMut(Self::Handle, &Self::JointConstraint));
    /// Mutable iterates through all the bodies on this set, applying the closure `f` on them.
    fn foreach_mut(&mut self, f: impl FnMut(Self::Handle, &mut Self::JointConstraint));

    /// Gets the handle of one joint that has been inserted.
    ///
    /// A joint set must keep track (using typically a stack or a queue) of every joint that has been
    /// inserted into it. This is used by nphysics to perform some internal setup actions, or
    /// physical actions like waking bodies attached to this joint.
    ///
    /// This method should return a removed joint handle only once.
    fn pop_insertion_event(&mut self) -> Option<(Self::Handle, BodyPartHandle<Bodies::Handle>, BodyPartHandle<Bodies::Handle>)>;
    /// Gets the handle of one joint that has been removed.
    ///
    /// A joint set must keep track (using typically a stack or a queue) of every joint that has been
    /// removed from it. This is used by nphysics to perform some internal cleanup actions, or
    /// physical actions like waking bodies that were attached to this joint.
    fn pop_removal_event(&mut self) -> Option<(Self::Handle, BodyPartHandle<Bodies::Handle>, BodyPartHandle<Bodies::Handle>)>;
    /// Remove a joint from this set.
    ///
    /// A constraint-based joint can be removed automatically by nphysics when one of its attached
    /// bodies is removed from the mechanical world.
    fn remove(&mut self, to_remove: Self::Handle);
}

/// A set containing all the joint-constraints added to the world.
///
/// It is based on an arena using generational indices to avoid the ABA problem.
pub struct DefaultJointConstraintSet<N: RealField, Bodies: BodySet<N> = DefaultBodySet<N>> {
    constraints: Arena<Box<dyn JointConstraint<N, Bodies>>>,
    inserted: Vec<(DefaultJointConstraintHandle, BodyPartHandle<Bodies::Handle>, BodyPartHandle<Bodies::Handle>)>,
    removed: Vec<(DefaultJointConstraintHandle, BodyPartHandle<Bodies::Handle>, BodyPartHandle<Bodies::Handle>)>,
}

impl<N: RealField, Bodies: BodySet<N>> DefaultJointConstraintSet<N, Bodies> {
    /// Creates an empty set.
    pub fn new() -> Self {
        DefaultJointConstraintSet {
            constraints: Arena::new(),
            inserted: Vec::new(),
            removed: Vec::new(),
        }
    }

    /// Adds a joint to this set.
    pub fn insert(&mut self, constraint: impl JointConstraint<N, Bodies>) -> DefaultJointConstraintHandle {
        self.insert_boxed(Box::new(constraint))
    }

    /// Adds a joint (represented as a boxed trait-object) to this set.
    pub fn insert_boxed(&mut self, constraint: Box<dyn JointConstraint<N, Bodies>>) -> DefaultJointConstraintHandle {
        let (part1, part2) = constraint.anchors();
        let handle = self.constraints.insert(constraint);
        self.inserted.push((handle, part1, part2));
        handle
    }

    /// Removes a joint from this set.
    pub fn remove(&mut self, to_remove: DefaultJointConstraintHandle) -> Option<Box<dyn JointConstraint<N, Bodies>>> {
        let res = self.constraints.remove(to_remove)?;
        let (part1, part2) = res.anchors();
        self.removed.push((to_remove, part1, part2));
        Some(res)
    }

    /// Check if this set contains a joint identified by `handle`.
    pub fn contains(&self, handle: DefaultJointConstraintHandle) -> bool {
        self.constraints.contains(handle)
    }

    /// Gets a reference to the joint identified by `handle`.
    pub fn get(&self, handle: DefaultJointConstraintHandle) -> Option<&dyn JointConstraint<N, Bodies>> {
        self.constraints.get(handle).map(|b| &**b)
    }

    /// Gets a mutable reference to the joint identified by `handle`.
    pub fn get_mut(&mut self, handle: DefaultJointConstraintHandle) -> Option<&mut dyn JointConstraint<N, Bodies>> {
        self.constraints.get_mut(handle).map(|b| &mut **b)
    }

    /// Iter through all the joints and their handles.
    pub fn iter(&self) -> impl Iterator<Item = (DefaultJointConstraintHandle, &dyn JointConstraint<N, Bodies>)> {
        self.constraints.iter().map(|b| (b.0, &**b.1))
    }

    /// Mutably iter through all the joints and their handles.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (DefaultJointConstraintHandle, &mut dyn JointConstraint<N, Bodies>)> {
        self.constraints.iter_mut().map(|b| (b.0, &mut **b.1))
    }
}

impl<N: RealField, Bodies: BodySet<N> + 'static> JointConstraintSet<N, Bodies> for DefaultJointConstraintSet<N, Bodies> {
    type JointConstraint = dyn JointConstraint<N, Bodies>;
    type Handle = DefaultJointConstraintHandle;

    fn get(&self, handle: Self::Handle) -> Option<&Self::JointConstraint> {
        self.get(handle)
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::JointConstraint> {
        self.get_mut(handle)
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.contains(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &Self::JointConstraint)) {
        for (h, b) in self.iter() {
            f(h, b)
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Self::Handle, &mut Self::JointConstraint)) {
        for (h, b) in self.iter_mut() {
            f(h, b)
        }
    }

    fn pop_insertion_event(&mut self) -> Option<(Self::Handle, BodyPartHandle<Bodies::Handle>, BodyPartHandle<Bodies::Handle>)> {
        self.inserted.pop()
    }

    fn pop_removal_event(&mut self) -> Option<(Self::Handle, BodyPartHandle<Bodies::Handle>, BodyPartHandle<Bodies::Handle>)> {
        self.removed.pop()
    }

    fn remove(&mut self, to_remove: Self::Handle) {
        let _ = self.remove(to_remove);
    }
}


/// The handle of a joint on a `DefaultJointConstraintsSet`.
pub type DefaultJointConstraintHandle = generational_arena::Index;

/// Trait implemented by joint that operate by generating constraints to restrict the relative motion of two body parts.
pub trait JointConstraint<N: RealField, Bodies: BodySet<N>>: NonlinearConstraintGenerator<N, Bodies> + Downcast + Send + Sync {
    /// Return `true` if the constraint is active.
    ///
    /// Typically, a constraint is disable if it is between two sleeping bodies, or, between bodies without any degrees of freedom.
    fn is_active(&self, bodies: &Bodies) -> bool {
        let (b1, b2) = self.anchors();
        let body1 = try_ret!(bodies.get(b1.0), false);
        let body2 = try_ret!(bodies.get(b2.0), false);

        let ndofs1 = body1.status_dependent_ndofs();
        let ndofs2 = body2.status_dependent_ndofs();

        (ndofs1 != 0 && body1.is_active()) || (ndofs2 != 0 && body2.is_active())
    }

    /// The maximum number of velocity constraints generated by this joint.
    fn num_velocity_constraints(&self) -> usize;
    /// The two body parts affected by this joint.
    fn anchors(&self) -> (BodyPartHandle<Bodies::Handle>, BodyPartHandle<Bodies::Handle>);
    /// Initialize and retrieve all the constraints appied to the bodies attached to this joint.
    fn velocity_constraints(
        &mut self,
        parameters: &IntegrationParameters<N>,
        bodies: &Bodies,
        ext_vels: &DVector<N>,
        ground_j_id: &mut usize,
        j_id: &mut usize,
        jacobians: &mut [N],
        velocity_constraints: &mut LinearConstraints<N, usize>,
    );
    /// Called after velocity constraint resolution, allows the joint to keep a cache of impulses generated for each constraint.
    fn cache_impulses(&mut self, constraints: &LinearConstraints<N, usize>);
}

impl_downcast!(JointConstraint<N, Bodies> where N: RealField, Bodies: BodySet<N>);
