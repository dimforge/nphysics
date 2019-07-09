#![allow(missing_docs)] // For downcast.

use downcast_rs::Downcast;
use generational_arena::Arena;
use na::{DVector, RealField};

use crate::object::{BodyPartHandle, BodySet, Body, BodyHandle, DefaultBodySet, DefaultBodyPartHandle};
use crate::solver::{LinearConstraints, IntegrationParameters, NonlinearConstraintGenerator};


pub trait JointConstraintSet<N: RealField, Bodies: BodySet<N>> {
    type JointConstraint: ?Sized + JointConstraint<N, Bodies>;
    type Handle: Copy;

    fn get(&self, handle: Self::Handle) -> Option<&Self::JointConstraint>;
    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::JointConstraint>;

    fn contains(&self, handle: Self::Handle) -> bool;

    fn foreach(&self, f: impl FnMut(Self::Handle, &Self::JointConstraint));
    fn foreach_mut(&mut self, f: impl FnMut(Self::Handle, &mut Self::JointConstraint));

    fn pop_insertion_event(&mut self) -> Option<(Self::Handle, BodyPartHandle<Bodies::Handle>, BodyPartHandle<Bodies::Handle>)>;
    fn pop_removal_event(&mut self) -> Option<(Self::Handle, BodyPartHandle<Bodies::Handle>, BodyPartHandle<Bodies::Handle>)>;
    fn remove(&mut self, to_remove: Self::Handle);
}

pub struct DefaultJointConstraintSet<N: RealField> {
    constraints: Arena<Box<JointConstraint<N, DefaultBodySet<N>>>>,
    inserted: Vec<(DefaultJointConstraintHandle, DefaultBodyPartHandle, DefaultBodyPartHandle)>,
    removed: Vec<(DefaultJointConstraintHandle, DefaultBodyPartHandle, DefaultBodyPartHandle)>,
}

impl<N: RealField> DefaultJointConstraintSet<N> {
    pub fn new() -> Self {
        DefaultJointConstraintSet {
            constraints: Arena::new(),
            inserted: Vec::new(),
            removed: Vec::new(),
        }
    }

    pub fn insert(&mut self, constraint: Box<JointConstraint<N, DefaultBodySet<N>>>) -> DefaultJointConstraintHandle {
        let (part1, part2) = constraint.anchors();
        let handle = self.constraints.insert(constraint);
        self.inserted.push((handle, part1, part2));
        handle
    }

    pub fn remove(&mut self, to_remove: DefaultJointConstraintHandle) -> Option<Box<JointConstraint<N, DefaultBodySet<N>>>> {
        let res = self.constraints.remove(to_remove)?;
        let (part1, part2) = res.anchors();
        self.removed.push((to_remove, part1, part2));
        Some(res)
    }

    pub fn get(&self, handle: DefaultJointConstraintHandle) -> Option<&JointConstraint<N, DefaultBodySet<N>>> {
        self.constraints.get(handle).map(|b| &**b)
    }

    pub fn get_mut(&mut self, handle: DefaultJointConstraintHandle) -> Option<&mut JointConstraint<N, DefaultBodySet<N>>> {
        self.constraints.get_mut(handle).map(|b| &mut **b)
    }

    pub fn iter(&self) -> impl Iterator<Item = (DefaultJointConstraintHandle, &JointConstraint<N, DefaultBodySet<N>>)> {
        self.constraints.iter().map(|b| (b.0, &**b.1))
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (DefaultJointConstraintHandle, &mut JointConstraint<N, DefaultBodySet<N>>)> {
        self.constraints.iter_mut().map(|b| (b.0, &mut **b.1))
    }
}

impl<N: RealField> JointConstraintSet<N, DefaultBodySet<N>> for DefaultJointConstraintSet<N> {
    type JointConstraint = JointConstraint<N, DefaultBodySet<N>>;
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

    fn pop_insertion_event(&mut self) -> Option<(Self::Handle, DefaultBodyPartHandle, DefaultBodyPartHandle)> {
        self.inserted.pop()
    }

    fn pop_removal_event(&mut self) -> Option<(Self::Handle, DefaultBodyPartHandle, DefaultBodyPartHandle)> {
        self.removed.pop()
    }

    fn remove(&mut self, to_remove: Self::Handle) {
        let _ = self.remove(to_remove);
    }
}


/// The handle of a constraint.
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
