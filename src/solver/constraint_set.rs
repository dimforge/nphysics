use crate::object::{BodyHandle, ColliderHandle};
use crate::solver::{
    BilateralConstraint, BilateralGroundConstraint, NonlinearUnilateralConstraint,
    UnilateralConstraint, UnilateralGroundConstraint,
};
use na::RealField;

/// Set of velocity-based constraints.
pub struct LinearConstraints<N: RealField, Id> {
    /// Unilateral velocity constraints involving a dynamic body and the ground (or a body without any degrees of freedoms).
    pub unilateral_ground: Vec<UnilateralGroundConstraint<N, Id>>,
    /// Unilateral velocity constraints between dynamic bodies.
    pub unilateral: Vec<UnilateralConstraint<N, Id>>,
    /// Bilateral velocity constraints involving a dynamic body and the ground (or a body without any degrees of freedoms).
    pub bilateral_ground: Vec<BilateralGroundConstraint<N, Id>>,
    /// Bilateral velocity constraints between dynamic bodies.
    pub bilateral: Vec<BilateralConstraint<N, Id>>,
}

impl<N: RealField, Id> LinearConstraints<N, Id> {
    /// Creates a new empty set of constraints.
    pub fn new() -> Self {
        LinearConstraints {
            unilateral_ground: Vec::new(),
            unilateral: Vec::new(),
            bilateral_ground: Vec::new(),
            bilateral: Vec::new(),
        }
    }

    /// The total number of constraints on this set.
    pub fn len(&self) -> usize {
        self.unilateral_ground.len()
            + self.unilateral.len()
            + self.bilateral_ground.len()
            + self.bilateral.len()
    }

    /// Remove all constraints from this set.
    pub fn clear(&mut self) {
        self.unilateral_ground.clear();
        self.unilateral.clear();
        self.bilateral_ground.clear();
        self.bilateral.clear();
    }
}

/// Set of non-linear position-based constraints.
pub struct NonlinearConstraints<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle> {
    /// Unilateral position-based constraints between two bodies.
    pub unilateral: Vec<NonlinearUnilateralConstraint<N, Handle, CollHandle>>,
}

impl<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle>
    NonlinearConstraints<N, Handle, CollHandle>
{
    /// Create an empty set of nonlinear position-based constraints.
    pub fn new() -> Self {
        NonlinearConstraints {
            unilateral: Vec::new(),
        }
    }

    /// The total number of constraints on this set.
    pub fn len(&self) -> usize {
        self.unilateral.len()
    }

    /// Remove all constraints from this set.
    pub fn clear(&mut self) {
        self.unilateral.clear();
    }
}

/// A set of all velocity constraints and non-linear position-based constraints.
pub struct ConstraintSet<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle, Id> {
    /// The velocity constraints constructed.
    pub velocity: LinearConstraints<N, Id>,
    /// The position constraints constructed.
    pub position: NonlinearConstraints<N, Handle, CollHandle>,
}

impl<N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle, Id>
    ConstraintSet<N, Handle, CollHandle, Id>
{
    /// Create a new empty set of constraints.
    pub fn new() -> Self {
        ConstraintSet {
            velocity: LinearConstraints::new(),
            position: NonlinearConstraints::new(),
        }
    }

    /// The total number of constraints on this set.
    pub fn len(&self) -> usize {
        self.velocity.len() + self.position.len()
    }

    /// Remove all constraints from this set.
    pub fn clear(&mut self) {
        self.velocity.clear();
        self.position.clear();
    }
}
