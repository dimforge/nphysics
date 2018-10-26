use na::Real;
use solver::{
    BilateralConstraint, BilateralGroundConstraint,
    MultibodyJointLimitsNonlinearConstraintGenerator, NonlinearUnilateralConstraint,
    UnilateralConstraint, UnilateralGroundConstraint,
};

/// Set of velocity-based constraints.
#[derive(Default)]
pub struct Constraints<N: Real> {
    /// Unilateral velocity constraints involving a dynamic body and the ground (or a body without any degrees of freedoms).
    pub unilateral_ground: Vec<UnilateralGroundConstraint<N>>,
    /// Unilateral velocity constraints between dynamic bodies.
    pub unilateral: Vec<UnilateralConstraint<N>>,
    /// Bilateral velocity constraints involving a dynamic body and the ground (or a body without any degrees of freedoms).
    pub bilateral_ground: Vec<BilateralGroundConstraint<N>>,
    /// Bilateral velocity constraints between dynamic bodies.
    pub bilateral: Vec<BilateralConstraint<N>>,
}

impl<N: Real> Constraints<N> {
    /// Creates a new empty set of constraints.
    pub fn new() -> Self {
        Constraints {
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
pub struct NonlinearConstraints<N: Real> {
    /// Unilateral position-based constraints between two bodies.
    pub unilateral: Vec<NonlinearUnilateralConstraint<N>>,
    /// Position-based constraints for joint limits.
    pub multibody_limits: Vec<MultibodyJointLimitsNonlinearConstraintGenerator>,
}

impl<N: Real> NonlinearConstraints<N> {
    /// Create an empty set of nonlinear position-based constraints.
    pub fn new() -> Self {
        NonlinearConstraints {
            unilateral: Vec::new(),
            multibody_limits: Vec::new(),
        }
    }

    /// The total number of constraints on this set.
    pub fn len(&self) -> usize {
        self.unilateral.len() + self.multibody_limits.len()
    }

    /// Remove all constraints from this set.
    pub fn clear(&mut self) {
        self.unilateral.clear();
        self.multibody_limits.clear();
    }
}

/// A set of all velocity constraints and non-linear position-based constraints.
pub struct ConstraintSet<N: Real> {
    /// The velocity constraints constructed.
    pub velocity: Constraints<N>,
    /// The position constraints constructed.
    pub position: NonlinearConstraints<N>,
}

impl<N: Real> ConstraintSet<N> {
    /// Create a new empty set of constraints.
    pub fn new() -> Self {
        ConstraintSet {
            velocity: Constraints::new(),
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
