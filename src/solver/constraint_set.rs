use na::Real;
use solver::{BilateralConstraint, BilateralGroundConstraint, UnilateralConstraint,
             UnilateralGroundConstraint};

pub struct ConstraintSet<N: Real> {
    pub unilateral_ground_constraints: Vec<UnilateralGroundConstraint<N>>,
    pub unilateral_constraints: Vec<UnilateralConstraint<N>>,
    pub bilateral_ground_constraints: Vec<BilateralGroundConstraint<N>>,
    pub bilateral_constraints: Vec<BilateralConstraint<N>>,
}

pub struct ConstraintSetIndices {
    pub first_unilateral_constraint: usize,
    pub first_unilateral_ground_constraint: usize,
    pub first_bilateral_constraint: usize,
    pub first_bilateral_ground_constraint: usize,
}

impl<N: Real> ConstraintSet<N> {
    pub fn new() -> Self {
        ConstraintSet {
            unilateral_ground_constraints: Vec::new(),
            unilateral_constraints: Vec::new(),
            bilateral_ground_constraints: Vec::new(),
            bilateral_constraints: Vec::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.unilateral_constraints.len() + self.unilateral_ground_constraints.len()
            + self.bilateral_ground_constraints.len() + self.bilateral_constraints.len()
    }

    pub fn clear(&mut self) {
        self.unilateral_ground_constraints.clear();
        self.unilateral_constraints.clear();
        self.bilateral_ground_constraints.clear();
        self.bilateral_constraints.clear();
    }

    pub fn current_indices(&self) -> ConstraintSetIndices {
        ConstraintSetIndices {
            first_unilateral_ground_constraint: self.unilateral_ground_constraints.len(),
            first_unilateral_constraint: self.unilateral_constraints.len(),
            first_bilateral_ground_constraint: self.bilateral_ground_constraints.len(),
            first_bilateral_constraint: self.bilateral_constraints.len(),
        }
    }
}
