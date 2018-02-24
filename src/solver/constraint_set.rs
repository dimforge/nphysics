use na::Real;
use solver::{BilateralConstraint, BilateralGroundConstraint, UnilateralConstraint,
             UnilateralGroundConstraint};

pub struct Constraints<N: Real> {
    pub unilateral_ground: Vec<UnilateralGroundConstraint<N>>,
    pub unilateral: Vec<UnilateralConstraint<N>>,
    pub bilateral_ground: Vec<BilateralGroundConstraint<N>>,
    pub bilateral: Vec<BilateralConstraint<N>>,
}

impl<N: Real> Constraints<N> {
    pub fn new() -> Self {
        Constraints {
            unilateral_ground: Vec::new(),
            unilateral: Vec::new(),
            bilateral_ground: Vec::new(),
            bilateral: Vec::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.unilateral_ground.len() + self.unilateral.len() + self.bilateral_ground.len()
            + self.bilateral.len()
    }

    pub fn clear(&mut self) {
        self.unilateral_ground.clear();
        self.unilateral.clear();
        self.bilateral_ground.clear();
        self.bilateral.clear();
    }
}

pub struct ConstraintSet<N: Real> {
    pub velocity: Constraints<N>,
    pub position: Constraints<N>,
}

impl<N: Real> ConstraintSet<N> {
    pub fn new() -> Self {
        ConstraintSet {
            velocity: Constraints::new(),
            position: Constraints::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.velocity.len() + self.position.len()
    }

    pub fn clear(&mut self) {
        self.position.clear();
        self.velocity.clear();
    }
}
