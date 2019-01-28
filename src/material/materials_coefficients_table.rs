use std::collections::HashMap;
use na::Real;
use ncollide::utils::SortedPair;

use crate::material::MaterialId;


pub struct MaterialsCoefficientsTable<N: Real> {
    friction: HashMap<SortedPair<u32>, N>,
    restitution: HashMap<SortedPair<u32>, N>,
}

impl<N: Real> MaterialsCoefficientsTable<N> {
    pub fn new() -> Self {
        MaterialsCoefficientsTable {
            friction: HashMap::new(),
            restitution: HashMap::new(),
        }
    }

    pub fn set_friction_coefficient(&mut self, m1: MaterialId, m2: MaterialId, coeff: N) {
        let _ = self.friction.insert(SortedPair::new(m1, m2), coeff);
    }

    pub fn set_restitution_coefficient(&mut self, m1: MaterialId, m2: MaterialId, coeff: N) {
        let _ = self.restitution.insert(SortedPair::new(m1, m2), coeff);
    }

    pub fn friction_coefficient(&self, m1: MaterialId, m2: MaterialId) -> Option<N> {
        self.friction.get(&SortedPair::new(m1, m2)).cloned()
    }

    pub fn restitution_coefficient(&self, m1: MaterialId, m2: MaterialId) -> Option<N> {
        self.restitution.get(&SortedPair::new(m1, m2)).cloned()
    }
}