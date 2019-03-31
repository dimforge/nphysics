use std::collections::HashMap;
use na::RealField;
use ncollide::utils::SortedPair;

use crate::material::MaterialId;


/// A lookup table for friction and restitution coefficient associated to certain pairs of materials.
pub struct MaterialsCoefficientsTable<N: RealField> {
    friction: HashMap<SortedPair<u32>, N>,
    restitution: HashMap<SortedPair<u32>, N>,
}

impl<N: RealField> MaterialsCoefficientsTable<N> {
    /// Initialize an empty table of friction and restitution coefficients.
    pub fn new() -> Self {
        MaterialsCoefficientsTable {
            friction: HashMap::new(),
            restitution: HashMap::new(),
        }
    }

    /// Sets the friction coefficient associated to this pair of materials.
    pub fn set_friction_coefficient(&mut self, m1: MaterialId, m2: MaterialId, coeff: N) {
        let _ = self.friction.insert(SortedPair::new(m1, m2), coeff);
    }

    /// Sets the restitution coefficient associated to this pair of materials.
    pub fn set_restitution_coefficient(&mut self, m1: MaterialId, m2: MaterialId, coeff: N) {
        let _ = self.restitution.insert(SortedPair::new(m1, m2), coeff);
    }

    /// Retrieves the friction coefficient associated to this pair of materials.
    ///
    /// Returns `None` if no coefficient was associated to this pair.
    pub fn friction_coefficient(&self, m1: MaterialId, m2: MaterialId) -> Option<N> {
        self.friction.get(&SortedPair::new(m1, m2)).cloned()
    }

    /// Retrieves the restitution coefficient associated to this pair of materials.
    ///
    /// Returns `None` if no coefficient was associated to this pair.
    pub fn restitution_coefficient(&self, m1: MaterialId, m2: MaterialId) -> Option<N> {
        self.restitution.get(&SortedPair::new(m1, m2)).cloned()
    }
}