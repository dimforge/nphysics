use std::mem;
use num::Zero;
use ncollide::utils::GenerationalId;

pub struct ImpulseCache<N> {
    cache: Vec<(GenerationalId, N)>,
}

impl<N: Copy + Zero> ImpulseCache<N> {
    pub fn new() -> Self {
        ImpulseCache { cache: Vec::new() }
    }

    pub fn clear(&mut self) {
        self.cache.clear();
    }

    pub fn get(&self, contact_id: GenerationalId) -> N {
        if contact_id.id < self.cache.len() && self.cache[contact_id.id].0 == contact_id {
            self.cache[contact_id.id].1
        } else {
            Zero::zero()
        }
    }

    pub fn set(&mut self, contact_id: GenerationalId, impulse: N) {
        if contact_id.id >= self.cache.len() {
            let zero = (GenerationalId::invalid(), N::zero());
            self.cache.resize(contact_id.id + 1, zero);
        }
        self.cache[contact_id.id] = (contact_id, impulse)
    }
}
