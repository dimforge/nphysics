use std::ops::{Index, IndexMut};
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

    pub fn contains(&self, contact_id: GenerationalId) -> bool {
        contact_id.id < self.cache.len() && self.cache[contact_id.id].0 == contact_id
    }

    pub fn get(&self, contact_id: GenerationalId) -> N {
        if self.contains(contact_id) {
            self.cache[contact_id.id].1
        } else {
            Zero::zero()
        }
    }

    pub fn entry_id(&mut self, contact_id: GenerationalId) -> usize {
        if contact_id.id >= self.cache.len() {
            let zero = (GenerationalId::invalid(), N::zero());
            self.cache.resize(contact_id.id + 1, zero);
        }
        self.cache[contact_id.id].0 = contact_id;
        contact_id.id
    }
}

impl<N> Index<usize> for ImpulseCache<N> {
    type Output = N;

    #[inline]
    fn index(&self, i: usize) -> &N {
        &self.cache[i].1
    }
}

impl<N> IndexMut<usize> for ImpulseCache<N> {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut N {
        &mut self.cache[i].1
    }
}
