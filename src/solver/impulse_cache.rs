use ncollide::utils::GenerationalId;
use num::Zero;
use std::ops::{Index, IndexMut};

/// A cache for impulses.
pub struct ImpulseCache<N> {
    cache: Vec<(GenerationalId, N)>,
}

impl<N: Copy + Zero> ImpulseCache<N> {
    /// Create a new empty cache.
    pub fn new() -> Self {
        ImpulseCache { cache: Vec::new() }
    }

    /// Clear the cache.
    pub fn clear(&mut self) {
        self.cache.clear();
    }

    /// The number of impulses on this cache.
    pub fn len(&self) -> usize {
        self.cache.len()
    }

    /// Test if the cache already contains the specified contact.
    pub fn contains(&self, contact_id: GenerationalId) -> bool {
        contact_id.id < self.cache.len() && self.cache[contact_id.id].0 == contact_id
    }

    /// Get the impulse stored for the specified contact.
    ///
    /// Returns 0 if no cache entry is registered for this contact.
    pub fn get(&self, contact_id: GenerationalId) -> N {
        if self.contains(contact_id) {
            self.cache[contact_id.id].1
        } else {
            Zero::zero()
        }
    }

    /// Retrieve the index on the cache vector associated to the given contact.
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
