#![doc(hidden)]

use std::iter;
use std::mem;
use std::hash::{Hash, Hasher};
use std::collections::HashMap;
use num::Float;
use na;
use na::IterableMut;
use math::{Scalar, Point};
use ncollide::utils::AsBytes;
use utils::DeterministicState;

#[derive(PartialEq)]
/// The identifier of a contact stored in the impulse cache.
pub struct ContactIdentifier {
    obj1:    usize,
    obj2:    usize,
    ccenter: Point
}

impl Eq for ContactIdentifier { } // NOTE: this is  wrong because of floats, but we dont care

impl Hash for ContactIdentifier {
    #[inline]
    fn hash<H: Hasher>(&self, state: &mut H) {
        state.write(self.ccenter.as_bytes())
    }
}

impl ContactIdentifier {
    pub fn new(obj1: usize, obj2: usize, center: Point, step: &Scalar) -> ContactIdentifier {
        let mut cell = center / *step;

        for x in cell.iter_mut() {
            *x = x.trunc()
        }

        ContactIdentifier {
            obj1:    obj1,
            obj2:    obj2,
            ccenter: cell
        }
    }
}

pub struct ImpulseCache {
    // XXX: simulations won't be reproductible because of the randomized HashMap.
    hash_prev:           HashMap<ContactIdentifier, (usize, usize), DeterministicState>,
    cache_prev:          Vec<Scalar>,
    hash_next:           HashMap<ContactIdentifier, (usize, usize), DeterministicState>,
    cache_next:          Vec<Scalar>,
    step:                Scalar,
    impulse_per_contact: usize
}

impl ImpulseCache {
    pub fn new(step: Scalar, impulse_per_contact: usize) -> ImpulseCache {

        ImpulseCache {
            hash_prev:           HashMap::with_capacity_and_hasher(32, DeterministicState::new()),
            hash_next:           HashMap::with_capacity_and_hasher(32, DeterministicState::new()),
            cache_prev:          iter::repeat(na::zero()).take(impulse_per_contact).collect(),
            cache_next:          iter::repeat(na::zero()).take(impulse_per_contact).collect(),
            step:                step,
            impulse_per_contact: impulse_per_contact
        }
    }

    pub fn insert(&mut self, cid: usize, obj1: usize, obj2: usize, center: Point) {
        let id = ContactIdentifier::new(obj1, obj2, center, &self.step);
        let imp =
            match self.hash_prev.get(&id).cloned() {
                Some((_, i)) => i,
                None         => 0
            };

        let _ = self.hash_next.insert(id, (cid, imp));
    }

    pub fn hash(&self) -> &HashMap<ContactIdentifier, (usize, usize), DeterministicState> {
        &self.hash_next
    }

    pub fn hash_mut(&mut self) -> &mut HashMap<ContactIdentifier, (usize, usize), DeterministicState> {
        &mut self.hash_next
    }

    pub fn push_impulsions(&mut self) -> &mut [Scalar] {
        let begin = self.cache_next.len();

        for _ in 0 .. self.impulse_per_contact {
            self.cache_next.push(na::zero());
        }

        let end = self.cache_next.len();

        &mut self.cache_next[begin .. end]
    }

    pub fn reserved_impulse_offset(&self) -> usize {
        self.impulse_per_contact
    }

    pub fn impulsions_at(&self, at: usize) -> &[Scalar] {
        &self.cache_prev[at .. at + self.impulse_per_contact]
    }

    pub fn len(&self) -> usize {
        self.hash_next.len()
    }

    pub fn clear(&mut self) {
        self.cache_prev.clear();
        self.hash_prev.clear();
        self.cache_next.clear();
        self.hash_next.clear();

        self.cache_prev.extend(iter::repeat(na::zero::<Scalar>()).take(self.impulse_per_contact));
        self.cache_next.extend(iter::repeat(na::zero::<Scalar>()).take(self.impulse_per_contact));
    }

    pub fn swap(&mut self) {
        mem::swap(&mut self.hash_prev, &mut self.hash_next);
        mem::swap(&mut self.cache_prev,&mut self.cache_next);
        self.hash_next.clear();
        self.cache_next.truncate(self.impulse_per_contact);
    }
}
