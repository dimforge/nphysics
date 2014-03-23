#[doc(hidden)];

use std::mem;
use std::hash::Hash;
use std::hash::sip::{SipHasher, SipState};
use rand::{IsaacRng, Rng};
use collections::HashMap;
use nalgebra::na;
use nalgebra::na::Iterable;
use ncollide::math::{Scalar, Vect};

#[deriving(Eq)]
/// The identifier of a contact stored in the impulse cache.
pub struct ContactIdentifier {
    priv obj1:    uint,
    priv obj2:    uint,
    priv ccenter: Vect
}

impl Hash for ContactIdentifier {
    #[inline]
    #[cfg(f32)]
    fn hash(&self, state: &mut SipState) {
        for e in self.ccenter.iter() {
            let _ = state.write_le_f32(*e);
        }
    }

    #[inline]
    #[cfg(f64)]
    fn hash(&self, state: &mut SipState) {
        for e in self.ccenter.iter() {
            let _ = state.write_le_f64(*e);
        }
    }
}

impl ContactIdentifier {
    pub fn new(obj1: uint, obj2: uint, center: Vect, step: &Scalar) -> ContactIdentifier {
        ContactIdentifier {
            obj1:    obj1,
            obj2:    obj2,
            ccenter: (center / *step).trunc()
        }
    }
}

pub struct ImpulseCache {
    priv hash_prev:           HashMap<ContactIdentifier, (uint, uint)>,
    priv cache_prev:          Vec<Scalar>,
    priv hash_next:           HashMap<ContactIdentifier, (uint, uint)>,
    priv cache_next:          Vec<Scalar>,
    priv step:                Scalar,
    priv impulse_per_contact: uint
}

impl ImpulseCache {
    pub fn new(step: Scalar, impulse_per_contact: uint) -> ImpulseCache {
        let mut rng = IsaacRng::new_unseeded();

        ImpulseCache {
            hash_prev:           HashMap::with_capacity_and_hasher(32, SipHasher::new_with_keys(rng.gen(), rng.gen())),
            hash_next:           HashMap::with_capacity_and_hasher(32, SipHasher::new_with_keys(rng.gen(), rng.gen())),
            cache_prev:          Vec::from_elem(impulse_per_contact, na::zero()),
            cache_next:          Vec::from_elem(impulse_per_contact, na::zero()),
            step:                step,
            impulse_per_contact: impulse_per_contact
        }
    }

    pub fn insert<'a>(&'a mut self, cid: uint, obj1: uint, obj2: uint, center: Vect) {
        let id = ContactIdentifier::new(obj1, obj2, center, &self.step);
        let imp =
            match self.hash_prev.find_copy(&id) {
                Some((_, i)) => i,
                None         => 0
            };

        self.hash_next.insert(id, (cid, imp));
    }

    pub fn hash<'a>(&'a self) -> &'a HashMap<ContactIdentifier, (uint, uint)> {
        &'a self.hash_next
    }

    pub fn hash_mut<'a>(&'a mut self) -> &'a mut HashMap<ContactIdentifier, (uint, uint)> {
        &'a mut self.hash_next
    }

    pub fn push_impulsions<'a>(&'a mut self) -> &'a mut [Scalar] {
        let begin = self.cache_next.len();

        for _ in range(0, self.impulse_per_contact) {
            self.cache_next.push(na::zero());
        }

        let end = self.cache_next.len();

        self.cache_next.mut_slice(begin, end)
    }

    pub fn reserved_impulse_offset(&self) -> uint {
        self.impulse_per_contact
    }

    pub fn impulsions_at<'a>(&'a self, at: uint) -> &'a [Scalar] {
        self.cache_prev.slice(at, at + self.impulse_per_contact)
    }

    pub fn len(&self) -> uint {
        self.hash_next.len()
    }

    pub fn clear(&mut self) {
        self.cache_prev.clear();
        self.hash_prev.clear();
        self.cache_next.clear();
        self.hash_next.clear();

        self.cache_prev.grow_set(self.impulse_per_contact, &na::zero(), na::zero());
        self.cache_next.grow_set(self.impulse_per_contact, &na::zero(), na::zero());
    }

    pub fn swap(&mut self) {
        mem::swap(&mut self.hash_prev, &mut self.hash_next);
        mem::swap(&mut self.cache_prev,&mut self.cache_next);
        self.hash_next.clear();
        self.cache_next.truncate(self.impulse_per_contact);
    }
}
