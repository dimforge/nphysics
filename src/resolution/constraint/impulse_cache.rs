#![doc(hidden)]

use std::mem;
use std::hash::{Hash, Writer};
use std::hash::sip::{SipHasher, SipState};
use rand::{IsaacRng, Rng};
use std::collections::HashMap;
use nalgebra::na;
use nalgebra::na::{Iterable, IterableMut};
use ncollide::math::{Scalar, Vect};

#[deriving(PartialEq)]
/// The identifier of a contact stored in the impulse cache.
pub struct ContactIdentifier {
    obj1:    uint,
    obj2:    uint,
    ccenter: Vect
}

impl Eq for ContactIdentifier { } // NOTE: this is  wrong because of floats, but we dont care

impl Hash for ContactIdentifier {
    #[inline]
    #[cfg(f32)]
    fn hash(&self, state: &mut SipState) {
        for e in self.ccenter.iter() {
            unsafe { // FIXME: could we do this without going unsafe?
                let _ = state.write(mem::transmute::<f32, [u8, ..4]>(*e));
            }
        }
    }

    #[inline]
    #[cfg(f64)]
    fn hash(&self, state: &mut SipState) {
        for e in self.ccenter.iter() {
            unsafe { // FIXME: could we do this without going unsafe?
                let _ = state.write(mem::transmute::<f64, [u8, ..8]>(*e));
            }
        }
    }
}

impl ContactIdentifier {
    pub fn new(obj1: uint, obj2: uint, center: Vect, step: &Scalar) -> ContactIdentifier {
        let mut cell = center / *step;

        for x in cell.mut_iter() {
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
    hash_prev:           HashMap<ContactIdentifier, (uint, uint), SipHasher>,
    cache_prev:          Vec<Scalar>,
    hash_next:           HashMap<ContactIdentifier, (uint, uint), SipHasher>,
    cache_next:          Vec<Scalar>,
    step:                Scalar,
    impulse_per_contact: uint
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

    pub fn hash<'a>(&'a self) -> &'a HashMap<ContactIdentifier, (uint, uint), SipHasher> {
        &self.hash_next
    }

    pub fn hash_mut<'a>(&'a mut self) -> &'a mut HashMap<ContactIdentifier, (uint, uint), SipHasher> {
        &mut self.hash_next
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
