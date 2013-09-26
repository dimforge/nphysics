use std::util;
use std::rand::{IsaacRng, Rng};
use std::num::Zero;
use std::vec;
use nalgebra::vec::Vec;
use std::hashmap::HashMap;

// This is a seed generated some day by the seed() function.
// It is a good seed for the IsaacRng generator we use to initialize the HashMap.
static seed: [u8, .. 1024] =
    [ 236, 159, 4, 254, 23, 58, 193, 215, 21, 219, 217, 172, 73, 130, 51, 136, 136, 203, 179, 78,
    27, 123, 207, 67, 3, 53, 12, 80, 159, 246, 149, 238, 167, 9, 188, 65, 25, 13, 205, 99, 124,
    143, 151, 203, 13, 181, 79, 71, 17, 145, 102, 191, 2, 7, 32, 122, 25, 111, 178, 242, 74, 44,
    103, 196, 208, 113, 1, 56, 29, 46, 63, 39, 66, 38, 178, 128, 142, 169, 217, 165, 34, 109, 73,
    68, 204, 39, 166, 90, 70, 166, 102, 163, 69, 41, 226, 52, 179, 60, 130, 84, 238, 80, 120, 93,
    94, 35, 69, 51, 8, 155, 222, 64, 87, 218, 201, 167, 106, 183, 176, 143, 134, 60, 34, 69, 140,
    13, 28, 161, 213, 48, 216, 139, 170, 93, 40, 229, 43, 58, 141, 0, 103, 107, 248, 51, 242, 129,
    110, 228, 51, 138, 198, 120, 193, 132, 208, 88, 91, 181, 207, 157, 34, 208, 252, 47, 214, 208,
    19, 40, 78, 181, 111, 229, 178, 135, 11, 67, 94, 206, 206, 94, 202, 19, 129, 6, 240, 99, 209,
    55, 182, 137, 19, 103, 58, 238, 118, 15, 42, 37, 175, 14, 61, 175, 107, 85, 253, 228, 49, 51,
    202, 119, 74, 209, 127, 51, 207, 35, 51, 78, 156, 145, 149, 55, 159, 90, 63, 245, 143, 95, 161,
    191, 186, 183, 71, 217, 206, 210, 199, 212, 124, 146, 143, 177, 41, 125, 197, 46, 41, 7, 209,
    122, 230, 185, 52, 53, 56, 131, 28, 88, 94, 217, 123, 137, 142, 179, 254, 91, 47, 113, 21, 52,
    207, 17, 48, 26, 163, 190, 1, 217, 206, 168, 13, 207, 127, 167, 141, 239, 43, 149, 188, 96,
    194, 174, 25, 179, 118, 199, 199, 129, 60, 55, 41, 42, 165, 156, 0, 251, 216, 252, 168, 181,
    30, 40, 0, 182, 63, 36, 224, 86, 214, 62, 51, 2, 51, 72, 146, 199, 167, 240, 102, 55, 130, 37,
    235, 234, 156, 148, 34, 17, 203, 208, 171, 214, 14, 147, 110, 118, 237, 211, 184, 82, 82, 90,
    62, 8, 55, 40, 147, 200, 230, 6, 127, 16, 9, 149, 66, 21, 189, 67, 155, 29, 112, 123, 156, 207,
    225, 27, 249, 26, 203, 128, 215, 73, 29, 110, 222, 224, 44, 13, 13, 48, 135, 20, 194, 182, 129,
    193, 117, 101, 118, 217, 30, 243, 191, 240, 81, 116, 7, 79, 237, 78, 37, 53, 100, 26, 214, 36,
    155, 130, 4, 137, 13, 241, 49, 124, 90, 228, 12, 15, 53, 222, 170, 9, 180, 98, 80, 149, 16,
    146, 247, 52, 84, 147, 12, 74, 203, 112, 109, 16, 138, 174, 122, 14, 2, 163, 25, 19, 216, 89,
    204, 168, 202, 216, 72, 187, 123, 31, 78, 162, 144, 206, 238, 85, 245, 3, 7, 94, 29, 107, 52,
    69, 74, 72, 131, 56, 9, 249, 41, 108, 72, 39, 98, 186, 54, 145, 195, 9, 52, 63, 21, 4, 13, 122,
    237, 196, 84, 217, 48, 130, 220, 45, 85, 212, 248, 72, 221, 60, 226, 35, 210, 69, 9, 203, 76,
    34, 94, 93, 109, 142, 13, 124, 153, 157, 142, 48, 179, 79, 235, 38, 111, 197, 84, 240, 142, 34,
    199, 216, 133, 1, 52, 87, 61, 192, 180, 99, 252, 95, 236, 11, 121, 188, 133, 201, 205, 200, 18,
    110, 76, 59, 241, 112, 247, 166, 195, 102, 216, 234, 44, 254, 254, 122, 73, 249, 34, 27, 99,
    86, 126, 77, 226, 146, 140, 92, 123, 74, 105, 42, 45, 180, 161, 137, 214, 45, 135, 188, 194,
    214, 151, 102, 69, 255, 74, 170, 119, 188, 7, 29, 2, 252, 183, 74, 167, 111, 125, 219, 105,
    166, 214, 240, 129, 240, 142, 49, 149, 167, 77, 178, 40, 244, 198, 47, 247, 130, 90, 71, 58,
    231, 102, 235, 82, 148, 146, 242, 65, 74, 243, 102, 201, 169, 64, 136, 157, 70, 186, 142, 190,
    227, 212, 38, 103, 83, 247, 206, 97, 140, 130, 177, 19, 26, 94, 16, 93, 130, 138, 11, 61, 132,
    95, 45, 71, 166, 15, 86, 238, 157, 162, 126, 177, 17, 112, 229, 198, 8, 31, 205, 222, 112, 124,
    130, 2, 201, 251, 224, 213, 27, 102, 133, 133, 204, 192, 148, 156, 122, 248, 216, 165, 11, 224,
    123, 99, 191, 61, 211, 13, 214, 85, 186, 192, 155, 198, 63, 221, 11, 237, 122, 114, 60, 253,
    55, 195, 68, 54, 176, 96, 186, 123, 38, 145, 59, 9, 117, 9, 243, 55, 182, 230, 207, 213, 6,
    176, 143, 145, 47, 96, 47, 75, 49, 228, 196, 245, 14, 104, 41, 107, 208, 164, 126, 197, 173,
    146, 1, 189, 19, 34, 126, 104, 232, 145, 157, 31, 155, 96, 158, 150, 1, 216, 90, 24, 200, 79,
    85, 51, 155, 248, 2, 136, 139, 16, 49, 91, 206, 199, 170, 250, 47, 73, 88, 199, 200, 12, 5,
    181, 69, 190, 175, 107, 243, 215, 3, 33, 52, 253, 62, 194, 18, 221, 58, 95, 163, 117, 102, 212,
    154, 199, 58, 94, 106, 5, 85, 0, 238, 15, 2, 62, 22, 144, 197, 83, 172, 5, 44, 161, 61, 107,
    136, 176, 208, 82, 212, 75, 61, 47, 147, 60, 114, 113, 30, 151, 60, 119, 235, 195, 77, 111, 15,
    182, 107, 95, 207, 55, 158, 56, 246, 4, 202, 159, 8, 71, 81, 99, 43, 99, 231, 163, 127, 69,
    142, 25, 37, 157, 199, 215, 226, 33, 156, 232, 55, 100, 213, 152, 15, 201, 121, 68, 115, 88,
    215, 31, 44, 30, 246, 252, 58, 28, 144, 10, 38, 246, 21, 125, 7, 3, 186, 236, 227, 210, 6, 251,
    120, 230, 39, 41, 226, 129, 36, 12, 110, 105, 134, 170, 215, 60, 252, 29, 15, 2, 168, 23, 234,
    246, 160, 226, 171, 153, 69, 173, 182, 181, 194, 19, 133, 62, 11, 181, 20, 154, 72, 45, 172,
    109, 93, 158, 161, 191, 10, 87, 129, 192, 150, 9, 16, 167, 186, 89, 150, 235, 125, 28, 233, 2,
    203, 4, 57, 196, 152 ];

#[deriving(Eq)]
struct ContactIdentifier<V> {
    obj1:    uint,
    obj2:    uint,
    ccenter: V
}

pub type Cb<'self> = &'self fn(buf: &[u8]) -> bool;

impl<V: IterBytes> IterBytes for ContactIdentifier<V> {
    #[inline]
    fn iter_bytes(&self, _lsb0: bool, f: Cb) -> bool {
        self.ccenter.iter_bytes(_lsb0, f)
    }
}

impl<V: Round + Vec<N>, N> ContactIdentifier<V> {
    pub fn new(obj1: uint, obj2: uint, center: V, step: &N) -> ContactIdentifier<V> {
        ContactIdentifier {
            obj1:    obj1,
            obj2:    obj2,
            ccenter: (center / *step).trunc()
        }
    }
}

// FIXME: make the fields priv
pub struct ImpulseCache<N, V> {
    priv hash_prev:           HashMap<ContactIdentifier<V>, (uint, uint)>,
    priv cache_prev:          ~[N],
    priv hash_next:           HashMap<ContactIdentifier<V>, (uint, uint)>,
    priv cache_next:          ~[N],
    priv step:                N,
    priv impulse_per_contact: uint
}

impl<N: Zero + Clone,
     V: Vec<N> + Round + IterBytes>
ImpulseCache<N, V> {
    pub fn new(step: N, impulse_per_contact: uint) -> ImpulseCache<N, V> {
        let mut rng = IsaacRng::new_seeded(seed);

        ImpulseCache {
            hash_prev:           HashMap::with_capacity_and_keys(rng.gen(), rng.gen(), 32),
            hash_next:           HashMap::with_capacity_and_keys(rng.gen(), rng.gen(), 32),
            cache_prev:          vec::from_elem(impulse_per_contact, Zero::zero()),
            cache_next:          vec::from_elem(impulse_per_contact, Zero::zero()),
            step:                step,
            impulse_per_contact: impulse_per_contact
        }
    }

    pub fn insert<'a>(&'a mut self, cid: uint, obj1: uint, obj2: uint, center: V) {
        let id = ContactIdentifier::new(obj1, obj2, center, &self.step);
        let imp =
            match self.hash_prev.find_copy(&id) {
                Some((_, i)) => i,
                None         => 0
            };

        self.hash_next.insert(id, (cid, imp));
    }

    pub fn hash<'a>(&'a self) -> &'a HashMap<ContactIdentifier<V>, (uint, uint)> {
        &'a self.hash_next
    }

    pub fn hash_mut<'a>(&'a mut self) -> &'a mut HashMap<ContactIdentifier<V>, (uint, uint)> {
        &'a mut self.hash_next
    }

    pub fn push_impulsions<'a>(&'a mut self) -> &'a mut [N] {
        let begin = self.cache_next.len();

        do self.impulse_per_contact.times {
            self.cache_next.push(Zero::zero());
        }

        let end = self.cache_next.len();

        self.cache_next.mut_slice(begin, end)
    }

    pub fn reserved_impulse_offset(&self) -> uint {
        self.impulse_per_contact
    }

    pub fn impulsions_at<'a>(&'a self, at: uint) -> &'a [N] {
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
    }

    pub fn swap(&mut self) {
        util::swap(&mut self.hash_prev, &mut self.hash_next);
        util::swap(&mut self.cache_prev,&mut self.cache_next);
        self.hash_next.clear();
        self.cache_next.truncate(self.impulse_per_contact);
    }
}
