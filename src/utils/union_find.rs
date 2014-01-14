//! The union find algorithm.

/// An element used by the union-find algorithm.
#[deriving(Clone)]
pub struct UnionFindSet {
    /// The parent of this union find element.
    parent: uint,
    /// The rank of this union find element.
    rank:   uint
}

impl UnionFindSet {
    /// Creates a new `UnionFindSet`.
    #[inline]
    pub fn new(key: uint) -> UnionFindSet {
        UnionFindSet {
            parent: key,
            rank:   0
        }
    }

    /// Reinitialize this set.
    #[inline]
    pub fn reinit(&mut self, key: uint) {
        self.parent = key;
        self.rank   = 0;
    }
}

/// Performs the `find` part of the union-find algorithm.
pub fn find(x: uint, sets: &mut [UnionFindSet]) -> uint {
    if sets[x].parent != x {
        sets[x].parent = find(sets[x].parent, sets);
    }

    sets[x].parent
}
 
/// Performs the `union` part of the union-find algorithm.
pub fn union(x: uint, y: uint, sets: &mut [UnionFindSet]) {
     let x_root = find(x, sets);
     let y_root = find(y, sets);

     if x_root == y_root {
         return
     }

     let rankx = sets[x_root].rank;
     let ranky = sets[y_root].rank;

     if rankx < ranky {
         sets[x_root].parent = y_root
     }
     else if rankx > ranky {
         sets[y_root].parent = x_root
     }
     else {
         sets[y_root].parent = x_root;
         sets[x_root].rank   = rankx + 1
     }
}
