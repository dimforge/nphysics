//! The union find algorithm.

/// An element used by the union-find algorithm.
#[derive(Copy, Clone, Debug, Hash)]
pub struct UnionFindSet {
    /// The parent of this union find element.
    parent: usize,
    /// The rank of this union find element.
    rank: usize,
}

impl UnionFindSet {
    /// Creates a new `UnionFindSet`.
    #[inline]
    pub fn new(key: usize) -> UnionFindSet {
        UnionFindSet {
            parent: key,
            rank: 0,
        }
    }

    /// Reinitialize this set.
    #[inline]
    pub fn reinit(&mut self, key: usize) {
        self.parent = key;
        self.rank = 0;
    }
}

/// Performs the `find` part of the union-find algorithm.
pub fn find(x: usize, sets: &mut [UnionFindSet]) -> usize {
    if sets[x].parent != x {
        sets[x].parent = find(sets[x].parent, sets);
    }

    sets[x].parent
}

/// Performs the `union` part of the union-find algorithm.
pub fn union(x: usize, y: usize, sets: &mut [UnionFindSet]) {
    let x_root = find(x, sets);
    let y_root = find(y, sets);

    if x_root == y_root {
        return;
    }

    let rankx = sets[x_root].rank;
    let ranky = sets[y_root].rank;

    if rankx < ranky {
        sets[x_root].parent = y_root
    } else if rankx > ranky {
        sets[y_root].parent = x_root
    } else {
        sets[y_root].parent = x_root;
        sets[x_root].rank = rankx + 1
    }
}
