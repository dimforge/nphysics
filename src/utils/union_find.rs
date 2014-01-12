#[deriving(Clone)]
pub struct UFindSet {
    parent: uint,
    rank:   uint
}

impl UFindSet {
    #[inline]
    pub fn new(key: uint) -> UFindSet {
        UFindSet {
            parent: key,
            rank:   0
        }
    }

    #[inline]
    pub fn reinit(&mut self, key: uint) {
        self.parent = key;
        self.rank   = 0;
    }
}

pub fn find(x: uint, sets: &mut [UFindSet]) -> uint {
    if sets[x].parent != x {
        sets[x].parent = find(sets[x].parent, sets);
    }

    sets[x].parent
}
 
pub fn union(x: uint, y: uint, sets: &mut [UFindSet]) {
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
