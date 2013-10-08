use std::ptr;
use std::num::{Zero, One, from_f32};
use nalgebra::na::AlgebraicVec;
use ncollide::util::hash_map::HashMap;
use ncollide::util::hash::UintTWHash;
use integration::Integrator;
use detection::detector::Detector;
use detection::constraint::{Constraint, RBRB, BallInSocket, Fixed};
use object::{Body, RB, SB};
use signal::signal::{SignalEmiter, BodyActivationRequestHandler,
                     CollisionSignalHandler};

struct BodyWithEnergy<N, LV, AV, M, II> {
    // NOTE: that is the place to put a `can_be_deactivated` flag if needed…
    body:   @mut Body<N, LV, AV, M, II>,
    energy: N
}

impl<N, LV, AV, M, II> BodyWithEnergy<N, LV, AV, M, II> {
    pub fn new(body: @mut Body<N, LV, AV, M, II>, energy: N) -> BodyWithEnergy<N, LV, AV, M, II> {
        BodyWithEnergy {
            body:   body,
            energy: energy
        }
    }
}

// NOTE: this could easily be made generic wrt the Body<...> and wrt Constraint<...>
pub struct IslandActivationManager<N, LV, AV, M, II> {
    events:         @mut SignalEmiter<N, Body<N, LV, AV, M, II>, Constraint<N, LV, AV, M, II>>,
    threshold:      N,
    mix_factor:     N,
    bodies:         HashMap<uint, BodyWithEnergy<N, LV, AV, M, II>, UintTWHash>,
    ufind:          ~[UFindSet],
    can_deactivate: ~[bool],
    collector:      ~[Constraint<N, LV, AV, M, II>]
}

impl<N:  'static + One + Zero + Num + FromPrimitive + Orderable + Algebraic + Clone,
     LV: 'static + AlgebraicVec<N> + Clone,
     AV: 'static + AlgebraicVec<N> + Clone,
     M:  'static,
     II: 'static>
IslandActivationManager<N, LV, AV, M, II> {
    pub fn new(events:     @mut SignalEmiter<N, Body<N, LV, AV, M, II>, Constraint<N, LV, AV, M, II>>,
               threshold:  N,
               mix_factor: N)
               -> @mut IslandActivationManager<N, LV, AV, M, II> {
        assert!(mix_factor >= Zero::zero() && threshold <= One::one(),
                "The energy mixing factor must be comprised between 0.0 and 1.0.");

        let res = @mut IslandActivationManager {
            events:         events,
            threshold:      threshold,
            mix_factor:     mix_factor,
            bodies:         HashMap::new(UintTWHash::new()),
            ufind:          ~[],
            can_deactivate: ~[],
            collector:      ~[]
        };

        let key = ptr::to_mut_unsafe_ptr(res) as uint;

        events.add_body_activation_request_handler(
            key,
            res as @mut BodyActivationRequestHandler<Body<N, LV, AV, M, II>>
        );

        // FIXME: instead of sending the activation message right away, maybe it could be more
        // performant to store a list of objects to activate, and perform the activation during the
        // next call to `interferences` ?
        events.add_collision_handler(key, res as @mut CollisionSignalHandler<Body<N, LV, AV, M, II>>);

        res
    }

    pub fn doit(&mut self) {
        self.mix_factor = self.threshold.clone();
    }

    fn activate(&mut self, b: @mut Body<N, LV, AV, M, II>) -> bool {
        if b.can_move() && !b.is_active() {
            b.activate();
            self.add(b);

            true
        }
        else {
            // add some virtual energy to the body
            // to ensure it wont fall asleep right after the activation message
            match self.bodies.find_mut(&(ptr::to_mut_unsafe_ptr(b) as uint)) {
                Some(ref mut b) => {
                    b.energy = self.threshold * from_f32(2.0).unwrap();
                },
                None => { }
            }

            false
        }
    }

    fn deactivate(&mut self, b: @mut Body<N, LV, AV, M, II>) {
        if b.is_active() {
            b.deactivate();

            self.remove(b);

            // XXX: this should really not be here!
            self.events.emit_body_deactivated(b);
        }
    }

}

impl<N:  Num + Ord + Algebraic,
     LV: AlgebraicVec<N> + Clone,
     AV: AlgebraicVec<N> + Clone,
     M,
     II>
IslandActivationManager<N, LV, AV, M, II> {
    fn can_deactivate(&mut self, body: uint) -> bool {
        self.bodies.elements()[body].value.energy < self.threshold
    }
}

impl<N:  'static + Num + Clone + FromPrimitive + Orderable + Algebraic,
     LV: 'static + AlgebraicVec<N> + Clone,
     AV: 'static + AlgebraicVec<N> + Clone,
     M:  'static,
     II: 'static>
Detector<N, Body<N, LV, AV, M, II>, Constraint<N, LV, AV, M, II>>
for IslandActivationManager<N, LV, AV, M, II> {
    fn add(&mut self, body: @mut Body<N, LV, AV, M, II>) {
        if body.can_move() && body.is_active() {
            if self.bodies.insert(ptr::to_mut_unsafe_ptr(body) as uint,
                                  BodyWithEnergy::new(body, self.threshold * from_f32(2.0).unwrap())) {
                self.ufind.push(UFindSet::new(0));
                self.can_deactivate.push(false);
            }
        }
        if !body.can_move() && body.is_active() {
            self.deactivate(body);
        }
    }

    fn remove(&mut self, b: @mut Body<N, LV, AV, M, II>) {
        if self.bodies.remove(&(ptr::to_mut_unsafe_ptr(b) as uint)) {
            self.ufind.pop();
            self.can_deactivate.pop();
        }
    }

    fn update(&mut self) {
        // Update bodies energy
        for b in self.bodies.elements_mut().mut_iter() {
            match *b.value.body {
                RB(ref rb) => {
                    // NOTE: this is not the kinetic energy, just a hacky value to detect stabilization
                    // FIXME: take the time in account (to make a true RWA)
                    let _1: N = One::one();
                    b.value.energy = 
                        (_1 - self.mix_factor) * b.value.energy +
                        self.mix_factor * (rb.lin_vel().sqnorm() + rb.ang_vel().sqnorm());

                    b.value.energy = b.value.energy.min(&(self.threshold * from_f32(4.0).unwrap()));
                },
                SB(_) => {
                    fail!("Sorry. Energy computation for soft bodies is not implemented yet.")
                }
            }
        }
    }

    fn interferences(&mut self, out: &mut ~[Constraint<N, LV, AV, M, II>]) {
        // here goes all the magic :p
        for d in self.can_deactivate.mut_iter() {
            *d = true
        }

        /*
         * Build islands to deactivate stuffs
         */
        for (i, b) in self.bodies.elements().iter().enumerate() {
            b.value.body.set_index(i as int)
        }

        // init and run the union find
        for (i, u) in self.ufind.mut_iter().enumerate() {
            u.reinit(i)
        }

        for c in out.iter() {
            match *c {
                RBRB(obj1, obj2, _) =>
                    // we compute islands on awaken objects only
                    if obj1.is_active() && obj2.is_active() { // FIXME: add a `propagates_forces` function?
                        union(obj1.index() as uint, obj2.index() as uint, self.ufind)
                    },
                BallInSocket(bis) => {
                    match bis.anchor1().body {
                        Some(b1) => {
                            if b1.is_active() {
                                match bis.anchor2().body {
                                    Some(b2) => {
                                        if b2.is_active() {
                                            union(b1.index() as uint, b2.index() as uint, self.ufind)
                                        }
                                    },
                                    None => { }
                                }
                            }
                        },
                        None => { }
                    }
                },
                Fixed(f) => { // FIXME: code duplication from the BallInSocket variant
                    match f.anchor1().body {
                        Some(b1) => {
                            if b1.is_active() {
                                match f.anchor2().body {
                                    Some(b2) => {
                                        if b2.is_active() {
                                            union(b1.index() as uint, b2.index() as uint, self.ufind)
                                        }
                                    },
                                    None => { }
                                }
                            }
                        },
                        None => { }
                    }
                }
            }
        }

        // find out whether islands can be deactivated
        for i in range(0u, self.ufind.len()) {
            let root = find(i, self.ufind);
            self.can_deactivate[root] = self.can_deactivate[root] && self.can_deactivate(i)
        }

        // NOTE: this one is tricky
        // deactivate islands having only deactivable objects

        let mut to_deactivate = ~[]; // FIXME: avoid allocation at each update

        for i in range(0u, self.ufind.len()) {
            let root = find(i, self.ufind);
            if self.can_deactivate[root] { // everybody in this set can be deactivacted
                let o = self.bodies.elements()[i].value.body;

                if true { // FIXME: ??? o.can_be_deactivated() {
                    to_deactivate.push(o)
                }
            }
        }

        /*
         * NOTE: from here, we dont need the object index any more…
         */
        // … so we can really deactivate objects now
        for o in to_deactivate.iter() {
            self.deactivate(*o)
        }

        // remove every collision between pair of deactivated bodies
        do out.retain |o| {
            match *o {
                RBRB(obj1, obj2, _) => obj1.is_active() || obj2.is_active(),
                BallInSocket(bis)   => {
                    let good = match bis.anchor1().body {
                        None    => false,
                        Some(b) => b.is_active()
                    };

                    good || match bis.anchor2().body {
                            None    => false,
                            Some(b) => b.is_active()
                        }
                }
                Fixed(f) => { // FIXME: code duplication from the BallInSocket varient
                    let good = match f.anchor1().body {
                        None    => false,
                        Some(b) => b.is_active()
                    };

                    good || match f.anchor2().body {
                            None    => false,
                            Some(b) => b.is_active()
                        }
                }
            }
        }

        /*
         * Activate any deactivated object interfering with an active object
         */
        let mut i = 0u;
        while i != out.len() { // we use a `while` loop because `out.len()` might change
            let mut to_activate = None;

            match out[i] {
                RBRB(obj1, obj2, _) => {
                    if !obj1.is_active() && obj1.can_move() { // FIXME: ? && obj1.is_activable() {
                        to_activate = Some(obj1);
                    }
                    else if !obj2.is_active() && obj2.can_move() { // FIXME: ? && obj2.is_activable() {
                        to_activate = Some(obj2);
                    }
                },
                BallInSocket(bis) => {
                    match bis.anchor1().body {
                        None    => { },
                        Some(b) => if !b.is_active() && b.can_move() { to_activate = Some(b) }
                    }

                    match bis.anchor2().body {
                        None    => { },
                        Some(b) => if !b.is_active() && b.can_move() { to_activate = Some(b) }
                    }
                },
                Fixed(f) => { // FIXME: code duplication from BallInSocket
                    match f.anchor1().body {
                        None    => { },
                        Some(b) => if !b.is_active() && b.can_move() { to_activate = Some(b) }
                    }

                    match f.anchor2().body {
                        None    => { },
                        Some(b) => if !b.is_active() && b.can_move() { to_activate = Some(b) }
                    }
                }
            }

            match to_activate {
                None    => { },
                Some(o) => {
                    if self.activate(o) {
                        self.events.emit_body_activated(o, out);
                    };
                }
            }

            i = i + 1;
        }
    }

    #[inline]
    fn priority(&self) -> f64 {
        100.0
    }
}

/*
 * Union find algorithm to build the collision islands
 */
struct UFindSet {
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

fn find(x: uint, sets: &mut [UFindSet]) -> uint {
    if sets[x].parent != x {
        sets[x].parent = find(sets[x].parent, sets);
    }

    sets[x].parent
}
 
fn union(x: uint, y: uint, sets: &mut [UFindSet]) {
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

impl<N:  'static + One + Zero + Num + FromPrimitive + Orderable + Algebraic + Clone,
     LV: 'static + AlgebraicVec<N> + Clone,
     AV: 'static + AlgebraicVec<N> + Clone,
     M:  'static,
     II: 'static>
CollisionSignalHandler<Body<N, LV, AV, M, II>> for IslandActivationManager<N, LV, AV, M, II> {
    fn handle_collision_started_signal(&mut self,
                                       a: @mut Body<N, LV, AV, M, II>,
                                       b: @mut Body<N, LV, AV, M, II>) {
        if self.activate(a) {
            self.events.emit_body_activated(a, &mut self.collector);
            self.collector.clear();
        }
        if self.activate(b) {
            self.events.emit_body_activated(b, &mut self.collector);
            self.collector.clear();
        }
    }

    fn handle_collision_ended_signal(&mut self,
                                     a: @mut Body<N, LV, AV, M, II>,
                                     b: @mut Body<N, LV, AV, M, II>) {
        if self.activate(a) {
            self.events.emit_body_activated(a, &mut self.collector);
            self.collector.clear();
        }
        if self.activate(b) {
            self.events.emit_body_activated(b, &mut self.collector);
            self.collector.clear();
        }
    }
}

impl<N:  'static + One + Zero + Num + FromPrimitive + Orderable + Algebraic + Clone,
     LV: 'static + AlgebraicVec<N> + Clone,
     AV: 'static + AlgebraicVec<N> + Clone,
     M:  'static,
     II: 'static>
BodyActivationRequestHandler<Body<N, LV, AV, M, II>>
for IslandActivationManager<N, LV, AV, M, II> {
    fn handle_body_activation_request(&mut self, b: @mut Body<N, LV, AV, M, II>) {
        if self.activate(b) {
            self.events.emit_body_activated(b, &mut self.collector);
            self.collector.clear();
        };
    }

    fn handle_body_deactivation_request(&mut self, b: @mut Body<N, LV, AV, M, II>) {
        self.deactivate(b)
    }
}
