use std::num::Zero;
use object::{Body, RB, SB};

pub struct Anchor<N, LV, AV, M, II, P> {
    body:     Option<@mut Body<N, LV, AV, M, II>>,
    position: P
}

impl<N, LV, AV, M, II, P> Anchor<N, LV, AV, M, II, P> {
    pub fn new(body: Option<@mut Body<N, LV, AV, M, II>>, position: P) -> Anchor<N, LV, AV, M, II, P> {
        Anchor {
            body:     body,
            position: position
        }
    }
}

impl<N: Clone, LV: Clone + Zero, AV, M, II, P> Anchor<N, LV, AV, M, II, P> {
    pub fn center_of_mass(&self) -> LV {
        match self.body {
            Some(b) => {
                match *b {
                    RB(ref rb) => rb.center_of_mass().clone(),
                    SB(_)      => fail!("Not yet implemented.")
                }
            },
            None => Zero::zero()
        }
    }
}
