use std::num::Zero;
use nalgebra::mat::Transform;
use detection::joint::anchor::Anchor;
use object::{RB, SB};

pub struct BallInSocket<N, LV, AV, M, II> {
    priv up_to_date: bool,
    priv anchor1:    Anchor<N, LV, AV, M, II>,
    priv anchor2:    Anchor<N, LV, AV, M, II>,
}

impl<N, LV, AV, M, II> BallInSocket<N, LV, AV, M, II> {
    pub fn new(anchor1: Anchor<N, LV, AV, M, II>,
               anchor2: Anchor<N, LV, AV, M, II>)
               -> BallInSocket<N, LV, AV, M, II> {
        BallInSocket {
            up_to_date: false,
            anchor1:    anchor1,
            anchor2:    anchor2
        }
    }

    pub fn up_to_date(&self) -> bool {
        self.up_to_date
    }

    pub fn update(&mut self) {
        self.up_to_date = true
    }

    pub fn anchor1<'r>(&'r self) -> &'r Anchor<N, LV, AV, M, II> {
        &self.anchor1
    }

    pub fn anchor2<'r>(&'r self) -> &'r Anchor<N, LV, AV, M, II> {
        &self.anchor2
    }
}

impl<N, LV: Eq, AV, M, II> BallInSocket<N, LV, AV, M, II> {
    pub fn set_local1(&mut self, local1: LV) {
        if local1 != self.anchor1.position {
            self.up_to_date = false;
            self.anchor1.position = local1
        }
    }

    pub fn set_local2(&mut self, local2: LV) {
        if local2 != self.anchor2.position {
            self.up_to_date = false;
            self.anchor2.position = local2
        }
    }
}

impl<N: Clone, LV: Clone, AV, M: Transform<LV>, II> BallInSocket<N, LV, AV, M, II> {
    pub fn anchor1_pos(&self) -> LV {
        match self.anchor1.body {
            Some(b) => {
                match *b {
                    RB(rb) => rb.transform_ref().transform(&self.anchor1.position),
                    SB(_)   => fail!("Not yet implemented.")
                }
            },
            None => self.anchor1.position.clone()
        }
    }

    pub fn anchor2_pos(&self) -> LV {
        match self.anchor2.body {
            Some(b) => {
                match *b {
                    RB(rb) => rb.transform_ref().transform(&self.anchor2.position),
                    SB(_)   => fail!("Not yet implemented.")
                }
            },
            None => self.anchor2.position.clone()
        }
    }
}

impl<N: Clone, LV: Clone + Zero, AV, M, II> BallInSocket<N, LV, AV, M, II> {
    pub fn center_of_mass1(&self) -> LV {
        match self.anchor1.body {
            Some(b) => {
                match *b {
                    RB(rb) => rb.center_of_mass().clone(),
                    SB(_)   => fail!("Not yet implemented.")
                }
            },
            None => Zero::zero()
        }
    }

    pub fn center_of_mass2(&self) -> LV {
        match self.anchor2.body {
            Some(b) => {
                match *b {
                    RB(rb) => rb.center_of_mass().clone(),
                    SB(_)   => fail!("Not yet implemented.")
                }
            },
            None => Zero::zero()
        }
    }
}
