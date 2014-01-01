use nalgebra::na::Transform;
use ncollide::math::LV;
use detection::joint::anchor::Anchor;
use object::{RB, SB};

pub struct BallInSocket {
    priv up_to_date: bool,
    priv anchor1:    Anchor<LV>,
    priv anchor2:    Anchor<LV>,
}

impl BallInSocket {
    pub fn new(anchor1: Anchor<LV>, anchor2: Anchor<LV>) -> BallInSocket {
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

    pub fn anchor1<'r>(&'r self) -> &'r Anchor<LV> {
        &self.anchor1
    }

    pub fn anchor2<'r>(&'r self) -> &'r Anchor<LV> {
        &self.anchor2
    }
}

impl BallInSocket {
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

    pub fn anchor1_pos(&self) -> LV {
        match self.anchor1.body {
            Some(b) => {
                match *b {
                    RB(ref rb) => rb.transform_ref().transform(&self.anchor1.position),
                    SB(_)      => fail!("Not yet implemented.")
                }
            },
            None => self.anchor1.position.clone()
        }
    }

    pub fn anchor2_pos(&self) -> LV {
        match self.anchor2.body {
            Some(b) => {
                match *b {
                    RB(ref rb) => rb.transform_ref().transform(&self.anchor2.position),
                    SB(_)      => fail!("Not yet implemented.")
                }
            },
            None => self.anchor2.position.clone()
        }
    }
}
