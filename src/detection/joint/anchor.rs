use object::{Body, RB, SB};
use nalgebra::na;
use ncollide::math::LV;

pub struct Anchor<P> {
    body:     Option<@mut Body>,
    position: P
}

impl<P> Anchor<P> {
    pub fn new(body: Option<@mut Body>, position: P) -> Anchor<P> {
        Anchor {
            body:     body,
            position: position
        }
    }
}

impl<P>
Anchor<P> {
    pub fn center_of_mass(&self) -> LV {
        match self.body {
            Some(b) => {
                match *b {
                    RB(ref rb) => rb.center_of_mass().clone(),
                    SB(_)      => fail!("Not yet implemented.")
                }
            },
            None => na::zero()
        }
    }
}
