use object::{Body, RB, SB};
use nalgebra::na;
use aliases::traits::{NPhysicsScalar, NPhysicsDirection, NPhysicsOrientation, NPhysicsTransform,
                      NPhysicsInertia};

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

impl<N:  Clone + NPhysicsScalar,
     LV: Clone + NPhysicsDirection<N, AV>,
     AV: Clone + NPhysicsOrientation<N>,
     M:  NPhysicsTransform<LV, AV>,
     II: Clone + NPhysicsInertia<N, LV, AV, M>,
     P>
Anchor<N, LV, AV, M, II, P> {
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
