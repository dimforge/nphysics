use std::rc::Rc;
use std::cell::RefCell;
use object::RigidBody;
use nalgebra::na;
use ncollide::math::LV;

pub struct Anchor<P> {
    body:     Option<Rc<RefCell<RigidBody>>>,
    position: P
}

impl<P> Anchor<P> {
    pub fn new(body: Option<Rc<RefCell<RigidBody>>>, position: P) -> Anchor<P> {
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
            Some(ref b) => {
                let bb = b.borrow().borrow();
                let rb = bb.get();

                rb.center_of_mass().clone()
            },
            None => na::zero()
        }
    }
}
