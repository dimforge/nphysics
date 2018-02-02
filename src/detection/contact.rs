use na::Real;
use ncollide::query::Contact;

use object::BodyHandle;
use math::Point;

#[derive(Clone, Debug)]
pub struct ContactConstraint<N: Real> {
    pub b1: BodyHandle,
    pub b2: BodyHandle,
    pub contact: Contact<Point<N>>,
}

impl<N: Real> ContactConstraint<N> {
    pub fn new(b1: BodyHandle, b2: BodyHandle, contact: Contact<Point<N>>) -> Self {
        ContactConstraint { b1, b2, contact }
    }
}
