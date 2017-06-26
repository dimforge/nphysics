//! Data structure to describe a constraint between two rigid bodies.

use alga::general::Real;
use ncollide::query::Contact;
use object::RigidBody;
use detection::joint::{Fixed, BallInSocket};
use math::Point;

/// A constraint between two rigid bodies.
pub enum Constraint<N: Real> {
    /// A contact.
    RBRB(::Rc<RigidBody<N>>, ::Rc<RigidBody<N>>, Contact<Point<N>>),
    /// A ball-in-socket joint.
    BallInSocket(::Rc<BallInSocket<N>>),
    /// A fixed joint.
    Fixed(::Rc<Fixed<N>>),
}

impl<N: Real> Clone for Constraint<N> {
    fn clone(&self) -> Constraint<N> {
        match *self {
            Constraint::RBRB(ref a, ref b, ref c) => Constraint::RBRB(a.clone(), b.clone(), c.clone()),
            Constraint::BallInSocket(ref bis)     => Constraint::BallInSocket(bis.clone()),
            Constraint::Fixed(ref f)              => Constraint::Fixed(f.clone()),
        }
    }
}
