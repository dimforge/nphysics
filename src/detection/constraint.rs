//! Data structure to describe a constraint between two rigid bodies.

use std::rc::Rc;
use std::cell::RefCell;
use ncollide::geometry::Contact;
use object::RigidBody;
use detection::joint::{Fixed, BallInSocket};
use math::Point;

/// A constraint between two rigid bodies.
pub enum Constraint {
    /// A contact.
    RBRB(Rc<RefCell<RigidBody>>, Rc<RefCell<RigidBody>>, Contact<Point>),
    /// A ball-in-socket joint.
    BallInSocket(Rc<RefCell<BallInSocket>>),
    /// A fixed joint.
    Fixed(Rc<RefCell<Fixed>>),
}

impl Clone for Constraint {
    fn clone(&self) -> Constraint {
        match *self {
            Constraint::RBRB(ref a, ref b, ref c) => Constraint::RBRB(a.clone(), b.clone(), c.clone()),
            Constraint::BallInSocket(ref bis) => Constraint::BallInSocket(bis.clone()),
            Constraint::Fixed(ref f) => Constraint::Fixed(f.clone()),
        }
    }
}
