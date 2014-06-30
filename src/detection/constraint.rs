//! Data structure to describe a constraint between two rigid bodies.

use std::rc::Rc;
use std::cell::RefCell;
use ncollide::narrow::Contact;
use object::RigidBody;
use detection::joint::{Fixed, BallInSocket};

/// A constraint between two rigid bodies.
pub enum Constraint {
    /// A contact.
    RBRB(Rc<RefCell<RigidBody>>, Rc<RefCell<RigidBody>>, Contact),
    /// A ball-in-socket joint.
    BallInSocket(Rc<RefCell<BallInSocket>>),
    /// A fixed joint.
    Fixed(Rc<RefCell<Fixed>>),
}

impl Clone for Constraint {
    fn clone(&self) -> Constraint {
        match *self {
            RBRB(ref a, ref b, ref c) => RBRB(a.clone(), b.clone(), c.clone()),
            BallInSocket(ref bis)     => BallInSocket(bis.clone()),
            Fixed(ref f)              => Fixed(f.clone()),
        }
    }
}
