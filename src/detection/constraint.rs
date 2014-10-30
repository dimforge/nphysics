//! Data structure to describe a constraint between two rigid bodies.

use std::rc::Rc;
use std::cell::RefCell;
use ncollide::narrow::Contact;
use object::RigidBody;
use detection::joint::{Fixed, BallInSocket};
use math::{Scalar, Point, Vect};

/// A constraint between two rigid bodies.
pub enum Constraint {
    /// A contact.
    RBRB(Rc<RefCell<RigidBody>>, Rc<RefCell<RigidBody>>, Contact<Scalar, Point, Vect>),
    /// A ball-in-socket joint.
    BallInSocketConstraint(Rc<RefCell<BallInSocket>>),
    /// A fixed joint.
    FixedConstraint(Rc<RefCell<Fixed>>),
}

impl Clone for Constraint {
    fn clone(&self) -> Constraint {
        match *self {
            RBRB(ref a, ref b, ref c)       => RBRB(a.clone(), b.clone(), c.clone()),
            BallInSocketConstraint(ref bis) => BallInSocketConstraint(bis.clone()),
            FixedConstraint(ref f)          => FixedConstraint(f.clone()),
        }
    }
}
