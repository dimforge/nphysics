//! Data structure to describe a constraint between two rigid bodies.

use alga::general::Real;
use ncollide::query::Contact;
use detection::joint::{Fixed, BallInSocket};
use math::Point;

/// A constraint between two rigid bodies.
#[derive(Clone)]
pub enum Constraint<N: Real> {
    /// A contact.
    RBRB(usize, usize, Contact<Point<N>>),
    /// A ball-in-socket joint.
    BallInSocket(BallInSocket<N>),
    /// A fixed joint.
    Fixed(Fixed<N>),
}
