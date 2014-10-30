use std::rc::Rc;
use std::cell::RefCell;
use object::RigidBody;
use na;
use math::Point;

/// One of the two end points of a joint.
pub struct Anchor<P> {
    /// The body attached to this anchor.
    pub body:     Option<Rc<RefCell<RigidBody>>>,
    /// The attach position, in local coordinates of the attached body.
    pub position: P
}

impl<P> Anchor<P> {
    /// Creates a new `Anchor` at a given `position` on a `body` local space.
    ///
    /// If `body` is `None`, the anchor is concidered to be attached to the ground and `position`
    /// is the attach point in global coordinates.
    pub fn new(body: Option<Rc<RefCell<RigidBody>>>, position: P) -> Anchor<P> {
        Anchor {
            body:     body,
            position: position
        }
    }
}

impl<P> Anchor<P> {
    /// The center of mass of the body attached to this anchor.
    ///
    /// Returns the zero vector if no body is attached.
    pub fn center_of_mass(&self) -> Point {
        match self.body {
            Some(ref b) => {
                let rb = b.borrow();

                rb.center_of_mass().clone()
            },
            None => na::orig()
        }
    }
}
