use alga::general::Real;
use na;
use world::RigidBodyStorage;
use math::{Point, Isometry};

/// One of the two end points of a joint.
#[derive(Clone)]
pub struct Anchor<P> {
    /// The body attached to this anchor.
    pub body:     Option<usize>,
    /// The attach position, in local coordinates of the attached body.
    pub position: P
}

impl<P> Anchor<P> {
    /// Creates a new `Anchor` at a given `position` on a `body` local space.
    ///
    /// If `body` is `None`, the anchor is concidered to be attached to the ground and `position`
    /// is the attach point in global coordinates.
    pub fn new(body: Option<usize>, position: P) -> Anchor<P> {
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
    pub fn center_of_mass<N: Real>(&self, bodies: &RigidBodyStorage<N>) -> Point<N> {
        if let Some(b) = self.body {
            bodies[b].center_of_mass().clone()
        } else {
            na::origin()
        }
    }
}

impl<N: Real> Anchor<Point<N>> {
    /// The attach point in global coordinates.
    // TODO: inline ??
    #[inline]
    pub fn pos(&self, bodies: &RigidBodyStorage<N>) -> Point<N> {
        if let Some(b) = self.body {
            bodies[b].position() * self.position
        } else {
            self.position.clone()
        }
    }
}

impl<N: Real> Anchor<Isometry<N>> {
    /// The attach point in global coordinates.
    #[inline]
    pub fn pos(&self, bodies: &RigidBodyStorage<N>) -> Isometry<N> {
        if let Some(b) = self.body {
            bodies[b].position() * self.position
        } else {
            self.position.clone()
        }
    }
}
