use na::Real;
use ncollide::query::{ContactManifold, TrackedContact};

use object::BodyHandle;
use math::Point;

/// A contact manifold between two bodies.
#[derive(Clone, Debug)]
pub struct BodyContactManifold<'a, N: Real> {
    pub b1: BodyHandle,
    pub b2: BodyHandle,
    pub margin1: N,
    pub margin2: N,
    pub manifold: &'a ContactManifold<Point<N>>,
}

impl<'a, N: Real> BodyContactManifold<'a, N> {
    pub fn new(
        b1: BodyHandle,
        b2: BodyHandle,
        margin1: N,
        margin2: N,
        manifold: &'a ContactManifold<Point<N>>,
    ) -> Self {
        BodyContactManifold {
            b1,
            b2,
            margin1,
            margin2,
            manifold,
        }
    }

    pub fn len(&self) -> usize {
        self.manifold.len()
    }

    pub fn contacts(&self) -> &[TrackedContact<Point<N>>] {
        self.manifold.contacts()
    }

    pub fn deepest_contact(&self) -> Option<&TrackedContact<Point<N>>> {
        self.manifold.deepest_contact()
    }
}
