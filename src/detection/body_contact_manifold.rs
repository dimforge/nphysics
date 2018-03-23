use na::Real;
use ncollide::query::{ContactManifold, TrackedContact};

use object::{BodyHandle, ColliderHandle};
use math::{Point, Isometry};

/// A contact manifold between two bodies.
#[derive(Clone, Debug)]
pub struct BodyContactManifold<'a, N: Real> {
    pub body1: BodyHandle,
    pub body2: BodyHandle,
    pub collider1: ColliderHandle,
    pub collider2: ColliderHandle, 
    pub collider_pos_wrt_body1: &'a Isometry<N>,
    pub collider_pos_wrt_body2: &'a Isometry<N>,
    pub margin1: N,
    pub margin2: N,
    pub manifold: &'a ContactManifold<Point<N>>,
}

impl<'a, N: Real> BodyContactManifold<'a, N> {
    pub fn new(
        body1: BodyHandle,
        body2: BodyHandle,
        collider1: ColliderHandle,
        collider2: ColliderHandle,
        collider_pos_wrt_body1: &'a Isometry<N>,
        collider_pos_wrt_body2: &'a Isometry<N>,
        margin1: N,
        margin2: N,
        manifold: &'a ContactManifold<Point<N>>,
    ) -> Self {
        BodyContactManifold {
            body1,
            body2,
            collider1,
            collider2,
            collider_pos_wrt_body1,
            collider_pos_wrt_body2,
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
