use na::Real;
use ncollide::query::{ContactManifold, TrackedContact};

use object::{BodyHandle, ColliderHandle, Collider};
use math::{Isometry, Point};

/// A contact manifold between two bodies.
#[derive(Clone)]
pub struct ColliderContactManifold<'a, N: Real> {
    pub collider1: &'a Collider<N>,
    pub collider2: &'a Collider<N>,
    pub manifold: &'a ContactManifold<Point<N>>,
}

impl<'a, N: Real> ColliderContactManifold<'a, N> {
    pub fn new(
        collider1: &'a Collider<N>,
        collider2: &'a Collider<N>,
        manifold: &'a ContactManifold<Point<N>>,
    ) -> Self {
        ColliderContactManifold {
            collider1,
            collider2,
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

    pub fn body1(&self) -> BodyHandle {
        self.collider1.data().body()
    }

    pub fn body2(&self) -> BodyHandle {
        self.collider2.data().body()
    }
}
