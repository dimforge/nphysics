use na::Real;
use ncollide::query::{ContactManifold, TrackedContact};

use object::{BodyHandle, Collider, ColliderHandle};
use math::{Isometry, Point};

/// A contact manifold between two bodies.
#[derive(Clone)]
pub struct ColliderContactManifold<'a, N: Real> {
    pub collider1: &'a Collider<N>,
    pub collider2: &'a Collider<N>,
    pub pos_wrt_body1: Isometry<N>,
    pub pos_wrt_body2: Isometry<N>,
    pub manifold: &'a ContactManifold<Point<N>>,
}

impl<'a, N: Real> ColliderContactManifold<'a, N> {
    pub fn new(
        collider1: &'a Collider<N>,
        collider2: &'a Collider<N>,
        manifold: &'a ContactManifold<Point<N>>,
    ) -> Self {
        let deepest = manifold.deepest_contact().unwrap();
        let deepest_fid1 = deepest.kinematic.feature1();
        let deepest_fid2 = deepest.kinematic.feature2();

        let dpos1 = collider1.shape().subshape_transform(deepest_fid1);
        let dpos2 = collider2.shape().subshape_transform(deepest_fid2);

        let pos_wrt_body1 = collider1.data().position_wrt_parent() * dpos1;
        let pos_wrt_body2 = collider2.data().position_wrt_parent() * dpos2;

        ColliderContactManifold {
            collider1,
            collider2,
            pos_wrt_body1,
            pos_wrt_body2,
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
