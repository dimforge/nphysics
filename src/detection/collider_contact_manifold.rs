use na::Real;
use ncollide::query::{ContactManifold, TrackedContact};

use math::Isometry;
use object::{BodyHandle, Collider};

/// A contact manifold between two bodies.
#[derive(Clone)]
pub struct ColliderContactManifold<'a, N: Real> {
    pub collider1: &'a Collider<N>,
    pub collider2: &'a Collider<N>,
    pub pos_wrt_body1: Isometry<N>,
    pub pos_wrt_body2: Isometry<N>,
    pub manifold: &'a ContactManifold<N>,
}

impl<'a, N: Real> ColliderContactManifold<'a, N> {
    pub fn new(
        collider1: &'a Collider<N>,
        collider2: &'a Collider<N>,
        manifold: &'a ContactManifold<N>,
    ) -> Self {
        let id1 = manifold.subshape_id1();
        let id2 = manifold.subshape_id2();

        let pos_wrt_body1;
        let pos_wrt_body2;

        if let Some(dpos1) = collider1.shape().subshape_transform(id1) {
            pos_wrt_body1 = collider1.data().position_wrt_body() * dpos1;
        } else {
            pos_wrt_body1 = *collider1.data().position_wrt_body()
        }

        if let Some(dpos2) = collider2.shape().subshape_transform(id2) {
            pos_wrt_body2 = collider2.data().position_wrt_body() * dpos2;
        } else {
            pos_wrt_body2 = *collider2.data().position_wrt_body()
        }

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

    pub fn contacts(&self) -> &[TrackedContact<N>] {
        self.manifold.contacts()
    }

    pub fn deepest_contact(&self) -> Option<&TrackedContact<N>> {
        self.manifold.deepest_contact()
    }

    pub fn body1(&self) -> BodyHandle {
        self.collider1.data().body()
    }

    pub fn body2(&self) -> BodyHandle {
        self.collider2.data().body()
    }
}
