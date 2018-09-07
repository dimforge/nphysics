use na::Real;
use ncollide::query::{ContactManifold, TrackedContact};

use math::Isometry;
use object::{BodyHandle, BodyPartHandle, Collider, ColliderAnchor};

/// A contact manifold between two bodies.
#[derive(Clone)]
pub struct ColliderContactManifold<'a, N: Real> {
    /// The first collider involved in the contact.
    pub collider1: &'a Collider<N>,
    /// The second collider involved in the contact.
    pub collider2: &'a Collider<N>,
    /// The position of the contact manifold wrt. the first body.
    /// 
    /// This is the frame in which the contact kinematic informations
    /// are expressed relative to the first body. This can be different
    /// from `collider1.position_wrt_body()` when the collider has a
    /// composite shape.
    pub pos_wrt_body1: Isometry<N>,
    /// The position of the contact manifold wrt. the second body.
    /// 
    /// This is the frame in which the contact kinematic informations
    /// are expressed relative to the second body. This can be different
    /// from `collider2.position_wrt_body()` when the collider has a
    /// composite shape.
    pub pos_wrt_body2: Isometry<N>,
    /// The contact manifold.
    pub manifold: &'a ContactManifold<N>,
}

impl<'a, N: Real> ColliderContactManifold<'a, N> {
    /// Initialize a new contact manifold.
    pub fn new(
        collider1: &'a Collider<N>,
        collider2: &'a Collider<N>,
        manifold: &'a ContactManifold<N>,
    ) -> Self {
        let id1 = manifold.subshape_id1();
        let id2 = manifold.subshape_id2();

        let pos_wrt_body1 = if let Some(dpos1) = collider1.shape().subshape_transform(id1) {
            match collider1.data().anchor() {
                ColliderAnchor::OnBodyPart { position_wrt_body_part, .. } => position_wrt_body_part * dpos1,
                ColliderAnchor::OnDeformableBody { .. } => dpos1
            }
        } else {
            match collider1.data().anchor() {
                ColliderAnchor::OnBodyPart { position_wrt_body_part, .. } => *position_wrt_body_part,
                ColliderAnchor::OnDeformableBody { .. } => Isometry::identity()
            }
        };


        let pos_wrt_body2 = if let Some(dpos2) = collider2.shape().subshape_transform(id2) {
            match collider2.data().anchor() {
                ColliderAnchor::OnBodyPart { position_wrt_body_part, .. } => position_wrt_body_part * dpos2,
                ColliderAnchor::OnDeformableBody { .. } => dpos2
            }
        } else {
            match collider2.data().anchor() {
                ColliderAnchor::OnBodyPart { position_wrt_body_part, .. } => *position_wrt_body_part,
                ColliderAnchor::OnDeformableBody { .. } => Isometry::identity()
            }
        };

        ColliderContactManifold {
            collider1,
            collider2,
            pos_wrt_body1,
            pos_wrt_body2,
            manifold,
        }
    }

    /// The number of contacts on the manifold.
    pub fn len(&self) -> usize {
        self.manifold.len()
    }

    /// Get all the contacts from the manifold.
    pub fn contacts(&self) -> &[TrackedContact<N>] {
        self.manifold.contacts()
    }

    /// Get the deepest contact, if any, from the manifold.
    pub fn deepest_contact(&self) -> Option<&TrackedContact<N>> {
        self.manifold.deepest_contact()
    }

    /// The handle of the first body involved in the contact.
    pub fn body1(&self) -> BodyHandle {
        self.collider1.data().body()
    }

    /// The handle of the first body involved in the contact.
    pub fn body2(&self) -> BodyHandle {
        self.collider2.data().body()
    }

    /// The handle of the first body part involved in the contact.
    pub fn body_part1(&self) -> BodyPartHandle {
        match self.collider1.data().anchor() {
            ColliderAnchor::OnBodyPart { body_part, .. } => *body_part,
            ColliderAnchor::OnDeformableBody { .. } => BodyPartHandle::ground(), // XXX: handle deformables
        }
    }

    /// The handle of the second body part involved in the contact.
    pub fn body_part2(&self) -> BodyPartHandle {
        match self.collider2.data().anchor() {
            ColliderAnchor::OnBodyPart { body_part, .. } => *body_part,
            ColliderAnchor::OnDeformableBody { .. } => BodyPartHandle::ground(), // XXX: handle deformables
        }
    }

    /// The anchor between the fist collider and the body it is attached to.
    pub fn anchor1(&self) -> &ColliderAnchor<N> {
        self.collider1.data().anchor()
    }

    /// The anchor between the fist collider and the body it is attached to.
    pub fn anchor2(&self) -> &ColliderAnchor<N> {
        self.collider2.data().anchor()
    }
}
