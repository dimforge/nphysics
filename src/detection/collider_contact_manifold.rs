use na::RealField;
use ncollide::query::{ContactManifold, TrackedContact};
use ncollide::shape::FeatureId;

use crate::object::{BodyHandle, BodyPartHandle, Collider, ColliderAnchor, ColliderHandle};

/// A contact manifold between two bodies.
#[derive(Clone)]
pub struct ColliderContactManifold<'a, N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle>
{
    /// The handle of the first collider involved in the contact.
    pub handle1: CollHandle,
    /// The first collider involved in the contact.
    pub collider1: &'a Collider<N, Handle>,
    /// The handle of the second collider involved in the contact.
    pub handle2: CollHandle,
    /// The second collider involved in the contact.
    pub collider2: &'a Collider<N, Handle>,
    /// The contact manifold.
    pub manifold: &'a ContactManifold<N>,
}

impl<'a, N: RealField, Handle: BodyHandle, CollHandle: ColliderHandle>
    ColliderContactManifold<'a, N, Handle, CollHandle>
{
    /// Initialize a new contact manifold.
    pub fn new(
        handle1: CollHandle,
        collider1: &'a Collider<N, Handle>,
        handle2: CollHandle,
        collider2: &'a Collider<N, Handle>,
        manifold: &'a ContactManifold<N>,
    ) -> Self {
        ColliderContactManifold {
            handle1,
            collider1,
            handle2,
            collider2,
            manifold,
        }
    }

    /// The number of contacts on the manifold.
    pub fn len(&self) -> usize {
        self.manifold.len()
    }

    /// Get all the contacts from the manifold.
    pub fn contacts(&self) -> impl Iterator<Item = &TrackedContact<N>> {
        self.manifold.contacts()
    }

    /// Get the deepest contact, if any, from the manifold.
    pub fn deepest_contact(&self) -> Option<&TrackedContact<N>> {
        self.manifold.deepest_contact()
    }

    /// The handle of the first body involved in the contact.
    pub fn body1(&self) -> Handle {
        self.collider1.body()
    }

    /// The handle of the first body involved in the contact.
    pub fn body2(&self) -> Handle {
        self.collider2.body()
    }

    /// The handle of the first body part involved in the given contact on the specified feature.
    ///
    /// The feature is assumed to belong to the first collider involved in this contact.
    pub fn body_part1(&self, feature1: FeatureId) -> BodyPartHandle<Handle> {
        match self.collider1.anchor() {
            ColliderAnchor::OnBodyPart { body_part, .. } => *body_part,
            ColliderAnchor::OnDeformableBody {
                body, body_parts, ..
            } => {
                let subshape_id = self.collider1.shape().subshape_containing_feature(feature1);
                if let Some(body_parts) = body_parts {
                    BodyPartHandle(*body, body_parts[subshape_id])
                } else {
                    BodyPartHandle(*body, subshape_id)
                }
            }
        }
    }

    /// The handle of the second body part involved in the given contact on the specified feature.
    ///
    /// The feature is assumed to belong to the second collider involved in this contact.
    pub fn body_part2(&self, feature2: FeatureId) -> BodyPartHandle<Handle> {
        match self.collider2.anchor() {
            ColliderAnchor::OnBodyPart { body_part, .. } => *body_part,
            ColliderAnchor::OnDeformableBody {
                body, body_parts, ..
            } => {
                let subshape_id = self.collider2.shape().subshape_containing_feature(feature2);
                if let Some(body_parts) = body_parts {
                    BodyPartHandle(*body, body_parts[subshape_id])
                } else {
                    BodyPartHandle(*body, subshape_id)
                }
            }
        }
    }

    /// The anchor between the fist collider and the body it is attached to.
    pub fn anchor1(&self) -> &ColliderAnchor<N, Handle> {
        self.collider1.anchor()
    }

    /// The anchor between the fist collider and the body it is attached to.
    pub fn anchor2(&self) -> &ColliderAnchor<N, Handle> {
        self.collider2.anchor()
    }
}
