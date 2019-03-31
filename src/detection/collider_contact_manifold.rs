use na::RealField;
use ncollide::query::{ContactManifold, TrackedContact};
use ncollide::shape::FeatureId;

use crate::object::{BodyHandle, BodyPartHandle, Collider, ColliderAnchor};

/// A contact manifold between two bodies.
#[derive(Clone)]
pub struct ColliderContactManifold<'a, N: RealField> {
    /// The first collider involved in the contact.
    pub collider1: &'a Collider<N>,
    /// The second collider involved in the contact.
    pub collider2: &'a Collider<N>,
    /// The contact manifold.
    pub manifold: &'a ContactManifold<N>,
}

impl<'a, N: RealField> ColliderContactManifold<'a, N> {
    /// Initialize a new contact manifold.
    pub fn new(
        collider1: &'a Collider<N>,
        collider2: &'a Collider<N>,
        manifold: &'a ContactManifold<N>,
    ) -> Self {
        ColliderContactManifold {
            collider1,
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
    pub fn body1(&self) -> BodyHandle {
        self.collider1.body()
    }

    /// The handle of the first body involved in the contact.
    pub fn body2(&self) -> BodyHandle {
        self.collider2.body()
    }

    /// The handle of the first body part involved in the given contact on the specified feature.
    ///
    /// The feature is assumed to belong to the first collider involved in this contact.
    pub fn body_part1(&self, feature1: FeatureId) -> BodyPartHandle {
        match self.collider1.anchor() {
            ColliderAnchor::OnBodyPart { body_part, .. } => *body_part,
            ColliderAnchor::OnDeformableBody { body, body_parts, .. } => {
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
    pub fn body_part2(&self, feature2: FeatureId) -> BodyPartHandle {
        match self.collider2.anchor() {
            ColliderAnchor::OnBodyPart { body_part, .. } => *body_part,
            ColliderAnchor::OnDeformableBody { body, body_parts, .. } => {
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
    pub fn anchor1(&self) -> &ColliderAnchor<N> {
        self.collider1.anchor()
    }

    /// The anchor between the fist collider and the body it is attached to.
    pub fn anchor2(&self) -> &ColliderAnchor<N> {
        self.collider2.anchor()
    }
}
