use std::sync::Arc;
use na::Real;
use ncollide::world::{CollisionObject, CollisionObjectHandle, CollisionObjects};
use ncollide::shape::FeatureId;

use math::Isometry;
use object::{BodyPartHandle, BodyHandle, Material};

/// Type of a reference to a collider.
pub type Colliders<'a, N> = CollisionObjects<'a, N, ColliderData<N>>;

/// Type of the handle of a collider.
pub type ColliderHandle = CollisionObjectHandle;
/// Type of a collider.
pub type Collider<N> = CollisionObject<N, ColliderData<N>>;

/// Type of the handle of a sensor.
pub type SensorHandle = CollisionObjectHandle;
/// Type of a sensor.
pub type Sensor<N> = CollisionObject<N, ColliderData<N>>;

/// Description of the way a collider is attached to a body.
pub enum ColliderAnchor<N: Real> {
    /// Attach of a collider with a body part.
    OnBodyPart {
        /// The attached body part handle.
        body_part: BodyPartHandle,
        /// Relative position of the collider wrt. the body part.
        position_wrt_body_part: Isometry<N>,
    },
    /// Attach of a collider with a deformable body.
    OnDeformableBody {
        /// The attached body handle.
        body: BodyHandle,
        /// Indices mapping degrees of freedom of the body with degrees of freedom of the collision object.
        /// If set to `None`, the mapping is trivial, i.e., the i-th degree of freedom of the body corresponds to
        /// the i-th degree of freedom of the collision object.
        // NOTE: we made it an ARC mostly because ot avoids some borrowing issue on simulation steps to
        // apply the deformation to attached colliders. Though it is still interesting per se to allow
        // sharing deformation index buffers between deformable colliders.
        indices: Option<Arc<Vec<usize>>>,
        /// A map between the collision objects parts and body part indices.
        ///
        /// The `i`-th part of the collision object corresponds to the `body_parts[i]`-th body part.
        /// If set to `None`, the mapping is trivial, i.e., `i`-th part of the collision object corresponds to the `i`-th body part.
        body_parts: Option<Arc<Vec<usize>>>,
    },
}

impl<N: Real> ColliderAnchor<N> {
    /// The body this anchor is attached to.
    #[inline]
    pub fn body(&self) -> BodyHandle {
        match self {
            ColliderAnchor::OnBodyPart { body_part, .. } => body_part.body_handle,
            ColliderAnchor::OnDeformableBody { body, .. } => *body
        }
    }
}

/// Data stored into each collider.
///
/// Those are needed by nphysics.
pub struct ColliderData<N: Real> {
    margin: N,
    anchor: ColliderAnchor<N>,
    // NOTE: needed for the collision filter.
    body_status_dependent_ndofs: usize,
    material: Material<N>,
}

impl<N: Real> ColliderData<N> {
    /// Initializes data for a collider.
    pub fn new(
        margin: N,
        anchor: ColliderAnchor<N>,
        body_status_dependent_ndofs: usize,
        material: Material<N>,
    ) -> Self {
        ColliderData {
            margin,
            anchor,
            body_status_dependent_ndofs,
            material,
        }
    }

    /// The collision margin surrounding this collider.
    #[inline]
    pub fn margin(&self) -> N {
        self.margin
    }

    /// Handle to the body this collider is attached to.
    pub fn body(&self) -> BodyHandle {
        self.anchor.body()
    }

    /// The anchor attaching this collider with a body part or deformable body.
    pub fn anchor(&self) -> &ColliderAnchor<N> {
        &self.anchor
    }

    /// The position of this collider geometry wrt. the body it is attached to.
    pub fn position_wrt_body(&self) -> Isometry<N> {
        if let ColliderAnchor::OnBodyPart { position_wrt_body_part, .. } = self.anchor {
            position_wrt_body_part
        } else {
            Isometry::identity()
        }
    }

    /// Handle to the body part containing the given subshape of this collider's shape.
    pub fn body_part(&self, subshape_id: usize) -> BodyPartHandle {
        match &self.anchor {
            ColliderAnchor::OnBodyPart { body_part, .. } => *body_part,
            ColliderAnchor::OnDeformableBody { body, body_parts, .. } => {
                if let Some(body_parts) = body_parts {
                    BodyPartHandle { body_handle: *body, part_id: body_parts[subshape_id] }
                } else {
                    BodyPartHandle { body_handle: *body, part_id: subshape_id }
                }
            }
        }
    }

    /// The material of this collider.
    #[inline]
    pub fn material(&self) -> &Material<N> {
        &self.material
    }

    #[inline]
    pub(crate) fn body_status_dependent_ndofs(&self) -> usize {
        self.body_status_dependent_ndofs
    }

    #[inline]
    pub(crate) fn set_body_status_dependent_ndofs(&mut self, ndofs: usize) {
        self.body_status_dependent_ndofs = ndofs
    }
}