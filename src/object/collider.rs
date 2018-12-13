use na::Real;
use ncollide::world::{CollisionObject, CollisionObjectHandle, CollisionObjects};

use math::Isometry;
use object::{BodyHandle, Material};

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

/// Data stored into each collider.
///
/// Those are needed by nphysics.
pub struct ColliderData<N: Real> {
    margin: N,
    body: BodyHandle,
    // NOTE: needed for the collision filter.
    body_status_dependent_ndofs: usize,
    position_wrt_body: Isometry<N>,
    material: Material<N>,
}

impl<N: Real> ColliderData<N> {
    /// Initializes data for a collider.
    pub fn new(
        margin: N,
        body: BodyHandle,
        body_status_dependent_ndofs: usize,
        position_wrt_body: Isometry<N>,
        material: Material<N>,
    ) -> Self {
        ColliderData {
            margin,
            body,
            body_status_dependent_ndofs,
            position_wrt_body,
            material,
        }
    }

    /// The handle of the body part this collider is attached to.
    #[inline]
    pub fn body(&self) -> BodyHandle {
        self.body
    }

    /// The collision margin surrounding this collider.
    #[inline]
    pub fn margin(&self) -> N {
        self.margin
    }

    /// The position of the collider relative to the body it is attached to.
    #[inline]
    pub fn position_wrt_body(&self) -> &Isometry<N> {
        &self.position_wrt_body
    }

    /// The material of this collider.
    #[inline]
    pub fn material(&self) -> &Material<N> {
        &self.material
    }

    /// Set the material of this collider.
    #[inline]
    pub fn set_material(&mut self, material: Material<N>) {
        self.material = material;
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
