use na::Real;
use ncollide::world::{CollisionObject, CollisionObjectHandle, CollisionObjects};

use object::{BodyHandle, Material};
use math::Isometry;

pub type Colliders<'a, N> = CollisionObjects<'a, N, ColliderData<N>>;

pub type ColliderHandle = CollisionObjectHandle;
pub type Collider<N> = CollisionObject<N, ColliderData<N>>;

pub type SensorHandle = CollisionObjectHandle;
pub type Sensor<N> = CollisionObject<N, ColliderData<N>>;

pub struct ColliderData<N: Real> {
    margin: N,
    body: BodyHandle,
    // NOTE: needed for the collision filter.
    body_status_dependent_ndofs: usize,
    position_wrt_body: Isometry<N>,
    material: Material<N>,
}

impl<N: Real> ColliderData<N> {
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

    #[inline]
    pub fn body(&self) -> BodyHandle {
        self.body
    }

    #[inline]
    pub fn margin(&self) -> N {
        self.margin
    }

    // XXX: rename this "position_wrt_body".
    #[inline]
    pub fn position_wrt_body(&self) -> &Isometry<N> {
        &self.position_wrt_body
    }

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
