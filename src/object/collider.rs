use na::Real;
use ncollide::world::{CollisionObject, CollisionObjectHandle, CollisionObjects};

use object::{BodyHandle, Material};
use math::{Isometry, Point};

pub type Colliders<'a, N> = CollisionObjects<'a, Point<N>, Isometry<N>, ColliderData<N>>;

pub type ColliderHandle = CollisionObjectHandle;
pub type Collider<N> = CollisionObject<Point<N>, Isometry<N>, ColliderData<N>>;

pub type SensorHandle = CollisionObjectHandle;
pub type Sensor<N> = CollisionObject<Point<N>, Isometry<N>, ColliderData<N>>;

pub struct ColliderData<N: Real> {
    margin: N,
    body: BodyHandle,
    position_wrt_parent: Isometry<N>,
    material: Material<N>,
}

impl<N: Real> ColliderData<N> {
    pub fn new(
        margin: N,
        body: BodyHandle,
        position_wrt_parent: Isometry<N>,
        material: Material<N>,
    ) -> Self {
        ColliderData {
            margin,
            body,
            position_wrt_parent,
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

    #[inline]
    pub fn position_wrt_parent(&self) -> &Isometry<N> {
        &self.position_wrt_parent
    }

    #[inline]
    pub fn material(&self) -> &Material<N> {
        &self.material
    }
}
