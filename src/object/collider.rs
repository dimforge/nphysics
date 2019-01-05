use std::sync::Arc;
use std::f64;
use either::Either;
use na::Real;
use ncollide::world::{CollisionObject, CollisionObjectHandle, CollisionObjects, GeometricQueryType, CollisionGroups};
use ncollide::shape::{FeatureId, ShapeHandle};

use crate::math::{Isometry, Vector};
use crate::object::{BodyPartHandle, BodyHandle, Material, Body, BodyPart};
use crate::world::{World, CollisionWorld};
use crate::volumetric::Volumetric;

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
            ColliderAnchor::OnBodyPart { body_part, .. } => body_part.0,
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
                    BodyPartHandle(*body, body_parts[subshape_id])
                } else {
                    BodyPartHandle(*body, subshape_id)
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

    // FIXME: move this to a collider wrapper.
    pub fn sync(cworld: &mut CollisionWorld<N>, handle: ColliderHandle, body: &Body<N>, body_part: Option<&BodyPart<N>>) {
        let new_pos;
        let collider = cworld
            .collision_object_mut(handle)
            .expect("Internal error: collider not found.");

        collider
            .data_mut()
            .set_body_status_dependent_ndofs(body.status_dependent_ndofs());

        match collider.data().anchor() {
            ColliderAnchor::OnBodyPart { position_wrt_body_part, .. } => {
                let part_pos = body_part.expect("Invalid body part.").position();
                new_pos = Either::Left(part_pos * position_wrt_body_part)
            }
            ColliderAnchor::OnDeformableBody { indices, .. } => {
                // (that's why this is an arc) to avoid borrowing issue.
                new_pos = Either::Right(indices.clone());
            }
        }

        match new_pos {
            Either::Left(pos) => cworld.set_position(handle, pos),
            Either::Right(indices) => cworld.set_deformations(handle, body.deformed_positions().unwrap().1, indices.as_ref().map(|idx| &idx[..]))
        }
    }
}

pub struct ColliderDesc<N: Real> {
    pub margin: N,
    pub groups: CollisionGroups,
    pub shape: ShapeHandle<N>,
    pub position: Isometry<N>,
    pub material: Material<N>,
    pub density: Option<N>,
    pub linear_prediction: N,
    pub angular_prediction: N,
    pub is_sensor: bool
}

impl<N: Real> ColliderDesc<N> {
    pub fn new(shape: ShapeHandle<N>) -> Self {
        let linear_prediction = na::convert(0.002);
        let angular_prediction = na::convert(f64::consts::PI / 180.0 * 5.0);

        ColliderDesc {
            shape,
            margin: na::convert(0.01),
            groups: CollisionGroups::default(),
            position: Isometry::identity(),
            material: Material::default(),
            density: None,
            linear_prediction,
            angular_prediction,
            is_sensor: false
        }
    }

    pub fn set_translation(&mut self, vector: Vector<N>) -> &mut Self {
        self.position.translation.vector = vector;
        self
    }

    pub fn set_shape(&mut self, shape: ShapeHandle<N>) -> &mut Self {
        self.shape = shape;
        self
    }

    pub fn set_density(&mut self, density: Option<N>) -> &mut Self {
        self.density = density;
        self
    }

    pub fn with_translation(mut self, vector: Vector<N>) -> Self {
        self.position.translation.vector = vector;
        self
    }

    pub fn with_shape(mut self, shape: ShapeHandle<N>) -> Self {
        self.shape = shape;
        self
    }

    pub fn with_density(mut self, density: Option<N>) -> Self {
        self.density = density;
        self
    }

    pub fn build_with_parent<'w>(&self, parent: BodyPartHandle, world: &'w mut World<N>) -> &'w mut Collider<N> {
        self.do_build(parent, world)
    }

    pub fn build<'w>(&self, world: &'w mut World<N>) -> &'w mut Collider<N> {
        self.do_build(BodyPartHandle::ground(), world)
    }

    fn do_build<'w>(&self, parent: BodyPartHandle, world: &'w mut World<N>) -> &'w mut Collider<N> {
        let (bodies, cworld) = world.bodies_mut_and_collision_world_mut();
        let body = bodies.body_mut(parent.0);
        let ndofs = body.status_dependent_ndofs();
        let part = body.part_mut(parent.1);
        self.build_with_infos(parent, ndofs, part, cworld)
    }

    pub(crate) fn build_with_infos<'w>(&self,
                                   parent: BodyPartHandle,
                                   parent_status_dependent_ndofs: usize,
                                   parent_part: &mut BodyPart<N>,
                                   cworld: &'w mut CollisionWorld<N>)
                                -> &'w mut Collider<N> {
        let query = if self.is_sensor {
            GeometricQueryType::Proximity(self.linear_prediction * na::convert(0.5f64))
        } else {
            GeometricQueryType::Contacts(
                self.margin + self.linear_prediction * na::convert(0.5f64),
                self.angular_prediction,
            )
        };

        let (pos, ndofs) = if parent.is_ground() {
            (self.position, 0)
        } else {
            if let Some(density) = self.density {
                let inertia = self.shape.inertia(density);
                // FIXME: contribute to the center of mass too.
                parent_part.add_local_inertia(inertia);
            }

            (
                parent_part.position() * self.position,
                parent_status_dependent_ndofs
            )
        };

        let anchor = ColliderAnchor::OnBodyPart { body_part: parent, position_wrt_body_part: self.position };
        let data = ColliderData::new(self.margin, anchor, ndofs, self.material.clone());
        cworld.add(pos, self.shape.clone(), self.groups, query, data)
    }
}