use std::sync::Arc;
use std::f64;
use std::mem;
use std::any::Any;
use na::Real;
use ncollide::world::{CollisionObject, CollisionObjectHandle, GeometricQueryType, CollisionGroups};
use ncollide::shape::{ShapeHandle, Shape};

use crate::math::{Isometry, Vector, Rotation};
use crate::object::{BodyPartHandle, BodyHandle, Material, MaterialHandle, Body, BodyPart};
use crate::world::{World, ColliderWorld};
use crate::volumetric::Volumetric;

/// Type of the handle of a collider.
pub type ColliderHandle = CollisionObjectHandle;

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
    material: MaterialHandle<N>,
    user_data: Option<Box<Any + Send + Sync>>,
}

impl<N: Real> ColliderData<N> {
    /// Initializes data for a collider.
    pub fn new(
        margin: N,
        anchor: ColliderAnchor<N>,
        body_status_dependent_ndofs: usize,
        material: MaterialHandle<N>,
    ) -> Self {
        ColliderData {
            margin,
            anchor,
            body_status_dependent_ndofs,
            material,
            user_data: None
        }
    }

    user_data_accessors!();

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
        &*self.material
    }

    /// A mutable reference to this collider's material.
    ///
    /// If the material is shared, then an internal clone is performed
    /// before returning the mutable reference (this effectively call
    /// the `Arc::make_mut` method to get a copy-on-write behavior).
    #[inline]
    pub fn material_mut(&mut self) -> &mut Material<N> {
        self.material.make_mut()
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


#[repr(transparent)]
pub struct Collider<N: Real>(pub CollisionObject<N, ColliderData<N>>);

impl<N: Real> Collider<N> {
    pub fn from_ref(co: &CollisionObject<N, ColliderData<N>>) -> &Self {
        unsafe {
            mem::transmute(co)
        }
    }

    pub fn from_mut(co: &mut CollisionObject<N, ColliderData<N>>) -> &mut Self {
        unsafe {
            mem::transmute(co)
        }
    }

    /*
     * Methods of ColliderData.
     */
    #[inline]
    pub fn user_data(&self) -> Option<&(Any + Send + Sync)> {
        self.0.data().user_data.as_ref().map(|d| &**d)
    }

    #[inline]
    pub fn user_data_mut(&mut self) -> Option<&mut (Any + Send + Sync)> {
        self.0.data_mut().user_data.as_mut().map(|d| &mut **d)
    }

    #[inline]
    pub fn set_user_data(&mut self, data: Option<Box<Any + Send + Sync>>) -> Option<Box<Any + Send + Sync>> {
        std::mem::replace(&mut self.0.data_mut().user_data, data)
    }

    #[inline]
    pub fn take_user_data(&mut self) -> Option<Box<Any + Send + Sync>> {
        self.0.data_mut().user_data.take()
    }

    /// The collision margin surrounding this collider.
    #[inline]
    pub fn margin(&self) -> N {
        self.0.data().margin()
    }

    /// Handle to the body this collider is attached to.
    pub fn body(&self) -> BodyHandle {
        self.0.data().body()
    }

    /// The anchor attaching this collider with a body part or deformable body.
    pub fn anchor(&self) -> &ColliderAnchor<N> {
        self.0.data().anchor()
    }

    /// The position of this collider geometry wrt. the body it is attached to.
    pub fn position_wrt_body(&self) -> Isometry<N> {
        self.0.data().position_wrt_body()
    }

    /// Handle to the body part containing the given subshape of this collider's shape.
    pub fn body_part(&self, subshape_id: usize) -> BodyPartHandle {
        self.0.data().body_part(subshape_id)
    }

    /// The material of this collider.
    #[inline]
    pub fn material(&self) -> &Material<N> {
        self.0.data().material()
    }

    /// Returns `true` if this collider is a sensor.
    #[inline]
    pub fn is_sensor(&self) -> bool {
        self.query_type().is_proximity_query()
    }

    /*
     * Original methods from the CollisionObject.
     */

    /// The collision object unique handle.
    #[inline]
    pub fn handle(&self) -> ColliderHandle {
        self.0.handle()
    }

    /// The collision object position.
    #[inline]
    pub fn position(&self) -> &Isometry<N> {
        self.0.position()
    }

    /// Sets the position of the collision object.
    #[inline]
    pub fn set_position(&mut self, pos: Isometry<N>) {
        self.0.set_position(pos)
    }

    /// Deforms the underlying shape if possible.
    ///
    /// Panics if the shape is not deformable.
    #[inline]
    pub fn set_deformations(&mut self, coords: &[N]) {
        self.0.set_deformations(coords)
    }

    /// The collision object shape.
    #[inline]
    pub fn shape(&self) -> &ShapeHandle<N> {
        self.0.shape()
    }

    /// The collision groups of the collision object.
    #[inline]
    pub fn collision_groups(&self) -> &CollisionGroups {
        self.0.collision_groups()
    }

    /// The kind of queries this collision object may generate.
    #[inline]
    pub fn query_type(&self) -> GeometricQueryType<N> {
        self.0.query_type()
    }
}

pub struct ColliderDesc<N: Real> {
    margin: N,
    collision_groups: CollisionGroups,
    shape: ShapeHandle<N>,
    position: Isometry<N>,
    material: Option<MaterialHandle<N>>,
    density: N,
    linear_prediction: N,
    angular_prediction: N,
    is_sensor: bool
}

impl<N: Real> ColliderDesc<N> {
    pub fn new(shape: ShapeHandle<N>) -> Self {
        let linear_prediction = na::convert(0.001);
        let angular_prediction = na::convert(f64::consts::PI / 180.0 * 5.0);

        ColliderDesc {
            shape,
            margin: Self::default_margin(),
            collision_groups: CollisionGroups::default(),
            position: Isometry::identity(),
            material: None,
            density: N::zero(),
            linear_prediction,
            angular_prediction,
            is_sensor: false
        }
    }

    pub fn default_margin() -> N {
        na::convert(0.01)
    }

    #[cfg(feature = "dim3")]
    desc_custom_setters!(
        self.with_rotation, set_rotation, axisangle: Vector<N> | { self.position.rotation = Rotation::new(axisangle) }
    );

    #[cfg(feature = "dim2")]
    desc_custom_setters!(
        self.with_rotation, set_rotation, angle: N | { self.position.rotation = Rotation::new(angle) }
    );


    desc_custom_setters!(
        self.with_translation, set_translation, vector: Vector<N> | { self.position.translation.vector = vector }
        self.with_material, set_material, material: MaterialHandle<N> | { self.material = Some(material) }
    );

    desc_setters!(
        with_shape, set_shape, shape: ShapeHandle<N>
        with_margin, set_margin, margin: N
        with_density, set_density, density: N
        with_collision_groups, set_collision_groups, collision_groups: CollisionGroups
        with_linear_prediction, set_linear_prediction, linear_prediction: N
        with_angular_prediction, set_angular_prediction, angular_prediction: N
        as_sensor, set_as_sensor, is_sensor: bool
        with_position, set_position, position: Isometry<N>
    );

    #[cfg(feature = "dim3")]
    desc_custom_getters!(
        self.rotation: Vector<N> | { self.position.rotation.scaled_axis() }
    );

    #[cfg(feature = "dim2")]
    desc_custom_getters!(
        self.rotation: N | { self.position.rotation.angle() }
    );

    desc_custom_getters!(
        self.shape: &Shape<N> | { &*self.shape }
        self.translation: &Vector<N> | { &self.position.translation.vector }
        self.material: Option<&Material<N>> | { self.material.as_ref().map(|m| &**m) }
    );

    desc_getters!(
        [val] margin: N
        [val] density: N
        [val] collision_groups: CollisionGroups
        [val] linear_prediction: N
        [val] angular_prediction: N
        [val] is_sensor: bool
        [ref] position: Isometry<N>
    );

    pub fn build_with_parent<'w>(&self, parent: BodyPartHandle, world: &'w mut World<N>) -> Option<&'w mut Collider<N>> {
        self.do_build(parent, world)
    }

    pub fn build<'w>(&self, world: &'w mut World<N>) -> &'w mut Collider<N> {
        self.do_build(BodyPartHandle::ground(), world).expect("The world should contain a Ground")
    }

    fn do_build<'w>(&self, parent: BodyPartHandle, world: &'w mut World<N>) -> Option<&'w mut Collider<N>> {
        let (bodies, cworld) = world.bodies_mut_and_collider_world_mut();
        let body = bodies.body_mut(parent.0)?;
        let ndofs = body.status_dependent_ndofs();
        let part = body.part_mut(parent.1)?;
        Some(self.build_with_infos(parent, ndofs, part, cworld))
    }

    pub(crate) fn build_with_infos<'w>(&self,
                                       parent: BodyPartHandle,
                                       parent_status_dependent_ndofs: usize,
                                       parent_part: &mut BodyPart<N>,
                                       cworld: &'w mut ColliderWorld<N>)
                                    -> &'w mut Collider<N> {
        let query = if self.is_sensor {
            GeometricQueryType::Proximity(self.linear_prediction)
        } else {
            GeometricQueryType::Contacts(
                self.margin + self.linear_prediction,
                self.angular_prediction,
            )
        };

        let (pos, ndofs) = if parent.is_ground() {
            (self.position, 0)
        } else {
            if !self.density.is_zero() {
                let com = self.shape.center_of_mass();
                let inertia = self.shape.inertia(self.density);
                parent_part.add_local_inertia_and_com(com, inertia);
            }

            (
                parent_part.position() * self.position,
                parent_status_dependent_ndofs
            )
        };

        let anchor = ColliderAnchor::OnBodyPart { body_part: parent, position_wrt_body_part: self.position };
        let material = self.material.clone().unwrap_or_else(|| cworld.default_material());
        let data = ColliderData::new(self.margin, anchor, ndofs, material);
        cworld.add(pos, self.shape.clone(), self.collision_groups, query, data)
    }
}


pub struct DeformableColliderDesc<N: Real> {
    margin: N,
    collision_groups: CollisionGroups,
    shape: ShapeHandle<N>,
    material: Option<MaterialHandle<N>>,
    linear_prediction: N,
    angular_prediction: N,
    is_sensor: bool,
    body_parts_mapping: Option<Arc<Vec<usize>>>
}

impl<N: Real> DeformableColliderDesc<N> {
    pub fn new(shape: ShapeHandle<N>) -> Self {
        assert!(shape.is_deformable_shape(), "The the shape of a deformable collider must be deformable.");
        let linear_prediction = na::convert(0.002);
        let angular_prediction = na::convert(f64::consts::PI / 180.0 * 5.0);

        DeformableColliderDesc {
            shape,
            margin: na::convert(0.01),
            collision_groups: CollisionGroups::default(),
            material: None,
            linear_prediction,
            angular_prediction,
            is_sensor: false,
            body_parts_mapping: None
        }
    }

    pub fn with_shape(mut self, shape: ShapeHandle<N>) -> Self {
        assert!(shape.is_deformable_shape(), "The the shape of a deformable collider must be deformable.");
        self.shape = shape;
        self
    }

    pub fn set_shape(&mut self, shape: ShapeHandle<N>) -> &mut Self {
        assert!(shape.is_deformable_shape(), "The the shape of a deformable collider must be deformable.");
        self.shape = shape;
        self
    }

    desc_custom_setters!(
        self.with_material, set_material, material: MaterialHandle<N> | { self.material = Some(material) }
    );

    desc_setters!(
        with_margin, set_margin, margin: N
        with_collision_groups, set_collision_groups, collision_groups: CollisionGroups
        with_linear_prediction, set_linear_prediction, linear_prediction: N
        with_angular_prediction, set_angular_prediction, angular_prediction: N
        as_sensor, set_as_sensor, is_sensor: bool
        with_body_parts_mapping, set_body_parts_mapping, body_parts_mapping: Option<Arc<Vec<usize>>>
    );

    desc_custom_getters!(
        self.shape: &Shape<N> | { &*self.shape }
        self.material: Option<&Material<N>> | { self.material.as_ref().map(|m| &**m) }

    );

    desc_getters!(
        [val] margin: N
        [val] collision_groups: CollisionGroups
        [val] linear_prediction: N
        [val] angular_prediction: N
        [val] is_sensor: bool
    );

    pub fn build_with_parent<'w>(&self, parent: BodyHandle, world: &'w mut World<N>) -> Option<&'w mut Collider<N>> {
        let (bodies, cworld) = world.bodies_mut_and_collider_world_mut();
        let parent = bodies.body(parent)?;
        Some(self.build_with_infos(parent, cworld))
    }

    pub(crate) fn build_with_infos<'w>(&self,
                                       parent: &Body<N>,
                                       cworld: &'w mut ColliderWorld<N>)
                                       -> &'w mut Collider<N> {
        let query = if self.is_sensor {
            GeometricQueryType::Proximity(self.linear_prediction)
        } else {
            GeometricQueryType::Contacts(
                self.margin + self.linear_prediction,
                self.angular_prediction,
            )
        };

        let parent_deformation_type = parent
            .deformed_positions()
            .expect("A deformable collider can only be attached to a deformable body.")
            .0;

        assert_eq!(
            parent_deformation_type,
            self.shape.as_deformable_shape().unwrap().deformations_type(),
            "Both the deformable shape and deformable body must support the same deformation types."
        );

        let body = parent.handle();
        let ndofs = parent.status_dependent_ndofs();
        let body_parts = self.body_parts_mapping.clone();
        let anchor = ColliderAnchor::OnDeformableBody { body, body_parts };
        let material = self.material.clone().unwrap_or_else(|| cworld.default_material());
        let data = ColliderData::new(self.margin, anchor, ndofs, material);
        cworld.add(Isometry::identity(), self.shape.clone(), self.collision_groups, query, data)
    }
}