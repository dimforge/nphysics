use std::sync::Arc;
use std::f64;
use std::mem;
use std::any::Any;
use na::RealField;
use ncollide::world::{CollisionObject, CollisionObjectHandle, GeometricQueryType, CollisionGroups};
use ncollide::shape::{ShapeHandle, Shape};

use crate::math::{Isometry, Vector, Rotation};
use crate::object::{BodyPartHandle, BodyHandle, Body};
use crate::material::{Material, MaterialHandle};
use crate::world::{World, ColliderWorld};
use crate::volumetric::Volumetric;
use crate::utils::{UserData, UserDataBox};


/// Type of the handle of a collider.
pub type ColliderHandle = CollisionObjectHandle;

/// Description of the way a collider is attached to a body.
pub enum ColliderAnchor<N: RealField> {
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

impl<N: RealField> ColliderAnchor<N> {
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
pub struct ColliderData<N: RealField> {
    name: String,
    margin: N,
    anchor: ColliderAnchor<N>,
    // Doubly linked list of colliders attached to a body.
    prev: Option<ColliderHandle>,
    next: Option<ColliderHandle>,
    // NOTE: needed for the collision filter.
    body_status_dependent_ndofs: usize,
    material: MaterialHandle<N>,
    user_data: Option<Box<Any + Send + Sync>>,
}

impl<N: RealField> ColliderData<N> {
    /// Initializes data for a collider.
    pub fn new(
        name: String,
        margin: N,
        anchor: ColliderAnchor<N>,
        body_status_dependent_ndofs: usize,
        material: MaterialHandle<N>,
    ) -> Self {
        ColliderData {
            name,
            margin,
            anchor,
            prev: None,
            next: None,
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


/// A geometric entity that can be attached to a body so it can be affected by contacts and proximity queries.
#[repr(transparent)]
pub struct Collider<N: RealField>(pub CollisionObject<N, ColliderData<N>>);

impl<N: RealField> Collider<N> {
    pub(crate) fn from_ref(co: &CollisionObject<N, ColliderData<N>>) -> &Self {
        unsafe {
            mem::transmute(co)
        }
    }

    pub(crate) fn from_mut(co: &mut CollisionObject<N, ColliderData<N>>) -> &mut Self {
        unsafe {
            mem::transmute(co)
        }
    }

    /*
     * Methods of ColliderData.
     */
    /// The user-data attached to this collider.
    #[inline]
    pub fn user_data(&self) -> Option<&(Any + Send + Sync)> {
        self.0.data().user_data.as_ref().map(|d| &**d)
    }

    /// Mutable reference to the user-data attached to this collider.
    #[inline]
    pub fn user_data_mut(&mut self) -> Option<&mut (Any + Send + Sync)> {
        self.0.data_mut().user_data.as_mut().map(|d| &mut **d)
    }

    /// Sets the user-data attached to this collider.
    #[inline]
    pub fn set_user_data(&mut self, data: Option<Box<Any + Send + Sync>>) -> Option<Box<Any + Send + Sync>> {
        std::mem::replace(&mut self.0.data_mut().user_data, data)
    }

    /// Replace the user-data of this collider by `None` and returns the old value.
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

    /// The user-defined name of this collider.
    #[inline]
    pub fn name(&self) -> &str {
        &self.0.data().name
    }

    /// Sets the name of this collider.
    #[inline]
    pub fn set_name(&mut self, name: String) {
        self.0.data_mut().name = name
    }

    /*
     * Collider chain.
     */

    #[inline]
    pub(crate) fn next(&self) -> Option<ColliderHandle> {
        self.0.data().next
    }

    #[inline]
    pub(crate) fn prev(&self) -> Option<ColliderHandle> {
        self.0.data().prev
    }

    #[inline]
    pub(crate) fn set_next(&mut self, next: Option<ColliderHandle>) {
        self.0.data_mut().next = next
    }

    #[inline]
    pub(crate) fn set_prev(&mut self, prev: Option<ColliderHandle>) {
        self.0.data_mut().prev = prev
    }
}

/// A non-deformable collider builder.
///
/// See https://www.nphysics.org/rigid_body_simulations_with_contacts/#colliders for details.
pub struct ColliderDesc<N: RealField> {
    name: String,
    user_data: Option<UserDataBox>,
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

impl<N: RealField> ColliderDesc<N> {
    /// Creates a new collider builder with the given shape.
    pub fn new(shape: ShapeHandle<N>) -> Self {
        let linear_prediction = na::convert(0.001);
        let angular_prediction = na::convert(f64::consts::PI / 180.0 * 5.0);

        ColliderDesc {
            name: String::new(),
            user_data: None,
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

    /// The default margin surrounding a collider: 0.01
    pub fn default_margin() -> N {
        na::convert(0.01)
    }

    user_data_desc_accessors!();

    #[cfg(feature = "dim3")]
    desc_custom_setters!(
        self.rotation, set_rotation, axisangle: Vector<N> | { self.position.rotation = Rotation::new(axisangle) }
    );

    #[cfg(feature = "dim2")]
    desc_custom_setters!(
        self.rotation, set_rotation, angle: N | { self.position.rotation = Rotation::new(angle) }
    );


    desc_custom_setters!(
        self.translation, set_translation, vector: Vector<N> | { self.position.translation.vector = vector }
        self.material, set_material, material: MaterialHandle<N> | { self.material = Some(material) }
    );

    desc_setters!(
        shape, set_shape, shape: ShapeHandle<N>
        margin, set_margin, margin: N
        density, set_density, density: N
        name, set_name, name: String
        collision_groups, set_collision_groups, collision_groups: CollisionGroups
        linear_prediction, set_linear_prediction, linear_prediction: N
        angular_prediction, set_angular_prediction, angular_prediction: N
        sensor, set_is_sensor, is_sensor: bool
        position, set_position, position: Isometry<N>
    );

    #[cfg(feature = "dim3")]
    desc_custom_getters!(
        self.get_rotation: Vector<N> | { self.position.rotation.scaled_axis() }
    );

    #[cfg(feature = "dim2")]
    desc_custom_getters!(
        self.get_rotation: N | { self.position.rotation.angle() }
    );

    desc_custom_getters!(
        self.get_shape: &Shape<N> | { &*self.shape }
        self.get_name: &str | { &self.name }
        self.get_translation: &Vector<N> | { &self.position.translation.vector }
        self.get_material: Option<&Material<N>> | { self.material.as_ref().map(|m| &**m) }
    );

    desc_getters!(
        [val] get_margin -> margin: N
        [val] get_density -> density: N
        [val] get_collision_groups -> collision_groups: CollisionGroups
        [val] get_linear_prediction -> linear_prediction: N
        [val] get_angular_prediction -> angular_prediction: N
        [val] is_sensor -> is_sensor: bool
        [ref] get_position -> position: Isometry<N>
    );

    /// Builds a collider into the `world` attached to the body part `parent`.
    pub fn build_with_parent<'w>(&self, parent: BodyPartHandle, world: &'w mut World<N>) -> Option<&'w mut Collider<N>> {
        self.do_build(parent, world)
    }

    /// Builds a collider into the `world`.
    pub fn build<'w>(&self, world: &'w mut World<N>) -> &'w mut Collider<N> {
        self.do_build(BodyPartHandle::ground(), world).expect("The world should contain a Ground")
    }

    fn do_build<'w>(&self, parent: BodyPartHandle, world: &'w mut World<N>) -> Option<&'w mut Collider<N>> {
        let (bodies, cworld) = world.bodies_mut_and_collider_world_mut();
        let body = bodies.body_mut(parent.0)?;
        self.build_with_infos(parent, body, cworld)
    }

    // Returns `None` if the given body part does not exist.
    pub(crate) fn build_with_infos<'w>(&self,
                                       parent: BodyPartHandle,
                                       body: &mut Body<N>,
                                       cworld: &'w mut ColliderWorld<N>)
                                    -> Option<&'w mut Collider<N>> {
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
                let com = self.position * self.shape.center_of_mass();
                let inertia = self.shape.inertia(self.density).transformed(&self.position);
                body.add_local_inertia_and_com(parent.1, com, inertia);
            }

            (
                body.part(parent.1)?.position() * self.position,
                body.status_dependent_ndofs()
            )
        };

        let anchor = ColliderAnchor::OnBodyPart { body_part: parent, position_wrt_body_part: self.position };
        let material = self.material.clone().unwrap_or_else(|| cworld.default_material());
        let mut data = ColliderData::new(self.name.clone(), self.margin, anchor, ndofs, material);
        data.user_data = self.user_data.as_ref().map(|data| data.0.to_any());
        Some(cworld.add(pos, self.shape.clone(), self.collision_groups, query, data))
    }
}


/// A deformable collider builder.
pub struct DeformableColliderDesc<N: RealField> {
    name: String,
    user_data: Option<UserDataBox>,
    margin: N,
    collision_groups: CollisionGroups,
    shape: ShapeHandle<N>,
    material: Option<MaterialHandle<N>>,
    linear_prediction: N,
    angular_prediction: N,
    is_sensor: bool,
    body_parts_mapping: Option<Arc<Vec<usize>>>
}

impl<N: RealField> DeformableColliderDesc<N> {
    /// Creates a deformable collider from the given shape.
    ///
    /// Panics if the shape is not deformable.
    pub fn new(shape: ShapeHandle<N>) -> Self {
        assert!(shape.is_deformable_shape(), "The the shape of a deformable collider must be deformable.");
        let linear_prediction = na::convert(0.002);
        let angular_prediction = na::convert(f64::consts::PI / 180.0 * 5.0);

        DeformableColliderDesc {
            name: String::new(),
            user_data: None,
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
}

impl<N: RealField> DeformableColliderDesc<N> {
    user_data_desc_accessors!();

    /// Sets the shape of this collider builder.
    ///
    /// Panics if the shape is not deformable.
    pub fn shape(mut self, shape: ShapeHandle<N>) -> Self {
        assert!(shape.is_deformable_shape(), "The the shape of a deformable collider must be deformable.");
        self.shape = shape;
        self
    }

    /// Sets the shape of this collider builder.
    ///
    /// Panics if the shape is not deformable.
    pub fn set_shape(&mut self, shape: ShapeHandle<N>) -> &mut Self {
        assert!(shape.is_deformable_shape(), "The the shape of a deformable collider must be deformable.");
        self.shape = shape;
        self
    }

    desc_custom_setters!(
        self.material, set_material, material: MaterialHandle<N> | { self.material = Some(material) }
    );

    desc_setters!(
        name, set_name, name: String
        margin, set_margin, margin: N
        collision_groups, set_collision_groups, collision_groups: CollisionGroups
        linear_prediction, set_linear_prediction, linear_prediction: N
        angular_prediction, set_angular_prediction, angular_prediction: N
        as_sensor, set_as_sensor, is_sensor: bool
        body_parts_mapping, set_body_parts_mapping, body_parts_mapping: Option<Arc<Vec<usize>>>
    );

    desc_custom_getters!(
        self.get_shape: &Shape<N> | { &*self.shape }
        self.get_name: &str | { &self.name }
        self.get_material: Option<&Material<N>> | { self.material.as_ref().map(|m| &**m) }

    );

    desc_getters!(
        [val] get_margin -> margin: N
        [val] get_collision_groups -> collision_groups: CollisionGroups
        [val] get_linear_prediction -> linear_prediction: N
        [val] get_angular_prediction -> angular_prediction: N
        [val] get_is_sensor -> is_sensor: bool
    );

    /// Builds a deformable collider attached to `parent` into the `world`.
    pub fn build_parent<'w>(&self, parent: BodyHandle, world: &'w mut World<N>) -> Option<&'w mut Collider<N>> {
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
        let mut data = ColliderData::new(self.name.clone(), self.margin, anchor, ndofs, material);
        data.user_data = self.user_data.as_ref().map(|data| data.0.to_any());
        cworld.add(Isometry::identity(), self.shape.clone(), self.collision_groups, query, data)
    }
}