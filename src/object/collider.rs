use std::f64;
use std::sync::Arc;

use na::RealField;
use ncollide::pipeline::{
    BroadPhaseProxyHandle, CollisionGroups, CollisionObject, CollisionObjectGraphIndex,
    CollisionObjectRef, CollisionObjectUpdateFlags, GeometricQueryType,
};
use ncollide::shape::{Shape, ShapeHandle};
use std::any::Any;

use crate::material::{BasicMaterial, Material, MaterialHandle};
use crate::math::{Isometry, Rotation, Vector};
use crate::object::{BodyHandle, BodyPartHandle};

use crate::utils::{UserData, UserDataBox};

/// Description of the way a collider is attached to a body.
#[derive(Clone)]
pub enum ColliderAnchor<N: RealField, Handle: BodyHandle> {
    /// Attach of a collider with a body part.
    OnBodyPart {
        /// The attached body part handle.
        body_part: BodyPartHandle<Handle>,
        /// Relative position of the collider wrt. the body part.
        position_wrt_body_part: Isometry<N>,
    },
    /// Attach of a collider with a deformable body.
    OnDeformableBody {
        /// The attached body handle.
        body: Handle,
        /// A map between the colliders parts and body part indices.
        ///
        /// The `i`-th part of the collider corresponds to the `body_parts[i]`-th body part.
        /// If set to `None`, the mapping is trivial, i.e., `i`-th part of the collider corresponds to the `i`-th body part.
        body_parts: Option<Arc<Vec<usize>>>,
    },
}

impl<N: RealField, Handle: BodyHandle> ColliderAnchor<N, Handle> {
    /// The body this anchor is attached to.
    #[inline]
    pub fn body(&self) -> Handle {
        match self {
            ColliderAnchor::OnBodyPart { body_part, .. } => body_part.0,
            ColliderAnchor::OnDeformableBody { body, .. } => *body,
        }
    }
}

/// The data a collider set must return after a collider has been removed.
pub struct ColliderRemovalData<N: RealField, Handle: BodyHandle> {
    pub(crate) anchor: ColliderAnchor<N, Handle>,
    pub(crate) shape: ShapeHandle<N>,
    pub(crate) density: N,
    pub(crate) proxy_handle: BroadPhaseProxyHandle,
    pub(crate) graph_index: CollisionObjectGraphIndex,
}

/// Data stored in each collider.
///
/// This is needed by nphysics.
pub struct ColliderData<N: RealField, Handle: BodyHandle> {
    margin: N,
    density: N,
    anchor: ColliderAnchor<N, Handle>,
    // NOTE: needed for the collision filter.
    body_status_dependent_ndofs: usize,
    material: MaterialHandle<N>,
    ccd_enabled: bool,
    user_data: Option<Box<dyn Any + Send + Sync>>,
}

impl<N: RealField, Handle: BodyHandle> ColliderData<N, Handle> {
    /// Initializes data for a collider.
    pub fn new(
        margin: N,
        density: N,
        anchor: ColliderAnchor<N, Handle>,
        body_status_dependent_ndofs: usize,
        material: MaterialHandle<N>,
    ) -> Self {
        ColliderData {
            margin,
            density,
            anchor,
            body_status_dependent_ndofs,
            material,
            ccd_enabled: false,
            user_data: None,
        }
    }

    user_data_accessors!();

    /// The collision margin surrounding this collider.
    #[inline]
    pub fn margin(&self) -> N {
        self.margin
    }

    /// Handle to the body this collider is attached to.
    pub fn body(&self) -> Handle {
        self.anchor.body()
    }

    /// The anchor attaching this collider with a body part or deformable body.
    pub fn anchor(&self) -> &ColliderAnchor<N, Handle> {
        &self.anchor
    }

    /// The density of this collider.
    pub fn density(&self) -> N {
        self.density
    }

    /// The position of this collider geometry wrt. the body it is attached to.
    pub fn position_wrt_body(&self) -> Isometry<N> {
        if let ColliderAnchor::OnBodyPart {
            position_wrt_body_part,
            ..
        } = self.anchor
        {
            position_wrt_body_part
        } else {
            Isometry::identity()
        }
    }

    /// Handle to the body part containing the given subshape of this collider's shape.
    pub fn body_part(&self, subshape_id: usize) -> BodyPartHandle<Handle> {
        match &self.anchor {
            ColliderAnchor::OnBodyPart { body_part, .. } => *body_part,
            ColliderAnchor::OnDeformableBody {
                body, body_parts, ..
            } => {
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
    pub fn material(&self) -> &dyn Material<N> {
        &*self.material
    }

    /// A mutable reference to this collider's material.
    ///
    /// If the material is shared, then an internal clone is performed
    /// before returning the mutable reference (this effectively calls
    /// the `Arc::make_mut` method to get a copy-on-write behavior).
    #[inline]
    pub fn material_mut(&mut self) -> &mut dyn Material<N> {
        self.material.make_mut()
    }
}

/// A geometric entity that can be attached to a body so it can be affected by contacts and proximity queries.
#[repr(transparent)]
pub struct Collider<N: RealField, Handle: BodyHandle>(
    pub(crate) CollisionObject<N, ColliderData<N, Handle>>,
); // FIXME: keep this pub(crate) or private?

impl<N: RealField, Handle: BodyHandle> Collider<N, Handle> {
    /// Computes the data that needs to be returned once this collider has been removed from a collider set.
    pub fn removal_data(&self) -> Option<ColliderRemovalData<N, Handle>> {
        Some(ColliderRemovalData {
            anchor: self.0.data().anchor.clone(),
            shape: self.shape_handle().clone(),
            density: self.0.data().density,
            proxy_handle: self.0.proxy_handle()?,
            graph_index: self.0.graph_index()?,
        })
    }

    /*
     * Methods of ColliderData.
     */
    /// The user-data attached to this collider.
    #[inline]
    pub fn user_data(&self) -> Option<&(dyn Any + Send + Sync)> {
        self.0.data().user_data.as_deref()
    }

    /// Mutable reference to the user-data attached to this collider.
    #[inline]
    pub fn user_data_mut(&mut self) -> Option<&mut (dyn Any + Send + Sync)> {
        self.0.data_mut().user_data.as_deref_mut()
    }

    /// Sets the user-data attached to this collider.
    #[inline]
    pub fn set_user_data(
        &mut self,
        data: Option<Box<dyn Any + Send + Sync>>,
    ) -> Option<Box<dyn Any + Send + Sync>> {
        std::mem::replace(&mut self.0.data_mut().user_data, data)
    }

    /// Replaces the user-data of this collider by `None` and returns the old value.
    #[inline]
    pub fn take_user_data(&mut self) -> Option<Box<dyn Any + Send + Sync>> {
        self.0.data_mut().user_data.take()
    }

    /// The collision margin surrounding this collider.
    #[inline]
    pub fn margin(&self) -> N {
        self.0.data().margin()
    }

    /// Sets the marging on this collider's shapes.
    #[inline]
    pub fn set_margin(&mut self, margin: N) {
        *self.0.update_flags_mut() |= CollisionObjectUpdateFlags::SHAPE_CHANGED;
        self.0.data_mut().margin = margin;
    }

    /// Clears all the internal flags tracking changes made to this collider.
    #[inline]
    pub fn clear_update_flags(&mut self) {
        self.0.clear_update_flags()
    }

    /// The density of this collider.
    #[inline]
    pub fn density(&self) -> N {
        self.0.data().density
    }

    /// Handle to the body this collider is attached to.
    #[inline]
    pub fn body(&self) -> Handle {
        self.0.data().body()
    }

    /// The anchor attaching this collider with a body part or deformable body.
    #[inline]
    pub fn anchor(&self) -> &ColliderAnchor<N, Handle> {
        self.0.data().anchor()
    }

    /// The position of this collider geometry wrt. the body it is attached to.
    #[inline]
    pub fn position_wrt_body(&self) -> Isometry<N> {
        self.0.data().position_wrt_body()
    }

    /// Handle to the body part containing the given subshape of this collider's shape.
    #[inline]
    pub fn body_part(&self, subshape_id: usize) -> BodyPartHandle<Handle> {
        self.0.data().body_part(subshape_id)
    }

    /// The material of this collider.
    #[inline]
    pub fn material(&self) -> &dyn Material<N> {
        self.0.data().material()
    }

    /// A mutable reference to this collider's material.
    ///
    /// If the material is shared, then an internal clone is performed
    /// before returning the mutable reference (this effectively calls
    /// the `Arc::make_mut` method to get a copy-on-write behavior).
    #[inline]
    pub fn material_mut(&mut self) -> &mut dyn Material<N> {
        self.0.data_mut().material_mut()
    }

    /// Returns `true` if this collider is a sensor.
    #[inline]
    pub fn is_sensor(&self) -> bool {
        self.query_type().is_proximity_query()
    }

    /// Returns `true` if this collider is subjected to Continuous Collision Detection (CCD).
    #[inline]
    pub fn is_ccd_enabled(&self) -> bool {
        self.0.data().ccd_enabled
    }

    /// Enables or disables Continuous Collision Detection (CCD) for this collider.
    #[inline]
    pub fn enable_ccd(&mut self, enabled: bool) {
        self.0.data_mut().ccd_enabled = enabled
    }

    #[inline]
    pub(crate) fn body_status_dependent_ndofs(&self) -> usize {
        self.0.data().body_status_dependent_ndofs
    }

    #[inline]
    pub(crate) fn set_body_status_dependent_ndofs(&mut self, ndofs: usize) {
        self.0.data_mut().body_status_dependent_ndofs = ndofs
    }

    /*
     * Original methods from the CollisionObject.
     */

    /// This collider's non-stable graph index.
    ///
    /// This index may change whenever a collider is removed from the world.
    #[inline]
    pub fn graph_index(&self) -> Option<CollisionObjectGraphIndex> {
        self.0.graph_index()
    }

    /// Sets the collider unique but non-stable graph index.
    #[inline]
    pub fn set_graph_index(&mut self, index: Option<CollisionObjectGraphIndex>) {
        self.0.set_graph_index(index)
    }

    /// The collider's broad phase proxy unique identifier.
    #[inline]
    pub fn proxy_handle(&self) -> Option<BroadPhaseProxyHandle> {
        self.0.proxy_handle()
    }

    /// Sets the collider's broad phase proxy unique identifier.
    #[inline]
    pub fn set_proxy_handle(&mut self, handle: Option<BroadPhaseProxyHandle>) {
        self.0.set_proxy_handle(handle)
    }

    /// The collider position.
    #[inline]
    pub fn position(&self) -> &Isometry<N> {
        self.0.position()
    }

    /// Sets the position of the collider.
    #[inline]
    pub fn set_position(&mut self, pos: Isometry<N>) {
        self.0.set_position(pos)
    }

    /// Sets the position of the collider.
    #[inline]
    pub fn set_position_with_prediction(&mut self, position: Isometry<N>, prediction: Isometry<N>) {
        self.0.set_position_with_prediction(position, prediction)
    }

    /// Deforms the underlying shape if possible.
    ///
    /// Panics if the shape is not deformable.
    #[inline]
    pub fn set_deformations(&mut self, coords: &[N]) {
        self.0.set_deformations(coords)
    }

    /// This collider's shape.
    #[inline]
    pub fn shape(&self) -> &dyn Shape<N> {
        &**self.0.shape()
    }

    /// This collider's shape.
    #[inline]
    pub fn shape_handle(&self) -> &ShapeHandle<N> {
        self.0.shape()
    }

    /// Sets this collider's shape.
    #[inline]
    pub fn set_shape(&mut self, shape: ShapeHandle<N>) {
        self.0.set_shape(shape)
    }

    /// The collision groups of the collider.
    #[inline]
    pub fn collision_groups(&self) -> &CollisionGroups {
        self.0.collision_groups()
    }

    /// Sets the collision groups of this collider.
    #[inline]
    pub fn set_collision_groups(&mut self, groups: CollisionGroups) {
        self.0.set_collision_groups(groups)
    }

    /// Returns the kind of queries this collider is expected to emit.
    #[inline]
    pub fn query_type(&self) -> GeometricQueryType<N> {
        self.0.query_type()
    }

    /// Sets the `GeometricQueryType` of the collider.
    /// Use `CollisionWorld::set_query_type` to use this method.
    #[inline]
    pub fn set_query_type(&mut self, query_type: GeometricQueryType<N>) {
        self.0.set_query_type(query_type);
    }
}

impl<N: RealField, Handle: BodyHandle> CollisionObjectRef<N> for Collider<N, Handle> {
    fn graph_index(&self) -> Option<CollisionObjectGraphIndex> {
        self.0.graph_index()
    }

    fn proxy_handle(&self) -> Option<BroadPhaseProxyHandle> {
        self.0.proxy_handle()
    }

    fn position(&self) -> &Isometry<N> {
        self.0.position()
    }

    fn predicted_position(&self) -> Option<&Isometry<N>> {
        self.0.predicted_position()
    }

    fn shape(&self) -> &dyn Shape<N> {
        self.0.shape().as_ref()
    }

    fn collision_groups(&self) -> &CollisionGroups {
        self.0.collision_groups()
    }

    fn query_type(&self) -> GeometricQueryType<N> {
        self.0.query_type()
    }

    fn update_flags(&self) -> CollisionObjectUpdateFlags {
        self.0.update_flags()
    }
}

/// A non-deformable collider builder.
///
/// See https://www.nphysics.org/rigid_body_simulations_with_contacts/#colliders for details.
pub struct ColliderDesc<N: RealField> {
    user_data: Option<UserDataBox>,
    margin: N,
    collision_groups: CollisionGroups,
    shape: ShapeHandle<N>,
    position: Isometry<N>,
    material: Option<MaterialHandle<N>>,
    density: N,
    linear_prediction: N,
    angular_prediction: N,
    is_sensor: bool,
    ccd_enabled: bool,
}

impl<N: RealField> ColliderDesc<N> {
    /// Creates a new collider builder with the given shape.
    pub fn new(shape: ShapeHandle<N>) -> Self {
        let linear_prediction = na::convert(0.001);
        let angular_prediction = na::convert(f64::consts::PI / 180.0 * 5.0);

        ColliderDesc {
            user_data: None,
            shape,
            margin: Self::default_margin(),
            collision_groups: CollisionGroups::default(),
            position: Isometry::identity(),
            material: None,
            density: N::zero(),
            linear_prediction,
            angular_prediction,
            is_sensor: false,
            ccd_enabled: false,
        }
    }

    /// The default margin surrounding a collider: 0.01
    pub fn default_margin() -> N {
        na::convert(0.01)
    }

    user_data_desc_accessors!();

    #[cfg(feature = "dim3")]
    desc_custom_setters!(
        self.rotation,
        set_rotation,
        axisangle: Vector<N> | { self.position.rotation = Rotation::new(axisangle) }
    );

    #[cfg(feature = "dim2")]
    desc_custom_setters!(
        self.rotation,
        set_rotation,
        angle: N | { self.position.rotation = Rotation::new(angle) }
    );

    desc_custom_setters!(
        self.translation, set_translation, vector: Vector<N> | { self.position.translation.vector = vector }
        self.material, set_material, material: MaterialHandle<N> | { self.material = Some(material) }
    );

    desc_setters!(
        shape, set_shape, shape: ShapeHandle<N>
        margin, set_margin, margin: N
        density, set_density, density: N
        collision_groups, set_collision_groups, collision_groups: CollisionGroups
        linear_prediction, set_linear_prediction, linear_prediction: N
        angular_prediction, set_angular_prediction, angular_prediction: N
        sensor, set_is_sensor, is_sensor: bool
        position, set_position, position: Isometry<N>
        ccd_enabled, set_ccd_enabled, ccd_enabled: bool
    );

    #[cfg(feature = "dim3")]
    desc_custom_getters!(self.get_rotation: Vector<N> | { self.position.rotation.scaled_axis() });

    #[cfg(feature = "dim2")]
    desc_custom_getters!(self.get_rotation: N | { self.position.rotation.angle() });

    desc_custom_getters!(
        self.get_shape: &dyn Shape<N> | { &*self.shape }
        self.get_translation: &Vector<N> | { &self.position.translation.vector }
        self.get_material: Option<&dyn Material<N>> | { self.material.as_deref() }
    );

    desc_getters!(
        [val] get_margin -> margin: N
        [val] get_density -> density: N
        [val] get_collision_groups -> collision_groups: CollisionGroups
        [val] get_linear_prediction -> linear_prediction: N
        [val] get_angular_prediction -> angular_prediction: N
        [val] is_sensor -> is_sensor: bool
        [val] get_ccd_enabled -> ccd_enabled: bool
        [ref] get_position -> position: Isometry<N>
    );

    /// Build a collider and configure it to be attached to the given parent body part.
    pub fn build<Handle: BodyHandle>(
        &self,
        parent_handle: BodyPartHandle<Handle>,
    ) -> Collider<N, Handle> {
        let query = if self.is_sensor {
            GeometricQueryType::Proximity(self.linear_prediction)
        } else {
            GeometricQueryType::Contacts(
                self.margin + self.linear_prediction,
                self.angular_prediction,
            )
        };

        let anchor = ColliderAnchor::OnBodyPart {
            body_part: parent_handle,
            position_wrt_body_part: self.position,
        };
        let material = self
            .material
            .clone()
            .unwrap_or_else(|| MaterialHandle::new(BasicMaterial::default()));
        let mut data = ColliderData::new(self.margin, self.density, anchor, 0, material);
        data.ccd_enabled = self.ccd_enabled;
        data.user_data = self.user_data.as_ref().map(|data| data.0.to_any());
        let co = CollisionObject::new(
            None,
            None,
            self.position,
            self.shape.clone(),
            self.collision_groups,
            query,
            data,
        );
        Collider(co)
    }
}

/// A deformable collider builder.
pub struct DeformableColliderDesc<N: RealField> {
    user_data: Option<UserDataBox>,
    margin: N,
    collision_groups: CollisionGroups,
    shape: ShapeHandle<N>,
    material: Option<MaterialHandle<N>>,
    linear_prediction: N,
    angular_prediction: N,
    is_sensor: bool,
    ccd_enabled: bool,
    body_parts_mapping: Option<Arc<Vec<usize>>>,
}

impl<N: RealField> DeformableColliderDesc<N> {
    /// Creates a deformable collider from the given shape.
    ///
    /// Panics if the shape is not deformable.
    pub fn new(shape: ShapeHandle<N>) -> Self {
        assert!(
            shape.is_deformable_shape(),
            "The the shape of a deformable collider must be deformable."
        );
        let linear_prediction = na::convert(0.002);
        let angular_prediction = na::convert(f64::consts::PI / 180.0 * 5.0);

        DeformableColliderDesc {
            user_data: None,
            shape,
            margin: na::convert(0.01),
            collision_groups: CollisionGroups::default(),
            material: None,
            linear_prediction,
            angular_prediction,
            is_sensor: false,
            ccd_enabled: false,
            body_parts_mapping: None,
        }
    }
}

impl<N: RealField> DeformableColliderDesc<N> {
    user_data_desc_accessors!();

    /// Sets the shape of this collider builder.
    ///
    /// Panics if the shape is not deformable.
    pub fn shape(mut self, shape: ShapeHandle<N>) -> Self {
        assert!(
            shape.is_deformable_shape(),
            "The the shape of a deformable collider must be deformable."
        );
        self.shape = shape;
        self
    }

    /// Sets the shape of this collider builder.
    ///
    /// Panics if the shape is not deformable.
    pub fn set_shape(&mut self, shape: ShapeHandle<N>) -> &mut Self {
        assert!(
            shape.is_deformable_shape(),
            "The the shape of a deformable collider must be deformable."
        );
        self.shape = shape;
        self
    }

    desc_custom_setters!(
        self.material,
        set_material,
        material: MaterialHandle<N> | { self.material = Some(material) }
    );

    desc_setters!(
        margin, set_margin, margin: N
        collision_groups, set_collision_groups, collision_groups: CollisionGroups
        linear_prediction, set_linear_prediction, linear_prediction: N
        angular_prediction, set_angular_prediction, angular_prediction: N
        as_sensor, set_as_sensor, is_sensor: bool
        ccd_enabled, set_ccd_enabled, ccd_enabled: bool
        body_parts_mapping, set_body_parts_mapping, body_parts_mapping: Option<Arc<Vec<usize>>>
    );

    desc_custom_getters!(
        self.get_shape: &dyn Shape<N> | { &*self.shape }
        self.get_material: Option<&dyn Material<N>> | { self.material.as_ref().map(|m| &**m) }

    );

    desc_getters!(
        [val] get_margin -> margin: N
        [val] get_collision_groups -> collision_groups: CollisionGroups
        [val] get_linear_prediction -> linear_prediction: N
        [val] get_angular_prediction -> angular_prediction: N
        [val] get_is_sensor -> is_sensor: bool
        [val] get_ccd_enabled -> ccd_enabled: bool
    );

    /// Builds a deformable collider and configures it to be attached to the given parent body.
    pub fn build<Handle: BodyHandle>(&self, parent_handle: Handle) -> Collider<N, Handle> {
        let query = if self.is_sensor {
            GeometricQueryType::Proximity(self.linear_prediction)
        } else {
            GeometricQueryType::Contacts(
                self.margin + self.linear_prediction,
                self.angular_prediction,
            )
        };

        let body_parts = self.body_parts_mapping.clone();
        let anchor = ColliderAnchor::OnDeformableBody {
            body: parent_handle,
            body_parts,
        };
        let material = self
            .material
            .clone()
            .unwrap_or_else(|| MaterialHandle::new(BasicMaterial::default()));
        let mut data = ColliderData::new(self.margin, N::zero(), anchor, 0, material);
        data.ccd_enabled = data.ccd_enabled;
        data.user_data = self.user_data.as_ref().map(|data| data.0.to_any());

        let co = CollisionObject::new(
            None,
            None,
            Isometry::identity(),
            self.shape.clone(),
            self.collision_groups,
            query,
            data,
        );
        Collider(co)
    }
}
