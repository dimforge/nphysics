use std::any::Any;
use na::{DVectorSlice, DVectorSliceMut, RealField};

use crate::math::{Force, Inertia, Isometry, Point, Rotation, Translation, Vector, Velocity,
                  SpatialVector, SPATIAL_DIM, DIM, Dim, ForceType};
use crate::object::{ActivationStatus, BodyPartHandle, BodyStatus, Body, BodyPart, BodyHandle,
                    ColliderDesc, BodyDesc, BodyUpdateStatus};
use crate::solver::{IntegrationParameters, ForceDirection};
use crate::world::{World, ColliderWorld};
use crate::utils::{UserData, UserDataBox};
use ncollide::shape::DeformationsType;

#[cfg(feature = "dim3")]
use crate::math::AngularVector;
#[cfg(feature = "dim3")]
use crate::utils::GeneralizedCross;


/// A rigid body.
#[derive(Debug)]
pub struct RigidBody<N: RealField> {
    name: String,
    handle: BodyHandle,
    position: Isometry<N>,
    velocity: Velocity<N>,
    local_inertia: Inertia<N>,
    inertia: Inertia<N>,
    local_com: Point<N>,
    com: Point<N>,
    augmented_mass: Inertia<N>,
    inv_augmented_mass: Inertia<N>,
    external_forces: Force<N>,
    acceleration: Velocity<N>,
    status: BodyStatus,
    gravity_enabled: bool,
    activation: ActivationStatus<N>,
    jacobian_mask: SpatialVector<N>,
    companion_id: usize,
    update_status: BodyUpdateStatus,
    user_data: Option<Box<Any + Send + Sync>>
}

impl<N: RealField> RigidBody<N> {
    /// Create a new rigid body with the specified handle and dynamic properties.
    fn new(handle: BodyHandle, position: Isometry<N>) -> Self {
        let inertia = Inertia::zero();
        let com = Point::from(position.translation.vector);

        RigidBody {
            name: String::new(),
            handle,
            position,
            velocity: Velocity::zero(),
            local_inertia: inertia,
            inertia,
            local_com: Point::origin(),
            com,
            augmented_mass: inertia,
            inv_augmented_mass: inertia.inverse(),
            external_forces: Force::zero(),
            acceleration: Velocity::zero(),
            status: BodyStatus::Dynamic,
            gravity_enabled: true,
            activation: ActivationStatus::new_active(),
            jacobian_mask: SpatialVector::repeat(N::one()),
            companion_id: 0,
            update_status: BodyUpdateStatus::all(),
            user_data: None
        }
    }

    user_data_accessors!();

    /// Mark some translational degrees of freedom as kinematic.
    pub fn set_translations_kinematic(&mut self, is_kinematic: Vector<bool>) {
        self.update_status.set_status_changed(true);
        for i in 0..DIM {
            self.jacobian_mask[i] = if is_kinematic[i] { N::zero() } else { N::one() }
        }
    }

    /// Mark some rotational degrees of freedom as kinematic.
    #[cfg(feature = "dim3")]
    pub fn set_rotations_kinematic(&mut self, is_kinematic: Vector<bool>) {
        self.update_status.set_status_changed(true);
        self.jacobian_mask[3] = if is_kinematic.x { N::zero() } else { N::one() };
        self.jacobian_mask[4] = if is_kinematic.y { N::zero() } else { N::one() };
        self.jacobian_mask[5] = if is_kinematic.z { N::zero() } else { N::one() };
    }

    /// Mark rotations as kinematic.
    #[cfg(feature = "dim2")]
    pub fn set_rotation_kinematic(&mut self, is_kinematic: bool) {
        self.update_status.set_status_changed(true);
        self.jacobian_mask[2] = if is_kinematic { N::zero() } else { N::one() };
    }


    /// Flags indicating which translational degrees of freedoms are kinematic.
    pub fn kinematic_translations(&self) -> Vector<bool> {
        self.jacobian_mask.fixed_rows::<Dim>(0).map(|m| m.is_zero())
    }

    /// Flags indicating which rotational degrees of freedoms are kinematic.
    #[cfg(feature = "dim3")]
    pub fn kinematic_rotations(&self) -> Vector<bool> {
        Vector::new(
            self.jacobian_mask[3].is_zero(),
            self.jacobian_mask[4].is_zero(),
            self.jacobian_mask[5].is_zero(),
        )
    }

    /// Flags indicating if rotations are kinematic.
    #[cfg(feature = "dim2")]
    pub fn kinematic_rotation(&self) -> bool {
        self.jacobian_mask[2].is_zero()
    }

    /// Disable all rotations of this rigid body.
    ///
    /// This is the same as setting all the rotations of this rigid body as kinematic and setting
    /// its angular velocity to zero. The rotations will still be controllable at the velocity level
    /// by the user afterwards.
    pub fn disable_all_rotations(&mut self) {
        self.update_status.set_velocity_changed(true);
        #[cfg(feature = "dim3")]
            {
                self.set_rotations_kinematic(Vector::repeat(true));
                self.velocity.angular = Vector::zeros();
            }
        #[cfg(feature = "dim2")]
            {
                self.set_rotation_kinematic(true);
                self.velocity.angular = N::zero();
            }
    }

    /// Enable all rotations for this rigid body.
    ///
    /// This is the same as setting all the rotations of this rigid body as non-kinematic.
    pub fn enable_all_rotations(&mut self) {
        #[cfg(feature = "dim3")]
            {
                self.set_rotations_kinematic(Vector::repeat(false))
            }
        #[cfg(feature = "dim2")]
            {
                self.set_rotation_kinematic(false)
            }
    }

    /// Disable all translations of this rigid body.
    ///
    /// This is the same as setting all the translations of this rigid body as kinematic and setting
    /// its linear velocity to zero. The translations will still be controllable at the velocity level
    /// by the user afterwards.
    pub fn disable_all_translations(&mut self) {
        self.update_status.set_velocity_changed(true);
        self.set_translations_kinematic(Vector::repeat(true));
        self.velocity.linear = Vector::zeros();
    }

    /// Enable all translations for this rigid body.
    ///
    /// This is the same as setting all the translations of this rigid body as non-kinematic.
    pub fn enable_all_translations(&mut self) {
        self.set_translations_kinematic(Vector::repeat(false))
    }

    /// The handle of this rigid body.
    #[inline]
    pub fn handle(&self) -> BodyHandle {
        self.handle
    }

    /// The part-handle of this rigid body.
    ///
    /// The part id is set to 0 though any value is acceptable.
    #[inline]
    pub fn part_handle(&self) -> BodyPartHandle {
        BodyPartHandle(self.handle, 0)
    }

    /// Mutable information regarding activation and deactivation (sleeping) of this rigid body.
    #[inline]
    pub fn activation_status_mut(&mut self) -> &mut ActivationStatus<N> {
        &mut self.activation
    }

    /// Set the center of mass of this rigid body, expressed in its local space.
    #[inline]
    pub fn set_local_center_of_mass(&mut self, local_com: Point<N>) {
        self.update_status.set_local_com_changed(true);
        self.local_com = local_com;
    }

    fn update_inertia_from_local_inertia(&mut self) {
        // Needed for 2D because the inertia is not updated on the `update_dynamics`.
        self.inertia = self.local_inertia.transformed(&self.position);
        self.augmented_mass = self.inertia;
        self.inv_augmented_mass = self.inertia.inverse();
    }

    /// Set the local inertia of this rigid body, expressed in its local space.
    #[inline]
    pub fn set_local_inertia(&mut self, local_inertia: Inertia<N>) {
        self.update_status.set_local_inertia_changed(true);
        self.local_inertia = local_inertia;
        self.update_inertia_from_local_inertia();
    }

    /// Set the mass of this rigid body.
    #[inline]
    pub fn set_mass(&mut self, mass: N) {
        self.update_status.set_local_inertia_changed(true);
        self.local_inertia.linear = mass;
        self.update_inertia_from_local_inertia();
    }

    /// Set the angular inertia of this rigid body, expressed in its local space.
    #[inline]
    #[cfg(feature = "dim2")]
    pub fn set_angular_inertia(&mut self, angular_inertia: N) {
        self.update_status.set_local_inertia_changed(true);
        self.local_inertia.angular = angular_inertia;
        self.update_inertia_from_local_inertia();
    }

    /// Set the angular inertia of this rigid body, expressed in its local space.
    #[inline]
    #[cfg(feature = "dim3")]
    pub fn set_angular_inertia(&mut self, angular_inertia: na::Matrix3<N>) {
        self.update_status.set_local_inertia_changed(true);
        self.local_inertia.angular = angular_inertia;
        self.update_inertia_from_local_inertia();
    }

    /// Sets the position of this rigid body.
    #[inline]
    pub fn set_position(&mut self, pos: Isometry<N>) {
        self.update_status.set_position_changed(true);
        self.position = pos;
        self.com = pos * self.local_com;
    }

    /// Set the velocity of this rigid body.
    #[inline]
    pub fn set_velocity(&mut self, vel: Velocity<N>) {
        self.update_status.set_velocity_changed(true);
        self.velocity = vel;
    }

    /// Set the linear velocity of this rigid body.
    #[inline]
    pub fn set_linear_velocity(&mut self, vel: Vector<N>) {
        self.update_status.set_velocity_changed(true);
        self.velocity.linear = vel;
    }

    #[cfg(feature = "dim2")]
    /// Set the angular velocity of this rigid body.
    #[inline]
    pub fn set_angular_velocity(&mut self, vel: N) {
        self.update_status.set_velocity_changed(true);
        self.velocity.angular = vel;
    }

    #[cfg(feature = "dim3")]
    /// Set the angular velocity of this rigid body.
    #[inline]
    pub fn set_angular_velocity(&mut self, vel: AngularVector<N>) {
        self.update_status.set_velocity_changed(true);
        self.velocity.angular = vel;
    }

    /// The augmented mass (inluding gyroscropic terms) in world-space of this rigid body.
    #[inline]
    pub fn augmented_mass(&self) -> &Inertia<N> {
        &self.augmented_mass
    }

    /// The inverse augmented mass (inluding gyroscropic terms) in world-space of this rigid body.
    #[inline]
    pub fn inv_augmented_mass(&self) -> &Inertia<N> {
        &self.inv_augmented_mass
    }

    /// The position of this rigid body.
    #[inline]
    pub fn position(&self) -> &Isometry<N> {
        &self.position
    }

    /// The velocity of this rigid body.
    #[inline]
    pub fn velocity(&self) -> &Velocity<N> {
        &self.velocity
    }

    #[inline]
    fn apply_displacement(&mut self, displacement: &Velocity<N>) {
        let rotation = Rotation::new(displacement.angular);
        let translation = Translation::from(displacement.linear);
        let shift = Translation::from(self.com.coords);
        let disp = translation * shift * rotation * shift.inverse();
        let new_pos = disp * self.position;
        self.set_position(new_pos);
    }
}


impl<N: RealField> Body<N> for RigidBody<N> {
    #[inline]
    fn name(&self) -> &str {
        &self.name
    }

    #[inline]
    fn set_name(&mut self, name: String) {
        self.name = name
    }

    #[inline]
    fn activation_status(&self) -> &ActivationStatus<N> {
        &self.activation
    }

    #[inline]
    fn activate_with_energy(&mut self, energy: N) {
        self.activation.set_energy(energy)
    }

    #[inline]
    fn deactivate(&mut self) {
        self.update_status.clear();
        self.activation.set_energy(N::zero());
        self.velocity = Velocity::zero();
    }

    #[inline]
    fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.activation.set_deactivation_threshold(threshold)
    }

    #[inline]
    fn update_status(&self) -> BodyUpdateStatus {
        self.update_status
    }

    #[inline]
    fn status(&self) -> BodyStatus {
        self.status
    }

    #[inline]
    fn set_status(&mut self, status: BodyStatus) {
        if status != self.status {
            self.update_status.set_status_changed(true);
        }
        self.status = status
    }

    #[inline]
    fn deformed_positions(&self) -> Option<(DeformationsType, &[N])> {
        None
    }

    #[inline]
    fn deformed_positions_mut(&mut self) -> Option<(DeformationsType, &mut [N])> {
        None
    }

    #[inline]
    fn companion_id(&self) -> usize {
        self.companion_id
    }

    #[inline]
    fn set_companion_id(&mut self, id: usize) {
        self.companion_id = id
    }

    #[inline]
    fn handle(&self) -> BodyHandle {
        self.handle
    }

    #[inline]
    fn ndofs(&self) -> usize {
        SPATIAL_DIM
    }

    #[inline]
    fn generalized_velocity(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(self.velocity.as_slice(), SPATIAL_DIM)
    }

    #[inline]
    fn generalized_velocity_mut(&mut self) -> DVectorSliceMut<N> {
        self.update_status.set_velocity_changed(true);
        DVectorSliceMut::from_slice(self.velocity.as_mut_slice(), SPATIAL_DIM)
    }

    #[inline]
    fn generalized_acceleration(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(self.acceleration.as_slice(), SPATIAL_DIM)
    }

    #[inline]
    fn integrate(&mut self, params: &IntegrationParameters<N>) {
        let disp = self.velocity * params.dt;
        self.apply_displacement(&disp);
    }

    fn clear_forces(&mut self) {
        self.external_forces = Force::zero();
    }

    fn clear_update_flags(&mut self) {
        self.update_status.clear();
    }

    fn update_kinematics(&mut self) {
    }

    #[allow(unused_variables)] // for params used only in 3D.
    fn update_dynamics(&mut self, dt: N) {
        if !self.update_status.inertia_needs_update() || self.status != BodyStatus::Dynamic {
            return;
        }

        if !self.is_active() {
            self.activate();
        }

        match self.status {
            #[cfg(feature = "dim3")]
            BodyStatus::Dynamic => {
                // The inverse inertia matrix is constant in 2D.
                self.inertia = self.local_inertia.transformed(&self.position);
                self.augmented_mass = self.inertia;

                let i = &self.inertia.angular;
                let w = &self.velocity.angular;
                let iw = i * w;
                let w_dt = w * dt;
                let w_dt_cross = w_dt.gcross_matrix();
                let iw_dt_cross = (iw * dt).gcross_matrix();
                self.augmented_mass.angular += w_dt_cross * i - iw_dt_cross;

                // NOTE: if we did not have the gyroscopic forces, we would not have to invert the inertia
                // matrix at each time-step => add a flag to disable gyroscopic forces?
                self.inv_augmented_mass = self.augmented_mass.inverse();
            }
            _ => {}
        }
    }

    fn update_acceleration(&mut self, gravity: &Vector<N>, _: &IntegrationParameters<N>) {
        self.acceleration = Velocity::zero();

        match self.status {
            BodyStatus::Dynamic => {
                // The inverse inertia matrix is constant in 2D.
                #[cfg(feature = "dim3")]
                    {
                        /*
                         * Compute acceleration due to gyroscopic forces.
                         */
                        let i = &self.inertia.angular;
                        let w = &self.velocity.angular;
                        let iw = i * w;
                        let gyroscopic = -w.cross(&iw);
                        self.acceleration.angular = self.inv_augmented_mass.angular * gyroscopic;
                    }

                if self.inv_augmented_mass.linear != N::zero() && self.gravity_enabled {
                    self.acceleration.linear = *gravity;
                }

                self.acceleration += self.inv_augmented_mass * self.external_forces;
                self.acceleration.as_vector_mut().component_mul_assign(&self.jacobian_mask);
            }
            _ => {}
        }
    }

    #[inline]
    fn part(&self, _: usize) -> Option<&BodyPart<N>> {
        Some(self)
    }

    #[inline]
    fn apply_displacement(&mut self, displacement: &[N]) {
        self.apply_displacement(&Velocity::from_slice(displacement));
    }

    #[inline]
    fn world_point_at_material_point(&self, _: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        self.position * point
    }

    #[inline]
    fn position_at_material_point(&self, _: &BodyPart<N>, point: &Point<N>) -> Isometry<N> {
        self.position * Translation::from(point.coords)
    }

    #[inline]
    fn material_point_at_world_point(&self, _: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        self.position.inverse_transform_point(point)
    }

    #[inline]
    fn gravity_enabled(&self) -> bool {
        self.gravity_enabled
    }

    #[inline]
    fn enable_gravity(&mut self, enabled: bool) {
        self.gravity_enabled = enabled
    }

    #[inline]
    fn fill_constraint_geometry(
        &self,
        _: &BodyPart<N>,
        _: usize,
        point: &Point<N>,
        force_dir: &ForceDirection<N>,
        j_id: usize,
        wj_id: usize,
        jacobians: &mut [N],
        inv_r: &mut N,
        ext_vels: Option<&DVectorSlice<N>>,
        out_vel: Option<&mut N>
    ) {
        let pos = point - self.com.coords;
        let force = force_dir.at_point(&pos);
        let mut masked_force = force.clone();
        masked_force.as_vector_mut().component_mul_assign(&self.jacobian_mask);

        match self.status {
            BodyStatus::Kinematic => {
                if let Some(out_vel) = out_vel {
                    // Don't use the masked force here so the locked
                    // DOF remain controllable at the velocity level.
                    *out_vel += force.as_vector().dot(&self.velocity.as_vector());
                }
            },
            BodyStatus::Dynamic => {
                jacobians[j_id..j_id + SPATIAL_DIM].copy_from_slice(masked_force.as_slice());

                let inv_mass = self.inv_augmented_mass();
                let imf = *inv_mass * masked_force;
                jacobians[wj_id..wj_id + SPATIAL_DIM].copy_from_slice(imf.as_slice());

                *inv_r += inv_mass.mass() + masked_force.angular_vector().dot(&imf.angular_vector());

                if let Some(out_vel) = out_vel {
                    // Don't use the masked force here so the locked
                    // DOF remain controllable at the velocity level.
                    *out_vel += force.as_vector().dot(&self.velocity.as_vector());

                    if let Some(ext_vels) = ext_vels {
                        *out_vel += masked_force.as_vector().dot(ext_vels)
                    }
                }
            },
            BodyStatus::Static | BodyStatus::Disabled => {},
        }
    }

    #[inline]
    fn has_active_internal_constraints(&mut self) -> bool {
        false
    }

    #[inline]
    fn setup_internal_velocity_constraints(&mut self, _: &DVectorSlice<N>, _: &IntegrationParameters<N>) {}

    #[inline]
    fn warmstart_internal_velocity_constraints(&mut self,  _: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_velocity_constraints(&mut self, _: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, _: &IntegrationParameters<N>) {}

    #[inline]
    fn add_local_inertia_and_com(&mut self, _: usize, com: Point<N>, inertia: Inertia<N>) {
        self.update_status.set_local_com_changed(true);
        self.update_status.set_local_inertia_changed(true);

        // Update center of mass.
        if !inertia.linear.is_zero() {
            let mass_sum = self.inertia.linear + inertia.linear;
            self.local_com = (self.local_com * self.inertia.linear + com.coords * inertia.linear) / mass_sum;
            self.com = self.position * self.local_com;
        }

        // Update local inertia.
        self.local_inertia += inertia;
        self.update_inertia_from_local_inertia();
    }

    /*
     * Application of forces/impulses.
     */
    fn apply_force(&mut self, _: usize, force: &Force<N>, force_type: ForceType, auto_wake_up: bool) {
        if self.status != BodyStatus::Dynamic {
            return;
        }

        if auto_wake_up {
            self.activate();
        }

        match force_type {
            ForceType::Force => {
                self.external_forces.as_vector_mut().cmpy(N::one(), force.as_vector(), &self.jacobian_mask, N::one())
            }
            ForceType::Impulse => {
                self.update_status.set_velocity_changed(true);
                let dvel = self.inv_augmented_mass * *force;
                self.velocity.as_vector_mut().cmpy(N::one(), dvel.as_vector(), &self.jacobian_mask, N::one())
            }
            ForceType::AccelerationChange => {
                let change = self.augmented_mass * *force;
                self.external_forces.as_vector_mut().cmpy(N::one(), change.as_vector(), &self.jacobian_mask, N::one())
            }
            ForceType::VelocityChange => {
                self.update_status.set_velocity_changed(true);
                self.velocity.as_vector_mut().cmpy(N::one(), force.as_vector(), &self.jacobian_mask, N::one())
            }
        }
    }

    fn apply_local_force(&mut self, _: usize, force: &Force<N>, force_type: ForceType, auto_wake_up: bool) {
        let world_force = force.transform_by(&self.position);
        self.apply_force(0, &world_force, force_type, auto_wake_up)
    }

    fn apply_force_at_point(&mut self, _: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        let force = Force::linear_at_point(*force, &(point - self.com.coords));
        self.apply_force(0, &force, force_type, auto_wake_up)
    }

    fn apply_local_force_at_point(&mut self, _: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        self.apply_force_at_point(0, &(self.position * force), point, force_type, auto_wake_up)
    }

    fn apply_force_at_local_point(&mut self, _: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        self.apply_force_at_point(0, force, &(self.position * point), force_type, auto_wake_up)
    }

    fn apply_local_force_at_local_point(&mut self, _: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        self.apply_force_at_point(0, &(self.position * force), &(self.position * point), force_type, auto_wake_up)
    }
}


impl<N: RealField> BodyPart<N> for RigidBody<N> {
    #[inline]
    fn is_ground(&self) -> bool {
        false
    }

    #[inline]
    fn part_handle(&self) -> BodyPartHandle {
        BodyPartHandle(self.handle, 0)
    }

    #[inline]
    fn velocity(&self) -> Velocity<N> {
        self.velocity
    }

    #[inline]
    fn position(&self) -> Isometry<N> {
        self.position
    }

    #[inline]
    fn local_inertia(&self) -> Inertia<N> {
        self.local_inertia
    }

    #[inline]
    fn inertia(&self) -> Inertia<N> {
        self.inertia
    }

    #[inline]
    fn center_of_mass(&self) -> Point<N> {
        self.com
    }
}


/// The description of a rigid body, used to build a new `RigidBody`.
///
/// This is the structure to use in order to create and add a rigid body
/// (as well as some attached colliders) to the `World`. It follows
/// the builder pattern and defines three kinds of methods:
///
/// * Methods with the `.with_` prefix: sets a property of `self` and returns `Self` itself.
/// * Methods with the `.set_`prefix: sets a property of `&mut self` and retuns the `&mut self` pointer.
/// * The `build` method: actually build the rigid body into the given `World` and returns a mutable reference to the newly created rigid body.
///   The `build` methods takes `self` by-ref so the same `RigidBodyDesc` can be re-used (possibly modified) to build other rigid bodies.
///
/// The `.with_` methods as well as the `.set_` method are designed to support chaining.
/// Because the `.with_` methods takes `self` by-move, it is useful to use when initializing the
/// `RigidBodyDesc` for the first time. The `.set_` methods are useful when modifying it after
/// this initialization (including after calls to `.build`).
#[derive(Clone)]
pub struct RigidBodyDesc<'a, N: RealField> {
    name: String,
    user_data: Option<UserDataBox>,
    gravity_enabled: bool,
    position: Isometry<N>,
    velocity: Velocity<N>,
    local_inertia: Inertia<N>,
    local_center_of_mass: Point<N>,
    status: BodyStatus,
    colliders: Vec<&'a ColliderDesc<N>>,
    sleep_threshold: Option<N>,
    kinematic_translations: Vector<bool>,
    #[cfg(feature = "dim3")]
    kinematic_rotations: Vector<bool>,
    #[cfg(feature = "dim2")]
    kinematic_rotation: bool,
}

impl<'a, N: RealField> RigidBodyDesc<'a, N> {
    /// A default rigid body builder.
    pub fn new() -> RigidBodyDesc<'a, N> {
        RigidBodyDesc {
            name: String::new(),
            user_data: None,
            gravity_enabled: true,
            position: Isometry::identity(),
            velocity: Velocity::zero(),
            local_inertia: Inertia::zero(),
            local_center_of_mass: Point::origin(),
            status: BodyStatus::Dynamic,
            colliders: Vec::new(),
            sleep_threshold: Some(ActivationStatus::default_threshold()),
            kinematic_translations: Vector::repeat(false),
            #[cfg(feature = "dim3")]
            kinematic_rotations: Vector::repeat(false),
            #[cfg(feature = "dim2")]
            kinematic_rotation: false
        }
    }

    user_data_desc_accessors!();

    #[cfg(feature = "dim3")]
    desc_custom_setters!(
        self.rotation, set_rotation, axisangle: Vector<N> | { self.position.rotation = Rotation::new(axisangle) }
        self.kinematic_rotations, set_rotations_kinematic, kinematic_rotations: Vector<bool> | { self.kinematic_rotations = kinematic_rotations }
        self.angular_inertia, set_angular_inertia, angular_inertia: na::Matrix3<N> | { self.local_inertia.angular = angular_inertia }
    );

    #[cfg(feature = "dim2")]
    desc_custom_setters!(
        self.rotation, set_rotation, angle: N | { self.position.rotation = Rotation::new(angle) }
        self.kinematic_rotation, set_rotation_kinematic, is_kinematic: bool | { self.kinematic_rotation = is_kinematic }
        self.angular_inertia, set_angular_inertia, angular_inertia: N | { self.local_inertia.angular = angular_inertia }
    );

    desc_custom_setters!(
        self.translation, set_translation, vector: Vector<N> | { self.position.translation.vector = vector }
        self.mass, set_mass, mass: N | { self.local_inertia.linear = mass }
        self.collider, add_collider, collider: &'a ColliderDesc<N> | { self.colliders.push(collider) }
    );

    desc_setters!(
        gravity_enabled, enable_gravity, gravity_enabled: bool
        status, set_status, status: BodyStatus
        name, set_name, name: String
        position, set_position, position: Isometry<N>
        velocity, set_velocity, velocity: Velocity<N>
        local_inertia, set_local_inertia, local_inertia: Inertia<N>
        local_center_of_mass, set_local_center_of_mass, local_center_of_mass: Point<N>
        sleep_threshold, set_sleep_threshold, sleep_threshold: Option<N>
        kinematic_translations, set_translations_kinematic, kinematic_translations: Vector<bool>
    );

    #[cfg(feature = "dim3")]
    desc_custom_getters!(
        self.get_rotation: Vector<N> | { self.position.rotation.scaled_axis() }
        self.get_kinematic_rotations: Vector<bool> | { self.kinematic_rotations }
        self.get_angular_inertia: &na::Matrix3<N> | { &self.local_inertia.angular }
    );

    #[cfg(feature = "dim2")]
    desc_custom_getters!(
        self.get_rotation: N | { self.position.rotation.angle() }
        self.get_kinematic_rotation: bool | { self.kinematic_rotation }
        self.get_angular_inertia: N | { self.local_inertia.angular }
    );

    desc_custom_getters!(
        self.get_translation: &Vector<N> | { &self.position.translation.vector }
        self.get_mass: N | { self.local_inertia.linear }
        self.get_name: &str | { &self.name }
        self.get_colliders: &[&'a ColliderDesc<N>] | { &self.colliders[..] }
    );

    desc_getters!(
        [val] is_gravity_enabled -> gravity_enabled: bool
        [val] get_status -> status: BodyStatus
        [val] get_sleep_threshold -> sleep_threshold: Option<N>
        [ref] get_position -> position: Isometry<N>
        [ref] get_velocity -> velocity: Velocity<N>
        [ref] get_local_inertia -> local_inertia: Inertia<N>
        [ref] get_local_center_of_mass -> local_center_of_mass: Point<N>
    );

    /// Builds a rigid body and all its attached colliders.
    pub fn build<'w>(&mut self, world: &'w mut World<N>) -> &'w mut RigidBody<N> {
        world.add_body(self)
    }
}

impl<'a, N: RealField> BodyDesc<N> for RigidBodyDesc<'a, N> {
    type Body = RigidBody<N>;

    fn build_with_handle(&self, cworld: &mut ColliderWorld<N>, handle: BodyHandle) -> RigidBody<N> {
        let mut rb = RigidBody::new(handle, self.position);
        rb.set_velocity(self.velocity);
        rb.set_local_inertia(self.local_inertia);
        rb.set_local_center_of_mass(self.local_center_of_mass);
        rb.set_status(self.status);
        rb.set_deactivation_threshold(self.sleep_threshold);
        rb.set_translations_kinematic(self.kinematic_translations);
        rb.enable_gravity(self.gravity_enabled);
        rb.set_name(self.name.clone());
        let _ = rb.set_user_data(self.user_data.as_ref().map(|data| data.0.to_any()));

        #[cfg(feature = "dim3")]
            {
                rb.set_rotations_kinematic(self.kinematic_rotations);
            }
        #[cfg(feature = "dim2")]
            {
                rb.set_rotation_kinematic(self.kinematic_rotation);
            }

        for desc in &self.colliders {
            let part_handle = rb.part_handle();
            let _ = desc.build_with_infos(part_handle, &mut rb, cworld);
        }

        rb
    }
}