#![allow(missing_docs)] // For downcast.

use downcast_rs::Downcast;

use na::{self, DVectorSlice, DVectorSliceMut, RealField};
use ncollide::shape::DeformationsType;

use crate::math::{Force, ForceType, Inertia, Isometry, Point, Vector, Velocity};
use crate::object::{BodyPartHandle, BodyHandle, BodyStatus, BodyUpdateStatus, ActivationStatus, BodyPart, Body};
use crate::solver::{IntegrationParameters, ForceDirection};

/// Trait implemented by all bodies supported by nphysics.
pub trait ImmutableBody<N: RealField> {
    /// Returns `true` if this body is the ground.
    fn is_ground(&self) -> bool {
        false
    }

    /// The flags tracking what modifications were applied to a body.
    fn update_status(&self) -> BodyUpdateStatus;

    /// The handle of this body.
    fn handle(&self) -> BodyHandle;

    /// The status of this body.
    fn status(&self) -> BodyStatus;

    /// Information regarding activation and deactivation (sleeping) of this body.
    fn activation_status(&self) -> &ActivationStatus<N>;


    /// The number of degrees of freedom of this body.
    fn ndofs(&self) -> usize;

    /// The generalized accelerations at each degree of freedom of this body.
    fn generalized_acceleration(&self) -> DVectorSlice<N>;

    /// The generalized velocities of this body.
    fn generalized_velocity(&self) -> DVectorSlice<N>;

    /// The companion ID of this body.
    fn companion_id(&self) -> usize;


    /// A reference to the specified body part.
    fn part(&self, i: usize) -> Option<&BodyPart<N>>;

    /// If this is a deformable body, returns its deformed positions.
    fn deformed_positions(&self) -> Option<(DeformationsType, &[N])>;

    /// Fills all the jacobians (and the jacobians multiplied by the inverse augmented mass matrix) for a
    /// constraint applying a force at the point `center` (relative to the body part's center of mass) and
    /// the direction `dir`.
    ///
    /// If the force is a torque, it is applied at the center of mass of the body part.
    fn fill_constraint_geometry(
        &self,
        part: &BodyPart<N>,
        ndofs: usize, // FIXME: keep this parameter?
        center: &Point<N>,
        dir: &ForceDirection<N>,
        j_id: usize,
        wj_id: usize,
        jacobians: &mut [N],
        inv_r: &mut N,
        ext_vels: Option<&DVectorSlice<N>>,
        out_vel: Option<&mut N>
    );

    /// Transform the given point expressed in material coordinates to world-space.
    fn world_point_at_material_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Point<N>;

    /// Transform the given point expressed in material coordinates to world-space.
    fn position_at_material_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Isometry<N>;

    /// Transform the given point expressed in material coordinates to world-space.
    fn material_point_at_world_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Point<N>;

    /// The number of degrees of freedom (DOF) of this body, taking its status into account.
    ///
    /// In particular, this returns 0 for any body with a status different than `BodyStatus::Dynamic`.
    #[inline]
    fn status_dependent_ndofs(&self) -> usize {
        if self.is_dynamic() {
            self.ndofs()
        } else {
            0
        }
    }

    /// The velocity of the specified body part, taking this body status into account.
    ///
    /// This will return a zero velocity for any body with a status different than `BodyStatus::Dynamic`.
    #[inline]
    fn status_dependent_body_part_velocity(&self, part: &BodyPart<N>) -> Velocity<N> {
        if self.is_dynamic() {
            part.velocity()
        } else {
            Velocity::zero()
        }
    }

    /// Check if this body is active.
    #[inline]
    fn is_active(&self) -> bool {
        match self.status() {
            BodyStatus::Dynamic => self.activation_status().is_active(),
            BodyStatus::Kinematic => true,
            BodyStatus::Static => false,
            BodyStatus::Disabled => false,
        }
    }

    /// Whether or not the status of this body is dynamic.
    #[inline]
    fn is_dynamic(&self) -> bool {
        self.status() == BodyStatus::Dynamic
    }

    /// Whether or not the status of this body is kinematic.
    #[inline]
    fn is_kinematic(&self) -> bool {
        self.status() == BodyStatus::Kinematic
    }

    /// Whether or not the status of this body is static.
    #[inline]
    fn is_static(&self) -> bool {
        self.status() == BodyStatus::Static
    }

    /// Whether this body is affected by gravity.
    #[inline]
    fn gravity_enabled(&self) -> bool;
}

// FIXME: simply rename this to `Body`.
pub trait MutableBody<N: RealField>: ImmutableBody<N> {
    /// Update whether this body needs to be waken up after a user-interaction.
    fn update_activation_status(&mut self) {
        if self.update_status().body_needs_wake_up() {
            self.activate()
        }
    }

    fn advance(&mut self, time_ratio: N) {
        unimplemented!()
    }
    fn validate_advancement(&mut self) { unimplemented!() }
    fn clamp_advancement(&mut self) { unimplemented!() }

    fn step_started(&mut self) { unimplemented!() }

    /// Updates the kinematics, e.g., positions and jacobians, of this body.
    fn update_kinematics(&mut self);

    /// Update the dynamics property of this body.
    fn update_dynamics(&mut self, dt: N);

    /// Update the acceleration of this body given the forces it is subject to and the gravity.
    fn update_acceleration(&mut self, gravity: &Vector<N>, params: &IntegrationParameters<N>);

    /// Reset the timestep-specific dynamic information of this body.
    fn clear_forces(&mut self);

    /// Clear all the update flags of this body.
    fn clear_update_flags(&mut self);

    /// Applies a generalized displacement to this body.
    fn apply_displacement(&mut self, disp: &[N]);

    /// Set the status of this body.
    fn set_status(&mut self, status: BodyStatus);

    /// Sets the energy bellow which this body is put to sleep.
    ///
    /// If set to `None` the body will never sleep.
    fn set_deactivation_threshold(&mut self, threshold: Option<N>);

    /// Set the companion ID of this body (may be reinitialized by nphysics).
    fn set_companion_id(&mut self, id: usize);

    /// The mutable generalized velocities of this body.
    fn generalized_velocity_mut(&mut self) -> DVectorSliceMut<N>;

    /// Integrate the position of this body.
    fn integrate(&mut self, params: &IntegrationParameters<N>);

    /// Force the activation of this body with the given level of energy.
    fn activate_with_energy(&mut self, energy: N);

    /// Put this body to sleep.
    fn deactivate(&mut self);

    /// If this is a deformable body, returns a mutable reference to its deformed positions.
    fn deformed_positions_mut(&mut self) -> Option<(DeformationsType, &mut [N])>;


    /// Returns `true` if this bodies contains internal constraints that need to be solved.
    fn has_active_internal_constraints(&mut self) -> bool;

    /// Initializes the internal velocity constraints of a body.
    fn setup_internal_velocity_constraints(&mut self, ext_vels: &DVectorSlice<N>, params: &IntegrationParameters<N>);

    /// For warmstarting the solver, modifies the delta velocity applied by the internal constraints of this body.
    fn warmstart_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>);

    /// Execute one step for the iterative resolution of this body's internal velocity constraints.
    fn step_solve_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>);

    /// Execute one step for the iterative resolution of this body's internal position constraints.
    #[inline]
    fn step_solve_internal_position_constraints(&mut self, params: &IntegrationParameters<N>);

    /// Add the given inertia to the local inertia of this body part.
    fn add_local_inertia_and_com(&mut self, _part_index: usize, _com: Point<N>, _inertia: Inertia<N>)
    {} // FIXME: don't auto-impl.


    /// Force the activation of this body.
    #[inline]
    fn activate(&mut self) {
        if let Some(threshold) = self.activation_status().deactivation_threshold() {
            self.activate_with_energy(threshold * na::convert(2.0));
        }
    }

    /// Enable or disable gravity for this body.
    #[inline]
    fn enable_gravity(&mut self, enabled: bool);

    /*
     * Application of forces/impulses.
     */
    /// Apply a force at the center of mass of a part of this body.
    fn apply_force(&mut self, part_id: usize, force: &Force<N>, force_type: ForceType, auto_wake_up: bool);

    /// Apply a local force at the center of mass of a part of this body.
    fn apply_local_force(&mut self, part_id: usize, force: &Force<N>, force_type: ForceType, auto_wake_up: bool);

    /// Apply a force at a given point of a part of this body.
    fn apply_force_at_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool);

    /// Apply a local force at a given point of a part of this body.
    fn apply_local_force_at_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool);

    /// Apply a force at a given local point of a part of this body.
    fn apply_force_at_local_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool);

    /// Apply a local force at a given local point of a part of this body.
    fn apply_local_force_at_local_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool);
}


impl<N: RealField> ImmutableBody<N> for Body<N> {
    /// Returns `true` if this body is the ground.
    fn is_ground(&self) -> bool {
        self.is_ground()
    }

    /// The flags tracking what modifications were applied to a body.
    fn update_status(&self) -> BodyUpdateStatus {
        self.update_status()
    }

    /// The handle of this body.
    fn handle(&self) -> BodyHandle {
        self.handle()
    }

    /// The status of this body.
    fn status(&self) -> BodyStatus {
        self.status()
    }

    /// Information regarding activation and deactivation (sleeping) of this body.
    fn activation_status(&self) -> &ActivationStatus<N> {
        self.activation_status()
    }


    /// The number of degrees of freedom of this body.
    fn ndofs(&self) -> usize {
        self.ndofs()
    }

    /// The generalized accelerations at each degree of freedom of this body.
    fn generalized_acceleration(&self) -> DVectorSlice<N> {
        self.generalized_acceleration()
    }

    /// The generalized velocities of this body.
    fn generalized_velocity(&self) -> DVectorSlice<N> {
        self.generalized_velocity()
    }

    /// The companion ID of this body.
    fn companion_id(&self) -> usize {
        self.companion_id()
    }


    /// A reference to the specified body part.
    fn part(&self, i: usize) -> Option<&BodyPart<N>> {
        self.part(i)
    }

    /// If this is a deformable body, returns its deformed positions.
    fn deformed_positions(&self) -> Option<(DeformationsType, &[N])> {
        self.deformed_positions()
    }

    /// Fills all the jacobians (and the jacobians multiplied by the inverse augmented mass matrix) for a
    /// constraint applying a force at the point `center` (relative to the body part's center of mass) and
    /// the direction `dir`.
    ///
    /// If the force is a torque, it is applied at the center of mass of the body part.
    fn fill_constraint_geometry(
        &self,
        part: &BodyPart<N>,
        ndofs: usize, // FIXME: keep this parameter?
        center: &Point<N>,
        dir: &ForceDirection<N>,
        j_id: usize,
        wj_id: usize,
        jacobians: &mut [N],
        inv_r: &mut N,
        ext_vels: Option<&DVectorSlice<N>>,
        out_vel: Option<&mut N>
    ) {
        self.fill_constraint_geometry(part, ndofs, center, dir, j_id, wj_id, jacobians, inv_r, ext_vels, out_vel)
    }

    /// Transform the given point expressed in material coordinates to world-space.
    fn world_point_at_material_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        self.world_point_at_material_point(part, point)
    }

    /// Transform the given point expressed in material coordinates to world-space.
    fn position_at_material_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Isometry<N> {
        self.position_at_material_point(part, point)
    }

    /// Transform the given point expressed in material coordinates to world-space.
    fn material_point_at_world_point(&self, part: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        self.material_point_at_world_point(part, point)
    }

    /// The number of degrees of freedom (DOF) of this body, taking its status into account.
    ///
    /// In particular, this returns 0 for any body with a status different than `BodyStatus::Dynamic`.
    #[inline]
    fn status_dependent_ndofs(&self) -> usize {
        self.status_dependent_ndofs()
    }

    /// The velocity of the specified body part, taking this body status into account.
    ///
    /// This will return a zero velocity for any body with a status different than `BodyStatus::Dynamic`.
    #[inline]
    fn status_dependent_body_part_velocity(&self, part: &BodyPart<N>) -> Velocity<N> {
        self.status_dependent_body_part_velocity(part)
    }

    /// Check if this body is active.
    #[inline]
    fn is_active(&self) -> bool {
        self.is_active()
    }

    /// Whether or not the status of this body is dynamic.
    #[inline]
    fn is_dynamic(&self) -> bool {
        self.is_dynamic()
    }

    /// Whether or not the status of this body is kinematic.
    #[inline]
    fn is_kinematic(&self) -> bool {
        self.is_kinematic()
    }

    /// Whether or not the status of this body is static.
    #[inline]
    fn is_static(&self) -> bool {
        self.is_static()
    }

    /// Whether this body is affected by gravity.
    #[inline]
    fn gravity_enabled(&self) -> bool {
        self.gravity_enabled()
    }
}

impl<N: RealField> MutableBody<N> for Body<N> {
    /// Update whether this body needs to be waken up after a user-interaction.
    fn update_activation_status(&mut self) {
        self.update_activation_status()
    }

    fn advance(&mut self, time_ratio: N) {
        self.advance(time_ratio)
    }
    fn validate_advancement(&mut self) {
        self.validate_advancement()
    }
    fn clamp_advancement(&mut self) {
        self.clamp_advancement()
    }

    fn step_started(&mut self) {
        self.step_started()
    }

    /// Updates the kinematics, e.g., positions and jacobians, of this body.
    fn update_kinematics(&mut self) {
        self.update_kinematics()
    }

    /// Update the dynamics property of this body.
    fn update_dynamics(&mut self, dt: N) {
        self.update_dynamics(dt)
    }

    /// Update the acceleration of this body given the forces it is subject to and the gravity.
    fn update_acceleration(&mut self, gravity: &Vector<N>, params: &IntegrationParameters<N>) {
        self.update_acceleration(gravity, params)
    }

    /// Reset the timestep-specific dynamic information of this body.
    fn clear_forces(&mut self) {
        self.clear_forces()
    }

    /// Clear all the update flags of this body.
    fn clear_update_flags(&mut self) {
        self.clear_update_flags()
    }

    /// Applies a generalized displacement to this body.
    fn apply_displacement(&mut self, disp: &[N]) {
        self.apply_displacement(disp)
    }

    /// Set the status of this body.
    fn set_status(&mut self, status: BodyStatus) {
        self.set_status(status)
    }

    /// Sets the energy bellow which this body is put to sleep.
    ///
    /// If set to `None` the body will never sleep.
    fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.set_deactivation_threshold(threshold)
    }

    /// Set the companion ID of this body (may be reinitialized by nphysics).
    fn set_companion_id(&mut self, id: usize) {
        self.set_companion_id(id)
    }

    /// The mutable generalized velocities of this body.
    fn generalized_velocity_mut(&mut self) -> DVectorSliceMut<N> {
        self.generalized_velocity_mut()
    }
    /// Integrate the position of this body.
    fn integrate(&mut self, params: &IntegrationParameters<N>)  {
        self.integrate(params)
    }

    /// Force the activation of this body with the given level of energy.
    fn activate_with_energy(&mut self, energy: N) {
        self.activate_with_energy(energy)
    }

    /// Put this body to sleep.
    fn deactivate(&mut self) {
        self.deactivate()
    }

    /// If this is a deformable body, returns a mutable reference to its deformed positions.
    fn deformed_positions_mut(&mut self) -> Option<(DeformationsType, &mut [N])> {
        self.deformed_positions_mut()
    }


    /// Returns `true` if this bodies contains internal constraints that need to be solved.
    fn has_active_internal_constraints(&mut self) -> bool {
        self.has_active_internal_constraints()
    }

    /// Initializes the internal velocity constraints of a body.
    fn setup_internal_velocity_constraints(&mut self, ext_vels: &DVectorSlice<N>, params: &IntegrationParameters<N>) {
        self.setup_internal_velocity_constraints(ext_vels, params)
    }

    /// For warmstarting the solver, modifies the delta velocity applied by the internal constraints of this body.
    fn warmstart_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {
        self.warmstart_internal_velocity_constraints(dvels)
    }

    /// Execute one step for the iterative resolution of this body's internal velocity constraints.
    fn step_solve_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {
        self.step_solve_internal_velocity_constraints(dvels)
    }

    /// Execute one step for the iterative resolution of this body's internal position constraints.
    #[inline]
    fn step_solve_internal_position_constraints(&mut self, params: &IntegrationParameters<N>) {
        self.step_solve_internal_position_constraints(params)
    }

    /// Add the given inertia to the local inertia of this body part.
    fn add_local_inertia_and_com(&mut self, part_index: usize, com: Point<N>, inertia: Inertia<N>) {
        self.add_local_inertia_and_com(part_index, com, inertia)
    }


    /// Force the activation of this body.
    #[inline]
    fn activate(&mut self) {
        self.activate()
    }

    /// Enable or disable gravity for this body.
    #[inline]
    fn enable_gravity(&mut self, enabled: bool) {
        self.enable_gravity(enabled)
    }

    /// Apply a force at the center of mass of a part of this body.
    fn apply_force(&mut self, part_id: usize, force: &Force<N>, force_type: ForceType, auto_wake_up: bool) {
        self.apply_force(part_id, force, force_type, auto_wake_up)
    }

    /// Apply a local force at the center of mass of a part of this body.
    fn apply_local_force(&mut self, part_id: usize, force: &Force<N>, force_type: ForceType, auto_wake_up: bool) {
        self.apply_local_force(part_id, force, force_type, auto_wake_up)
    }

    /// Apply a force at a given point of a part of this body.
    fn apply_force_at_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        self.apply_force_at_point(part_id, force, point, force_type, auto_wake_up)
    }

    /// Apply a local force at a given point of a part of this body.
    fn apply_local_force_at_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        self.apply_local_force_at_point(part_id, force, point, force_type, auto_wake_up)
    }

    /// Apply a force at a given local point of a part of this body.
    fn apply_force_at_local_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        self.apply_force_at_local_point(part_id, force, point, force_type, auto_wake_up)
    }

    /// Apply a local force at a given local point of a part of this body.
    fn apply_local_force_at_local_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        self.apply_local_force_at_local_point(part_id, force, point, force_type, auto_wake_up)
    }
}