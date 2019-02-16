#![allow(missing_docs)] // For downcast.

use downcast_rs::Downcast;

use na::{self, DVectorSlice, DVectorSliceMut, Real};
use ncollide::shape::DeformationsType;

use crate::math::{Force, ForceType, Inertia, Isometry, Point, Vector, Velocity};
use crate::object::{BodyPartHandle, BodyHandle};
use crate::solver::{IntegrationParameters, ForceDirection};

/// The status of a body.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub enum BodyStatus {
    /// The body is disabled and ignored by the physics engine.
    Disabled,
    /// The body is static and thus cannot move.
    Static,
    /// The body is dynamic and thus can move and is subject to forces.
    Dynamic,
    /// The body is kinematic so its velocity is controlled by the user and it is not affected by forces and constraints.
    Kinematic,
}

impl Default for BodyStatus {
    fn default() -> Self {
        BodyStatus::Dynamic
    }
}

/// The activation status of a body.
///
/// This controls whether a body is sleeping or not.
#[derive(Copy, Clone, Debug)]
pub struct ActivationStatus<N: Real> {
    threshold: Option<N>,
    energy: N,
}

impl<N: Real> ActivationStatus<N> {
    /// The default amount of energy bellow which a body can be put to sleep by nphysics.
    pub fn default_threshold() -> N {
        na::convert(0.01f64)
    }

    /// Create a new activation status initialised with the default activation threshold and is active.
    pub fn new_active() -> Self {
        ActivationStatus {
            threshold: Some(Self::default_threshold()),
            energy: Self::default_threshold() * na::convert(4.0),
        }
    }

    /// Create a new activation status initialised with the default activation threshold and is inactive.
    pub fn new_inactive() -> Self {
        ActivationStatus {
            threshold: Some(Self::default_threshold()),
            energy: N::zero(),
        }
    }

    /// Retuns `true` if the body is not asleep.
    #[inline]
    pub fn is_active(&self) -> bool {
        !self.energy.is_zero()
    }

    /// The threshold bellow which the body can be put to sleep.
    ///
    /// A value of `None` indicates that the body cannot sleep.
    #[inline]
    pub fn deactivation_threshold(&self) -> Option<N> {
        self.threshold
    }

    /// Set the threshold bellow which the body can be put to sleep.
    ///
    /// A value of `None` prevents the body from sleeping.
    #[inline]
    pub fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.threshold = threshold
    }

    /// The current energy averaged through several frames.
    #[inline]
    pub fn energy(&self) -> N {
        self.energy
    }

    /// Sets the current average energy of the body.
    #[inline]
    pub fn set_energy(&mut self, energy: N) {
        self.energy = energy
    }
}

/// Trait implemented by all bodies supported by nphysics.
pub trait Body<N: Real>: Downcast + Send + Sync {
    /// The name of this body.
    fn name(&self) -> &str;

    /// Sets the name of this body.
    fn set_name(&mut self, name: String);

    /// Returns `true` if this body is the ground.
    fn is_ground(&self) -> bool {
        false
    }

    /// Update whether this body needs to be waken up after a user-interaction.
    fn update_activation_status(&mut self) {
        if self.update_status().body_needs_wake_up() {
            self.activate()
        }
    }

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

    /// The flags tracking what modifications were applied to a body.
    fn update_status(&self) -> BodyUpdateStatus;

    /// Applies a generalized displacement to this body.
    fn apply_displacement(&mut self, disp: &[N]);

    /// The handle of this body.
    fn handle(&self) -> BodyHandle;

    /// The status of this body.
    fn status(&self) -> BodyStatus;

    /// Set the status of this body.
    fn set_status(&mut self, status: BodyStatus);

    /// Information regarding activation and deactivation (sleeping) of this body.
    fn activation_status(&self) -> &ActivationStatus<N>;

    /// Sets the energy bellow which this body is put to sleep.
    ///
    /// If set to `None` the body will never sleep.
    fn set_deactivation_threshold(&mut self, threshold: Option<N>);

    /// The number of degrees of freedom of this body.
    fn ndofs(&self) -> usize;

    /// The generalized accelerations at each degree of freedom of this body.
    fn generalized_acceleration(&self) -> DVectorSlice<N>;

    /// The generalized velocities of this body.
    fn generalized_velocity(&self) -> DVectorSlice<N>;

    /// The companion ID of this body.
    fn companion_id(&self) -> usize;

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

    /// A reference to the specified body part.
    fn part(&self, i: usize) -> Option<&BodyPart<N>>;

    /// If this is a deformable body, returns its deformed positions.
    fn deformed_positions(&self) -> Option<(DeformationsType, &[N])>;

    /// If this is a deformable body, returns a mutable reference to its deformed positions.
    fn deformed_positions_mut(&mut self) -> Option<(DeformationsType, &mut [N])>;

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

    /// Force the activation of this body.
    #[inline]
    fn activate(&mut self) {
        if let Some(threshold) = self.activation_status().deactivation_threshold() {
            self.activate_with_energy(threshold * na::convert(2.0));
        }
    }

    /// Whether this body is affected by gravity.
    #[inline]
    fn gravity_enabled(&self) -> bool;

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

/// Trait implemented by each part of a body supported by nphysics.
pub trait BodyPart<N: Real>: Downcast + Send + Sync {
    /// Returns `true` if this body part is the ground.
    fn is_ground(&self) -> bool {
        false
    }

    /// The handle of this body part.
    fn part_handle(&self) -> BodyPartHandle;

    /// The center of mass of this body part.
    fn center_of_mass(&self) -> Point<N>;

    /// The position of this body part wrt. the ground.
    fn position(&self) -> Isometry<N>;

    /// The velocity of this body part.
    fn velocity(&self) -> Velocity<N>;

    /// The world-space inertia of this body part.
    fn inertia(&self) -> Inertia<N>;

    /// The local-space inertia of this body part.
    fn local_inertia(&self) -> Inertia<N>;
}

impl_downcast!(Body<N> where N: Real);
impl_downcast!(BodyPart<N> where N: Real);



bitflags! {
    #[derive(Default)]
    struct BodyUpdateStatusFlags: u8 {
        const POSITION_CHANGED = 0b00000001;
        const VELOCITY_CHANGED = 0b00000010;
        const LOCAL_INERTIA_CHANGED = 0b000100;
        const LOCAL_COM_CHANGED = 0b001000;
        const DAMPING_CHANGED = 0b010000;
        const STATUS_CHANGED = 0b100000;
    }
}

macro_rules! bitflags_accessors(
    ($($get_name: ident, $set_name: ident, $variant: ident)*) => {$(
        #[inline]
        pub fn $get_name(&self) -> bool {
            self.0.contains(BodyUpdateStatusFlags::$variant)
        }

        #[inline]
        pub fn $set_name(&mut self, changed: bool) {
            self.0.set(BodyUpdateStatusFlags::$variant, changed)
        }
    )*}
);

#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq)]
pub struct BodyUpdateStatus(BodyUpdateStatusFlags);
impl BodyUpdateStatus {
    #[inline]
    pub fn all() -> Self {
        BodyUpdateStatus(BodyUpdateStatusFlags::all())
    }

    #[inline]
    pub fn empty() -> Self {
        BodyUpdateStatus(BodyUpdateStatusFlags::default())
    }

    bitflags_accessors!(
        position_changed, set_position_changed, POSITION_CHANGED
        velocity_changed, set_velocity_changed, VELOCITY_CHANGED
        local_inertia_changed, set_local_inertia_changed, LOCAL_INERTIA_CHANGED
        local_com_changed, set_local_com_changed, LOCAL_COM_CHANGED
        damping_changed, set_damping_changed, DAMPING_CHANGED
        status_changed, set_status_changed, STATUS_CHANGED
    );

    #[inline]
    pub fn inertia_needs_update(&self) -> bool {
        self.0.intersects(
            BodyUpdateStatusFlags::POSITION_CHANGED |
                BodyUpdateStatusFlags::VELOCITY_CHANGED |
                BodyUpdateStatusFlags::LOCAL_INERTIA_CHANGED |
                BodyUpdateStatusFlags::LOCAL_COM_CHANGED |
                BodyUpdateStatusFlags::DAMPING_CHANGED |
                BodyUpdateStatusFlags::STATUS_CHANGED
        )
    }

    #[inline]
    pub fn colliders_need_update(&self) -> bool {
        self.position_changed()
    }

    #[inline]
    pub fn body_needs_wake_up(&self) -> bool {
        self.0.intersects(
            BodyUpdateStatusFlags::POSITION_CHANGED |
                BodyUpdateStatusFlags::VELOCITY_CHANGED |
                BodyUpdateStatusFlags::STATUS_CHANGED
        )
    }

    #[inline]
    pub fn clear(&mut self) {
        self.0 = BodyUpdateStatusFlags::empty()
    }
}