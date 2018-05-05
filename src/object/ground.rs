use na::{self, DVectorSlice, DVectorSliceMut, Real};

use math::{Force, Inertia, Isometry, Point, Velocity};
use object::{ActivationStatus, BodyHandle, BodyStatus};
use solver::IntegrationParameters;

/// A singleton representing the ground.
///
/// Most of its methods are useless but provided anyway to be
/// similar to the other bodies.
pub struct Ground<N: Real> {
    companion_id: usize,
    activation: ActivationStatus<N>,
    data: [N; 0],
}

impl<N: Real> Ground<N> {
    pub(crate) fn new() -> Self {
        Ground {
            companion_id: usize::max_value(),
            activation: ActivationStatus::new_inactive(),
            data: [],
        }
    }

    /// The handle of the ground.
    #[inline]
    pub fn handle(&self) -> BodyHandle {
        BodyHandle::ground()
    }

    /// An inactive status.
    #[inline]
    pub fn activation_status(&self) -> &ActivationStatus<N> {
        &self.activation
    }

    /// Does nothing.
    #[inline]
    pub fn activate(&mut self) {
        if let Some(threshold) = self.activation.deactivation_threshold() {
            self.activate_with_energy(threshold * na::convert(2.0));
        }
    }

    /// Does nothing.
    #[inline]
    pub fn activate_with_energy(&mut self, energy: N) {
        self.activation.set_energy(energy)
    }

    /// Does nothing.
    #[inline]
    pub fn deactivate(&mut self) {
        self.activation.set_energy(N::zero());
    }

    /// Returns `false`.
    #[inline]
    pub fn is_active(&self) -> bool {
        self.activation.is_active()
    }

    /// Returns `false`.
    #[inline]
    pub fn is_dynamic(&self) -> bool {
        false
    }

    /// Returns `false`.
    #[inline]
    pub fn is_kinematic(&self) -> bool {
        false
    }

    /// Returns `true`.
    #[inline]
    pub fn is_static(&self) -> bool {
        true
    }

    /// Returns `BodyStatus::Static`.
    #[inline]
    pub fn status(&self) -> BodyStatus {
        BodyStatus::Static
    }

    /// The companion ID of the ground.
    ///
    /// May change at each step of the physics engine.
    #[inline]
    pub fn companion_id(&self) -> usize {
        self.companion_id
    }

    /// Sets the companion ID of the ground.
    ///
    /// This may be reset by the physics engine at each step.
    #[inline]
    pub fn set_companion_id(&mut self, id: usize) {
        self.companion_id = id
    }

    /// Returns 0.
    #[inline]
    pub fn ndofs(&self) -> usize {
        0
    }

    /// Returns an empty slice.
    #[inline]
    pub fn generalized_velocity(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(&self.data[..], 0)
    }

    /// Returns an empty slice.
    #[inline]
    pub fn generalized_velocity_mut(&mut self) -> DVectorSliceMut<N> {
        DVectorSliceMut::from_slice(&mut self.data[..], 0)
    }

    /// Returns an empty slice.
    #[inline]
    pub fn generalized_acceleration(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(&self.data[..], 0)
    }

    /// Does nothing.
    #[inline]
    pub fn integrate(&mut self, _: &IntegrationParameters<N>) {}

    /// Returns the origin.
    #[inline]
    pub fn center_of_mass(&self) -> Point<N> {
        Point::origin()
    }

    /// Returns the identity.
    #[inline]
    pub fn position(&self) -> Isometry<N> {
        Isometry::identity()
    }

    /// Returns zero.
    #[inline]
    pub fn velocity(&self) -> Velocity<N> {
        Velocity::zero()
    }

    /// Returns zero.
    #[inline]
    pub fn inertia(&self) -> Inertia<N> {
        Inertia::zero()
    }

    /// Returns zero.
    #[inline]
    pub fn local_inertia(&self) -> Inertia<N> {
        Inertia::zero()
    }

    /// Does nothing.
    #[inline]
    pub fn body_jacobian_mul_force(&self, _: &Force<N>, _: &mut [N]) {}

    /// Does nothing.
    #[inline]
    pub fn inv_mass_mul_generalized_forces(&self, _: &mut [N]) {}

    /// Does nothing.
    #[inline]
    pub fn inv_mass_mul_force(&self, _: &Force<N>, _: &mut [N]) {}

    /// Does nothing.
    #[inline]
    pub fn apply_force(&mut self, _: &Force<N>) {}
}
