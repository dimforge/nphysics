use na::{self, DVectorSlice, DVectorSliceMut, Real};

use math::{Force, Isometry, Point, Velocity};
use object::{ActivationStatus, BodyHandle, BodyStatus};
use solver::IntegrationParameters;

pub struct Ground<N: Real> {
    companion_id: usize,
    activation: ActivationStatus<N>,
    data: [N; 0],
}

impl<N: Real> Ground<N> {
    pub fn new() -> Self {
        Ground {
            companion_id: usize::max_value(),
            activation: ActivationStatus::new_inactive(),
            data: [],
        }
    }

    #[inline]
    pub fn handle(&self) -> BodyHandle {
        BodyHandle::ground()
    }

    #[inline]
    pub fn activation_status(&self) -> &ActivationStatus<N> {
        &self.activation
    }

    #[inline]
    pub fn activate(&mut self) {
        if let Some(threshold) = self.activation.deactivation_threshold() {
            self.activate_with_energy(threshold * na::convert(2.0));
        }
    }

    #[inline]
    pub fn activate_with_energy(&mut self, energy: N) {
        self.activation.set_energy(energy)
    }

    #[inline]
    pub fn deactivate(&mut self) {
        self.activation.set_energy(N::zero());
    }

    #[inline]
    pub fn is_active(&self) -> bool {
        self.activation.is_active()
    }

    #[inline]
    pub fn is_dynamic(&self) -> bool {
        false
    }

    #[inline]
    pub fn is_kinematic(&self) -> bool {
        false
    }

    #[inline]
    pub fn is_static(&self) -> bool {
        true
    }

    #[inline]
    pub fn status(&self) -> BodyStatus {
        BodyStatus::Static
    }

    #[inline]
    pub fn companion_id(&self) -> usize {
        self.companion_id
    }

    #[inline]
    pub fn set_companion_id(&mut self, id: usize) {
        self.companion_id = id
    }

    #[inline]
    pub fn ndofs(&self) -> usize {
        0
    }

    #[inline]
    pub fn generalized_velocity(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(&self.data[..], 0)
    }

    #[inline]
    pub fn generalized_velocity_mut(&mut self) -> DVectorSliceMut<N> {
        DVectorSliceMut::from_slice(&mut self.data[..], 0)
    }

    #[inline]
    pub fn generalized_acceleration(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(&self.data[..], 0)
    }

    #[inline]
    pub fn integrate(&mut self, _: &IntegrationParameters<N>) {}

    #[inline]
    pub fn center_of_mass(&self) -> Point<N> {
        Point::origin()
    }

    #[inline]
    pub fn position(&self) -> Isometry<N> {
        Isometry::identity()
    }

    #[inline]
    pub fn velocity(&self) -> Velocity<N> {
        Velocity::zero()
    }

    #[inline]
    pub fn body_jacobian_mul_force(&self, _: &Force<N>, _: &mut [N]) {}

    #[inline]
    pub fn inv_mass_mul_generalized_forces(&self, _: &mut [N]) {}

    #[inline]
    pub fn inv_mass_mul_force(&self, _: &Force<N>, _: &mut [N]) {}

    #[inline]
    pub fn apply_force(&mut self, _: &Force<N>) {}
}
