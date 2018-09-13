use na::{DVectorSlice, DVectorSliceMut, Real};

use ncollide::shape::DeformationsType;
use math::{Force, Inertia, Isometry, Point, Vector, Velocity};
use object::{ActivationStatus, BodyPartHandle, BodyStatus, Body, BodyPart, BodyHandle};
use solver::IntegrationParameters;

/// A singleton representing the ground.
///
/// Most of its methods are useless but provided anyway to be
/// similar to the other bodies.
#[derive(Clone, Debug)]
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
}

impl<N: Real> Body<N> for Ground<N> {
    #[inline]
    fn update_kinematics(&mut self) {}

    #[inline]
    fn update_dynamics(&mut self, _: &Vector<N>, _: &IntegrationParameters<N>) {}

    #[inline]
    fn clear_dynamics(&mut self) {}

    #[inline]
    fn part(&self, handle: BodyPartHandle) -> &BodyPart<N> {
        self
    }

    #[inline]
    fn part_mut(&mut self, handle: BodyPartHandle) -> &mut BodyPart<N> {
        self
    }

    #[inline]
    fn contains_part(&self, handle: BodyPartHandle) -> bool {
        handle.is_ground()
    }

    #[inline]
    fn set_handle(&mut self, handle: Option<BodyHandle>) {}

    #[inline]
    fn is_ground(&self) -> bool {
        true
    }

    #[inline]
    fn apply_displacement(&mut self, _: &[N]) {}

    #[inline]
    fn handle(&self) -> Option<BodyHandle> {
        Some(BodyHandle::ground())
    }

    #[inline]
    fn status(&self) -> BodyStatus {
        BodyStatus::Static
    }

    #[inline]
    fn activation_status(&self) -> &ActivationStatus<N> {
        &self.activation
    }

    #[inline]
    fn is_active(&self) -> bool {
        false
    }

    #[inline]
    fn is_dynamic(&self) -> bool {
        false
    }

    #[inline]
    fn is_kinematic(&self) -> bool {
        false
    }

    #[inline]
    fn is_static(&self) -> bool {
        true
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
    fn ndofs(&self) -> usize {
        0
    }

    #[inline]
    fn generalized_acceleration(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(&self.data[..], 0)
    }

    #[inline]
    fn generalized_velocity(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(&self.data[..], 0)
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
    fn generalized_velocity_mut(&mut self) -> DVectorSliceMut<N> {
        DVectorSliceMut::from_slice(&mut self.data[..], 0)
    }

    #[inline]
    fn integrate(&mut self, _: &IntegrationParameters<N>) {}

    #[inline]
    fn activate(&mut self) {}

    #[inline]
    fn activate_with_energy(&mut self, _: N) {}

    #[inline]
    fn deactivate(&mut self) {}

    #[inline]
    fn body_part_jacobian_mul_force(&self, _: &BodyPart<N>, _: &Force<N>, _: &mut [N]) {}

    #[inline]
    fn inv_mass_mul_generalized_forces(&self, _: &mut [N]) {}

    #[inline]
    fn inv_mass_mul_body_part_force(&self, _: &BodyPart<N>, _: &Force<N>, _: &mut [N]) {}
}

impl<N: Real> BodyPart<N> for Ground<N> {
    #[inline]
    fn is_ground(&self) -> bool {
        true
    }

    #[inline]
    fn handle(&self) -> Option<BodyPartHandle> {
        Some(BodyPartHandle::ground())
    }

    #[inline]
    fn center_of_mass(&self) -> Point<N> {
        Point::origin()
    }

    #[inline]
    fn position(&self) -> Isometry<N> {
        Isometry::identity()
    }

    #[inline]
    fn velocity(&self) -> Velocity<N> {
        Velocity::zero()
    }

    #[inline]
    fn inertia(&self) -> Inertia<N> {
        Inertia::zero()
    }

    #[inline]
    fn local_inertia(&self) -> Inertia<N> {
        Inertia::zero()
    }

    #[inline]
    fn apply_force(&mut self, _: &Force<N>) {}
}