use na::{self, DVectorSlice, DVectorSliceMut, Real};

use math::{Force, Inertia, Isometry, Point, Rotation, Translation, Vector, Velocity, SPATIAL_DIM};
use object::{ActivationStatus, BodyPartHandle, BodyStatus, Body, BodyPart, BodyHandle};
use solver::IntegrationParameters;

#[cfg(feature = "dim3")]
use math::AngularVector;
#[cfg(feature = "dim3")]
use utils::GeneralizedCross;

/// A rigid body.
#[derive(Clone, Debug)]
pub struct RigidBody<N: Real> {
    handle: Option<BodyHandle>,
    local_to_world: Isometry<N>,
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
    activation: ActivationStatus<N>,
    companion_id: usize,
}

impl<N: Real> RigidBody<N> {
    /// Create a new rigid body with the specified handle and dynamic properties.
    pub fn new(
        position: Isometry<N>,
        local_inertia: Inertia<N>,
        local_com: Point<N>,
    ) -> Self {
        let inertia = local_inertia.transformed(&position);
        let com = position * local_com;

        RigidBody {
            handle: None,
            local_to_world: position,
            velocity: Velocity::zero(),
            local_inertia,
            inertia,
            local_com,
            com,
            augmented_mass: inertia,
            inv_augmented_mass: inertia.inverse(),
            external_forces: Force::zero(),
            acceleration: Velocity::zero(),
            status: BodyStatus::Dynamic,
            activation: ActivationStatus::new_active(),
            companion_id: 0,
        }
    }

    /// Mutable information regarding activation and deactivation (sleeping) of this rigid body.
    #[inline]
    pub fn activation_status_mut(&mut self) -> &mut ActivationStatus<N> {
        &mut self.activation
    }

    /// Sets the position of this rigid body.
    #[inline]
    pub fn set_position(&mut self, pos: Isometry<N>) {
        self.local_to_world = pos;
        self.com = pos * self.local_com;
    }

    /// Set the velocity of this rigid body.
    #[inline]
    pub fn set_velocity(&mut self, vel: Velocity<N>) {
        self.velocity = vel
    }

    /// Set the linear velocity of this rigid body.
    #[inline]
    pub fn set_linear_velocity(&mut self, vel: Vector<N>) {
        self.velocity.linear = vel
    }

    #[cfg(feature = "dim2")]
    /// Set the angular velocity of this rigid body.
    #[inline]
    pub fn set_angular_velocity(&mut self, vel: N) {
        self.velocity.angular = vel
    }

    #[cfg(feature = "dim3")]
    /// Set the angular velocity of this rigid body.
    #[inline]
    pub fn set_angular_velocity(&mut self, vel: AngularVector<N>) {
        self.velocity.angular = vel
    }

    /// Set the status of this body.
    #[inline]
    pub fn set_status(&mut self, status: BodyStatus) {
        self.status = status
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

    /// Update the timestep-specific dynamic information of this rigid body.
    #[allow(unused_variables)] // for params used only in 3D.
    pub fn update_dynamics(&mut self, gravity: &Vector<N>, params: &IntegrationParameters<N>) {
        match self.status {
            BodyStatus::Dynamic => {
                // The inverse inertia matrix is constant in 2D.
                #[cfg(feature = "dim3")]
                    {
                        self.inertia = self.local_inertia.transformed(&self.local_to_world);
                        self.augmented_mass += self.inertia;

                        let i = &self.inertia.angular;
                        let w = &self.velocity.angular;
                        let iw = i * w;
                        let w_dt = w * params.dt;
                        let w_dt_cross = w_dt.gcross_matrix();
                        let iw_dt_cross = (iw * params.dt).gcross_matrix();
                        self.augmented_mass.angular += w_dt_cross * i - iw_dt_cross;

                        // NOTE: if we did not have the gyroscopic forces, we would not have to invert the inertia
                        // matrix at each time-step => add a flag to disable gyroscopic forces?
                        self.inv_augmented_mass = self.augmented_mass.inverse();

                        /*
                         * Compute acceleration due to gyroscopic forces.
                         */
                        let gyroscopic = -w.cross(&iw);
                        self.acceleration.angular += self.inv_augmented_mass.angular * gyroscopic;
                    }

                self.acceleration.linear += *gravity;
                self.acceleration += self.inv_augmented_mass * self.external_forces
            }
            _ => {}
        }
    }

    #[inline]
    fn apply_displacement(&mut self, displacement: &Velocity<N>) {
        let rotation = Rotation::new(displacement.angular);
        let translation = Translation::from_vector(displacement.linear);
        let shift = Translation::from_vector(self.com.coords);
        let disp = translation * shift * rotation * shift.inverse();
        let new_pos = disp * self.local_to_world;
        self.set_position(new_pos);
    }
}


impl<N: Real> Body<N> for RigidBody<N> {
    #[inline]
    fn set_handle(&mut self, handle: Option<BodyHandle>) {
        self.handle = handle
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
        self.activation.set_energy(N::zero());
        self.velocity = Velocity::zero();
    }

    #[inline]
    fn status(&self) -> BodyStatus {
        self.status
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
    fn handle(&self) -> Option<BodyHandle> {
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

    #[inline]
    fn clear_dynamics(&mut self) {
        self.augmented_mass = Inertia::zero();
        self.acceleration = Velocity::zero();
        self.external_forces = Force::zero();
    }

    #[inline]
    fn update_kinematics(&mut self) {}

    #[inline]
    fn part(&self, _: BodyPartHandle) -> &BodyPart<N> {
        self
    }

    #[inline]
    fn part_mut(&mut self, _: BodyPartHandle) -> &mut BodyPart<N> {
        self
    }

    #[inline]
    fn contains_part(&self, handle: BodyPartHandle) -> bool {
        self.handle.is_some() && handle.body_handle == self.handle.unwrap()
    }

    #[inline]
    fn apply_displacement(&mut self, displacement: &[N]) {
        self.apply_displacement(&Velocity::from_slice(displacement))
    }

    #[inline]
    fn inv_mass_mul_generalized_forces(&self, out: &mut [N]) {
        let force = Force::from_slice(out);
        self.inv_mass_mul_body_part_force(self, &force, out)
    }

    #[inline]
    fn body_part_jacobian_mul_force(&self, _: &BodyPart<N>, force: &Force<N>, out: &mut [N]) {
        out[..SPATIAL_DIM].copy_from_slice(force.as_slice());
    }

    #[inline]
    fn inv_mass_mul_body_part_force(&self, _: &BodyPart<N>, force: &Force<N>, out: &mut [N]) {
        let acc = self.inv_augmented_mass * *force;
        out[..SPATIAL_DIM].copy_from_slice(acc.as_slice());
    }
}

impl<N: Real> BodyPart<N> for RigidBody<N> {
    #[inline]
    fn is_ground(&self) -> bool {
        false
    }

    #[inline]
    fn handle(&self) -> Option<BodyPartHandle> {
        self.handle.map(|h| BodyPartHandle::new(h, 0))
    }

    #[inline]
    fn velocity(&self) -> Velocity<N> {
        self.velocity
    }

    #[inline]
    fn position(&self) -> Isometry<N> {
        self.local_to_world
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

    #[inline]
    fn apply_force(&mut self, force: &Force<N>) {
        self.external_forces.linear += force.linear;
        self.external_forces.angular += force.angular;
    }
}