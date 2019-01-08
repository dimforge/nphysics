use na::{DVectorSlice, DVectorSliceMut, Real};

use crate::math::{Force, Inertia, Isometry, Point, Rotation, Translation, Vector, Velocity, SPATIAL_DIM};
use crate::object::{ActivationStatus, BodyPartHandle, BodyStatus, Body, BodyPart, BodyHandle,
                    ColliderHandle, ColliderDesc, ColliderData, BodyDesc};
use crate::solver::{IntegrationParameters, ForceDirection};
use crate::world::{World, ColliderWorld};
use ncollide::shape::DeformationsType;
use ncollide::utils::IsometryOps;

#[cfg(feature = "dim3")]
use crate::math::AngularVector;
#[cfg(feature = "dim3")]
use crate::utils::GeneralizedCross;


/// A rigid body.
#[derive(Debug)]
pub struct RigidBody<N: Real> {
    handle: BodyHandle,
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
    fn new(handle: BodyHandle, position: Isometry<N>) -> Self {
        let inertia = Inertia::zero();
        let com = Point::from_coordinates(position.translation.vector);

        RigidBody {
            handle,
            local_to_world: position,
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
            activation: ActivationStatus::new_active(),
            companion_id: 0,
        }
    }

    #[inline]
    pub fn handle(&self) -> BodyHandle {
        self.handle
    }

    #[inline]
    fn part_handle(&self) -> BodyPartHandle {
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
        self.local_com = local_com;
    }

    /// Set the center of mass of this rigid body, expressed in its local space.
    #[inline]
    pub fn set_local_inertia(&mut self, local_inertia: Inertia<N>) {
        self.local_inertia = local_inertia;
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
        self.velocity = vel;
    }

    /// Set the linear velocity of this rigid body.
    #[inline]
    pub fn set_linear_velocity(&mut self, vel: Vector<N>) {
        self.velocity.linear = vel;
    }

    #[cfg(feature = "dim2")]
    /// Set the angular velocity of this rigid body.
    #[inline]
    pub fn set_angular_velocity(&mut self, vel: N) {
        self.velocity.angular = vel;
    }

    #[cfg(feature = "dim3")]
    /// Set the angular velocity of this rigid body.
    #[inline]
    pub fn set_angular_velocity(&mut self, vel: AngularVector<N>) {
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
    fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.activation.set_deactivation_threshold(threshold)
    }


    #[inline]
    fn status(&self) -> BodyStatus {
        self.status
    }

    #[inline]
    fn set_status(&mut self, status: BodyStatus) {
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
        #[cfg(feature = "dim3")]
            {
                self.augmented_mass = Inertia::zero();
                self.inv_augmented_mass = Inertia::zero();
            }
        self.acceleration = Velocity::zero();
        self.external_forces = Force::zero();
    }

    fn update_kinematics(&mut self) {
    }

    #[allow(unused_variables)] // for params used only in 3D.
    fn update_dynamics(&mut self, gravity: &Vector<N>, params: &IntegrationParameters<N>) {
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

                if self.inv_augmented_mass.linear != N::zero() {
                    self.acceleration.linear += *gravity;
                }
                self.acceleration += self.inv_augmented_mass * self.external_forces
            }
            _ => {}
        }
    }

    #[inline]
    fn part(&self, _: usize) -> Option<&BodyPart<N>> {
        Some(self)
    }

    #[inline]
    fn part_mut(&mut self, _: usize) -> Option<&mut BodyPart<N>> {
        Some(self)
    }

    #[inline]
    fn apply_displacement(&mut self, displacement: &[N]) {
        self.apply_displacement(&Velocity::from_slice(displacement));
    }

    #[inline]
    fn world_point_at_material_point(&self, _: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        self.local_to_world * point
    }

    #[inline]
    fn position_at_material_point(&self, _: &BodyPart<N>, point: &Point<N>) -> Isometry<N> {
        self.local_to_world * Translation::from_vector(point.coords)
    }

    #[inline]
    fn material_point_at_world_point(&self, _: &BodyPart<N>, point: &Point<N>) -> Point<N> {
        self.local_to_world.inverse_transform_point(point)
    }

    #[inline]
    fn fill_constraint_geometry(
        &self,
        _: &BodyPart<N>,
        ndofs: usize,
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

        match self.status {
            BodyStatus::Kinematic => {
                if let Some(out_vel) = out_vel {
                    *out_vel += force.as_vector().dot(&self.velocity.as_vector());
                }
            },
            BodyStatus::Dynamic => {
                jacobians[j_id..j_id + SPATIAL_DIM].copy_from_slice(force.as_slice());

                let inv_mass = self.inv_augmented_mass();
                let imf = *inv_mass * force;
                jacobians[wj_id..wj_id + SPATIAL_DIM].copy_from_slice(imf.as_slice());

                *inv_r += inv_mass.mass() + force.angular_vector().dot(&imf.angular_vector());

                if let Some(out_vel) = out_vel {
                    *out_vel += force.as_vector().dot(&self.velocity.as_vector());

                    if let Some(ext_vels) = ext_vels {
                        *out_vel += force.as_vector().dot(ext_vels)
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
    fn setup_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {}

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, params: &IntegrationParameters<N>) {}
}

impl<N: Real> BodyPart<N> for RigidBody<N> {
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
    fn add_local_inertia(&mut self, inertia: Inertia<N>) {
        self.local_inertia += inertia;

        // Needed for 2D because the inertia is not updated on the `update_dynamics`.
        self.inertia = self.local_inertia.transformed(&self.local_to_world);
        self.inv_augmented_mass = self.inertia.inverse();
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


#[derive(Clone)]
pub struct RigidBodyDesc<'a, N: Real> {
    pub position: Isometry<N>,
    pub velocity: Velocity<N>,
    pub local_inertia: Inertia<N>,
    pub local_com: Point<N>,
    pub status: BodyStatus,
    pub colliders: Vec<&'a ColliderDesc<N>>,
    pub sleep_threshold: Option<N>
}

impl<'a, N: Real> RigidBodyDesc<'a, N> {
    desc_custom_setters!(
        self.with_translation, set_translation, vector: Vector<N> | { self.position.translation.vector = vector }
        self.with_collider, add_collider, collider: &'a ColliderDesc<N> | { self.colliders.push(collider) }
    );

    desc_setters!(
        with_status, set_status, status: BodyStatus
        with_position, set_position, position: Isometry<N>
        with_velocity, set_velocity, velocity: Velocity<N>
        with_local_inertia, set_local_inertia, local_inertia: Inertia<N>
        with_local_center_of_mass, set_local_center_of_mass, local_com: Point<N>
        with_sleep_threshold, set_sleep_threshold, sleep_threshold: Option<N>
    );

    desc_custom_getters!(
        self.translation: &Vector<N> | { &self.position.translation.vector }
        self.colliders: &[&'a ColliderDesc<N>] | { &self.colliders[..] }
    );

    desc_getters!(
        [val] status: BodyStatus
        [val] sleep_threshold: Option<N>
        [ref] position: Isometry<N>
        [ref] velocity: Velocity<N>
        [ref] local_inertia: Inertia<N>
        [ref] local_com: Point<N>
    );

    pub fn build<'w>(&mut self, world: &'w mut World<N>) -> &'w mut RigidBody<N> {
        world.add_body(self)
    }
}

impl<'a, N: Real> BodyDesc<N> for RigidBodyDesc<'a, N> {
    type Body = RigidBody<N>;

    fn build_with_handle(&self, cworld: &mut ColliderWorld<N>, handle: BodyHandle) -> RigidBody<N> {
        let mut rb = RigidBody::new(handle, self.position);
        rb.set_velocity(self.velocity);
        rb.set_local_inertia(self.local_inertia);
        rb.set_local_center_of_mass(self.local_com);
        rb.set_status(self.status);
        rb.set_deactivation_threshold(self.sleep_threshold);

        for desc in &self.colliders {
            let part_handle = rb.part_handle();
            let ndofs = rb.status_dependent_ndofs();
            let handle = desc.build_with_infos(part_handle, ndofs, &mut rb, cworld);
        }

        rb
    }
}

impl<'a, N: Real> Default for RigidBodyDesc<'a, N> {
    fn default() -> Self {
        RigidBodyDesc {
            position: Isometry::identity(),
            velocity: Velocity::zero(),
            local_inertia: Inertia::zero(),
            local_com: Point::origin(),
            status: BodyStatus::Dynamic,
            colliders: Vec::new(),
            sleep_threshold: Some(ActivationStatus::default_threshold())
        }
    }
}