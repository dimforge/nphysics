use std::ops::{AddAssign, SubAssign};
use std::iter;
use std::collections::{HashMap, HashSet};
use std::marker::PhantomData;
use std::any::Any;
use either::Either;

use na::{self, RealField, DVector, DVectorSlice, DVectorSliceMut, Unit, VectorSliceMutN};
#[cfg(feature = "dim3")]
use na::Vector2;
use ncollide::utils::DeterministicState;
#[cfg(feature = "dim3")]
use ncollide::procedural;
use ncollide::shape::{DeformationsType, Multiball, ShapeHandle, Ball};
#[cfg(feature = "dim3")]
use ncollide::shape::TriMesh;

use crate::object::{Body, BodyPart, BodyStatus, BodyUpdateStatus,
                    ActivationStatus, FiniteElementIndices};
use crate::solver::{IntegrationParameters, ForceDirection};
use crate::math::{Force, ForceType, Inertia, Velocity, Vector, Point, Isometry, DIM, Dim, Translation};
use crate::object::{fem_helper, DeformableColliderDesc};
use crate::volumetric::Volumetric;

use crate::utils::{UserData, UserDataBox};

/// A particle of a SPH fluid.
#[derive(Clone)]
pub struct SPHElement<N: RealField> {
    index: usize,
    phantom: PhantomData<N>,
}

impl<N: RealField> SPHElement<N> {
    fn new(index: usize) -> Self {
        Self {
            index,
            phantom: PhantomData
        }
    }
}

/// A fluid modeled with the Smoothed Particle Hydrodynamics (SPH) method.
pub struct SPHFluid<N: RealField> {
    elements: Vec<SPHElement<N>>,
    kinematic_particles: DVector<bool>,
    positions: DVector<N>, // FIXME: would it be a good idea to reuse the position vector from the multiball?
    velocities_wo_pressure: DVector<N>,
    velocities: DVector<N>,
    accelerations: DVector<N>,
    forces: DVector<N>,
    impulses: DVector<N>,
    densities: DVector<N>,
    pressures: DVector<N>,

    multiball: ShapeHandle<N>,

    stiffness_constant: N,
    h: N,
    density: N,
    particle_mass: N,
    inv_particle_mass: N,

    companion_id: usize,
    gravity_enabled: bool,
    activation: ActivationStatus<N>,
    status: BodyStatus,
    update_status: BodyUpdateStatus,

    user_data: Option<Box<dyn Any + Send + Sync>>,
}


impl<N: RealField> SPHFluid<N> {
    /// Creates a new SPH fluid with the given density, particle radius, and initial particle positions.
    pub fn new(density: N, radius: N, centers: Vec<Point<N>>) -> Self {
        let num_particles = centers.len();
        let mut positions = DVector::repeat(num_particles * DIM, N::zero());

        for (i, c) in centers.iter().enumerate() {
            for k in 0..DIM {
                positions[i * DIM + k] = c[k]
            }
        }

        let multiball = Multiball::new(radius, centers);
        let particle_mass = Ball::new(radius).mass(density);

        Self {
            elements: (0..num_particles).map(|i| SPHElement::new(i)).collect(),
            kinematic_particles: DVector::repeat(num_particles, false),
            stiffness_constant: na::convert(10.8),
            h: radius * na::convert(2.0),
            density,
            positions,
            velocities: DVector::repeat(num_particles * DIM, N::zero()),
            velocities_wo_pressure: DVector::repeat(num_particles * DIM, N::zero()),
            accelerations: DVector::repeat(num_particles * DIM, N::zero()),
            forces: DVector::repeat(num_particles * DIM, N::zero()),
            impulses: DVector::repeat(num_particles, N::zero()),
            densities: DVector::repeat(num_particles, N::zero()),
            pressures: DVector::repeat(num_particles, N::zero()),
            multiball: ShapeHandle::new_shared_mutable(multiball),
            companion_id: 0,
            gravity_enabled: true,
            activation: ActivationStatus::new_active(),
            status: BodyStatus::Dynamic,
            update_status: BodyUpdateStatus::all(),
            particle_mass,
            inv_particle_mass: N::one() / particle_mass,
            user_data: None,
        }
    }
    /// The number of particle forming this SPH fluid.
    pub fn num_particles(&self) -> usize {
        self.positions.len() / DIM
    }

    /// The collider descriptor configured for this fluid.
    pub fn particles_collider_desc(&self) -> DeformableColliderDesc<N> {
        DeformableColliderDesc::new(self.multiball.clone())
    }

    /// Restrict the specified node acceleration to always be zero so
    /// it can be controlled manually by the user at the velocity level.
    pub fn set_node_kinematic(&mut self, i: usize, is_kinematic: bool) {
        assert!(i < self.positions.len() / DIM, "Node index out of bounds.");
        self.update_status.set_status_changed(true);
        self.update_status.set_local_inertia_changed(true);
        self.kinematic_particles[i] = is_kinematic;
    }

    /// Mark all nodes as non-kinematic.
    pub fn clear_kinematic_particles(&mut self) {
        self.update_status.set_status_changed(true);
        self.update_status.set_local_inertia_changed(true);
        self.kinematic_particles.fill(false)
    }

    // Equation 5 from "SPH Fluids in Computer Graphics", Eurographics 2014
    // https://cg.informatik.uni-freiburg.de/publications/2014_EG_SPH_STAR.pdf
    fn kernel_function(q: N) -> N {
        let _2: N = na::convert(2.0);
        assert!(q >= N::zero());
        if q >= _2 {
            return N::zero();
        }

        let _1: N = N::one();
        let _3: N = na::convert(3.0);
        let _6: N = na::convert(6.0);
        let coeff = _3 / (_2 * N::pi());

        if q < _1 {
            let qq = q * q;
            coeff * (_2 / _3 - qq + _1 / _2 * qq * q)
        } else { // q < _2
            let rhs = _2 - q;
            coeff * (_1 / _6 * rhs * rhs * rhs)
        }
    }

    // Derivative of the equation 5 from "SPH Fluids in Computer Graphics", Eurographics 2014
    // https://cg.informatik.uni-freiburg.de/publications/2014_EG_SPH_STAR.pdf
    fn kernel_function_derivative(q: N) -> N {
        let _2: N = na::convert(2.0);

        if q >= _2 {
            return N::zero();
        }

        let _1: N = N::one();
        let _3: N = na::convert(3.0);
        let coeff = _3 / (_2 * N::pi());

        if q < _1 {
            coeff * (-_2 + _3 / _2 * q) * q
        } else { // q < _2
            let rhs = _2 - q;
            coeff * (-_1 / _2 * rhs * rhs)
        }
    }

    fn kernel_function2(q: N) -> N {
        let _2: N = na::convert(2.0);
        assert!(q >= N::zero());
        if q >= _2 {
            return N::zero();
        }

        let _1: N = N::one();
        let _3: N = na::convert(3.0);
        let _4: N = na::convert(4.0);
        let _6: N = na::convert(6.0);
        let _7: N = na::convert(7.0);
        let _10: N = na::convert(10.0);
        let coeff = _10 / (_7 * N::pi());

        if q < _1 {
            let qq = q * q;
            coeff * (_1 - qq * _3 / _2 * (_1 - q / _2))
        } else { // q < _2
            let rhs = _2 - q;
            coeff * (_1 / _4 * rhs * rhs * rhs)
        }
    }

    fn kernel_function_derivative2(q: N) -> N {
        let _2: N = na::convert(2.0);

        if q >= _2 {
            return N::zero();
        }

        let _1: N = N::one();
        let _3: N = na::convert(3.0);
        let _4: N = na::convert(4.0);
        let _7: N = na::convert(7.0);
        let _10: N = na::convert(10.0);
        let coeff = _10 / (_7 * N::pi());

        if q < _1 {
            coeff * (-q * _3 + q * q * _3 * _3 / _4)
        } else { // q < _2
            let rhs = _2 - q;
            coeff * (-_3 / _4 * rhs * rhs)
        }
    }

    // Equation 4 from "SPH Fluids in Computer Graphics", Eurographics 2014
    // https://cg.informatik.uni-freiburg.de/publications/2014_EG_SPH_STAR.pdf
    fn kernel(p1: &Point<N>, p2: &Point<N>, h: N) -> N {
        let dist = na::distance(p1, p2) / h;
        N::one() / h.powi(DIM as i32) * Self::kernel_function(dist)
    }


    // Gradient of equation 4 from "SPH Fluids in Computer Graphics", Eurographics 2014
    // https://cg.informatik.uni-freiburg.de/publications/2014_EG_SPH_STAR.pdf
    fn kernel_gradient(p1: &Point<N>, p2: &Point<N>, h: N) -> Vector<N> {
        if let Some((dir, dist)) = Unit::try_new_and_get(p1 - p2, N::default_epsilon()) {
            *dir * (Self::kernel_function_derivative(dist / h) / (h.powi(DIM as i32)))
        } else {
            Vector::zeros()
        }
    }


    fn update_densities_and_pressures(&mut self) {
        let multiball_ref = self.multiball.as_ref();
        let multiball = multiball_ref.downcast_ref::<Multiball<N>>().unwrap();
        let num_particles = self.elements.len();
        let search_radius = N::zero();

    }

    fn update_internal_accelerations2(&mut self, params: &IntegrationParameters<N>) {
        let multiball_ref = self.multiball.as_ref();
        let multiball = multiball_ref.downcast_ref::<Multiball<N>>().unwrap();

        let num_particles = self.elements.len();
        self.velocities_wo_pressure.copy_from(&self.velocities);
        self.velocities_wo_pressure.axpy(params.dt(), &self.accelerations, N::one());
        let mut predicted_positions = self.positions.clone();
        predicted_positions.axpy(params.dt(), &self.velocities_wo_pressure, N::one());
        let mut lambdas = DVector::zeros(num_particles);
        let mut position_changes = DVector::zeros(num_particles * DIM);

        let niters = 4;
        for _ in 0..niters {
            /*
             *
             * Compute densities.
             *
             */
            for i in 0..num_particles {
                let pi = Point::from(predicted_positions.fixed_rows::<Dim>(i * DIM).into_owned());
                let mut density = N::zero();

                for j in multiball.balls_close_to_point(&pi, self.h * na::convert(2.0)) {
                    let pj = Point::from(predicted_positions.fixed_rows::<Dim>(j * DIM).into_owned());
                    let kernel_value = Self::kernel(&pi, &pj, self.h);

                    density += self.particle_mass * kernel_value;
                }

                self.densities[i] = density;
            }

//            println!("densities: {:?}", self.densities);

            /*
             *
             * Compute lambdas.
             *
             */
            for i in 0..num_particles {
                let pi = Point::from(predicted_positions.fixed_rows::<Dim>(i * DIM).into_owned());
                let mut denominator = N::zero();
                let mut total_gradient = Vector::zeros();
                let ci = self.densities[i] / self.density - N::one();

                for j in multiball.balls_close_to_point(&pi, self.h * na::convert(2.0)) {
                    let pj = Point::from(predicted_positions.fixed_rows::<Dim>(j * DIM).into_owned());
                    let kernel_gradient = Self::kernel_gradient(&pi, &pj, self.h);
                    total_gradient += Self::kernel_gradient(&pi, &pj, self.h);

                    if i != j {
                        denominator += (-kernel_gradient * (self.particle_mass / self.density)).norm_squared();
                    }
                }

                denominator += (total_gradient * (self.particle_mass / self.density)).norm_squared();

                if !denominator.is_zero() {
                    lambdas[i] = -ci / (denominator + na::convert(20.0));
                }
            }

//            println!("lambdas: {:?}", lambdas);

            /*
             *
             * Compute position changes.
             *
             */
            for i in 0..num_particles {
                let pi = Point::from(predicted_positions.fixed_rows::<Dim>(i * DIM).into_owned());
                let mut dpos = Vector::zeros();

                for j in multiball.balls_close_to_point(&pi, self.h * na::convert(2.0)) {
                    let pj = Point::from(predicted_positions.fixed_rows::<Dim>(j * DIM).into_owned());
                    let kernel_gradient = Self::kernel_gradient(&pi, &pj, self.h);
                    dpos += kernel_gradient * ((lambdas[i] + lambdas[j]) * (self.particle_mass / self.density));
                }

                position_changes.fixed_rows_mut::<Dim>(i * DIM).copy_from(&dpos);
            }

//            println!("Position changes: {:?}", position_changes);

            /*
             *
             * Apply position changes.
             *
             */
            predicted_positions += &position_changes;
        }

        // Compute actual velocities.
        let mut velocities = (&predicted_positions - &self.positions) / params.dt();

        // Add XSPH viscosity
        let mut viscosity_velocities = DVector::zeros(num_particles * DIM);

        for i in 0..num_particles {
            let pi = Point::from(predicted_positions.fixed_rows::<Dim>(i * DIM).into_owned());
            let vi = velocities.fixed_rows::<Dim>(i * DIM);

            let mut extra_vel = Vector::zeros();

            for j in multiball.balls_close_to_point(&pi, self.h * na::convert(2.0)) {
                let pj = Point::from(predicted_positions.fixed_rows::<Dim>(j * DIM).into_owned());
                let kernel_value = Self::kernel(&pi, &pj, self.h);
                let vj = velocities.fixed_rows::<Dim>(j * DIM);

                extra_vel += (vj - vi) * kernel_value;
            }

            viscosity_velocities.fixed_rows_mut::<Dim>(i * DIM).copy_from(&extra_vel);
        }

        let c: N = na::convert(0.01);
        velocities.axpy(c, &viscosity_velocities, N::one());

        // Compute final accelerations.
        self.accelerations = (velocities - &self.velocities) / params.dt();
    }

    fn update_internal_accelerations(&mut self, params: &IntegrationParameters<N>) {
        let multiball_ref = self.multiball.as_ref();
        let multiball = multiball_ref.downcast_ref::<Multiball<N>>().unwrap();
        let num_particles = self.elements.len();

        let search_radius = N::zero();

        /*
        // Add non-pressure forces.
        for i in 0..num_particles {
            let mut viscosity = Vector::zeros();

// XXX           for j in multiball.neighbors_of(i, search_radius) {
            for j in 0..num_particles {
                let pi = Point::from(self.positions.fixed_rows::<Dim>(i * DIM).into_owned()); // &multiball.centers()[i];
                let pj = Point::from(self.positions.fixed_rows::<Dim>(j * DIM).into_owned()); // &multiball.centers()[j];
                let sqd_i = self.densities[i] * self.densities[i];
                let sqd_j = self.densities[j] * self.densities[j];
            }

            let f = viscosity / self.particle_mass;
            println!("Found pressure acceleration: {}", f);

            self.accelerations.fixed_rows_mut::<Dim>(i * DIM).add_assign(&f);
        }*/

        // Compute the predicted velocity.
        self.velocities_wo_pressure.copy_from(&self.velocities);
        self.velocities_wo_pressure.axpy(params.dt(), &self.accelerations, N::one());
        let mut pressure_forces = DVector::zeros(self.elements.len() * DIM);

        println!("loop");
        for _ in 0..5 {
            let mut density_error = N::zero();
            // Update densities and pressure taking the new velocity into account.
            for i in 0..num_particles {
                let mut density = N::zero();

// XXX           for j in multiball.neighbors_of(i, search_radius) {
                for j in 0..num_particles {
                    let shifti = self.velocities_wo_pressure.fixed_rows::<Dim>(i * DIM) * params.dt();
                    let shiftj = self.velocities_wo_pressure.fixed_rows::<Dim>(j * DIM) * params.dt();
                    let pi = Point::from(self.positions.fixed_rows::<Dim>(i * DIM).into_owned()) + shifti; // &multiball.centers()[i];
                    let pj = Point::from(self.positions.fixed_rows::<Dim>(j * DIM).into_owned()) + shiftj; // &multiball.centers()[j];
                    let kernel_value = Self::kernel(&pi, &pj, self.h);
                    density += self.particle_mass * kernel_value;
                }

//            println!("Density: {}", density);
                density_error = density_error.max((density - self.density).abs());
//                println!("Extra vel: {}", self.velocities_wo_pressure);
                let pressure = self.stiffness_constant * /*(density - self.density); */ ((density / self.density).powi(7) - N::one());

                let max_pressure: N = na::convert(50.0);
                self.densities[i] = density;
                self.pressures[i] = *na::clamp(&pressure, &-max_pressure, &max_pressure);
            }

            // Add pressure force.
            for i in 0..num_particles {
                let mut pressure_force = Vector::zeros();

// XXX           for j in multiball.neighbors_of(i, search_radius) {
                for j in 0..num_particles {
                    let shifti = self.velocities_wo_pressure.fixed_rows::<Dim>(i * DIM) * params.dt();
                    let shiftj = self.velocities_wo_pressure.fixed_rows::<Dim>(j * DIM) * params.dt();
//                    println!("shift i: {:?}, shift j: {:?}", shifti, shiftj);
                    let pi = Point::from(self.positions.fixed_rows::<Dim>(i * DIM).into_owned()); // + shifti; // &multiball.centers()[i];
                    let pj = Point::from(self.positions.fixed_rows::<Dim>(j * DIM).into_owned()); //  + shiftj; // &multiball.centers()[j];
                    let sqd_i = self.densities[i] * self.densities[i];
                    let sqd_j = self.densities[j] * self.densities[j];
                    let kernel_gradient = Self::kernel_gradient(&pi, &pj, self.h);
                    let pressure_gradient = kernel_gradient * (self.densities[i] * self.particle_mass * (self.pressures[i] / sqd_i + self.pressures[j] / sqd_j));
                    pressure_force -= pressure_gradient * (self.particle_mass / self.densities[i]);
                }

                let pressure_acc = pressure_force / self.particle_mass;
//            println!("Found pressure acceleration: {}", f);

                pressure_forces.fixed_rows_mut::<Dim>(i * DIM).add_assign(&pressure_acc);
                self.velocities_wo_pressure.fixed_rows_mut::<Dim>(i * DIM).axpy(params.dt(), &pressure_acc, N::one());
                self.accelerations.fixed_rows_mut::<Dim>(i * DIM).add_assign(&pressure_acc);
            }

//            self.velocities_wo_pressure.axpy(params.dt(), &pressure_forces, N::one());
            pressure_forces.fill(N::zero());
//            println!("Density error: {}", density_error);
        }
    }
}

impl<N: RealField> Body<N> for SPHFluid<N> {
    #[inline]
    fn gravity_enabled(&self) -> bool {
        self.gravity_enabled
    }

    #[inline]
    fn enable_gravity(&mut self, enabled: bool) {
        self.gravity_enabled = enabled
    }

    fn update_kinematics(&mut self) {
        if self.update_status.position_changed() {
            self.update_densities_and_pressures();
        }
    }

    fn update_dynamics(&mut self, _: N) {
        if self.update_status.inertia_needs_update() && self.status == BodyStatus::Dynamic && !self.is_active() {
            self.activate();
        }
    }

    fn update_acceleration(&mut self, gravity: &Vector<N>, parameters: &IntegrationParameters<N>) {
        self.accelerations.copy_from(&self.forces);

        if self.gravity_enabled {
            let gravity_acc = gravity;

            for i in 0..self.positions.len() / DIM {
                if !self.kinematic_particles[i] {
                    let mut acc = self.accelerations.fixed_rows_mut::<Dim>(i * DIM);
                    acc += gravity_acc
                }
            }
        }

        self.update_internal_accelerations2(parameters);
    }

    fn clear_forces(&mut self) {
        self.forces.fill(N::zero())
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        self.update_status.set_position_changed(true);
        let disp = DVectorSlice::from_slice(disp, self.positions.len());
        self.positions += disp;
    }

    fn status(&self) -> BodyStatus {
        self.status
    }

    fn set_status(&mut self, status: BodyStatus) {
        self.update_status.set_status_changed(true);
        self.status = status
    }

    fn clear_update_flags(&mut self) {
        self.update_status.clear()
    }

    fn update_status(&self) -> BodyUpdateStatus {
        self.update_status
    }

    fn activation_status(&self) -> &ActivationStatus<N> {
        &self.activation
    }

    fn set_deactivation_threshold(&mut self, threshold: Option<N>) {
        self.activation.set_deactivation_threshold(threshold)
    }

    fn ndofs(&self) -> usize {
        self.positions.len()
    }

    fn generalized_acceleration(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(self.accelerations.as_slice(), self.accelerations.len())
    }

    fn generalized_velocity(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(self.velocities.as_slice(), self.velocities.len())
    }

    fn companion_id(&self) -> usize {
        self.companion_id
    }

    fn set_companion_id(&mut self, id: usize) {
        self.companion_id = id
    }

    fn generalized_velocity_mut(&mut self) -> DVectorSliceMut<N> {
        self.update_status.set_velocity_changed(true);
        let len = self.velocities.len();
        DVectorSliceMut::from_slice(self.velocities.as_mut_slice(), len)
    }

    fn integrate(&mut self, parameters: &IntegrationParameters<N>) {
        self.update_status.set_position_changed(true);
        self.positions.axpy(parameters.dt(), &self.velocities, N::one())
    }

    fn activate_with_energy(&mut self, energy: N) {
        self.activation.set_energy(energy)
    }

    fn deactivate(&mut self) {
        self.update_status.clear();
        self.activation.set_energy(N::zero());
        self.velocities.fill(N::zero());
    }

    fn part(&self, id: usize) -> Option<&dyn BodyPart<N>> {
        self.elements.get(id).map(|e| e as &dyn BodyPart<N>)
    }

    fn deformed_positions(&self) -> Option<(DeformationsType, &[N])> {
        Some((DeformationsType::Vectors, self.positions.as_slice()))
    }

    fn world_point_at_material_point(&self, part: &dyn BodyPart<N>, point: &Point<N>) -> Point<N> {
        unimplemented!()
    }

    fn position_at_material_point(&self, part: &dyn BodyPart<N>, point: &Point<N>) -> Isometry<N> {
        unimplemented!()
    }

    fn material_point_at_world_point(&self, part: &dyn BodyPart<N>, point: &Point<N>) -> Point<N> {
        unimplemented!()
    }

    fn fill_constraint_geometry(
        &self,
        part: &dyn BodyPart<N>,
        _: usize, // FIXME: keep this parameter?
        center: &Point<N>,
        force_dir: &ForceDirection<N>,
        j_id: usize,
        wj_id: usize,
        jacobians: &mut [N],
        inv_r: &mut N,
        ext_vels: Option<&DVectorSlice<N>>,
        out_vel: Option<&mut N>
    ) {
        if self.status == BodyStatus::Static || self.status == BodyStatus::Disabled {
            return;
        }

        let ndofs = self.ndofs();
        let elt = part.downcast_ref::<SPHElement<N>>().expect("The provided body part must be a triangular mass-spring element");


        // Needed by the non-linear SOR-prox.
        // FIXME: should this `fill` be done by the non-linear SOR-prox itself?
        if self.status == BodyStatus::Dynamic {
            DVectorSliceMut::from_slice(&mut jacobians[j_id..], ndofs).fill(N::zero());
        }

        if let ForceDirection::Linear(dir) = force_dir {
            if self.status == BodyStatus::Dynamic && !self.kinematic_particles[elt.index] {
                VectorSliceMutN::<N, Dim>::from_slice(&mut jacobians[j_id + elt.index * DIM..]).copy_from(&dir);
            }

            if let Some(out_vel) = out_vel {
                let va = self.velocities.fixed_rows::<Dim>(elt.index * DIM);
                *out_vel += va.dot(&dir);

                if self.status == BodyStatus::Dynamic && !self.kinematic_particles[elt.index] {
                    if let Some(ext_vels) = ext_vels {
                        *out_vel += ext_vels.fixed_rows::<Dim>(elt.index * DIM).dot(&dir);
                    }
                }
            }

            if self.status == BodyStatus::Dynamic {
                for i in 0..ndofs {
                    jacobians[wj_id + i] = jacobians[j_id + i] * self.inv_particle_mass;
                }

                // NOTE: the commented code bellow is equivalent to:
                *inv_r += self.inv_particle_mass;
                // *inv_r += DVectorSlice::from_slice(&jacobians[j_id..], ndofs).dot(&DVectorSlice::from_slice(&jacobians[wj_id..], ndofs));
            }
        }
    }

    #[inline]
    fn has_active_internal_constraints(&mut self) -> bool {
        true
    }

    #[inline]
    fn setup_internal_velocity_constraints(&mut self, _: &DVectorSlice<N>, _: &IntegrationParameters<N>) {}

    #[inline]
    fn warmstart_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {
    }

    #[inline]
    fn step_solve_internal_velocity_constraints(&mut self, dvels: &mut DVectorSliceMut<N>) {
    }

    #[inline]
    fn step_solve_internal_position_constraints(&mut self, _: &IntegrationParameters<N>) {
    }


    fn apply_force_at_local_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        if self.status != BodyStatus::Dynamic {
            return;
        }

        if auto_wake_up {
            self.activate()
        }

        let element = &self.elements[part_id];

        match force_type {
            ForceType::Force => {
                if !self.kinematic_particles[part_id] {
                    self.forces.fixed_rows_mut::<Dim>(part_id * DIM).add_assign(*force);
                }
            }
            ForceType::Impulse => {
                if !self.kinematic_particles[part_id] {
                    self.velocities.fixed_rows_mut::<Dim>(part_id * DIM).add_assign(force * self.inv_particle_mass);
                }
            }
            ForceType::AccelerationChange => {
                if !self.kinematic_particles[part_id] {
                    self.forces.fixed_rows_mut::<Dim>(part_id * DIM).add_assign(force * self.particle_mass);
                }
            }
            ForceType::VelocityChange => {
                if !self.kinematic_particles[part_id] {
                    self.velocities.fixed_rows_mut::<Dim>(part_id * DIM).add_assign(force);
                }
            }
        }
    }

    fn apply_force(&mut self, part_id: usize, force: &Force<N>, force_type: ForceType, auto_wake_up: bool) {
        self.apply_force_at_local_point(part_id, &force.linear, &Point::origin(), force_type, auto_wake_up)
    }

    fn apply_local_force(&mut self, part_id: usize, force: &Force<N>, force_type: ForceType, auto_wake_up: bool) {
        // FIXME: compute an approximate rotation for the conserned element (just like the FEM bodies)?
        self.apply_force(part_id, &force, force_type, auto_wake_up);
    }

    fn apply_force_at_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        let local_point = self.material_point_at_world_point(&self.elements[part_id], point);
        self.apply_force_at_local_point(part_id, &force, &local_point, force_type, auto_wake_up)
    }

    fn apply_local_force_at_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        // FIXME: compute an approximate rotation for the conserned element (just like the FEM bodies)?
        let local_point = self.material_point_at_world_point(&self.elements[part_id], point);
        self.apply_force_at_local_point(part_id, &force, &local_point, force_type, auto_wake_up);
    }


    fn apply_local_force_at_local_point(&mut self, part_id: usize, force: &Vector<N>, point: &Point<N>, force_type: ForceType, auto_wake_up: bool) {
        // FIXME: compute an approximate rotation for the conserned element (just like the FEM bodies)?
        self.apply_force_at_local_point(part_id, &force, &point, force_type, auto_wake_up);
    }
}


impl<N: RealField> BodyPart<N> for SPHElement<N> {
    fn center_of_mass(&self) -> Point<N> {
        unimplemented!()
    }
    fn local_center_of_mass(&self) -> Point<N> {
        unimplemented!()
    }

    fn position(&self) -> Isometry<N> {
        Isometry::new(Vector::<N>::y() * na::convert::<_, N>(100.0f64), na::zero()) // XXX
    }

    fn velocity(&self) -> Velocity<N> {
        unimplemented!()
    }

    fn inertia(&self) -> Inertia<N> {
        unimplemented!()
    }

    fn local_inertia(&self) -> Inertia<N> {
        unimplemented!()
    }
}


/*

enum SPHFluidDescGeometry<'a, N: RealField> {
    Quad(usize, usize),
    Polyline(&'a Polyline<N>),
    #[cfg(feature = "dim3")]
    TriMesh(&'a TriMesh<N>),
}

/// A builder of a mass-constraint system.
pub struct SPHFluidDesc<'a, N: RealField> {
    user_data: Option<UserDataBox>,
    geom: SPHFluidDescGeometry<'a, N>,
    stiffness: Option<N>,
    sleep_threshold: Option<N>,
    //    damping_ratio: N,
    mass: N,
    plasticity: (N, N, N),
    kinematic_particles: Vec<usize>,
    status: BodyStatus,
    gravity_enabled: bool,
}


impl<'a, N: RealField> SPHFluidDesc<'a, N> {
    fn with_geometry(geom: SPHFluidDescGeometry<'a, N>) -> Self {
        SPHFluidDesc {
            user_data: None,
            gravity_enabled: true,
            geom,
            stiffness: Some(na::convert(1.0e3)),
            sleep_threshold: Some(ActivationStatus::default_threshold()),
//            damping_ratio: na::convert(0.2),
            mass: N::one(),
            plasticity: (N::zero(), N::zero(), N::zero()),
            kinematic_particles: Vec::new(),
            status: BodyStatus::Dynamic,
        }
    }

    user_data_desc_accessors!();

    /// Create a mass-constraints system form a triangle mesh.
    #[cfg(feature = "dim3")]
    pub fn from_trimesh(mesh: &'a TriMesh<N>) -> Self {
        Self::with_geometry(SPHFluidDescGeometry::TriMesh(mesh))
    }

    /// Create a mass-constraints system form a polygonal line.
    pub fn from_polyline(polyline: &'a Polyline<N>) -> Self {
        Self::with_geometry(SPHFluidDescGeometry::Polyline(polyline))
    }

    /// Create a quad-shaped body.
    pub fn quad(subdiv_x: usize, subdiv_y: usize) -> Self {
        Self::with_geometry(SPHFluidDescGeometry::Quad(subdiv_x, subdiv_y))
    }

    /// Mark all nodes as non-kinematic.
    pub fn clear_kinematic_particles(&mut self) -> &mut Self {
        self.kinematic_particles.clear();
        self
    }

    desc_custom_setters!(
        self.plasticity, set_plasticity, strain_threshold: N, creep: N, max_force: N | { self.plasticity = (strain_threshold, creep, max_force) }
        self.kinematic_particles, set_nodes_kinematic, nodes: &[usize] | { self.kinematic_particles.extend_from_slice(nodes) }
    );

    desc_setters!(
        gravity_enabled, enable_gravity, gravity_enabled: bool
        stiffness, set_stiffness, stiffness: Option<N>
        sleep_threshold, set_sleep_threshold, sleep_threshold: Option<N>
//        damping_ratio, set_damping_ratio, damping_ratio: N
        mass, set_mass, mass: N
        status, set_status, status: BodyStatus
    );

    desc_custom_getters!(
        self.get_plasticity_strain_threshold: N | { self.plasticity.0 }
        self.get_plasticity_creep: N | { self.plasticity.1 }
        self.get_plasticity_max_force: N | { self.plasticity.2 }
        self.get_kinematic_particles: &[usize] | { &self.kinematic_particles[..] }
    );

    desc_getters!(
        [val] is_gravity_enabled -> gravity_enabled: bool
        [val] get_stiffness -> stiffness: Option<N>
        [val] get_sleep_threshold -> sleep_threshold: Option<N>
//        [val] get_damping_ratio -> damping_ratio: N
        [val] get_mass -> mass: N
        [val] get_status -> status: BodyStatus
    );

    /// Builds a mass-constraint based deformable body from this description.
    pub fn build(&self) -> SPHFluid<N> {
        let mut vol = match self.geom {
            SPHFluidDescGeometry::Quad(nx, ny) => {
                let polyline = Polyline::quad(nx, ny);
                SPHFluid::from_polyline(&polyline, self.mass, self.stiffness)
            }
            SPHFluidDescGeometry::Polyline(polyline) => {
                SPHFluid::from_polyline(polyline, self.mass, self.stiffness)
            }
            #[cfg(feature = "dim3")]
            SPHFluidDescGeometry::TriMesh(trimesh) => {
                SPHFluid::from_trimesh(trimesh, self.mass, self.stiffness)
            }
        };

        vol.set_deactivation_threshold(self.sleep_threshold);
        vol.set_plasticity(self.plasticity.0, self.plasticity.1, self.plasticity.2);
        vol.enable_gravity(self.gravity_enabled);
        vol.set_status(self.status);
        let _ = vol.set_user_data(self.user_data.as_ref().map(|data| data.0.to_any()));

        for i in &self.kinematic_particles {
            vol.set_node_kinematic(*i, true)
        }

        vol
    }
}
*/