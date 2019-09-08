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
use ncollide::shape::{DeformationsType, Multiball, ShapeHandle, Ball, FeatureId};
#[cfg(feature = "dim3")]
use ncollide::shape::TriMesh;
use ncollide::partitioning::HGrid;

use crate::object::{Body, BodyPart, BodySet, BodyStatus, BodyUpdateStatus, ColliderSet,
                    ActivationStatus, FiniteElementIndices, BodyPartHandle};
use crate::solver::{IntegrationParameters, ForceDirection, MechanicalSolver};
use crate::math::{Force, ForceType, Inertia, Velocity, Vector, Point, Isometry, DIM, Dim, Translation};
use crate::object::{fem_helper, DeformableColliderDesc, SPHKernel, Poly6Kernel, CubicSplineKernel, SpikyKernel, ViscosityKernel, LinearKernel, FluidBody};
use crate::volumetric::Volumetric;
use crate::joint::JointConstraintSet;
use crate::world::GeometricalWorld;
use crate::counters::Counters;
use crate::material::MaterialsCoefficientsTable;

struct ParticleContact<N: RealField> {
    i: usize,
    j: usize,
    weight: N,
    gradient: Vector<N>,
}

struct BoundaryContact<N: RealField> {
    i: usize,
    j: usize,
    weight: N,
    gradient: Vector<N>,
}


pub struct PBFSolver<N: RealField, Kernel: SPHKernel = Poly6Kernel, KernelGradient: SPHKernel = SpikyKernel, KernelLaplacian: SPHKernel = ViscosityKernel> {
    boundary: HGrid<N, usize>,
    num_particles: usize,
    particle_contacts: Vec<ParticleContact<N>>,
    boundary_contacts: Vec<BoundaryContact<N>>,
    positions: DVector<N>,
    velocities: DVector<N>,
    volumes: DVector<N>,
    boundary_positions: DVector<N>,
    boundary_volumes: DVector<N>,
    densities0: DVector<N>,
    h: N,
    phantoms: PhantomData<(Kernel, KernelGradient, KernelLaplacian)>,
}


impl<N, Kernel, KernelGradient, KernelLaplacian> PBFSolver<N, Kernel, KernelGradient, KernelLaplacian>
    where N: RealField,
          Kernel: SPHKernel,
          KernelGradient: SPHKernel,
          KernelLaplacian: SPHKernel {
    pub fn new() -> Self {
        let h = na::convert(0.4);
        Self {
            boundary: HGrid::new(h),
            num_particles: 0,
            particle_contacts: Vec::new(),
            boundary_contacts: Vec::new(),
            positions: DVector::zeros(0),
            velocities: DVector::zeros(0),
            volumes: DVector::zeros(0),
            boundary_positions: DVector::zeros(0),
            boundary_volumes: DVector::zeros(0),
            densities0: DVector::zeros(0),
            h,
            phantoms: PhantomData
        }
    }

    fn init_boundary<Bodies: BodySet<N>, Colliders: ColliderSet<N, Bodies::Handle>>(
        &mut self,
        bodies: &Bodies,
        active_bodies: &[Bodies::Handle],
        gworld: &GeometricalWorld<N, Bodies::Handle, Colliders::Handle>,
        colliders: &Colliders,
    ) {
        let mut positions = Vec::new();
        let mut num_boundary_particles = 0;

        self.boundary.clear();

        for handle in active_bodies {
            if let Some(fluid) = bodies.get_as::<FluidBody<N>>(*handle) {
                let contacts_iter = gworld
                    .body_colliders(*handle)
                    .unwrap()
                    .iter()
                    .flat_map(|coll_handle| gworld.contacts_with(colliders, *coll_handle, true).unwrap());

                for (ch1, c1, ch2, c2, _, manifold) in contacts_iter {
                    for contact in manifold.contacts() {
                        let position = if c1.body() == *handle {
                            contact.contact.world2//  + *contact.contact.normal * (self.h * na::convert(0.5))
                        } else {
                            contact.contact.world1//  - *contact.contact.normal * (self.h * na::convert(0.5))
                        };

                        self.boundary.insert(&position, num_boundary_particles); // FIXME: associate with the handle?
                        positions.extend_from_slice(position.coords.as_slice());
                        num_boundary_particles += 1;
                    }
                }
            }
        }

        println!("Num boundary particles: {}", num_boundary_particles);
        self.boundary_positions = DVector::from_vec(positions);
        self.boundary_volumes = DVector::zeros(num_boundary_particles);

        Self::compute_boundary_volumes(&self.boundary, self.h, &self.boundary_positions, &mut self.boundary_volumes);
    }

    fn compute_contacts(output: &mut Vec<ParticleContact<N>>, positions: &DVector<N>, multiball: &Multiball<N>, h: N, num_particles_i: usize, assembly_id1: usize, assembly_id2: usize) {

        for i in assembly_id1..assembly_id1 + num_particles_i {
            let pi = Point::from(positions.fixed_rows::<Dim>(i * DIM).into_owned());

            for j in multiball.balls_close_to_point(&pi, h).map(|j| assembly_id2 + j) {
                // Don't output the same contact twice.
                if i <= j {
                    let pj = Point::from(positions.fixed_rows::<Dim>(j * DIM).into_owned());
                    let weight = Kernel::points_apply(&pi, &pj, h);

                    if !weight.is_zero() {
                        let gradient = KernelGradient::points_apply_diff1(&pi, &pj, h);
                        output.push(ParticleContact {
                            weight,
                            gradient,
                            i,
                            j,
                        });
                    }
                }
            }
        }
    }

    fn compute_boundary_contacts(output: &mut Vec<BoundaryContact<N>>, positions: &DVector<N>, hgrid: &HGrid<N, usize>, boundary_positions: &DVector<N>, h: N, num_particles_i: usize, assembly_id1: usize) {
        for i in assembly_id1..assembly_id1 + num_particles_i {
            let pi = Point::from(positions.fixed_rows::<Dim>(i * DIM).into_owned());

            for j in hgrid.elements_close_to_point(&pi, h).cloned() {
                let pj = Point::from(boundary_positions.fixed_rows::<Dim>(j * DIM).into_owned());
                let weight = Kernel::points_apply(&pi, &pj, h);

                if !weight.is_zero() {
                    let gradient = KernelGradient::points_apply_diff1(&pi, &pj, h);
                    output.push(BoundaryContact {
                        weight,
                        gradient,
                        i,
                        j,
                    });
                }
            }
        }
    }


    fn update_contacts(
        contacts: &mut [ParticleContact<N>],
        boundary_contacts: &mut [BoundaryContact<N>],
        positions: &DVector<N>,
        boundary_positions: &DVector<N>,
        h: N
    ) {
        for c in contacts {
            let pi = Point::from(positions.fixed_rows::<Dim>(c.i * DIM).into_owned());
            let pj = Point::from(positions.fixed_rows::<Dim>(c.j * DIM).into_owned());

            c.weight = Kernel::points_apply(&pi, &pj, h);
            c.gradient = KernelGradient::points_apply_diff1(&pi, &pj, h);
        }

        for c in boundary_contacts {
            let pi = Point::from(positions.fixed_rows::<Dim>(c.i * DIM).into_owned());
            let pj = Point::from(boundary_positions.fixed_rows::<Dim>(c.j * DIM).into_owned());

            c.weight = Kernel::points_apply(&pi, &pj, h);
            c.gradient = KernelGradient::points_apply_diff1(&pi, &pj, h);
        }
    }

    fn init<Bodies: BodySet<N>>(&mut self, bodies: &mut Bodies, active_bodies: &[Bodies::Handle], dt: N) {
        // First compute companion indices and assembly indices.
        self.num_particles = 0;
        let mut total_fluids = 0;

        for handle in active_bodies {
            if let Some(fluid) = bodies.get_as_mut::<FluidBody<N>>(*handle) {
                fluid.set_companion_id(self.num_particles);
                self.num_particles += fluid.num_particles();
                total_fluids += 1;
            }
        }

        // Resize workspace.
        let ndofs = self.num_particles * DIM;

        if self.positions.len() < ndofs {
            self.densities0 = DVector::zeros(self.num_particles);
            self.volumes = DVector::zeros(self.num_particles);
            self.positions = DVector::zeros(ndofs);
            self.velocities = DVector::zeros(ndofs);
        }

        // Copy data from the fluid bodies to our workspace.
        for handle in active_bodies {
            if let Some(fluid) = bodies.get_as_mut::<FluidBody<N>>(*handle) {
                let start = fluid.companion_id() * DIM;
                let nparts = fluid.num_particles();
                let ndofs = fluid.positions().len();

                let mut velocities = self.velocities.rows_mut(start, ndofs);
                velocities.copy_from(fluid.velocities());
                velocities.axpy(dt, &fluid.generalized_acceleration(), N::one());
                fluid.clear_acceleration();

                self.positions.rows_mut(start, ndofs).copy_from(fluid.positions());
                self.densities0.rows_mut(start, nparts).fill(fluid.rest_density());
                self.volumes.rows_mut(start, nparts).fill(fluid.particle_volume());
            }
        }
    }

    fn output_results<Bodies: BodySet<N>>(&mut self, bodies: &mut Bodies, active_bodies: &[Bodies::Handle]) {
        // First compute companion indices and assembly indices.
        let mut total_particles = 0;

        // Copy data from the fluid bodies to our workspace.
        for handle in active_bodies {
            if let Some(fluid) = bodies.get_as_mut::<FluidBody<N>>(*handle) {
                let start = fluid.companion_id() * DIM;
                let ndofs = fluid.positions().len();

                fluid.positions_mut().copy_from(&self.positions.rows_mut(start, ndofs));
                fluid.velocities_mut().copy_from(&self.velocities.rows_mut(start, ndofs));
            }
        }
    }

    fn reset_contact_list<Bodies: BodySet<N>, Colliders: ColliderSet<N, Bodies::Handle>>(
        &mut self,
        gworld: &GeometricalWorld<N, Bodies::Handle, Colliders::Handle>,
        colliders: &Colliders,
        bodies: &Bodies,
        active_bodies: &[Bodies::Handle],
    ) {
        self.particle_contacts.clear();
        self.boundary_contacts.clear();

        for handle in active_bodies {
            if let Some(fluid) = bodies.get_as::<FluidBody<N>>(*handle) {
                let multiball_ref = fluid.shared_multiball().as_ref();
                let multiball = multiball_ref.downcast_ref::<Multiball<N>>().unwrap();
                let num_particles = fluid.num_particles();
                let assembly_id = fluid.companion_id();

                Self::compute_contacts(
                    &mut self.particle_contacts,
                    &self.positions,
                    multiball,
                    self.h,
                    num_particles,
                    assembly_id,
                    assembly_id,
                );

                Self::compute_boundary_contacts(
                    &mut self.boundary_contacts,
                    &self.positions,
                    &self.boundary,
                    &self.boundary_positions,
                    self.h,
                    num_particles,
                    assembly_id
                )

                // FIXME: take into account contacts with other fluids.
            }
        }
        println!("Num bcontacts: {}", self.boundary_contacts.len());
    }

    fn compute_boundary_volumes(
        boundary: &HGrid<N, usize>,
        h: N,
        boundary_positions: &DVector<N>,
        boundary_volumes: &mut DVector<N>
    ) {
        let num_boundary_particles = boundary_volumes.len();
        boundary_volumes.fill(N::zero());

        for i in 0..num_boundary_particles {
            let pi = Point::from(boundary_positions.fixed_rows::<Dim>(i * DIM).into_owned());
            let mut denominator = N::zero();

            for j in boundary.elements_close_to_point(&pi, h).cloned() {
                let pj = Point::from(boundary_positions.fixed_rows::<Dim>(j * DIM).into_owned());
                let weight = Kernel::points_apply(&pi, &pj, h);
                denominator += weight;
            }

            boundary_volumes[i] = N::one() / denominator;
        }
    }

    fn compute_densities(
        contacts: &[ParticleContact<N>],
        boundary_contacts: &[BoundaryContact<N>],
        particle_volumes: &DVector<N>,
        boundary_volumes: &DVector<N>,
        densities: &mut DVector<N>
    ) {
        densities.fill(N::zero());

        for c in contacts {
            densities[c.i] += particle_volumes[c.j] * c.weight;

            if c.i != c.j {
                densities[c.j] += particle_volumes[c.i] * c.weight;
            }
        }

        for c in boundary_contacts {
            densities[c.i] += boundary_volumes[c.j] * c.weight;
        }
    }

    fn compute_average_density_error(num_particles: usize, densities: &DVector<N>, densities0: &DVector<N>) -> N {
        let mut avg_density_err = N::zero();

        for i in 0..num_particles {
            avg_density_err += densities0[i] * (densities[i] - N::one()).max(N::zero());
        }

        avg_density_err / na::convert(num_particles as f64)
    }

    fn compute_lambdas(
        num_particles: usize,
        contacts: &[ParticleContact<N>],
        boundary_contacts: &[BoundaryContact<N>],
        densities: &DVector<N>,
        particle_volumes: &DVector<N>,
        boundary_volumes: &DVector<N>,
        lambdas: &mut DVector<N>
    ) {
        let mut total_gradient: Vec<_> = (0..num_particles).map(|_| Vector::zeros()).collect();
        let mut denominator: Vec<_> = (0..num_particles).map(|_| N::zero()).collect();

        for c in contacts {
            if c.i != c.j {
                let grad_i = c.gradient * particle_volumes[c.j];
                let grad_j = c.gradient * particle_volumes[c.i];

                denominator[c.i] += grad_i.norm_squared();
                denominator[c.j] += grad_j.norm_squared();

                total_gradient[c.i] += grad_i;
                total_gradient[c.j] -= grad_j;
            }
        }

        for c in boundary_contacts {
            total_gradient[c.i] += c.gradient;
            denominator[c.i] += (c.gradient * boundary_volumes[c.j]).norm_squared();
        }

        for i in 0..num_particles {
            let denominator = denominator[i] + total_gradient[i].norm_squared();
            let ci = (densities[i] - N::one()).max(N::zero());
            lambdas[i] = -ci / (denominator + na::convert(1.0e-6));
        }
    }

    fn compute_position_changes(
        contacts: &[ParticleContact<N>],
        boundary_contacts: &[BoundaryContact<N>],
        lambdas: &DVector<N>,
        particle_volumes: &DVector<N>,
        boundary_volumes: &DVector<N>,
        densities0: &DVector<N>,
        h: N,
        position_changes: &mut DVector<N>
    ) {
        position_changes.fill(N::zero());

        for c in contacts {
            // Compute virtual pressure.
            let k: N = na::convert(0.001);
            let n = 4;
            let dq = N::zero();
            let scorr = -k * (c.weight / Kernel::scalar_apply(dq, h)).powi(n);

            // Compute velocity change.
            let coeff = particle_volumes[c.j] * (lambdas[c.i] + densities0[c.j] / densities0[c.i] * lambdas[c.j])/* + scorr*/;
            position_changes.fixed_rows_mut::<Dim>(c.i * DIM).axpy(coeff, &c.gradient, N::one());

            if c.i != c.j {
                let coeff = particle_volumes[c.i] * (lambdas[c.j] + densities0[c.i] / densities0[c.j] * lambdas[c.i])/* + scorr*/;
                position_changes.fixed_rows_mut::<Dim>(c.j * DIM).axpy(-coeff, &c.gradient, N::one());
            }
        }

        for c in boundary_contacts {
            let coeff = boundary_volumes[c.j] * (lambdas[c.i] + lambdas[c.i])/* + scorr*/;
            println!("Force: {}", coeff);
            position_changes.fixed_rows_mut::<Dim>(c.i * DIM).axpy(coeff, &c.gradient, N::one());
            // XXX: apply the force to the body too.
        }
    }

    fn apply_viscosity(&mut self) {
        let viscosity: N = na::convert(0.01); // XXX

        // Add XSPH viscosity
        let mut viscosity_velocities = DVector::zeros(self.velocities.len());

        for c in &self.particle_contacts {
            if c.i != c.j {
                let vi = self.velocities.fixed_rows::<Dim>(c.i * DIM);
                let vj = self.velocities.fixed_rows::<Dim>(c.j * DIM);
                let extra_vel = (vj - vi) * c.weight;

                viscosity_velocities.fixed_rows_mut::<Dim>(c.i * DIM).add_assign(&extra_vel);
                viscosity_velocities.fixed_rows_mut::<Dim>(c.j * DIM).sub_assign(&extra_vel);
            }
        }

        self.velocities.axpy(viscosity, &viscosity_velocities, N::one());
    }
}

impl<N, Bodies, Colliders, Constraints, Kernel, KernelGradient, KernelLaplacian> MechanicalSolver<N, Bodies, Colliders, Constraints> for PBFSolver<N, Kernel, KernelGradient, KernelLaplacian>
    where N: RealField,
          Bodies: BodySet<N>,
          Colliders: ColliderSet<N, Bodies::Handle>,
          Constraints: JointConstraintSet<N, Bodies>,
          Kernel: SPHKernel,
          KernelGradient: SPHKernel,
          KernelLaplacian: SPHKernel,
{
    fn solve(
        &mut self,
        gworld: &mut GeometricalWorld<N, Bodies::Handle, Colliders::Handle>,
        counters: &mut Counters,
        bodies: &mut Bodies,
        colliders: &mut Colliders,
        constraints: &mut Constraints,
        parameters: &IntegrationParameters<N>,
        material_coefficients: &MaterialsCoefficientsTable<N>,
        active_bodies: &[Bodies::Handle], // FIXME: we should create a dedicated structure for islands.
    ) {
        let num_substeps = 1;
        let mut substep_params = parameters.clone();
        substep_params.set_dt(parameters.dt() / na::convert(num_substeps as f64));

        for _ in 0..num_substeps {
            self.init(bodies, active_bodies, substep_params.dt());
            self.init_boundary(bodies, active_bodies, gworld, colliders);
            self.reset_contact_list(gworld, colliders, bodies, active_bodies);

            let niters = 4;

            let mut lambdas = DVector::zeros(self.num_particles);
            let mut densities = DVector::zeros(self.num_particles);
            let mut position_changes = DVector::zeros(self.num_particles * DIM);
            let mut predicted_positions = self.positions.clone();
            predicted_positions.axpy(substep_params.dt(), &self.velocities, N::one());

            for loop_i in 0..niters {
                if loop_i > 0 { // At iteration 0, the contact are already up-to-date.
                    Self::update_contacts(&mut self.particle_contacts, &mut self.boundary_contacts, &predicted_positions, &self.boundary_positions, self.h);
                }

                Self::compute_densities(&self.particle_contacts, &self.boundary_contacts, &self.volumes, &self.boundary_volumes, &mut densities);
                let err = Self::compute_average_density_error(self.num_particles, &densities, &self.densities0);
                Self::compute_lambdas(self.num_particles, &self.particle_contacts, &self.boundary_contacts, &densities, &self.volumes, &self.boundary_volumes, &mut lambdas);
                Self::compute_position_changes(&self.particle_contacts, &self.boundary_contacts, &lambdas, &self.volumes, &self.boundary_volumes, &self.densities0, self.h, &mut position_changes);

//            println!("Densities: {}", densities);
                println!("Average density error: {}", err);
                println!("Average density error ratio: {}", err / self.densities0[0]);

                predicted_positions += &position_changes;
            }

            // Compute actual velocities.
            self.velocities = (&predicted_positions - &self.positions) * substep_params.inv_dt();
            self.positions = predicted_positions;
            self.apply_viscosity();
            self.output_results(bodies, active_bodies);

            for handle in active_bodies {
                if let Some(fluid) = bodies.get_as_mut::<FluidBody<N>>(*handle) {
                    fluid.sync_multiball();
                }
            }
        }
    }
}