use std::ops::{AddAssign, Range};
use std::iter;
use std::collections::HashMap;

use alga::linear::FiniteDimInnerSpace;
use na::{self, Real, DMatrix, DVector, DVectorSlice, DVectorSliceMut, LU, Dynamic, Vector2};
use ncollide::utils;
use ncollide::procedural::{self, IndexBuffer};
use ncollide::shape::{TriMesh, DeformationsType, TetrahedronPointLocation, Tetrahedron};
use ncollide::query::PointQueryWithLocation;

use object::{Body, BodyPart, BodyHandle, BodyPartHandle, BodyStatus, ActivationStatus};
use solver::{IntegrationParameters, ForceDirection};
use math::{Force, Inertia, Velocity, Vector, Point, Isometry, DIM, Dim};

/// A triangular element of the mass-spring surface.
#[derive(Clone)]
pub struct MassSpringElement<N: Real> {
    handle: Option<BodyPartHandle>,
    indices: Point<usize>,
    surface: N,
}

#[derive(Clone)]
struct Spring<N: Real> {
    elements: (usize, usize),
    rest_length: N,
}

impl<N: Real> Spring<N> {
    fn new(elements: (usize, usize), rest_length: N) -> Self {
        Spring {
            elements,
            rest_length,
        }
    }

    fn from_positions(elements: (usize, usize), positions: &[N]) -> Self {
        let p0 = Point::from_slice(&positions[elements.0..elements.0 + DIM]);
        let p1 = Point::from_slice(&positions[elements.1..elements.1 + DIM]);
        let rest_length = na::distance(&p0, &p1);

        Spring {
            elements,
            rest_length,
        }
    }
}

/// A deformable surface using a mass-spring model with triangular elements.
#[derive(Clone)]
pub struct MassSpringSurface<N: Real> {
    handle: Option<BodyHandle>,
    springs: Vec<Spring<N>>,
    elements: Vec<MassSpringElement<N>>,
    positions: DVector<N>,
    velocities: DVector<N>,
    accelerations: DVector<N>,
    augmented_mass: DMatrix<N>,
    inv_augmented_mass: LU<N, Dynamic, Dynamic>,

    companion_id: usize,
    activation: ActivationStatus<N>,
    status: BodyStatus,
    stiffness: N,
    mass: N,
}

impl<N: Real> MassSpringSurface<N> {
    /// Creates a new deformable surface following the mass-spring model.
    ///
    /// The surface is initialized with a set of links corresponding to each trimesh edges.
    pub fn new(mesh: &TriMesh<N>, mass: N, stiffness: N) -> Self {
        let ndofs = mesh.vertices().len() * DIM;
        let mut springs = HashMap::new();
        let mut elements = Vec::with_capacity(mesh.indices().len());
        let mut positions = DVector::zeros(ndofs);

        for (i, pos) in positions.as_mut_slice().chunks_mut(DIM).enumerate() {
            pos.copy_from_slice(mesh.vertices()[i].coords.as_slice())
        }

        for idx in mesh.indices() {
            let idx = idx * DIM;
            let elt = MassSpringElement {
                handle: None,
                indices: idx,
                surface: N::zero(),
            };

            elements.push(elt);

            fn key(i: usize, j: usize) -> (usize, usize) {
                if i <= j {
                    (i, j)
                } else {
                    (j, i)
                }
            }

            let _ = springs.entry(key(idx.x, idx.y)).or_insert_with(|| {
                Spring::from_positions((idx.x, idx.y), positions.as_slice())
            });
            let _ = springs.entry(key(idx.y, idx.z)).or_insert_with(|| {
                Spring::from_positions((idx.y, idx.z), positions.as_slice())
            });
            let _ = springs.entry(key(idx.z, idx.x)).or_insert_with(|| {
                Spring::from_positions((idx.z, idx.x), positions.as_slice())
            });
        }


        MassSpringSurface {
            handle: None,
            springs: springs.values().cloned().collect(),
            elements,
            positions,
            velocities: DVector::zeros(ndofs),
            accelerations: DVector::zeros(ndofs),
            augmented_mass: DMatrix::zeros(ndofs, ndofs),
            inv_augmented_mass: LU::new(DMatrix::zeros(0, 0)),
            companion_id: 0,
            activation: ActivationStatus::new_active(),
            status: BodyStatus::Dynamic,
            stiffness,
            mass,
        }
    }

    /// Creates a rectangular-shaped quad.
    pub fn quad(transform: &Isometry<N>, extents: &Vector2<N>, nx: usize, ny: usize, mass: N, stiffness: N) -> Self {
        let mesh = procedural::quad(extents.x, extents.y, nx, ny);
        let vertices = mesh.coords.iter().map(|pt| transform * pt).collect();
        let indices = mesh.indices.unwrap_unified().into_iter().map(|tri| na::convert(tri)).collect();
        let trimesh = TriMesh::new(vertices, indices, None);
        Self::new(&trimesh, mass, stiffness)
    }

    /// The triangle mesh corresponding to this mass-spring-surface structural elements.
    pub fn mesh(&self) -> TriMesh<N> {
        let vertices = self.positions.as_slice().chunks(DIM).map(|pt| Point::from_coordinates(Vector::from_row_slice(pt))).collect();
        let indices = self.elements.iter().map(|elt| elt.indices / DIM).collect();

        TriMesh::new(vertices, indices, None)
    }

    fn update_gravity_force(&mut self, gravity: &Vector<N>) {
        let split_mass = self.mass / na::convert(self.elements.len() as f64);
        let gravity_force = gravity * split_mass;

        for i in 0..self.positions.len() / DIM {
            self.accelerations.fixed_rows_mut::<Dim>(i * DIM).copy_from(&gravity_force)
        }
    }

    fn update_augmented_mass_and_forces(&mut self, gravity: &Vector<N>, params: &IntegrationParameters<N>) {
        let split_mass = self.mass / na::convert(self.elements.len() as f64);
        self.augmented_mass.fill_diagonal(split_mass);

//        for spring in self.springs {
//        }

        self.update_gravity_force(gravity);
        self.inv_augmented_mass = LU::new(self.augmented_mass.clone());
        assert!(self.inv_augmented_mass.solve_mut(&mut self.accelerations));
    }
}

impl<N: Real> Body<N> for MassSpringSurface<N> {
    fn update_kinematics(&mut self) {}

    fn update_dynamics(&mut self, gravity: &Vector<N>, params: &IntegrationParameters<N>) {
        self.update_augmented_mass_and_forces(gravity, params)
    }

    fn clear_dynamics(&mut self) {
        self.accelerations.fill(N::zero());
        self.augmented_mass.fill(N::zero());
    }

    fn apply_displacement(&mut self, disp: &[N]) {
        let disp = DVectorSlice::from_slice(disp, self.positions.len());
        self.positions += disp;
    }

    fn set_handle(&mut self, handle: Option<BodyHandle>) {
        self.handle = handle;

        for (i, element) in self.elements.iter_mut().enumerate() {
            element.handle = handle.map(|h| BodyPartHandle { body_handle: h, part_id: i })
        }
    }

    fn handle(&self) -> Option<BodyHandle> {
        self.handle
    }

    fn status(&self) -> BodyStatus {
        self.status
    }

    fn set_status(&mut self, status: BodyStatus) {
        self.status = status
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
        DVectorSlice::from_slice(self.velocities.as_slice(), self.accelerations.len())
    }

    fn companion_id(&self) -> usize {
        self.companion_id
    }

    fn set_companion_id(&mut self, id: usize) {
        self.companion_id = id
    }

    fn generalized_velocity_mut(&mut self) -> DVectorSliceMut<N> {
        DVectorSliceMut::from_slice(self.velocities.as_mut_slice(), self.accelerations.len())
    }

    fn integrate(&mut self, params: &IntegrationParameters<N>) {
        self.positions.axpy(params.dt, &self.velocities, N::one())
    }

    fn activate_with_energy(&mut self, energy: N) {
        self.activation.set_energy(energy)
    }

    fn deactivate(&mut self) {
        self.activation.set_energy(N::zero());
        self.velocities.fill(N::zero());
    }

    fn part(&self, handle: BodyPartHandle) -> &BodyPart<N> {
        &self.elements[handle.part_id]
    }

    fn part_mut(&mut self, handle: BodyPartHandle) -> &mut BodyPart<N> {
        &mut self.elements[handle.part_id]
    }

    fn contains_part(&self, handle: BodyPartHandle) -> bool {
        if let Some(me) = self.handle {
            handle.body_handle == me && handle.part_id < self.elements.len()
        } else {
            false
        }
    }

    fn deformed_positions(&self) -> Option<(DeformationsType, &[N])> {
        Some((DeformationsType::Vectors, self.positions.as_slice()))
    }

    fn deformed_positions_mut(&mut self) -> Option<(DeformationsType, &mut [N])> {
        Some((DeformationsType::Vectors, self.positions.as_mut_slice()))
    }

    fn inv_mass_mul_generalized_forces(&self, generalized_forces: &mut [N]) {
        let mut out = DVectorSliceMut::from_slice(generalized_forces, self.ndofs());
        assert!(self.inv_augmented_mass.solve_mut(&mut out))
    }

    fn body_part_jacobian_mul_unit_force(&self, part: &BodyPart<N>, point: &Point<N>, force_dir: &ForceDirection<N>, out: &mut [N]) {}

    fn body_part_point_velocity(&self, part: &BodyPart<N>, point: &Point<N>, force_dir: &ForceDirection<N>) -> N {
        N::zero()
    }
}


impl<N: Real> BodyPart<N> for MassSpringElement<N> {
    fn handle(&self) -> Option<BodyPartHandle> {
        self.handle
    }

    fn center_of_mass(&self) -> Point<N> {
        // XXX
        Point::origin()
    }

    fn position(&self) -> Isometry<N> {
        // XXX
        Isometry::identity()
    }

    fn velocity(&self) -> Velocity<N> {
        unimplemented!()
    }

    fn inertia(&self) -> Inertia<N> {
        // XXX
        Inertia::zero()
    }

    fn local_inertia(&self) -> Inertia<N> {
        // XXX
        Inertia::zero()
    }

    fn apply_force(&mut self, force: &Force<N>) {
        unimplemented!()
    }
}